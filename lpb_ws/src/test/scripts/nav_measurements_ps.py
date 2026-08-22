#!/usr/bin/env python3
# Preliminary Study metrics node.
# This script combines the preliminary goal/environment set and LiDAR collision
# logic with path-length-based timeout handling.
import rospy
import math
import time
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray, GoalID
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from datetime import datetime
from pathlib import Path as FSPath
from tf.transformations import quaternion_from_euler

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
import tf2_ros

# ─────────────────────────────────────────────────────────────────────────────
# Goal poses: x, y, theta in radians.
GOAL_POINTS = [
    {'x': 2.5, 'y': 1.5, 'theta':  math.radians(270)}, #1
    {'x': 0.5, 'y': 2.5, 'theta':  math.radians(90)},  #2
    {'x': 1.5, 'y': 0.5, 'theta':  math.radians(180)}, #3
    {'x': 2.5, 'y': 2.5, 'theta':  math.radians(0)},   #4
    {'x': 0.5, 'y': 0.5, 'theta':  math.radians(0)}    #0 (başlangıç)
]
# ─────────────────────────────────────────────────────────────────────────────

# GLOBAL STATE
collision_in_trial = False

robot_position = None
previous_position = None
total_distance = 0.0
last_twist = None

amcl_pose = None

last_plan_rx_time = rospy.Time(0)
last_plan_length = 0.0

navigation_start_time = None

# First global-plan tracking.
navigation_active = False        # Set immediately before publishing each goal.
first_plan_received = False

# Early trial termination after collision.
collision_abort = False

# TF
tfBuffer = None
tfListener = None

# Synchronization tolerances; values may be overridden through ROS parameters.
START_TOL_TF = 0.25
START_TOL_AMCL = 0.25
START_TOL_TF_AMCL = 0.25
SYNC_LOG_PERIOD = 1.0
SYNC_DWELL_SEC = 0.5
MAX_START_SYNC_WAIT = 10.0  # 0 disables the startup synchronization timeout.
MAX_TRIAL_RESETS = 3
# ─────────────────────────────────────────────────────────────────────────────

# No-motion update parameters.
NOMOTION_REPEATS = 8
NOMOTION_DT = 0.12

# Callback used to trigger a no-motion AMCL update after prolonged synchronization.
NOMOTION_POKE_FN = None

# === TELEPORT HELPERS ========================================================
def _safe_service(proxy_fn, *args, **kwargs):
    try:
        return proxy_fn(*args, **kwargs)
    except Exception as e:
        rospy.logwarn("Servis çağrısı başarısız (devam ediliyor): %s", str(e))
        return None

class TeleportHelper:
    def __init__(self, model_name="turtlebot3_burger", frame_id="map",
                 nomotion_repeats=8, nomotion_dt=0.12):
        self.model_name = model_name
        self.frame_id = frame_id
        self.nomotion_repeats = int(nomotion_repeats)
        self.nomotion_dt = float(nomotion_dt)

        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
        except rospy.ROSException:
            rospy.logwarn("/gazebo/set_model_state 10 sn içinde hazır değil (teleport yine de denenir).")

        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # clear_costmaps best-effort
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=3.0)
        except rospy.ROSException:
            rospy.logwarn("/move_base/clear_costmaps 3 sn içinde hazır değil (best-effort).")
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        # AMCL nomotion update (varsa)
        self.req_nomotion = None
        try:
            rospy.wait_for_service("/request_nomotion_update", timeout=2.0)
            self.req_nomotion = rospy.ServiceProxy("/request_nomotion_update", Empty)
            rospy.loginfo("AMCL /request_nomotion_update servisi hazır.")
        except rospy.ROSException:
            rospy.logwarn("AMCL /request_nomotion_update bulunamadı. Nomotion update atlanacak.")

        self.pub_cancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.pub_init = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def cancel_goal(self):
        self.pub_cancel.publish(GoalID())
        rospy.sleep(0.10)

    def _stop(self):
        self.pub_cmd.publish(Twist())
        rospy.sleep(0.05)

    def _publish_initialpose(self, x, y, qx, qy, qz, qw, repeats=3, hz=10.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0]*36
        cov[0] = cov[7] = 0.10**2
        cov[35] = math.radians(5.0)**2
        msg.pose.covariance = cov

        rate = rospy.Rate(hz)
        for _ in range(int(repeats)):
            msg.header.stamp = rospy.Time.now()
            self.pub_init.publish(msg)
            rate.sleep()

    def _nomotion_poke(self):
        if self.req_nomotion is None:
            return
        for _ in range(max(1, self.nomotion_repeats)):
            _safe_service(self.req_nomotion)
            rospy.sleep(self.nomotion_dt)

    def teleport_to_pose(self, x, y, theta=None, z=0.0):
        self.cancel_goal()
        self._stop()

        if theta is None:
            theta = 0.0
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(theta))

        ms = ModelState()
        ms.model_name = self.model_name
        ms.pose.position.x = float(x)
        ms.pose.position.y = float(y)
        ms.pose.position.z = float(z)
        ms.pose.orientation.x = qx
        ms.pose.orientation.y = qy
        ms.pose.orientation.z = qz
        ms.pose.orientation.w = qw
        ms.twist = Twist()
        ms.reference_frame = "world"

        _safe_service(self.set_state, ms)

        self._publish_initialpose(float(x), float(y), qx, qy, qz, qw, repeats=3, hz=10.0)
        _safe_service(self.clear_costmaps)

        # Trigger AMCL after teleporting the robot.
        self._nomotion_poke()

        self._stop()
        rospy.sleep(0.20)
# ============================================================================

# === BASIC UTILS =============================================================
def _amcl_store(msg):
    global amcl_pose
    amcl_pose = msg.pose.pose

def calculate_distance(pos1, pos2):
    return math.sqrt((pos1.position.x - pos2.position.x)**2 + (pos1.position.y - pos2.position.y)**2)

def odom_callback(msg):
    global robot_position, previous_position, total_distance, last_twist
    robot_position = msg.pose.pose
    last_twist = msg.twist.twist
    if previous_position is not None:
        total_distance += calculate_distance(previous_position, robot_position)
    previous_position = robot_position

def dist_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def get_robot_xy_from_tf(timeout=0.5):
    global tfBuffer
    if tfBuffer is None:
        return None
    try:
        trans = tfBuffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(timeout))
        return (trans.transform.translation.x, trans.transform.translation.y)
    except Exception:
        return None

def as_xy(p):
    try:
        if hasattr(p, 'position') and hasattr(p.position, 'x') and hasattr(p.position, 'y'):
            return (p.position.x, p.position.y)
        if hasattr(p, 'x') and hasattr(p, 'y'):
            return (p.x, p.y)
        if isinstance(p, dict):
            return (p['x'], p['y'])
    except Exception:
        pass
    return (0.0, 0.0)
# ============================================================================

# === GLOBAL PLAN CALLBACK ====================================================
def global_plan_callback(msg):
    global last_plan_rx_time, last_plan_length
    global navigation_active, first_plan_received

    if not navigation_active:
        return

    if first_plan_received:
        return

    if not msg.poses:
        return

    path_len = 0.0
    if len(msg.poses) >= 2:
        last_pose = msg.poses[0].pose.position
        for pose_stamped in msg.poses[1:]:
            pos = pose_stamped.pose.position
            path_len += math.hypot(pos.x - last_pose.x, pos.y - last_pose.y)
            last_pose = pos

    last_plan_rx_time = rospy.Time.now()
    last_plan_length = path_len
    first_plan_received = True

def wait_for_first_plan_since(since_time, timeout=20.0):
    global last_plan_rx_time, last_plan_length
    start = rospy.Time.now()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if (rospy.Time.now() - start).to_sec() > timeout:
            return float('nan')
        if first_plan_received and last_plan_rx_time >= since_time:
            return last_plan_length
        rate.sleep()
# ============================================================================

# === LIDAR COLLISION DETECTION ===============================================
def _min_valid_range(ranges):
    vals = [r for r in ranges if math.isfinite(r) and r > 0.0]
    return min(vals) if vals else float('inf')

in_collision = False
collision_enter_started = None
collision_exit_started  = None
ENTER_THRESHOLD = 0.12
EXIT_THRESHOLD  = 0.15
DWELL_ENTER     = 2.0
DWELL_EXIT      = 0.5

def lidar_callback(msg):
    global in_collision, collision_enter_started, collision_exit_started
    global collision_abort, collision_in_trial

    rng = _min_valid_range(msg.ranges)
    now = rospy.Time.now().to_sec()

    if rng < ENTER_THRESHOLD:
        collision_exit_started = None
        if not in_collision:
            if collision_enter_started is None:
                collision_enter_started = now
            elif (now - collision_enter_started) >= DWELL_ENTER:
                rospy.logwarn("Collision detected via LiDAR.")
                in_collision = True
                collision_in_trial = True
                collision_enter_started = None
                collision_abort = True
        return

    if rng >= EXIT_THRESHOLD:
        collision_enter_started = None
        if in_collision:
            if collision_exit_started is None:
                collision_exit_started = now
            elif (now - collision_exit_started) >= DWELL_EXIT:
                in_collision = False
                collision_exit_started = None
        return

    return
# ============================================================================

# === AMCL/TF SYNC (DWELL STABILITY) =========================================
def wait_for_localization_ready(expected_xy, label="", rate_hz=10.0):
    global amcl_pose, NOMOTION_POKE_FN, last_twist
    rate = rospy.Rate(rate_hz)

    last_log_t = 0.0
    dwell_start = None
    warn_after = rospy.Time.now().to_sec() + (MAX_START_SYNC_WAIT if MAX_START_SYNC_WAIT and MAX_START_SYNC_WAIT > 0 else 1e18)

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()

        tf_xy = get_robot_xy_from_tf(timeout=0.5)
        amcl_xy = None
        if amcl_pose is not None:
            amcl_xy = (amcl_pose.position.x, amcl_pose.position.y)

        ok = True
        reasons = []

        if tf_xy is None:
            ok = False
            reasons.append("TF yok")
        else:
            d_tf = dist_xy(tf_xy, expected_xy)
            if d_tf > START_TOL_TF:
                ok = False
                reasons.append(f"TF uzak (d={d_tf:.3f} > {START_TOL_TF:.2f})")

        if amcl_xy is None:
            ok = False
            reasons.append("AMCL yok")
        else:
            d_amcl = dist_xy(amcl_xy, expected_xy)
            if d_amcl > START_TOL_AMCL:
                ok = False
                reasons.append(f"AMCL uzak (d={d_amcl:.3f} > {START_TOL_AMCL:.2f})")

        if tf_xy is not None and amcl_xy is not None:
            d_ta = dist_xy(tf_xy, amcl_xy)
            if d_ta > START_TOL_TF_AMCL:
                ok = False
                reasons.append(f"TF-AMCL uyumsuz (d={d_ta:.3f} > {START_TOL_TF_AMCL:.2f})")

        if ok:
            if dwell_start is None:
                dwell_start = now
            if (now - dwell_start) >= SYNC_DWELL_SEC:
                rospy.loginfo(f"[SYNC OK] {label} expected=({expected_xy[0]:.2f},{expected_xy[1]:.2f}) "
                              f"tf=({tf_xy[0]:.2f},{tf_xy[1]:.2f}) amcl=({amcl_xy[0]:.2f},{amcl_xy[1]:.2f}) "
                              f"dwell={SYNC_DWELL_SEC:.2f}s")
                return (tf_xy, amcl_xy)
        else:
            dwell_start = None

        if now - last_log_t >= SYNC_LOG_PERIOD:
            last_log_t = now
            msg = ", ".join(reasons) if reasons else "hazır değil"
            rospy.logwarn(f"[SYNC WAIT] {label}: {msg} | expected=({expected_xy[0]:.2f},{expected_xy[1]:.2f})")

        if now >= warn_after:
            rospy.logwarn(f"[SYNC LONG WAIT] {label}: {MAX_START_SYNC_WAIT:.1f}s aşıldı, hâlâ bekleniyor...")

            # Request a no-motion update after prolonged synchronization.
            try:
                # Trigger only while the robot is stationary when possible.
                moving = False
                if last_twist is not None:
                    v = abs(last_twist.linear.x)
                    w = abs(last_twist.angular.z)
                    moving = (v > 0.02) or (w > 0.05)

                if (not moving) and (NOMOTION_POKE_FN is not None):
                    rospy.logwarn(f"[SYNC LONG WAIT] {label}: AMCL nomotion update tetikleniyor...")
                    NOMOTION_POKE_FN()
            except Exception as e:
                rospy.logwarn("Nomotion poke başarısız (devam): %s", str(e))

            warn_after = now + (MAX_START_SYNC_WAIT if MAX_START_SYNC_WAIT and MAX_START_SYNC_WAIT > 0 else 1e18)

        rate.sleep()

    return (None, None)
# ============================================================================

# === PUBLISH INITIALPOSE / GOAL ==============================================
def wait_for_connections(pub, timeout=2.0):
    start = time.time()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        elapsed = time.time() - start
        rospy.logwarn(f"/initialpose abone bekleniyor... ({elapsed:.1f}s)")
        if elapsed > timeout:
            return False
        time.sleep(0.5)
    return True

def wait_for_amcl_update(since_time, timeout=5.0):
    got = {'ok': False}
    def _cb(msg):
        if msg.header.stamp >= since_time:
            got['ok'] = True

    sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, _cb)
    start = rospy.Time.now()
    rate = rospy.Rate(50)
    try:
        while not rospy.is_shutdown():
            if got['ok']:
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()
    finally:
        sub.unregister()

def publish_initial_pose(pub_initial):
    if not wait_for_connections(pub_initial, timeout=10.0):
        rospy.logerr("10 sn boyunca /initialpose abone bağlanmadı! Kapatılıyor.")
        rospy.signal_shutdown("AMCL/initialpose subscriber yok")
        return False

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = 0.5
    initial_pose.pose.pose.position.y = 0.5
    initial_pose.pose.pose.position.z = 0.0

    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose.pose.pose.orientation.x = qx
    initial_pose.pose.pose.orientation.y = qy
    initial_pose.pose.pose.orientation.z = qz
    initial_pose.pose.pose.orientation.w = qw

    cov = [0.0]*36
    cov[0]  = 0.10**2
    cov[7]  = 0.10**2
    cov[35] = math.radians(5.0)**2
    initial_pose.pose.covariance = cov

    rate = rospy.Rate(10)
    for _ in range(3):
        initial_pose.header.stamp = rospy.Time.now()
        pub_initial.publish(initial_pose)
        rate.sleep()

    rospy.loginfo("Initial pose published.")
    return True

def publish_goal(pub_goal, goal):
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"
    goal_msg.pose.position.x = float(goal['x'])
    goal_msg.pose.position.y = float(goal['y'])
    goal_msg.pose.position.z = float(goal.get('z', 0.0))

    theta = float(goal.get('theta', 0.0))
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
    goal_msg.pose.orientation.x = qx
    goal_msg.pose.orientation.y = qy
    goal_msg.pose.orientation.z = qz
    goal_msg.pose.orientation.w = qw

    pub_goal.publish(goal_msg)
    rospy.loginfo(f"Goal published: x={goal['x']}, y={goal['y']}, theta={theta:.2f} rad")
    return goal_msg.header.stamp
# ============================================================================

# === GOAL REACHED ============================================================
def wait_until_robot_stopped(v_th=0.02, w_th=0.05, dwell=0.5, timeout=3.0):
    start = rospy.Time.now()
    ok_since = None
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if (rospy.Time.now() - start).to_sec() > timeout:
            return False
        if last_twist is None:
            rate.sleep()
            continue
        v = abs(last_twist.linear.x)
        w = abs(last_twist.angular.z)
        moving = (v > v_th) or (w > w_th)
        if moving:
            ok_since = None
        else:
            if ok_since is None:
                ok_since = rospy.Time.now()
            elif (rospy.Time.now() - ok_since).to_sec() >= dwell:
                return True
        rate.sleep()

def check_goal_reached(since_time, timeout_t):
    global collision_abort, world

    goal_reached = False

    def status_callback(msg):
        nonlocal goal_reached
        for st in msg.status_list:
            if st.status == 3 and st.goal_id.stamp >= since_time:
                goal_reached = True
                break

    sub = rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)

    while (not rospy.is_shutdown()
           and not goal_reached
           and not collision_abort
           and rospy.Time.now() < timeout_t):
        rospy.sleep(0.1)

    sub.unregister()

    if goal_reached and (not collision_abort):
        rospy.loginfo("Goal reached!")
        _ = wait_until_robot_stopped()
        return True

    if collision_abort:
        rospy.logwarn("Deneme çarpışma nedeniyle erken sonlandırıldı.")
    else:
        rospy.logwarn("Goal not reached within timeout.")

    return False
# ============================================================================

# === CSV =====================================================================
def write_trial_row(writer, trial_idx, start_xy, goal_xy,
                    nav_time, avg_speed, collision, timeout, success):
    writer.writerow({
        'Trial': trial_idx,
        'From (x,y)': f"({start_xy[0]:.2f}, {start_xy[1]:.2f})",
        'To (x,y)': f"({goal_xy[0]:.2f}, {goal_xy[1]:.2f})",
        'Navigation Time (s)': f"{nav_time:.4f}",
        'Average Speed (m/s)': f"{avg_speed:.4f}",
        'Collision': ('Yes' if collision else 'No'),
        'Timeout': ('Yes' if timeout else 'No'),
        'Success': ('Yes' if success else 'No')
    })
# ============================================================================

def main():
    rospy.init_node('sınama_düğümü', anonymous=True)

    global total_distance, previous_position
    global in_collision, collision_in_trial
    global navigation_active, first_plan_received
    global collision_abort, tfBuffer, tfListener
    global START_TOL_TF, START_TOL_AMCL, START_TOL_TF_AMCL, SYNC_LOG_PERIOD, SYNC_DWELL_SEC, MAX_START_SYNC_WAIT
    global world, local_planner
    global last_plan_rx_time, last_plan_length
    global NOMOTION_REPEATS, NOMOTION_DT
    global NOMOTION_POKE_FN

    SCRIPT_DIR = FSPath(__file__).resolve().parent
    TEST_ROOT = SCRIPT_DIR.parent
    RESULTS_ROOT = TEST_ROOT / "docs_tables_media" / "preliminary_study" / "raw_results"

    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    config        = rospy.get_param('~config', "same")
    world         = rospy.get_param('~world', "bosOrt")
    local_planner = rospy.get_param('~local_planner', "dwa")

    # Synchronization parameters can be overridden through ROS parameters.
    START_TOL_TF = rospy.get_param('~start_tol_tf', START_TOL_TF)
    START_TOL_AMCL = rospy.get_param('~start_tol_amcl', START_TOL_AMCL)
    START_TOL_TF_AMCL = rospy.get_param('~start_tol_tf_amcl', START_TOL_TF_AMCL)
    SYNC_LOG_PERIOD = rospy.get_param('~sync_log_period', SYNC_LOG_PERIOD)
    SYNC_DWELL_SEC = rospy.get_param('~sync_dwell_sec', SYNC_DWELL_SEC)
    MAX_START_SYNC_WAIT = rospy.get_param('~max_start_sync_wait', MAX_START_SYNC_WAIT)

    # No-motion update parameters can be overridden through ROS parameters.
    NOMOTION_REPEATS = int(rospy.get_param('~nomotion_repeats', NOMOTION_REPEATS))
    NOMOTION_DT = float(rospy.get_param('~nomotion_dt', NOMOTION_DT))

    teleport_enabled = rospy.get_param('~teleport_on_timeout', True)
    model_name       = rospy.get_param('~model_name', 'turtlebot3_burger')
    repeats          = rospy.get_param('~repeats', 2)

    directory = RESULTS_ROOT / config / world
    directory.mkdir(parents=True, exist_ok=True)
    filename  = f"{local_planner}_{timestamp}.csv"
    filepath  = directory / filename

    pub_initial = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, global_plan_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, _amcl_store)

    helper = TeleportHelper(
        model_name=model_name,
        frame_id="map",
        nomotion_repeats=NOMOTION_REPEATS,
        nomotion_dt=NOMOTION_DT
    )

    # The synchronization routine calls this callback after prolonged waiting.
    NOMOTION_POKE_FN = helper._nomotion_poke

    # TF init
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    if not tfBuffer.can_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(5.0)):
        rospy.logerr("TF map→base_footprint hazır değil (5 sn)!")
        rospy.signal_shutdown("TF not ready")
        return

    # initial pose
    since = rospy.Time.now()
    if not publish_initial_pose(pub_initial):
        return
    if not wait_for_amcl_update(since_time=since, timeout=10.0):
        rospy.logerr("Initialpose sonrası AMCL güncellemesi alınamadı.")
        rospy.signal_shutdown("AMCL not updating")
        return

    with open(filepath, 'w', newline='') as csvfile:
        fieldnames = ['Trial',
                      'From (x,y)',
                      'To (x,y)',
                      'Navigation Time (s)',
                      'Average Speed (m/s)',
                      'Collision',
                      'Timeout',
                      'Success']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        sum_nav = sum_speed = 0.0
        count_collision_yes = 0
        count_rows = 0
        count_timeout_yes = 0
        count_success_yes = 0

        for i in range(repeats):
            if rospy.is_shutdown():
                break

            total_distance = 0.0
            previous_position = None

            for gi, goal in enumerate(GOAL_POINTS, start=1):
                if rospy.is_shutdown():
                    break

                if gi == 1:
                    expected = GOAL_POINTS[-1]
                else:
                    expected = GOAL_POINTS[gi - 2]
                expected_xy = (float(expected['x']), float(expected['y']))

                trial_reset_count = 0
                while not rospy.is_shutdown():
                    collision_abort = False
                    collision_in_trial = False
                    in_collision = False
                    first_plan_received = False
                    navigation_active = False

                    tf_xy, amcl_xy = wait_for_localization_ready(
                        expected_xy=expected_xy,
                        label=f"rep{i+1}_trial{(i*len(GOAL_POINTS))+gi}_start_sync",
                        rate_hz=10.0
                    )
                    if tf_xy is None:
                        trial_reset_count += 1
                        rospy.logwarn(f"Trial {(i * len(GOAL_POINTS)) + gi} start sync failed. Retrying ({trial_reset_count}/{MAX_TRIAL_RESETS}).")
                        helper.cancel_goal()
                        if trial_reset_count > MAX_TRIAL_RESETS:
                            rospy.logerr("Trial setup failed after %d resets; stopping experiment.", MAX_TRIAL_RESETS)
                            rospy.signal_shutdown("Trial setup failed")
                            return
                        helper.teleport_to_pose(
                            x=float(expected['x']),
                            y=float(expected['y']),
                            theta=float(expected.get('theta', 0.0)),
                            z=float(expected.get('z', 0.0))
                        )
                        continue

                    current_start_point = {'x': tf_xy[0], 'y': tf_xy[1]}
                    prev_total_distance = total_distance

                    last_plan_rx_time = rospy.Time(0)
                    last_plan_length = 0.0
                    first_plan_received = False

                    navigation_active = True
                    goal_since = publish_goal(pub_goal, goal)
                    planned_len = wait_for_first_plan_since(goal_since, timeout=20.0)

                    if not math.isfinite(planned_len) or planned_len <= 0.0:
                        navigation_active = False
                        trial_reset_count += 1
                        rospy.logwarn(f"Trial {(i * len(GOAL_POINTS)) + gi} initial global plan was not obtained. Retrying ({trial_reset_count}/{MAX_TRIAL_RESETS}).")
                        helper.cancel_goal()
                        if trial_reset_count > MAX_TRIAL_RESETS:
                            rospy.logerr("Initial global plan unavailable after %d resets; stopping experiment.", MAX_TRIAL_RESETS)
                            rospy.signal_shutdown("Initial global plan unavailable")
                            return
                        helper.teleport_to_pose(
                            x=float(expected['x']),
                            y=float(expected['y']),
                            theta=float(expected.get('theta', 0.0)),
                            z=float(expected.get('z', 0.0))
                        )
                        continue

                    # 0.22 m/s: TurtleBot3 Burger maximum linear velocity.
                    # Preliminary Study timeout rule:
                    #   bosOrt      -> 3 times nominal duration.
                    #   duraganOrt3 -> 5 times nominal duration.
                    if world == "bosOrt":
                        timeout_duration = (planned_len / 0.22) * (1 + 2.00)
                    elif world in ("duraganOrt", "duraganOrt3"):
                        timeout_duration = (planned_len / 0.22) * (1 + 4.00)
                    else:
                        rospy.logerr("Bilinmeyen world için timeout tanımlı değil: %s", world)
                        navigation_active = False
                        rospy.signal_shutdown("Unknown world for timeout")
                        return

                    break

                if rospy.is_shutdown():
                    break

                goal_nav_start = rospy.Time.now()
                timeout_t = goal_nav_start + rospy.Duration(timeout_duration)

                reached = check_goal_reached(goal_since, timeout_t)
                navigation_active = False

                goal_end = rospy.Time.now()
                goal_distance = total_distance - prev_total_distance

                goal_nav_time = (goal_end - goal_nav_start).to_sec()
                goal_avg_speed = (goal_distance / goal_nav_time) if goal_nav_time > 0 else 0.0

                collided = bool(collision_in_trial)
                timeout_flag = (not reached) and (not collided)

                start_xy = as_xy(current_start_point)

                write_trial_row(
                    writer=writer,
                    trial_idx=(i * len(GOAL_POINTS)) + gi,
                    start_xy=start_xy,
                    goal_xy=(goal['x'], goal['y']),
                    nav_time=goal_nav_time,
                    avg_speed=goal_avg_speed,
                    collision=collided,
                    timeout=timeout_flag,
                    success=reached
                )

                sum_nav += goal_nav_time
                sum_speed += goal_avg_speed
                count_collision_yes += (1 if collided else 0)
                count_timeout_yes += (1 if timeout_flag else 0)
                count_success_yes += (1 if reached else 0)
                count_rows += 1

                rospy.loginfo(f"{(i * len(GOAL_POINTS)) + gi}. trial completed.")

                if (timeout_flag or collided) and teleport_enabled:
                    try:
                        helper.cancel_goal()

                        helper.teleport_to_pose(
                            x=float(goal['x']),
                            y=float(goal['y']),
                            theta=float(goal.get('theta', 0.0)),
                            z=float(goal.get('z', 0.0))
                        )

                        navigation_active = False
                        first_plan_received = False

                        wait_for_localization_ready(
                            expected_xy=(float(goal['x']), float(goal['y'])),
                            label=f"rep{i+1}_trial{(i*len(GOAL_POINTS))+gi}_post_teleport_sync",
                            rate_hz=10.0
                        )

                        rospy.logwarn("Deneme sonlandırıldı ({}). Teleport+Sync tamam."
                                      .format("collision" if collided else "timeout"))
                    except Exception as e:
                        rospy.logerr("Teleport denemesi başarısız: %s", str(e))

        avg_nav   = (sum_nav / count_rows) if count_rows else 0.0
        avg_speed = (sum_speed / count_rows) if count_rows else 0.0

        writer.writerow({
            'Trial': 'General Result',
            'From (x,y)': '',
            'To (x,y)': '',
            'Navigation Time (s)': f"{avg_nav:.4f}",
            'Average Speed (m/s)': f"{avg_speed:.4f}",
            'Collision': int(count_collision_yes),
            'Timeout': int(count_timeout_yes),
            'Success': int(count_success_yes)
        })

    rospy.loginfo("Navigation completed. Metrics saved to %s", str(filepath))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
