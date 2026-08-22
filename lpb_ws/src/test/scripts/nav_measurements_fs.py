#!/usr/bin/env python3
import rospy
import math
import os
import time
import csv
import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray, GoalID
from gazebo_msgs.msg import ContactsState
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
    {'x': 2.5, 'y': 2.0, 'theta':  math.radians(45)},  #1
    {'x': 0.5, 'y': 2.5, 'theta':  math.radians(180)}, #2
    {'x': 2.5, 'y': 0.5, 'theta': -math.radians(45)},  #3
    {'x': 0.5, 'y': 2.0, 'theta':  math.radians(90)},  #4
    {'x': 2.5, 'y': 1.0, 'theta': -math.radians(90)},  #5
    {'x': 1.0, 'y': 2.5, 'theta':  math.radians(0)},   #6
    {'x': 1.0, 'y': 0.5, 'theta':  math.radians(45)},  #7
    {'x': 2.0, 'y': 2.5, 'theta':  math.radians(90)},  #8
    {'x': 2.0, 'y': 0.5, 'theta':  math.radians(45)},  #9
    {'x': 0.5, 'y': 0.5, 'theta':  math.radians(180)}  #10-0 (başlangıç)
]
# ─────────────────────────────────────────────────────────────────────────────

# Static objects in the empty environment.
STATIC_OBJECTS_BOS_ORT = [
    {'x': 1.65,   'y': -0.075, 'width': 3.0, 'height': 0.15},
    {'x': -0.075, 'y': 1.65,   'width': 3.0, 'height': 0.15},
    {'x': 1.65,   'y': 3.075,  'width': 3.0, 'height': 0.15},
    {'x': 3.075,  'y': 1.65,   'width': 3.0, 'height': 0.15}
]

# Static objects in the static-obstacle environment.
STATIC_OBJECTS_DURAGAN_ORT = [
    # Walls
    {'x': 1.65,   'y': -0.075, 'width': 3.0, 'height': 0.15},
    {'x': -0.075, 'y': 1.65,   'width': 3.0, 'height': 0.15},
    {'x': 1.65,   'y': 3.075,  'width': 3.0, 'height': 0.15},
    {'x': 3.075,  'y': 1.65,   'width': 3.0, 'height': 0.15},
    # Unit Boxes
    {'x': 1, 'y': 1, 'width': 0.25, 'height': 0.25},
    {'x': 2, 'y': 1, 'width': 0.25, 'height': 0.25},
    {'x': 1, 'y': 2, 'width': 0.25, 'height': 0.25},
    {'x': 2, 'y': 2, 'width': 0.25, 'height': 0.25}
]

# ─────────────────────────────────────────────────────────────────────────────
# GLOBAL STATE
static_collision_counter = 0
dynamic_collision_counter = 0

deviation_timer = None
deviation_time = 0.0

robot_position = None
previous_position = None
total_distance = 0.0
last_twist = None

amcl_pose = None

global_path = []
global_path_length = 0.0
last_plan_stamp = rospy.Time(0)
last_plan_length = 0.0

navigation_start_time = None

# Local planner failure detection through replanning.
navigation_active = False        # Set immediately before publishing each goal.
first_plan_received = False
replan_detected = False

# Global plan tracking state.
last_plan_rx_time = rospy.Time(0)
last_plan_msg_stamp = rospy.Time(0)
last_plan_length = 0.0

# Early trial termination after collision.
collision_abort = False
static_collision_in_trial = False
dynamic_collision_in_trial = False

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

# === GLOBAL PLAN CALLBACK (Local Planner Failure via replan) ============================
def global_plan_callback(msg):
    global global_path, global_path_length
    global last_plan_rx_time, last_plan_msg_stamp, last_plan_length
    global navigation_active, first_plan_received, replan_detected

    if not navigation_active:
        return

    rx_time = rospy.Time.now()

    if first_plan_received:
        if not replan_detected:
            rospy.logwarn("Replan detected: ikinci global plan geldi. Local Planner Failure işaretleniyor.")
        replan_detected = True
        return

    first_plan_received = True

    if not msg.poses:
        rospy.logwarn("Global plan mesajı geldi ama poses boş.")
        return

    global_path = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]

    path_len = 0.0
    for a, b in zip(msg.poses[:-1], msg.poses[1:]):
        ax, ay = a.pose.position.x, a.pose.position.y
        bx, by = b.pose.position.x, b.pose.position.y
        path_len += math.hypot(bx - ax, by - ay)

    global_path_length = path_len
    last_plan_length = path_len
    last_plan_msg_stamp = msg.header.stamp
    last_plan_rx_time = rx_time

    rospy.loginfo(
        "Global plan alındı | rx=%.3f msg_stamp=%.3f poses=%d len=%.3f",
        rx_time.to_sec(),
        msg.header.stamp.to_sec(),
        len(msg.poses),
        path_len
    )

def wait_for_first_plan_since(since_time, timeout=20.0):
    global last_plan_rx_time, last_plan_length
    start = rospy.Time.now()
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("İlk global plan %0.1f sn içinde alınamadı.", timeout)
            return float('nan')

        if last_plan_rx_time >= since_time:
            return last_plan_length

        rate.sleep()
# ============================================================================

# === CONTACT-SENSOR COLLISION DETECTION ======================================
in_collision = False

STATIC_OBSTACLE_NAMES = ("bosOrt", "duraganOrt")
DYNAMIC_OBSTACLE_NAMES = ("hareketliEngel", "hareketliEngel_clone")

BUMPER_TOPICS = [
    '/bumper_states_base_body',
    '/bumper_states_caster_back',
    '/bumper_states_base_scan',
    '/bumper_states_wheel_left',
    '/bumper_states_wheel_right',
]

def _classify_contact_pair(name1, name2):
    """
    Bir temas çiftini sınıflandırır.
    Sadece robot <-> anlamlı engel temasları dikkate alınır.
    Geri dönüş:
      "static" / "dynamic" / None
    """
    if not name1 or not name2:
        return None

    n1 = name1.lower()
    n2 = name2.lower()

    robot1 = "turtlebot3_burger" in n1
    robot2 = "turtlebot3_burger" in n2

    # Robot hiç yoksa bu temas bizi ilgilendirmez
    if not (robot1 or robot2):
        return None

    # Robot olmayan tarafı al
    other = n2 if robot1 else n1

    # Gürültüleri ele
    if "ground_plane" in other:
        return None

    # Statik engel
    if any(s.lower() in other for s in STATIC_OBSTACLE_NAMES):
        return "static"

    # Dinamik engel
    if any(d.lower() in other for d in DYNAMIC_OBSTACLE_NAMES):
        return "dynamic"

    return None

def bumper_callback(msg):
    global in_collision
    global static_collision_counter, dynamic_collision_counter
    global collision_abort, navigation_active
    global static_collision_in_trial, dynamic_collision_in_trial

    # Sadece aktif navigasyon sırasında çarpışma say
    if not navigation_active:
        return

    # Bu trial zaten çarpışma ile bitirildiyse, devamını yok say
    if collision_abort:
        return

    detected_type = None

    for st in msg.states:
        detected_type = _classify_contact_pair(st.collision1_name, st.collision2_name)
        if detected_type is not None:
            break

    if detected_type is None:
        return

    if detected_type == "static":
        if not static_collision_in_trial:
            static_collision_counter += 1
            static_collision_in_trial = True
            rospy.logwarn("Static collision detected via contact sensor!")
    else:
        if not dynamic_collision_in_trial:
            dynamic_collision_counter += 1
            dynamic_collision_in_trial = True
            rospy.logwarn("Dynamic collision detected via contact sensor!")

    collision_abort = True
    in_collision = True
# ============================================================================

# === DEVIATION TIMER =========================================================
def check_deviation():
    global deviation_timer, deviation_time, global_path, amcl_pose
    if not global_path or amcl_pose is None:
        return

    min_distance = float('inf')
    for point in global_path:
        d = math.hypot(amcl_pose.position.x - point['x'], amcl_pose.position.y - point['y'])
        min_distance = min(min_distance, d)

    if min_distance > 0.1:
        if deviation_timer is None:
            deviation_timer = rospy.Time.now().to_sec()
    else:
        if deviation_timer is not None:
            deviation_time += (rospy.Time.now().to_sec() - deviation_timer)
            deviation_timer = None
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

            try:
                moving = False
                if last_twist is not None:
                    v = abs(last_twist.linear.x)
                    w = abs(last_twist.angular.z)
                    moving = (v > 0.02) or (w > 0.05)

                if (not moving) and (NOMOTION_POKE_FN is not None):
                    rospy.logwarn(f"[SYNC LONG WAIT] {label}: AMCL nomotion update tetikleniyor...")
                    NOMOTION_POKE_FN()
                    rospy.sleep(1.0)
            except Exception as e:
                rospy.logwarn("Nomotion poke başarısız (devam): %s", str(e))

            rospy.logerr(f"[SYNC FAIL] {label}: senkron alınamadı, trial reset istenecek.")
            return (None, None)

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
    global collision_abort, replan_detected, world

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
           and not replan_detected
           and rospy.Time.now() < timeout_t):
        rospy.sleep(0.1)

    sub.unregister()

    if goal_reached and (not collision_abort) and (not replan_detected):
        rospy.loginfo("Goal reached!")
        _ = wait_until_robot_stopped()
        return True

    if collision_abort:
        rospy.logwarn("Deneme çarpışma nedeniyle erken sonlandırıldı.")
    elif replan_detected:
        rospy.logwarn("Deneme local planner fail (replan) nedeniyle erken sonlandırıldı.")
    else:
        rospy.logwarn("Goal not reached within timeout.")

    return False
# ============================================================================

# === CSV =====================================================================
def write_trial_row(writer, trial_idx, start_xy, goal_xy,
                    dev_time, nav_time, pln_path, exc_path, avg_speed,
                    stat_col, dyn_col, local_fail, timeout, success):
    writer.writerow({
        'Trial': trial_idx,
        'From (x,y)': f"({start_xy[0]:.2f}, {start_xy[1]:.2f})",
        'To (x,y)': f"({goal_xy[0]:.2f}, {goal_xy[1]:.2f})",
        'Deviation Time (s)': f"{dev_time:.4f}",
        'Navigation Time (s)': f"{nav_time:.4f}",
        'Planned Path Length (m)': (f"{pln_path:.4f}" if isinstance(pln_path, float) and math.isfinite(pln_path) else ""),
        'Executed Path Length (m)': f"{exc_path:.4f}",
        'Average Speed (m/s)': f"{avg_speed:.4f}",
        'Static Collisions': ('Yes' if stat_col > 0 else 'No'),
        'Dynamic Collisions': ('Yes' if dyn_col > 0 else 'No'),
        'Local Planner Failure': ('Yes' if local_fail else 'No'),
        'Timeout': ('Yes' if timeout else 'No'),
        'Success': ('Yes' if success else 'No')
    })
# ============================================================================

def main():
    rospy.init_node('sınama_düğümü', anonymous=True)

    global deviation_time, deviation_timer, total_distance, previous_position
    global in_collision, static_collision_counter, dynamic_collision_counter
    global static_collision_in_trial, dynamic_collision_in_trial
    global navigation_active, first_plan_received, replan_detected
    global collision_abort, tfBuffer, tfListener
    global START_TOL_TF, START_TOL_AMCL, START_TOL_TF_AMCL, SYNC_LOG_PERIOD, SYNC_DWELL_SEC, MAX_START_SYNC_WAIT
    global world, local_planner
    global global_path, global_path_length, last_plan_stamp, last_plan_length, last_plan_rx_time, last_plan_msg_stamp
    global NOMOTION_REPEATS, NOMOTION_DT
    global NOMOTION_POKE_FN

    SCRIPT_DIR = FSPath(__file__).resolve().parent
    TEST_ROOT = SCRIPT_DIR.parent
    RESULTS_ROOT = TEST_ROOT / "docs_tables_media" / "full_study" / "raw_results"

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

    for topic in BUMPER_TOPICS:
        rospy.Subscriber(topic, ContactsState, bumper_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, global_plan_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, _amcl_store)

    rospy.Timer(rospy.Duration(0.1), lambda event: check_deviation())

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
                      'Deviation Time (s)',
                      'Navigation Time (s)',
                      'Planned Path Length (m)',
                      'Executed Path Length (m)',
                      'Average Speed (m/s)',
                      'Static Collisions',
                      'Dynamic Collisions',
                      'Local Planner Failure',
                      'Timeout',
                      'Success']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        # aggregates
        sum_dev = sum_nav = sum_planned = sum_exec = sum_speed = 0.0
        sum_stat = sum_dyn = 0
        count_rows = 0
        count_planned = 0
        count_timeout_yes = 0
        count_success_yes = 0
        count_local_fail_yes = 0

        for i in range(repeats):
            if rospy.is_shutdown():
                break

            static_collision_counter = 0
            dynamic_collision_counter = 0
            in_collision = False

            total_distance = 0.0
            deviation_time = 0.0
            previous_position = None
            deviation_timer = None

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
                    in_collision = False
                    replan_detected = False
                    first_plan_received = False
                    navigation_active = False
                    static_collision_in_trial = False
                    dynamic_collision_in_trial = False

                    tf_xy, amcl_xy = wait_for_localization_ready(
                        expected_xy=expected_xy,
                        label=f"rep{i+1}_trial{(i*len(GOAL_POINTS))+gi}_start_sync",
                        rate_hz=10.0
                    )

                    # Retry the same trial from a reset pose if startup synchronization fails.
                    if tf_xy is None:
                        trial_reset_count += 1
                        rospy.logwarn(f"Trial {(i * len(GOAL_POINTS)) + gi} start sync başarısız. Reset deneniyor ({trial_reset_count}/{MAX_TRIAL_RESETS}).")

                        if trial_reset_count > MAX_TRIAL_RESETS:
                            rospy.logerr("Maksimum trial reset sayısı aşıldı; test durduruluyor.")
                            rospy.signal_shutdown("Trial reset limit exceeded")
                            return

                        try:
                            helper.cancel_goal()
                            helper.teleport_to_pose(
                                x=float(expected['x']),
                                y=float(expected['y']),
                                theta=float(expected.get('theta', 0.0)),
                                z=float(expected.get('z', 0.0))
                            )
                        except Exception as e:
                            rospy.logerr("Trial reset teleport başarısız: %s", str(e))

                        continue

                    current_start_point = {'x': tf_xy[0], 'y': tf_xy[1]}

                    prev_total_distance = total_distance
                    prev_deviation_time = deviation_time

                    # Clear stale global-plan state before each trial.
                    global_path = []
                    global_path_length = 0.0
                    last_plan_stamp = rospy.Time(0)
                    last_plan_length = 0.0
                    first_plan_received = False
                    replan_detected = False
                    deviation_timer = None

                    last_plan_rx_time = rospy.Time(0)
                    last_plan_msg_stamp = rospy.Time(0)
                    last_plan_length = 0.0

                    navigation_active = True
                    goal_since = publish_goal(pub_goal, goal)
                    planned_len = wait_for_first_plan_since(goal_since, timeout=20.0)

                    # Retry the trial from a reset pose if no initial global plan arrives.
                    if not math.isfinite(planned_len) or planned_len <= 0.0:
                        navigation_active = False
                        trial_reset_count += 1
                        rospy.logwarn(f"Trial {(i * len(GOAL_POINTS)) + gi} için global plan alınamadı. Reset deneniyor ({trial_reset_count}/{MAX_TRIAL_RESETS}).")

                        if trial_reset_count > MAX_TRIAL_RESETS:
                            rospy.logerr("Maksimum trial reset sayısı aşıldı; test durduruluyor.")
                            rospy.signal_shutdown("Trial reset limit exceeded")
                            return

                        try:
                            helper.cancel_goal()
                            helper.teleport_to_pose(
                                x=float(expected['x']),
                                y=float(expected['y']),
                                theta=float(expected.get('theta', 0.0)),
                                z=float(expected.get('z', 0.0))
                            )
                        except Exception as e:
                            rospy.logerr("Trial reset teleport başarısız: %s", str(e))

                        continue

                    # 0.22 m/s is the TurtleBot3 Burger maximum translational velocity used for timeout scaling.
                    if world == "bosOrt":
                        timeout_duration = (planned_len/0.22) * (1+3.00)
                    elif world == "duraganOrt":
                        timeout_duration = (planned_len/0.22) * (1+5.00)
                    elif world == ("hareketliOrt"):
                        timeout_duration = (planned_len/0.22) * (1+7.00)
                    elif world == ("karmaOrt"):
                        timeout_duration = (planned_len/0.22) * (1+9.00)

                    goal_nav_start = rospy.Time.now()
                    timeout_t = goal_nav_start + rospy.Duration(timeout_duration)

                    reached = check_goal_reached(goal_since, timeout_t)
                    navigation_active = False

                    goal_end = rospy.Time.now()
                    goal_distance = total_distance - prev_total_distance

                    tmp_total_dev = deviation_time
                    if deviation_timer is not None:
                        tmp_total_dev += (goal_end.to_sec() - deviation_timer)
                    goal_deviation = tmp_total_dev - prev_deviation_time

                    goal_nav_time = (goal_end - goal_nav_start).to_sec()
                    goal_avg_speed = (goal_distance / goal_nav_time) if goal_nav_time > 0 else 0.0

                    static_collisions = 1 if static_collision_in_trial else 0
                    dynamic_collisions = 1 if dynamic_collision_in_trial else 0

                    collided = (static_collisions + dynamic_collisions) > 0
                    local_fail_flag = bool(replan_detected)
                    timeout_flag = (not reached) and (not collided) and (not local_fail_flag)

                    start_xy = as_xy(current_start_point)

                    write_trial_row(
                        writer=writer,
                        trial_idx=(i * len(GOAL_POINTS)) + gi,
                        start_xy=start_xy,
                        goal_xy=(goal['x'], goal['y']),
                        dev_time=goal_deviation,
                        nav_time=goal_nav_time,
                        pln_path=planned_len,
                        exc_path=goal_distance,
                        avg_speed=goal_avg_speed,
                        stat_col=static_collisions,
                        dyn_col=dynamic_collisions,
                        local_fail=local_fail_flag,
                        timeout=timeout_flag,
                        success=reached
                    )

                    sum_dev += goal_deviation
                    sum_nav += goal_nav_time
                    if math.isfinite(planned_len):
                        sum_planned += planned_len
                        count_planned += 1
                    sum_exec += goal_distance
                    sum_speed += goal_avg_speed
                    sum_stat += static_collisions
                    sum_dyn += dynamic_collisions
                    count_timeout_yes += (1 if timeout_flag else 0)
                    count_success_yes += (1 if reached else 0)
                    count_local_fail_yes += (1 if local_fail_flag else 0)
                    count_rows += 1

                    rospy.loginfo(f"{(i * len(GOAL_POINTS)) + gi}. trial completed.")

                    if (timeout_flag or collided or local_fail_flag) and teleport_enabled:
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
                            replan_detected = False

                            wait_for_localization_ready(
                                expected_xy=(float(goal['x']), float(goal['y'])),
                                label=f"rep{i+1}_trial{(i*len(GOAL_POINTS))+gi}_post_teleport_sync",
                                rate_hz=10.0
                            )

                            rospy.logwarn("Deneme sonlandırıldı ({}). Teleport+Sync tamam."
                                        .format("collision" if collided else ("local_fail" if local_fail_flag else "timeout")))
                        except Exception as e:
                            rospy.logerr("Teleport denemesi başarısız: %s", str(e))

                    break

        avg_dev   = (sum_dev / count_rows) if count_rows else 0.0
        avg_nav   = (sum_nav / count_rows) if count_rows else 0.0
        avg_exec  = (sum_exec / count_rows) if count_rows else 0.0
        avg_speed = (sum_speed / count_rows) if count_rows else 0.0
        avg_plan  = (sum_planned / count_planned) if count_planned > 0 else None

        writer.writerow({
            'Trial': 'General Result',
            'From (x,y)': '',
            'To (x,y)': '',
            'Deviation Time (s)': f"{avg_dev:.4f}",
            'Navigation Time (s)': f"{avg_nav:.4f}",
            'Planned Path Length (m)': (f"{avg_plan:.4f}" if avg_plan is not None else ""),
            'Executed Path Length (m)': f"{avg_exec:.4f}",
            'Average Speed (m/s)': f"{avg_speed:.4f}",
            'Static Collisions': int(sum_stat),
            'Dynamic Collisions': int(sum_dyn),
            'Local Planner Failure': int(count_local_fail_yes),
            'Timeout': int(count_timeout_yes),
            'Success': int(count_success_yes)
        })

    rospy.loginfo("Navigation completed. Metrics saved to %s", str(filepath))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
