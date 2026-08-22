#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class ModelController:
    def __init__(self):
        rospy.init_node('gazebo_model_movement', anonymous=True)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def move_model(self, model_name, point_a, point_b, speed):
        rate = rospy.Rate(100)  # 100 Hz

        # Segment length.
        dx = point_b[0] - point_a[0]
        dy = point_b[1] - point_a[1]
        dz = point_b[2] - point_a[2]
        length = math.sqrt(dx*dx + dy*dy + dz*dz)

        if length == 0:
            rospy.logwarn("Start and end positions are the same. No movement required.")
            return

        # Full round-trip period.
        T = 2.0 * length / speed

        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t0
            phase = math.fmod(t, T)  # 0 .. T

            if phase <= T / 2.0:
                # A -> B
                s = speed * phase                 # Distance traveled from A.
            else:
                # B -> A
                s = speed * (T - phase)           # Distance traveled back toward A.

            lam = s / length  # 0 .. 1

            current_position = [
                point_a[0] + dx * lam,
                point_a[1] + dy * lam,
                point_a[2] + dz * lam,
            ]

            model_state = ModelState()
            model_state.model_name = model_name
            model_state.pose.position.x = current_position[0]
            model_state.pose.position.y = current_position[1]
            model_state.pose.position.z = current_position[2]
            model_state.pose.orientation.x = 0.0
            model_state.pose.orientation.y = 0.0
            model_state.pose.orientation.z = 0.0
            model_state.pose.orientation.w = 1.0

            try:
                self.set_model_state(model_state)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = ModelController()
        model_name = 'hareketliEngel'  # Gazebo model name.
        start_position = [0.5, 1.5, 0]  # First target position
        end_position = [2.5, 1.5, 0]  # Second target position
        speed = 0.2  # Movement speed (units per second)

        controller.move_model(model_name, start_position, end_position, speed)
    except rospy.ROSInterruptException:
        pass
