#!/usr/bin/env python
import rospy
import random
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent

class TurtlebotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller')

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)

        # State variables
        self.bumped = False
        self.laser_ranges = []
        self.distance_traveled = 0.0
        self.last_turn_time = rospy.Time.now()

        # Parameters
        self.forward_speed = 0.2  # m/s
        self.turn_speed = 0.5     # rad/s
        self.turn_interval = 1.0  # meters
        self.last_pose_time = rospy.Time.now()

        rospy.loginfo("Turtlebot controller initialized.")

    def bumper_callback(self, msg):
        if msg.state == BumperEvent.PRESSED:
            rospy.logwarn("Bumper pressed! Halting.")
            self.bumped = True

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            twist = Twist()

            # 1. Halt if bumper pressed
            if self.bumped:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # 2. Escape symmetric obstacles
            elif self.detect_symmetric_obstacle():
                twist.angular.z = self.escape_turn()

            # 3. Avoid asymmetric obstacles
            elif self.detect_asymmetric_obstacle():
                twist.angular.z = self.avoid_turn()

            # 4. Random turn every 1ft (~0.3 m)
            elif self.should_random_turn():
                twist.angular.z = random.uniform(-math.radians(15), math.radians(15))
                rospy.loginfo("Random small turn: %.2f deg" % math.degrees(twist.angular.z))

            # 5. Default: drive forward
            else:
                twist.linear.x = self.forward_speed

            self.cmd_pub.publish(twist)
            rate.sleep()

    # -------- Behavior helpers --------
    def detect_symmetric_obstacle(self):
        """Check if obstacles are close in front-left and front-right within 0.3 m (~1ft)."""
        if not self.laser_ranges:
            return False
        ranges = self.laser_ranges
        front = ranges[len(ranges)//2]
        left = ranges[len(ranges)//3]
        right = ranges[2*len(ranges)//3]
        return (front < 0.3 and abs(left - right) < 0.1)

    def escape_turn(self):
        """Escape by turning ~180+-30 degrees."""
        angle = math.radians(150 + random.uniform(0, 60))
        rospy.loginfo("Escape turn: %.2f deg" % math.degrees(angle))
        return angle

    def detect_asymmetric_obstacle(self):
        if not self.laser_ranges:
            return False
        left = min(self.laser_ranges[:len(self.laser_ranges)//3])
        right = min(self.laser_ranges[2*len(self.laser_ranges)//3:])
        return (left < 0.3 or right < 0.3)

    def avoid_turn(self):
        """Turn away from closer obstacle."""
        left = min(self.laser_ranges[:len(self.laser_ranges)//3])
        right = min(self.laser_ranges[2*len(self.laser_ranges)//3:])
        if left < right:
            rospy.loginfo("Avoiding left obstacle -> turning right")
            return -self.turn_speed
        else:
            rospy.loginfo("Avoiding right obstacle -> turning left")
            return self.turn_speed

    def should_random_turn(self):
        now = rospy.Time.now()
        if (now - self.last_turn_time).to_sec() > 3.0:  # crude timer instead of odom
            self.last_turn_time = now
            return True
        return False


if __name__ == '__main__':
    try:
        controller = TurtlebotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

