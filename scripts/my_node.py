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

        # Asymetric void
        self.one_foot = 0.3048
        self.asym_margin = 0.08
        self.clear_bonus = 0.05
        self.front_fov_deg = 90     
        self.creep_speed = 0.05     
        self.turn_gain = 3.5         

        self.laser_msg = None

        # Escape fixed-action
        self.escape_active = False
        self.escape_end_time = rospy.Time.now()
        self.escape_angular_sign = 1.0

    def bumper_callback(self, msg):
        if msg.state == BumperEvent.PRESSED:
            rospy.logwarn("Bumper pressed! Halting.")
            self.bumped = True

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.laser_msg = msg

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()

            if self.laser_msg:
                L, C, R = self._front_sector_mins()
                rospy.loginfo_throttle(1.0, "L=%.2f  C=%.2f  R=%.2f  range_min=%.2f" %
                                       (L, C, R, getattr(self.laser_msg, 'range_min', float('nan'))))

            # Fixed Action escape
            if self.escape_active:
                if rospy.Time.now() < self.escape_end_time:
                    twist.linear.x = 0.0
                    twist.angular.z = self.escape_angular_sign * self.turn_speed
                    self.cmd_pub.publish(twist)
                    rate.sleep()
                    continue
                else:
                    self.escape_active = False 

            # 1. Halt if bumper pressed
            if self.bumped:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # 2. Escape symmetric obstacles start 180 degree turn
            elif self.detect_symmetric_obstacle():
                # Turn towards clearer direction
                L, C, R = self._front_sector_mins()
                turn_left_preferred = (L >= R)
                self.start_escape_fixed_action(turn_left_preferred)

                twist.linear.x = 0.0
                twist.angular.z = self.escape_angular_sign * self.turn_speed

            # 3. Avoid asymmetric obstacles
            elif self.detect_asymmetric_obstacle():
                # Turn from obstacle
                twist.linear.x = 0.0
                twist.angular.z = self.avoid_turn_reflex()

            # 4. Random turn or keep going forward
            elif self.should_random_turn():
                twist.linear.x = 0.0
                twist.angular.z = random.choice([-1, 1]) * self.turn_speed
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0

            self.cmd_pub.publish(twist)
            rate.sleep()

    def detect_symmetric_obstacle(self):
        """Check if obstacles are close in front-left and front-right within 0.3 m (~1ft)."""
        if not self.laser_msg:
            return False
        # Use sector-based minima and a danger threshold that respects sensor range_min
        L, C, R = self._front_sector_mins()
        m = self.laser_msg
        rm = getattr(m, 'range_min', 0.25)
        danger = max(self.one_foot, rm + 0.05)
        return (L < danger and R < danger and abs(L - R) < 0.1)

    def escape_turn(self):
        """Escape from (roughly) symmetric obstacle by turning in place."""
        # choose a random direction
        return random.choice([-1, 1]) * self.turn_speed

    # Fixed escape of 180 +- 30 degrees
    def start_escape_fixed_action(self, turn_left_preferred):
        # Pick angle = 180 deg +- 30 deg
        extra = random.uniform(-30.0, 30.0)
        angle_deg = 180.0 + extra
        angle_rad = math.radians(angle_deg)

        # Choose direction sign
        self.escape_angular_sign = 1.0 if turn_left_preferred else -1.0

        # Duration based on constant turn speed
        duration = angle_rad / max(self.turn_speed, 1e-3)
        self.escape_end_time = rospy.Time.now() + rospy.Duration.from_sec(duration)
        self.escape_active = True
        rospy.loginfo("ESCAPE: committing to %.1f deg turn %s (duration %.2fs)" %
                      (angle_deg, "left" if self.escape_angular_sign > 0 else "right", duration))

    def _min_in_sector(self, ranges, angle_min, angle_inc, start_deg, end_deg):
        if not ranges:
            return float('inf')
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        a0, a1 = sorted([start_rad, end_rad])
        i0 = max(0, int((a0 - angle_min) / angle_inc))
        i1 = min(len(ranges) - 1, int((a1 - angle_min) / angle_inc))

        vals = []
        for r in ranges[i0:i1 + 1]:
            if r is None:
                continue
            if (not math.isnan(r)) and (not math.isinf(r)) and (r > 0.0):
                vals.append(r)
        return min(vals) if vals else float('inf')

    def _front_sector_mins(self):
        if not self.laser_msg:
            return (float('inf'), float('inf'), float('inf'))
        m = self.laser_msg
        half = self.front_fov_deg

        right_min  = self._min_in_sector(m.ranges, m.angle_min, m.angle_increment, -half, 0)
        center_min = self._min_in_sector(m.ranges, m.angle_min, m.angle_increment, -half / 2.0, half / 2.0)
        left_min   = self._min_in_sector(m.ranges, m.angle_min, m.angle_increment, 0, half)
        return (left_min, center_min, right_min)

    def detect_asymmetric_obstacle(self):
        if not self.laser_msg:
            return False
        L, C, R = self._front_sector_mins()
        m = self.laser_msg
        rm = getattr(m, 'range_min', 0.25)
        danger = max(self.one_foot, rm + 0.05)

        #  At least one front sector is by an obstacle
        any_close = (L < danger) or (C < danger) or (R < danger)
        symmetric = (L < danger and R < danger and abs(L - R) < 0.1)
        return any_close and (not symmetric)

    # Reflexive avoid turn
    def avoid_turn_reflex(self):
        L, C, R = self._front_sector_mins()
        m = self.laser_msg
        rm = getattr(m, 'range_min', 0.25)
        danger = max(self.one_foot, rm + 0.05)

        # Consider only sectors that are actually "too close"
        close_vals = []
        if L < danger: close_vals.append(("L", L))
        if C < danger: close_vals.append(("C", C))
        if R < danger: close_vals.append(("R", R))

        # If multiple are close, choose the closest
        if not close_vals:
            return 0.0

        closest_label, closest_dist = min(close_vals, key=lambda t: t[1])

        # Turn away from the closest
        if closest_label == "L":
            turn_sign = -1.0  # turn right
        elif closest_label == "R":
            turn_sign = 1.0   # turn left
        else:
            # obstacle centered: turn toward the side that is clearer
            turn_sign = 1.0 if L >= R else -1.0

        diff = (R - L)
        z = self.turn_gain * (turn_sign if closest_label != "C" else (1.0 if diff > 0 else -1.0))
        if z > self.turn_speed:
            z = self.turn_speed
        if z < -self.turn_speed:
            z = -self.turn_speed

        rospy.loginfo("AVOID: closest=%s(%.2f m) -> turning %s" %
                      (closest_label, closest_dist, "left" if z > 0 else "right"))
        return z

    def should_random_turn(self):
        now = rospy.Time.now()
        if (now - self.last_turn_time).to_sec() > 3.0:
            self.last_turn_time = now
            return True
        return False


if __name__ == '__main__':
    try:
        controller = TurtlebotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
