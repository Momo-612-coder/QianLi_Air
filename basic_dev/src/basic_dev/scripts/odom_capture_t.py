#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import csv
import tty
import termios
import select
import threading

import rospy
from nav_msgs.msg import Odometry


class OdomCaptureOnKey:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/airsim_node/drone_1/drone_state")
        self.output_file = rospy.get_param("~output_file", "/tmp/airsim_odom_snapshots.csv")

        self._lock = threading.Lock()
        self._latest_odom = None

        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        self._init_csv_if_needed()

        self._sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)

        rospy.loginfo("[odom_capture_t] Subscribed odom topic: %s", self.odom_topic)
        rospy.loginfo("[odom_capture_t] Output file: %s", self.output_file)
        rospy.loginfo("[odom_capture_t] Press 't' to save latest odom, 'q' to quit.")

    def _init_csv_if_needed(self):
        if os.path.exists(self.output_file) and os.path.getsize(self.output_file) > 0:
            return

        with open(self.output_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "record_time_sec",
                "odom_stamp_sec",
                "frame_id",
                "child_frame_id",
                "px", "py", "pz",
                "qx", "qy", "qz", "qw",
                "vx", "vy", "vz",
                "wx", "wy", "wz",
            ])

    def _odom_cb(self, msg):
        with self._lock:
            self._latest_odom = msg

    def _snapshot(self):
        with self._lock:
            msg = self._latest_odom

        if msg is None:
            rospy.logwarn("[odom_capture_t] No odom received yet, skip snapshot.")
            return

        row = [
            rospy.Time.now().to_sec(),
            msg.header.stamp.to_sec(),
            msg.header.frame_id,
            msg.child_frame_id,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]

        with open(self.output_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)

        rospy.loginfo(
            "[odom_capture_t] Saved snapshot: p=(%.3f, %.3f, %.3f), q=(%.3f, %.3f, %.3f, %.3f)",
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def spin(self):
        if not sys.stdin.isatty():
            rospy.logerr("[odom_capture_t] stdin is not a tty; cannot read keyboard input.")
            rospy.logerr("[odom_capture_t] Launch with a real terminal, e.g. launch-prefix='xterm -e'.")
            rospy.signal_shutdown("stdin is not a tty")
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while not rospy.is_shutdown():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue

                ch = sys.stdin.read(1)
                if ch in ("t", "T"):
                    self._snapshot()
                elif ch in ("q", "Q"):
                    rospy.loginfo("[odom_capture_t] Quit requested by keyboard.")
                    rospy.signal_shutdown("user requested quit")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rospy.init_node("odom_capture_t")
    node = OdomCaptureOnKey()
    node.spin()


if __name__ == "__main__":
    main()
