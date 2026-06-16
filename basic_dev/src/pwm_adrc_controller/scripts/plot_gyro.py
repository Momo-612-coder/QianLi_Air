#!/usr/bin/env python3
"""
Subscribe to a nav_msgs/Odometry or sensor_msgs/Imu topic and plot angular velocity (wx, wy, wz)
Saves `gyro_plot.png` (or custom path) on shutdown (Ctrl-C).

Usage:
  python3 plot_gyro.py --topic /airsim_node/drone_1/drone_state --outfile /tmp/gyro_plot.png
"""
import argparse
import collections
import signal
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class GyroPlotter:
    def __init__(self, topic, is_imu, max_points=10000):
        self.topic = topic
        self.is_imu = is_imu
        self.times = collections.deque(maxlen=max_points)
        self.wx = collections.deque(maxlen=max_points)
        self.wy = collections.deque(maxlen=max_points)
        self.wz = collections.deque(maxlen=max_points)
        self.sub = None

    def start(self):
        if self.is_imu:
            self.sub = rospy.Subscriber(self.topic, Imu, self._cb)
        else:
            self.sub = rospy.Subscriber(self.topic, Odometry, self._cb)

    def _cb(self, msg):
        t = rospy.Time.now().to_sec()
        if self.is_imu:
            wx = msg.angular_velocity.x
            wy = msg.angular_velocity.y
            wz = msg.angular_velocity.z
        else:
            wx = msg.twist.twist.angular.x
            wy = msg.twist.twist.angular.y
            wz = msg.twist.twist.angular.z

        self.times.append(t)
        self.wx.append(wx)
        self.wy.append(wy)
        self.wz.append(wz)

    def save(self, outfile):
        if len(self.times) == 0:
            print('No data collected; nothing to save.')
            return

        t0 = self.times[0]
        times = np.array(self.times) - t0

        plt.figure(figsize=(10,4))
        plt.plot(times, np.array(self.wx), label='wx')
        plt.plot(times, np.array(self.wy), label='wy')
        plt.plot(times, np.array(self.wz), label='wz')
        plt.xlabel('time (s)')
        plt.ylabel('angular velocity (rad/s)')
        plt.title('Gyroscope / Angular Velocity')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(outfile, dpi=150)
        print('Saved gyro plot to', outfile)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', '-t', default='/airsim_node/drone_1/drone_state', help='Odometry or IMU topic')
    parser.add_argument('--imu', action='store_true', help='Treat topic as sensor_msgs/Imu instead of nav_msgs/Odometry')
    parser.add_argument('--outfile', '-o', default='gyro_plot.png', help='Output PNG path')
    parser.add_argument('--max-points', type=int, default=20000, help='Max samples to keep in memory')
    args = parser.parse_args()

    rospy.init_node('gyro_plotter', anonymous=True)

    gp = GyroPlotter(args.topic, args.imu, max_points=args.max_points)
    gp.start()

    def handle_sigint(signum, frame):
        rospy.signal_shutdown('user')

    signal.signal(signal.SIGINT, handle_sigint)

    print('Subscribed to', args.topic)
    print('Press Ctrl-C to stop and save to', args.outfile)

    # wait until shutdown
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    gp.save(args.outfile)


if __name__ == '__main__':
    main()
