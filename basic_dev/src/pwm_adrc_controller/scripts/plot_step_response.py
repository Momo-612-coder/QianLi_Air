#!/usr/bin/env python3
import argparse
import csv
import math
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rospy
from airsim_ros.msg import RotorPWM
from nav_msgs.msg import Odometry


def quat_to_euler(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=float)


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def odom_to_state(msg):
    p = msg.pose.pose.position
    v = msg.twist.twist.linear
    return {
        "stamp": msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec(),
        "pos": np.array([p.x, p.y, p.z], dtype=float),
        "euler": quat_to_euler(msg.pose.pose.orientation),
        "vel": np.array([v.x, v.y, v.z], dtype=float),
        "frame": msg.header.frame_id,
        "child": msg.child_frame_id,
    }


class StepResponseRecorder:
    def __init__(self, args):
        self.args = args
        self.actual = None
        self.target = None
        self.pwm = None
        self.last_target = None
        self.recording = False
        self.done = False
        self.record_now_started = False
        self.start_wall = None
        self.start_ros = None
        self.last_pwm_print_ros = None
        self.samples = []

        os.makedirs(args.outdir, exist_ok=True)
        rospy.Subscriber(args.odom_topic, Odometry, self.actual_cb, queue_size=1)
        rospy.Subscriber(args.target_topic, Odometry, self.target_cb, queue_size=1)
        rospy.Subscriber(args.pwm_topic, RotorPWM, self.pwm_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / args.rate), self.sample)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.status)

    def actual_cb(self, msg):
        self.actual = odom_to_state(msg)

    def target_cb(self, msg):
        state = odom_to_state(msg)
        if self.last_target is not None and not self.recording and not self.done:
            dp = np.linalg.norm(state["pos"] - self.last_target["pos"])
            de = np.max(np.abs([angle_diff(a, b) for a, b in zip(state["euler"], self.last_target["euler"])]))
            if dp >= self.args.pos_threshold or de >= self.args.att_threshold:
                self.start_recording("target step detected")
        self.target = state
        self.last_target = state

    def pwm_cb(self, msg):
        self.pwm = np.array([msg.rotorPWM0, msg.rotorPWM1, msg.rotorPWM2, msg.rotorPWM3], dtype=float)

    def start_recording(self, reason):
        if self.recording or self.done:
            return
        if self.pwm is None:
            rospy.logwarn("Target step detected, but PWM topic has not arrived yet: %s", self.args.pwm_topic)
            return
        self.recording = True
        self.start_wall = time.time()
        self.start_ros = rospy.Time.now().to_sec()
        self.last_pwm_print_ros = None
        self.samples = []
        rospy.loginfo(
            "%s. Recording %.2f seconds at %.1f Hz.",
            reason,
            self.args.duration,
            self.args.rate,
        )

    def sample(self, _event):
        if (
            self.args.record_now
            and not self.record_now_started
            and not self.recording
            and not self.done
            and self.actual is not None
            and self.target is not None
        ):
            self.record_now_started = True
            self.start_recording("--record-now armed")

        if not self.recording or self.actual is None or self.target is None:
            return

        now = rospy.Time.now().to_sec()
        t = now - self.start_ros
        pwm = self.pwm.copy() if self.pwm is not None else np.full(4, np.nan, dtype=float)
        self.samples.append({
            "t": t,
            "actual_pos": self.actual["pos"].copy(),
            "target_pos": self.target["pos"].copy(),
            "actual_euler": self.actual["euler"].copy(),
            "target_euler": self.target["euler"].copy(),
            "pwm": pwm,
            "target_frame": self.target["frame"],
            "target_child": self.target["child"],
        })

        pwm_print_rate = self.args.print_pwm_rate or self.args.rate
        if self.args.print_pwm and pwm_print_rate > 0.0:
            should_print = (
                self.last_pwm_print_ros is None
                or now - self.last_pwm_print_ros >= 1.0 / pwm_print_rate
            )
            if should_print:
                self.last_pwm_print_ros = now
                rospy.loginfo(
                    "t=%.3f pwm=[%.4f, %.4f, %.4f, %.4f]",
                    t, pwm[0], pwm[1], pwm[2], pwm[3],
                )

        if t >= self.args.duration:
            self.recording = False
            self.done = self.args.oneshot
            self.save()
            if self.args.oneshot:
                rospy.signal_shutdown("oneshot step response saved")

    def status(self, _event):
        if self.recording or self.done:
            return
        if self.actual is None:
            rospy.logwarn_throttle(2.0, "Waiting for odometry topic: %s", self.args.odom_topic)
            return
        if self.target is None:
            rospy.logwarn_throttle(2.0, "Waiting for ADRC target topic: %s", self.args.target_topic)
            return
        if self.pwm is None:
            rospy.logwarn_throttle(2.0, "Waiting for PWM topic: %s", self.args.pwm_topic)
            return
        rospy.loginfo_throttle(
            2.0,
            "Ready. Waiting for target step (pos >= %.3f m or attitude >= %.2f deg). Use --record-now to save immediately.",
            self.args.pos_threshold,
            self.args.att_threshold * 180.0 / math.pi,
        )

    def save(self):
        if not self.samples:
            rospy.logwarn("No samples collected.")
            return

        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.args.outdir, "adrc_step_response_" + stamp)
        csv_path = base + ".csv"
        png_path = base + ".png"

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t",
                "actual_x", "actual_y", "actual_z",
                "target_x", "target_y", "target_z",
                "actual_roll", "actual_pitch", "actual_yaw",
                "target_roll", "target_pitch", "target_yaw",
                "pwm0", "pwm1", "pwm2", "pwm3",
                "target_frame", "target_child",
            ])
            for s in self.samples:
                writer.writerow([
                    s["t"],
                    *s["actual_pos"].tolist(),
                    *s["target_pos"].tolist(),
                    *s["actual_euler"].tolist(),
                    *s["target_euler"].tolist(),
                    *s["pwm"].tolist(),
                    s["target_frame"],
                    s["target_child"],
                ])

        t = np.array([s["t"] for s in self.samples])
        actual_pos = np.array([s["actual_pos"] for s in self.samples])
        target_pos = np.array([s["target_pos"] for s in self.samples])
        actual_euler = np.array([s["actual_euler"] for s in self.samples]) * 180.0 / math.pi
        target_euler = np.array([s["target_euler"] for s in self.samples]) * 180.0 / math.pi
        pwm = np.array([s["pwm"] for s in self.samples])

        labels_pos = ["x", "y", "z"]
        labels_att = ["roll", "pitch", "yaw"]

        fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=True)
        fig.suptitle("ADRC keyboard/planner step response")

        for i in range(3):
            ax = axes[i, 0]
            ax.plot(t, target_pos[:, i], "k--", label="target")
            ax.plot(t, actual_pos[:, i], "b-", label="actual")
            ax.scatter(t, actual_pos[:, i], s=4, alpha=0.35)
            ax.set_ylabel(labels_pos[i] + " (m)")
            ax.grid(True)
            if i == 0:
                ax.legend(loc="best")

        for i in range(3):
            ax = axes[i, 1]
            ax.plot(t, target_euler[:, i], "k--", label="target")
            ax.plot(t, actual_euler[:, i], "r-", label="actual")
            ax.scatter(t, actual_euler[:, i], s=4, alpha=0.35)
            ax.set_ylabel(labels_att[i] + " (deg)")
            ax.grid(True)
            if i == 0:
                ax.legend(loc="best")

        ax = axes[3, 0]
        for i in range(4):
            ax.plot(t, pwm[:, i], label="pwm" + str(i))
        ax.set_ylabel("motor PWM")
        ax.set_xlabel("time (s)")
        ax.grid(True)
        ax.legend(loc="best", ncol=2)

        axes[3, 1].axis("off")

        axes[-1, 0].set_xlabel("time (s)")
        axes[2, 1].set_xlabel("time (s)")
        fig.tight_layout()
        fig.savefig(png_path, dpi=160)
        plt.close(fig)

        rospy.loginfo("Saved step response plot: %s", png_path)
        rospy.loginfo("Saved step response csv: %s", csv_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom-topic", default="/airsim_node/drone_1/drone_state")
    parser.add_argument("--target-topic", default="/pwm_adrc_controller/target_debug")
    parser.add_argument("--pwm-topic", default="/airsim_node/drone_1/rotor_pwm_cmd")
    parser.add_argument("--outdir", default="/tmp/adrc_step_response")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--pos-threshold", type=float, default=0.05)
    parser.add_argument("--att-threshold", type=float, default=math.radians(2.0))
    parser.add_argument("--continuous", action="store_true", help="keep running and save every detected step")
    parser.add_argument("--record-now", action="store_true", help="record immediately after odom and target topics are available")
    parser.add_argument("--print-pwm", action="store_true", default=True, help="print PWM values while recording")
    parser.add_argument("--no-print-pwm", action="store_false", dest="print_pwm", help="disable PWM printing while recording")
    parser.add_argument("--print-pwm-rate", type=float, default=None, help="PWM print rate in Hz while recording; defaults to --rate")
    args = parser.parse_args(rospy.myargv()[1:])
    args.oneshot = not args.continuous

    rospy.init_node("adrc_step_response_plotter", anonymous=True)
    StepResponseRecorder(args)
    rospy.spin()


if __name__ == "__main__":
    main()
