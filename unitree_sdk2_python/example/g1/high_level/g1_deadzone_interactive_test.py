#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import signal
import threading
from collections import deque

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


class InteractiveCmdVelTester:
    def __init__(self, network_interface):
        # ===== SDK 初始化 =====
        ChannelFactoryInitialize(0, network_interface)

        self.client = LocoClient()
        self.client.SetTimeout(10.0)
        self.client.Init()

        # ===== 测试参数 =====
        self.send_freq = 50.0
        self.send_dt = 1.0 / self.send_freq
        self.send_duration = 2.0   # 持续发送 2s
        self.stop_duration = 2.0   # 停车 2s

        # ===== 当前反馈 =====
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        # 保存一轮测试中的反馈
        self.samples = deque()
        self.lock = threading.Lock()
        self.running = True

        # ===== DDS 订阅 =====
        self.odom_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.odom_sub.Init(self.odom_callback)

    def odom_callback(self, msg: SportModeState_):
        with self.lock:
            self.current_vx = float(msg.velocity[0])
            self.current_vy = float(msg.velocity[1])
            self.current_wz = float(msg.yaw_speed)

    def stop_robot(self):
        try:
            self.client.StopMove()
        except Exception as e:
            print(f"[WARN] StopMove failed: {e}")

    def move_robot(self, vx, vy, wz):
        try:
            self.client.Move(vx, vy, wz)
        except Exception as e:
            print(f"[WARN] Move failed: {e}")

    def clear_samples(self):
        with self.lock:
            self.samples.clear()

    def record_sample(self):
        with self.lock:
            self.samples.append((
                time.time(),
                self.current_vx,
                self.current_vy,
                self.current_wz
            ))

    def print_current_state(self):
        with self.lock:
            vx = self.current_vx
            vy = self.current_vy
            wz = self.current_wz
        print(f"[STATE] vx={vx:+.4f}, vy={vy:+.4f}, wz={wz:+.4f}")

    def summarize_samples(self):
        with self.lock:
            data = list(self.samples)

        if not data:
            return None

        arr = np.array(data, dtype=float)
        # columns: time, vx, vy, wz
        return {
            "n": len(arr),
            "mean_vx": float(np.mean(arr[:, 1])),
            "std_vx": float(np.std(arr[:, 1])),
            "max_abs_vx": float(np.max(np.abs(arr[:, 1]))),

            "mean_vy": float(np.mean(arr[:, 2])),
            "std_vy": float(np.std(arr[:, 2])),
            "max_abs_vy": float(np.max(np.abs(arr[:, 2]))),

            "mean_wz": float(np.mean(arr[:, 3])),
            "std_wz": float(np.std(arr[:, 3])),
            "max_abs_wz": float(np.max(np.abs(arr[:, 3]))),
        }

    def run_one_test(self, vx, vy, wz):
        print("\n" + "-" * 70)
        print(f"[TEST] send cmd = ({vx:+.4f}, {vy:+.4f}, {wz:+.4f})")
        print(f"[INFO] Sending for {self.send_duration:.1f}s at {self.send_freq:.1f} Hz")
        print("-" * 70)

        self.clear_samples()

        start_time = time.time()
        next_print_time = start_time

        while self.running and (time.time() - start_time < self.send_duration):
            self.move_robot(vx, vy, wz)
            self.record_sample()

            now = time.time()
            if now >= next_print_time:
                self.print_current_state()
                next_print_time += 0.5

            time.sleep(self.send_dt)

        print("[INFO] Stop robot")
        self.stop_robot()
        time.sleep(self.stop_duration)

        summary = self.summarize_samples()
        if summary is None:
            print("[WARN] No samples collected.")
            return

        print("[RESULT]")
        print(f"  samples    = {summary['n']}")
        print(f"  mean_vx    = {summary['mean_vx']:+.4f}, std_vx = {summary['std_vx']:.4f}, max|vx| = {summary['max_abs_vx']:.4f}")
        print(f"  mean_vy    = {summary['mean_vy']:+.4f}, std_vy = {summary['std_vy']:.4f}, max|vy| = {summary['max_abs_vy']:.4f}")
        print(f"  mean_wz    = {summary['mean_wz']:+.4f}, std_wz = {summary['std_wz']:.4f}, max|wz| = {summary['max_abs_wz']:.4f}")
        print("[INFO] 请结合机器人实际运动现象，判断该输入是否越过死区。")

    def parse_user_input(self, s):
        s = s.strip().lower()

        if s in ["q", "quit", "exit"]:
            return "quit"
        if s in ["s", "stop"]:
            return "stop"
        if s in ["h", "help"]:
            return "help"

        parts = s.split()
        if len(parts) != 3:
            return None

        try:
            vx, vy, wz = map(float, parts)
            return (vx, vy, wz)
        except ValueError:
            return None

    def print_help(self):
        print("\n可用输入格式：")
        print("  vx vy wz")
        print("例如：")
        print("  0.10 0 0")
        print("  0 0.12 0")
        print("  0 0 0.08")
        print("  -0.10 0 0")
        print("特殊命令：")
        print("  h / help   查看帮助")
        print("  s / stop   立即停车")
        print("  q / quit   退出脚本\n")

    def main_loop(self):
        print("=" * 72)
        print("Unitree G1 Interactive cmd_vel Tester")
        print("=" * 72)
        print("说明：")
        print("1. 每次直接输入: vx vy wz")
        print("2. 脚本会持续发送这组命令 2 秒")
        print("3. 然后自动停车，并打印实际反馈统计")
        print("4. 你根据机器人运动情况继续输入下一组值")
        print("=" * 72)
        print("[安全提醒] 请确保机器人已进入可运动模式，并处于安全测试环境。")
        print("[建议] 测死区时一次只改一个轴，其余两个轴保持 0。")
        print("=" * 72)

        self.print_help()

        print("[INFO] 当前反馈预览：")
        for _ in range(5):
            self.print_current_state()
            time.sleep(0.2)

        while self.running:
            user_in = input("请输入 vx vy wz > ")
            parsed = self.parse_user_input(user_in)

            if parsed == "quit":
                break
            elif parsed == "stop":
                print("[INFO] Manual stop")
                self.stop_robot()
                continue
            elif parsed == "help":
                self.print_help()
                continue
            elif parsed is None:
                print("[WARN] 输入格式错误。请输入三个浮点数，例如: 0.1 0 0")
                continue

            vx, vy, wz = parsed
            self.run_one_test(vx, vy, wz)

        print("[INFO] Exiting tester...")
        self.stop_robot()


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <network_interface>")
        sys.exit(1)

    network_interface = sys.argv[1]
    tester = InteractiveCmdVelTester(network_interface)

    def handle_sigint(sig, frame):
        print("\n[INFO] Ctrl+C detected, stopping robot...")
        tester.running = False
        tester.stop_robot()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        tester.main_loop()
    finally:
        tester.stop_robot()


if __name__ == "__main__":
    main()