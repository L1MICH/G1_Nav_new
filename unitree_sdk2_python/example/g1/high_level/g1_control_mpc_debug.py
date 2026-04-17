#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import sys
import numpy as np
import osqp
import scipy.sparse as sp

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

class MPCController:
    def __init__(self, network_interface):
        rospy.loginfo("========================================")
        rospy.loginfo("Initializing Unitree MPC Controller (Debug Mode)...")
        rospy.loginfo("========================================")

        # 1. 初始化 SDK
        try:
            ChannelFactoryInitialize(0, network_interface)
        except Exception as e:
            print(f"[ERROR] 网络初始化失败: {e}")
            sys.exit(-1)

        # 2. 初始化客户端
        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        try:
            self.sport_client.Init()
        except Exception as e:
            print(f"[ERROR] 机器人连接失败: {e}")
            sys.exit(-1)

        self.can_move = False
        self.control_freq = 50.0
        self.dt = 1.0 / self.control_freq

        self.is_print_info = True  # 是否打印机器人状态信息
        self.print_frequent = 4.0  # 每 1 / self.print_frequent 秒打印一次信息流
        self.last_print_timestamp = -1  # 上次打印信息时的时间戳

        # 约束参数
        self.max_vx = 2.5
        self.max_vy = 1.0
        self.max_wz = 2.5
        self.max_acc_v = 1.5
        self.max_acc_w = 1.5
        
        # MPC 权重
        self.Q_v = 10.0  # 增大 Q_v 更加重视跟踪目标速度，响应更快
        self.R_v = 3.0  # 增大 R_v 输出更平滑、加减速更温和

        # 低速死区补偿参数
        self.stop_epsilon = 0.05      # 初始值值0.01
        self.min_effective_vx = 0.20  # 0.2以下机器人不动
        self.min_effective_vy = 0.20  # 0.2以下机器人不动
        self.min_effective_wz = 0.30   # 0.3以下机器人不动  
        #self.min_effective_wz = 0.60
        
        # 状态变量
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0

        # 订阅
        # self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel_smooth", Twist, self.cmd_vel_callback)
        
        self.path_sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)
        
        # DDS 订阅
        self.odom_dds_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.odom_dds_sub.Init(self.dds_odom_callback)

        self.control_timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
        rospy.loginfo("🚀 控制器已启动。等待导航路径...")

    def dds_odom_callback(self, msg: SportModeState_):
        self.current_vx = msg.velocity[0]
        self.current_vy = msg.velocity[1]
        self.current_wz = msg.yaw_speed

    def path_callback(self, msg: Path):
        # 打印路径接收情况 (调试用)
        # rospy.loginfo(f"收到路径，点数: {len(msg.poses)}")
        self.can_move = len(msg.poses) > 0

    def cmd_vel_callback(self, msg: Twist):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z

    def solve_mpc_step(self, v_current, v_target, v_last_cmd, max_v, max_acc):
        # 【重要修复】钳位限制，防止 OSQP 崩溃
        v_current = np.clip(v_current, -max_v, max_v)
        v_last_cmd = np.clip(v_last_cmd, -max_v, max_v)
        
        P = sp.csc_matrix([[2 * (self.Q_v + self.R_v)]])
        q = np.array([-2 * (self.Q_v * v_target + self.R_v * v_last_cmd)])
        
        acc_limit = max_acc * self.dt
        # 关键：约束窗口围绕上一拍命令，而不是当前实测速度。
        # 否则在底盘低速死区下，速度测量长期接近 0，会把命令永久卡在很小范围内。
        lower_bound = np.array([max(-max_v, v_last_cmd - acc_limit)])
        upper_bound = np.array([min(max_v, v_last_cmd + acc_limit)])
        
        A_box = sp.csc_matrix([[1.0]])
        
        prob = osqp.OSQP()
        prob.setup(P, q, A_box, lower_bound, upper_bound, verbose=False, eps_abs=1e-3, eps_rel=1e-3)
        res = prob.solve()
        
        if res.info.status != 'solved':
            rospy.logwarn(f"[WARNING] MPC 求解失败，状态: {res.info.status}")
            return 0.0
        return res.x[0]

    def apply_deadzone_compensation(self, cmd, target, min_effective, max_v):
        # 目标接近 0 时直接回零，避免抖动。
        if abs(target) <= self.stop_epsilon:
            return 0.0

        # 目标明显非零但命令落在死区内时，提升到最小有效值。
        if abs(target) >= min_effective and abs(cmd) < min_effective:
            cmd = np.sign(target) * min_effective

        return float(np.clip(cmd, -max_v, max_v))

    def control_loop(self, event):
        # --- 调试打印区 ---
        now_timestamp = int(rospy.get_time() * self.print_frequent)  
        self.is_print_info = (now_timestamp != self.last_print_timestamp) 
        if self.is_print_info:
            self.last_print_timestamp = now_timestamp

        if self.is_print_info: 
            state_str = "LOCKED" if not self.can_move else "ACTIVE"
            print(f"[State: {state_str}] "
                  f"Target: ({self.target_vx:.2f}, {self.target_vy:.2f}, {self.target_wz:.2f}) | "
                  f"Current: ({self.current_vx:.2f}, {self.current_vy:.2f}, {self.current_wz:.2f})")
        # ------------------

        if not self.can_move:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        cmd_vx = self.solve_mpc_step(self.current_vx, self.target_vx, self.last_cmd_vx, self.max_vx, self.max_acc_v)
        cmd_vy = self.solve_mpc_step(self.current_vy, self.target_vy, self.last_cmd_vy, self.max_vy, self.max_acc_v)
        cmd_wz = self.solve_mpc_step(self.current_wz, self.target_wz, self.last_cmd_wz, self.max_wz, self.max_acc_w)

        cmd_vx = self.apply_deadzone_compensation(cmd_vx, self.target_vx, self.min_effective_vx, self.max_vx)
        cmd_vy = self.apply_deadzone_compensation(cmd_vy, self.target_vy, self.min_effective_vy, self.max_vy)
        cmd_wz = self.apply_deadzone_compensation(cmd_wz, self.target_wz, self.min_effective_wz, self.max_wz)

        is_stop_command = (abs(cmd_vx) < self.stop_epsilon and abs(cmd_vy) < self.stop_epsilon and abs(cmd_wz) < self.stop_epsilon)
        is_in_deadzone = (abs(cmd_vx) < self.min_effective_vx and abs(cmd_vy) < self.min_effective_vy and abs(cmd_wz) < self.min_effective_wz) 



        # 打印机器人运动状态，确认数据流
        if self.is_print_info: 
            if is_in_deadzone and not is_stop_command:
                rospy.logwarn(f"cmd_vel低于死区: ({cmd_vx:.2f}, {cmd_vy:.2f}, {cmd_wz:.2f})")
            else:
                print(f"cmd_vel: ({cmd_vx:.2f}, {cmd_vy:.2f}, {cmd_wz:.2f})")
        # ------------------

        if is_stop_command:
            self.sport_client.StopMove()
        elif is_in_deadzone:
            # 导航模块和MPC均输出了一个小于死区的命令，这会导致机器人被死区卡住
            # 这时我们强行输出一个最小有效命令，帮助它“跳出”死区
            # 补偿策略：分别将vx vy vz的绝对值对死区值进行归一化，选择归一化值最大的速度分量进行速度补偿到死区值
            # 计算各分量归一化后绝对值
            norm_vx = abs(cmd_vx) / self.min_effective_vx
            norm_vy = abs(cmd_vy) / self.min_effective_vy
            norm_wz = abs(cmd_wz) / self.min_effective_wz

            # 找到最大归一化分量
            max_norm = max(norm_vx, norm_vy, norm_wz)
            if max_norm == norm_wz :
                # cmd_wz = np.sign(cmd_wz) * self.min_effective_wz
                cmd_wz = np.sign(cmd_wz) * self.min_effective_wz * 3.0
            elif max_norm == norm_vx:
                cmd_vx = np.sign(cmd_vx) * self.min_effective_vx
            else:
                cmd_vy = np.sign(cmd_vy) * self.min_effective_vy
            
            if self.is_print_info:
                rospy.logwarn(f"命令被死区限制，应用补偿: ({cmd_vx:.2f}, {cmd_vy:.2f}, {cmd_wz:.2f})")
            self.sport_client.Move(cmd_vx, cmd_vy, cmd_wz)
        else:
            self.sport_client.Move(cmd_vx, cmd_vy, cmd_wz)
            # self.sport_client.Move(0.3, cmd_vy, cmd_wz)

        
        self.last_cmd_vx = cmd_vx
        self.last_cmd_vy = cmd_vy
        self.last_cmd_wz = cmd_wz

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 g1_control_mpc_debug.py <network_interface>")
        sys.exit(-1)

    network_interface = sys.argv[1]
    rospy.init_node("unitree_mpc_controller", anonymous=False)
    
    try:
        controller = MPCController(network_interface)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
