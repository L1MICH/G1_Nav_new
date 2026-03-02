#!/usr/bin/env python3
"""
多目标导航脚本（集成语音和动作版）
"""

import rospy
import actionlib
import threading
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# ========== 新增：导入 Unitree SDK ==========
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
# ===========================================

class RobotController:
    """机器人控制器：管理语音和动作"""
    
    def __init__(self, network_interface):
        rospy.loginfo("正在初始化语音和动作系统...")
        
        # 初始化 ChannelFactory
        ChannelFactoryInitialize(0, network_interface)
        
        # 初始化音频客户端
        self.audio_client = AudioClient()
        self.audio_client.Init()
        self.audio_client.SetTimeout(10.0)
        
        # 初始化动作客户端
        self.arm_client = G1ArmActionClient()
        self.arm_client.Init()
        
        # 执行音频唤醒咒语（必须！否则没声音）
        self._wakeup_audio()
        
        rospy.loginfo("✅ 语音和动作系统初始化完成")
    
    def _wakeup_audio(self):
        """唤醒音频硬件的关键咒语"""
        rospy.loginfo("🔊 正在唤醒音频硬件...")
        
        # 等待服务连接
        for i in range(10):
            try:
                self.audio_client.GetVolume()
                rospy.loginfo("✅ 音频服务已连接")
                break
            except:
                rospy.logwarn(f"⏳ 等待音频服务... ({i+1}/10)")
                time.sleep(1)
        
        # 关键咒语：先中文，再英文
        self.audio_client.SetVolume(100)
        time.sleep(0.5)
        
        # 第1句：中文
        self.audio_client.TtsMaker("大家好", 0)
        time.sleep(1.0)
        
        # 第2句：英文（参数1是关键！）
        self.audio_client.TtsMaker("Hello Hello Hello", 1)
        time.sleep(3.0)
        
        rospy.loginfo("✅ 音频硬件唤醒完成")
    
    def say_hello_and_wave(self):
        """说 hello 并执行第一个动作（举手）"""
        rospy.loginfo("🎤 播放: hello")
        self.audio_client.TtsMaker("hello", 0)
        
        rospy.loginfo("🤖 执行动作 ID: 18（举手）")
        
        # 执行动作ID 18
        self.arm_client.ExecuteAction(18)
        time.sleep(1.0)  # 等待动作开始
        
        # 复位（释放手臂）
        self.arm_client.ExecuteAction(99)
        time.sleep(2.0)  # 等待复位完成
        
        rospy.loginfo("✅ 动作完成")


def optimize_move_base_params():
    """
    动态调整 move_base 的局部规划器参数，解决振荡和超调
    """
    rospy.loginfo("--- 正在应用导航优化参数 ---")
    
    # 位置公差
    rospy.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.35)
    
    # 角度公差（你可以根据需要调整，0.15/8.5*20 = 0.35，约20度）
    rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance', 0.15/8.5*20)
    
    # 最大速度
    rospy.set_param('/move_base/DWAPlannerROS/max_vel_x', 0.2) 
    
    # 刹车力度
    rospy.set_param('/move_base/DWAPlannerROS/acc_lim_x', 2.0)

    rospy.loginfo("--- 参数优化完成 ---")
    rospy.sleep(0.5)


def navigate_to_waypoints(waypoints, robot_controller):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待move_base服务器启动...")
    client.wait_for_server()
    optimize_move_base_params()

    for idx, waypoint in enumerate(waypoints):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint["x"]
        goal.target_pose.pose.position.y = waypoint["y"]
        goal.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, waypoint["yaw"])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"发送第{idx+1}个目标: ({waypoint['x']}, {waypoint['y']})")
        client.send_goal(goal)
        
        # --- 自定义等待循环：检测被卡住 ---
        start_time = time.time()
        last_spoke_time = 0
        speak_interval = 10.0  # 避免太频繁
        
        while not rospy.is_shutdown():
            state = client.get_state()
            
            # 1. 成功
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"✅ 到达第{idx+1}个点")
                threading.Thread(target=robot_controller.say_hello_and_wave, daemon=True).start()
                rospy.loginfo("休息 5 秒...")
                rospy.sleep(5.0)
                break
            
            # 2. 失败
            if state == actionlib.GoalStatus.ABORTED:
                rospy.logwarn("⚠️ 导航失败，尝试下一个点")
                break
                
            # 3. 检测被卡住 (修改点：使用 PENDING 状态)
            elapsed = time.time() - start_time
            now = time.time()
            
            # 如果机器人一直在“规划中”(PENDING)且超过了5秒，说明路径被堵死无法规划
            if state == actionlib.GoalStatus.PENDING:
                if elapsed > 5.0 and (now - last_spoke_time > speak_interval):
                    rospy.logwarn("🚧 检测到路径受阻（规划中）")
                    robot_controller.speak("请让一让") # 按你的要求
                    last_spoke_time = now
            
            rospy.sleep(0.1)



if __name__ == '__main__':
    try:
        # 初始化 ROS 节点
        rospy.init_node('multi_waypoint_nav')
        
        # 从参数服务器获取网络接口
        # 运行方式: rosrun your_package multi_waypoint_nav_pro.py eno1
        import sys
        if len(sys.argv) < 2:
            rospy.logerr("请提供网络接口: rosrun package_name script.py network_interface")
            sys.exit(1)
        
        network_interface = sys.argv[1]
        
        # ========== 新增：初始化机器人控制器 ==========
        robot_controller = RobotController(network_interface)
        # ===========================================
        
        waypoints = [
            {"x": 0.20, "y": -0.43, "yaw": 0.22},   
            {"x": 2.07, "y": -0.25, "yaw": 1.72},  
            {"x": 0.38, "y": -0.40, "yaw": -3.08},  
        ]
        
        navigate_to_waypoints(waypoints, robot_controller)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("导航脚本被中断")
