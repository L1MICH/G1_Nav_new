#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class CmdVelController:
    def __init__(self, network_interface):
        # 初始化 Unitree SDK
        rospy.loginfo("Initializing Unitree LocoClient...")
        ChannelFactoryInitialize(0, network_interface)

        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        self.can_move = False  # 标志位：是否可以开始运动

        # 订阅 /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.loginfo("Subscribed to /cmd_vel")

        # 订阅全局路径
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)
        rospy.loginfo("Subscribed to global path topic /move_base/GlobalPlanner/plan")

    def path_callback(self, msg: Path):
        if len(msg.poses) > 0:
            if not self.can_move:
                rospy.loginfo("Global path received, robot can start moving.")
            self.can_move = True
        else:
            if self.can_move:
                rospy.logwarn("Global path is empty, robot cannot move.")
            self.can_move = False

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x      # 前后移动
        vy = msg.linear.y      # 横向移动
        wz = msg.angular.z     # 旋转

        # 如果 cmd_vel 全 0，则认为不能移动
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            if self.can_move:
                rospy.logwarn("Received cmd_vel is all zeros. Robot will stop.")
            self.can_move = False
            return

        # 如果之前因为路径为空而不能动，也忽略
        if not self.can_move:
            rospy.logwarn("Global path not received or empty. Ignoring cmd_vel.")
            return

        rospy.loginfo(f"Received cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
        try:
            self.sport_client.Move(vx, vy, wz)
        except Exception as e:
            rospy.logerr(f"Failed to send Move command: {e}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: rosrun your_package cmd_vel_control.py networkInterface")
        sys.exit(-1)

    network_interface = sys.argv[1]

    rospy.init_node("unitree_cmd_vel_controller", anonymous=False)
    rospy.logwarn("Make sure the robot is in a safe environment before sending cmd_vel commands!")

    controller = CmdVelController(network_interface)

    rospy.spin()