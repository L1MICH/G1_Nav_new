#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from datetime import datetime
import rospy
import json
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from unitree_sdk2_python.unitree_sdk2py.idl.builtin_interfaces import msg


STATUS_NAME_MAP = {
	0: "PENDING",
	1: "ACTIVE",
	2: "PREEMPTED",
	3: "SUCCEEDED",
	4: "ABORTED",
	5: "REJECTED",
	6: "PREEMPTING",
	7: "RECALLING",
	8: "RECALLED",
	9: "LOST",
}


class NavStatusLogger:
	def __init__(self) -> None:
		rospy.init_node("nav_status_logger", anonymous=False)

		self.pub = rospy.Publisher("/navigation", String, queue_size=10)

		# 仅在状态切换时发布，避免 /move_base/status 持续重复消息导致刷屏。
		self._last_status_code = None
		self._last_status_text = None

		rospy.Subscriber("/move_base/status", GoalStatusArray, self._status_callback, queue_size=50)
		rospy.loginfo("nav_status_logger started, listening /move_base/status and publishing /nav/status")

	def _status_callback(self, msg: GoalStatusArray) -> None:
		if not msg.status_list:
			return

		# move_base 会周期发布状态列表，这里取最后一项作为最新状态。
		latest = msg.status_list[-1]
		status_code = int(latest.status)
		status_text = (latest.text or "").strip()

		if status_code == self._last_status_code and status_text == self._last_status_text:
			return

		self._last_status_code = status_code
		self._last_status_text = status_text

		status_name = STATUS_NAME_MAP.get(status_code, "UNKNOWN")
		# out = f"status={status_code}({status_name}), text={status_text}"
		# out = json.dumps({"status_code": status_code, "status_name": status_name, "status_text": status_text},indent=4,
        #     ensure_ascii=False,)

		body = json.dumps(
            {
                "module": "Navigation",
                "payload": {"message": str(status_name), "state": str(status_code), "status_text": str(status_text)},
            },
            indent=4,
            ensure_ascii=False,
        )
		
		json_msg = String()
		json_msg.data = body

		self.pub.publish(json_msg)
		rospy.loginfo("/nav/status -> %s", json_msg)
		rospy.loginfo("time -> %s", datetime.now().strftime("%H:%M:%S:%f")[:-3])

if __name__ == "__main__":
	try:
		NavStatusLogger()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
