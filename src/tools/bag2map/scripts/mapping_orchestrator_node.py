#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from multiprocessing import Event, Lock
from slam_toolbox_msgs.srv import SaveMap
import os


SAVE_MAP_SERVICE_TIMEOUT = 60  # seconds


class Orchestrator:
    FIRST_ODOM_MSG_TIMEOUT = 60  # seconds
    MAX_TIME_BETWEEN_ODOM_MSGS = 2  # seconds

    def __init__(self):
        self._received_event = Event()
        self._pub = rospy.Publisher(
            '/tf_static', TFMessage, queue_size=10, latch=True)
        self._static_tfs_lock = Lock()
        self._timer = rospy.Timer(rospy.Duration(secs=1), self._relay_tfs)
        self._static_tfs = None
        self._tf_sub = rospy.Subscriber(
            "/tf_static_replayed", TFMessage, self._on_static_tf)
        self._odom_sub = rospy.Subscriber("/odom", Odometry, self._on_odom)
        self._received_event.wait(self.FIRST_ODOM_MSG_TIMEOUT)

    def _relay_tfs(self, _):
        with self._static_tfs_lock:
            if self._static_tfs is None:
                return
            self._pub.publish(self._static_tfs)

    def _on_static_tf(self, msg: TFMessage):
        rospy.loginfo(f"Got static transforms")
        with self._static_tfs_lock:
            if self._static_tfs is None:
                self._static_tfs = msg
                return
            self._static_tfs.transforms.extend(msg.transforms)

    def _on_odom(self, _):
        self._received_event.set()

    def is_alive(self) -> bool:
        ret = self._received_event.wait(self.MAX_TIME_BETWEEN_ODOM_MSGS)
        self._received_event.clear()
        return ret


def main():
    rospy.init_node("orchestrator", anonymous=False)
    orchestrator = Orchestrator()
    save_map_client = rospy.ServiceProxy("/slam_toolbox/save_map", SaveMap)
    rospy.loginfo("Waiting for save map service")
    output_dir = rospy.get_param("~output_dir")
    save_map_client.wait_for_service(SAVE_MAP_SERVICE_TIMEOUT)
    rospy.loginfo("Got save map service")
    while True:
        if rospy.is_shutdown():
            rospy.logerr("Ros master died, aborting mapping.")
            return -1
        if not orchestrator.is_alive():
            rospy.loginfo(
                "Didn't get any odom message in time, proceeding to save map.")
            break
    os.makedirs(output_dir, exist_ok=True)
    save_map_client.call(name=String(data=output_dir))


if __name__ == '__main__':
    main()
