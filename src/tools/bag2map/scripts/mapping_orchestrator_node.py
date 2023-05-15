#! /usr/bin/env python3
from rosbag import Bag
import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from multiprocessing import Event, Lock
from slam_toolbox_msgs.srv import SaveMap
import os
from pathlib import Path


SAVE_MAP_SERVICE_TIMEOUT = 60  # seconds


class Orchestrator:
    FIRST_ODOM_MSG_TIMEOUT = 60  # seconds

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


def main():
    rospy.init_node("orchestrator", anonymous=False)
    orchestrator = Orchestrator()
    output_dir = rospy.get_param("~output_dir")
    end_timestamp: float = float(rospy.get_param("~end_timestamp", 0))
    bag_path = Path(rospy.get_param("~bag_path"))
    assert bag_path.is_file(
    ), f"Bag file doesn't exist in {bag_path.resolve()}"

    bag_end_timestamp = Bag(bag_path.resolve()).get_end_time()

    if end_timestamp == 0:
        end_timestamp = bag_end_timestamp
    else:
        assert end_timestamp < bag_end_timestamp, f"The provided end time is past the bag's end time."
        "{end_timestamp} vs {bag_end_timestamp}"

    rospy.loginfo("Got save map service")
    os.makedirs(output_dir, exist_ok=True)

    save_map_client = rospy.ServiceProxy("/slam_toolbox/save_map", SaveMap)
    rospy.loginfo("Waiting for save map service")
    save_map_client.wait_for_service(SAVE_MAP_SERVICE_TIMEOUT)

    rate = rospy.Rate(hz=1)

    while True:
        if rospy.is_shutdown():
            rospy.logerr("Ros master died, aborting mapping.")
            return -1
        rate.sleep()
        if rospy.Time.now().to_time() >= end_timestamp:
            rospy.loginfo("Finished mapping, proceeding to save the map.")
            break
    save_map_client.call(name=String(data=output_dir))


if __name__ == '__main__':
    main()
