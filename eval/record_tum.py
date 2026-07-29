#!/usr/bin/env python3
"""Record a nav_msgs/Odometry topic to a TUM trajectory file (artifact ①).

TUM line: `timestamp tx ty tz qx qy qz qw`, one per odom message, using the message
header stamp. This is the single normalized trajectory format all accuracy metrics
(start-end drift, evo plots) consume — independent of which system produced it.
"""
import argparse

import rospy
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True, help="nav_msgs/Odometry topic to record")
    ap.add_argument("--out", required=True, help="output TUM file path")
    args = ap.parse_args()

    out = open(args.out, "w")

    def cb(msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        out.write(
            "%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n"
            % (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        )
        out.flush()

    rospy.init_node("record_tum", anonymous=True)
    rospy.Subscriber(args.topic, Odometry, cb, queue_size=2000)
    rospy.loginfo("recording %s -> %s", args.topic, args.out)
    try:
        rospy.spin()
    finally:
        out.close()


if __name__ == "__main__":
    main()
