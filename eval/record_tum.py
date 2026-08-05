#!/usr/bin/env python3
"""Record a nav_msgs/Odometry topic to a TUM trajectory file (artifact ①).

TUM line: `timestamp tx ty tz qx qy qz qw`, one per odom message, using the message
header stamp. This is the single normalized trajectory format all accuracy metrics
(start-end drift, evo plots) consume — independent of which system produced it.

It also watches the stream go by. This is the only process holding the poses while a run
is still playing, so the divergence detector lives here rather than in a second subscriber
that would have to be raced into place alongside this one. Recording continues after the
verdict: the part of the trajectory that flew away is the evidence, and aggregate.py's
v_max_mps and compare.py's truncation both read it.
"""
import argparse

import divergence
import rospy
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True, help="nav_msgs/Odometry topic to record")
    ap.add_argument("--out", required=True, help="output TUM file path")
    ap.add_argument(
        "--diverged-file",
        # Its appearance is the signal, the same shape as --ready-file in record_frames.py:
        # this process cannot see run_system.sh's playback PID, and run_system.sh cannot
        # see the poses. A file is what they share.
        help="write here, and keep recording, when the trajectory is judged diverged; "
        "run_system.sh watches for it and stops playback",
    )
    args = ap.parse_args()

    out = open(args.out, "w")
    detector = divergence.Detector()

    def cb(msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        out.write(
            "%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n"
            % (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        )
        out.flush()

        reason = detector.push(t, p.x, p.y, p.z)
        if reason is None:
            return
        rospy.logwarn("trajectory diverged: %s", reason)
        if not args.diverged_file:
            return
        try:
            with open(args.diverged_file, "w") as fh:
                fh.write(reason + "\n")
        except OSError as e:
            # Never at the cost of the recording: a run that cannot be aborted is still a
            # run, and aggregate.py reaches the same verdict offline from this very file.
            rospy.logerr("could not write %s: %s", args.diverged_file, e)

    rospy.init_node("record_tum", anonymous=True)
    rospy.Subscriber(args.topic, Odometry, cb, queue_size=2000)
    rospy.loginfo("recording %s -> %s", args.topic, args.out)
    try:
        rospy.spin()
    finally:
        out.close()


if __name__ == "__main__":
    main()
