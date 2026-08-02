#!/usr/bin/env python3
"""Input-scan / output-odom arrival trace (artifact ③, the timing half).

Records two kinds of event with the wall clock at which this node received them:
`in` for every scan on the system's input topic, `out` for every odometry message the
system publishes. Nothing is paired or derived here — eval/aggregate.py does that, so
improving the pairing rule later never costs another bag replay.

Output CSV: `kind,stamp,wall`. `stamp` is the message header's sensor time; `wall` is
`time.time()` — NOT `rospy.Time.now()`, which under the `use_sim_time` every run sets is
bag time and therefore says nothing about how long anything took.
"""
import argparse
import struct
import sys
import time

import rospy
from nav_msgs.msg import Odometry

# One scan on the input topic is ~280 kB (MID-360 CustomMsg). rospy's default 64 kB
# receive buffer would split it across several recv() calls and add jitter to exactly the
# arrival time this file exists to measure.
SCAN_BUFF_SIZE = 2 ** 22

# The callback is O(1) — a 12-byte unpack and one line of output — so this only has to
# absorb scheduling hiccups, not sustained backlog. 200 scans is 20 s of buffer at the
# rig's 10 Hz. Dropping an input here would inflate out_ratio, which is why it is not
# left at the default.
SCAN_QUEUE = 200


def parse_header_stamp(buf):
    """Seconds from a serialized ROS message's leading std_msgs/Header.

    Header is the first field of both livox_ros_driver/CustomMsg and
    sensor_msgs/PointCloud2, and serializes as uint32 seq, uint32 secs, uint32 nsecs. So
    the stamp is readable without deserializing the point data behind it — which keeps
    this node off the CPU that artifact ③ is measuring — and without depending on either
    message type being built.
    """
    _seq, secs, nsecs = struct.unpack("<III", buf[:12])
    return secs + nsecs * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-topic", required=True, help="the system's input scan topic")
    ap.add_argument("--odom", required=True, help="the system's output odometry topic")
    ap.add_argument("--out", required=True, help="output frame_events.csv path")
    ap.add_argument(
        "--ready-file",
        help="touched once both subscriptions are registered, so the caller can hold "
        "playback until then rather than lose the opening scans",
    )
    args = ap.parse_args()

    out = open(args.out, "w")
    out.write("kind,stamp,wall\n")
    out.flush()
    seen = {"in": 0, "out": 0}

    def write(kind, stamp, wall):
        seen[kind] += 1
        # Flushed per event, like record_tum.py: a run cut short by Ctrl-C still leaves
        # everything it had measured up to that point.
        out.write("%s,%.9f,%.6f\n" % (kind, stamp, wall))
        out.flush()

    def on_scan(msg):
        write("in", parse_header_stamp(msg._buff), time.time())

    def on_odom(msg):
        write("out", msg.header.stamp.to_sec(), time.time())

    rospy.init_node("record_frames", anonymous=True)
    # tcp_nodelay is here for the odometry side. An odom message is a few hundred bytes,
    # so without it Nagle holds each one until the previous message's ACK returns, and
    # that ACK is itself delayed by up to 40 ms — once a system publishes faster than
    # that, poses arrive in batches and `wall` times the batch, not the pose. Measured on
    # point_lio, the share of consecutive outputs under 1 ms apart went 0.0% at 1x and 2x
    # (100 and 50 ms apart) to 16.3% at 3x and 45.0% at 5x, inflating lat_p50 from 21.2 to
    # 35.1 ms on a run whose queue was not growing at all. Setting it on the scan side too
    # is free: a 280 kB message is far past the MSS, so Nagle never held it anyway.
    #
    # Input subscribed first: run_system.sh gates playback on the ODOM subscription
    # appearing at the master, and that gate is only sound if this one is already there.
    rospy.Subscriber(
        args.in_topic, rospy.AnyMsg, on_scan,
        queue_size=SCAN_QUEUE, buff_size=SCAN_BUFF_SIZE, tcp_nodelay=True,
    )
    rospy.Subscriber(args.odom, Odometry, on_odom, queue_size=2000, tcp_nodelay=True)
    if args.ready_file:
        open(args.ready_file, "w").close()
    rospy.loginfo(
        "recording %s + %s -> %s", args.in_topic, args.odom, args.out
    )
    try:
        rospy.spin()
    finally:
        out.close()
        # A silent header-only file would reach aggregate.py as "not measured" and show
        # up as a dash in the table, which reads like the metric does not apply rather
        # than like the topic was wrong.
        if not seen["in"]:
            print(
                "record_frames: nothing ever arrived on %s — every latency metric for "
                "this run will be null" % args.in_topic,
                file=sys.stderr,
            )


if __name__ == "__main__":
    main()
