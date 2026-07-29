#!/usr/bin/env python3
"""External per-process resource sampler (artifact ③).

Finds the system-under-test by process name, then samples CPU% (share of one core;
may exceed 100 on multi-threaded systems) and RSS from /proc at a fixed interval.
Fully external — never trusts a system's self-reported numbers. No third-party deps.

Output CSV: `wall_s,cpu_pct,rss_mb`.
"""
import argparse
import glob
import os
import time


def find_pid(name):
    for path in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            with open(path, "rb") as f:
                cmd = f.read().replace(b"\x00", b" ").decode(errors="ignore")
        except (IOError, OSError):
            continue
        if name in cmd:
            return int(path.split("/")[2])
    return None


def read_cpu_ticks(pid):
    # utime (field 14) + stime (field 15); fields after comm which may contain spaces.
    with open("/proc/%d/stat" % pid) as f:
        data = f.read()
    fields = data[data.rfind(")") + 2:].split()
    return int(fields[11]) + int(fields[12])  # utime + stime (0-indexed after comm)


def read_rss_mb(pid):
    with open("/proc/%d/status" % pid) as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    return 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--proc", required=True, help="process-name substring to sample")
    ap.add_argument("--out", required=True)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--startup-timeout", type=float, default=60.0)
    args = ap.parse_args()

    hz = os.sysconf("SC_CLK_TCK")

    deadline = time.time() + args.startup_timeout
    pid = None
    while pid is None and time.time() < deadline:
        pid = find_pid(args.proc)
        if pid is None:
            time.sleep(0.2)
    if pid is None:
        print("sample_resource: process '%s' never appeared" % args.proc)
        return

    out = open(args.out, "w")
    out.write("wall_s,cpu_pct,rss_mb\n")
    prev_ticks, prev_w = read_cpu_ticks(pid), time.time()
    try:
        while os.path.exists("/proc/%d" % pid):
            time.sleep(args.interval)
            try:
                cur_ticks, cur_w = read_cpu_ticks(pid), time.time()
                rss = read_rss_mb(pid)
            except (IOError, OSError):
                break
            dt = cur_w - prev_w
            cpu = 100.0 * (cur_ticks - prev_ticks) / hz / dt if dt > 0 else 0.0
            out.write("%.3f,%.1f,%.1f\n" % (cur_w, cpu, rss))
            out.flush()
            prev_ticks, prev_w = cur_ticks, cur_w
    finally:
        out.close()


if __name__ == "__main__":
    main()
