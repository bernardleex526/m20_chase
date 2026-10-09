#!/usr/bin/env python3
"""Measure rs_follow CPU/RSS against synthetic clouds of realistic size.

Starts one node, drives it with synthetic PointCloud2 frames at a given point
count, and samples /proc/<pid>/stat so the number quoted is pure algorithm cost
(no gzserver, no simulator).

Usage: python3 bench_load.py            # runs all cases
"""
import os
import signal
import subprocess
import sys
import time

ROS_SETUP = "source /opt/ros/humble/setup.bash && source $HOME/m20_chase/install/setup.bash"

CASES = [
    # (points, hz, seconds, note)
    (6144,   10, 12, "Gazebo ray sensor (what we tested with)"),
    (32768,  10, 12, "RoboSense RS-Helios-16 class"),
    (65536,  10, 12, "RoboSense RS-Ruby lite class"),
    (131072, 10, 12, "dense 128-beam class"),
]


def node_pid():
    out = subprocess.run(["pgrep", "-f", "lib/rs_follow/rs_follow_node"],
                         capture_output=True, text=True).stdout.split()
    return int(out[0]) if out else None


def proc_cpu_jiffies(pid):
    with open(f"/proc/{pid}/stat") as f:
        f = f.read().split()
    return int(f[13]) + int(f[14])


def rss_kb(pid):
    try:
        with open(f"/proc/{pid}/status") as f:
            for l in f:
                if l.startswith("VmRSS"):
                    return int(l.split()[1])
    except OSError:
        pass
    return None


def cleanup():
    for pat in ("[l]ib/rs_follow/rs_follow_node", "[r]os2 run rs_follow"):
        subprocess.run(f"pkill -f '{pat}'", shell=True)
    time.sleep(1.0)


def run_case(npts, hz, secs, note):
    print(f"\n{'='*70}")
    print(f"{npts} points @ {hz} Hz for {secs}s  -  {note}")
    print("=" * 70)
    cleanup()

    node = subprocess.Popen(
        ["bash", "-lc",
         f"{ROS_SETUP} && exec ros2 run rs_follow rs_follow_node --ros-args "
         f"-p input_topic:=/rslidar_points -p cmd_vel_topic:=/cmd_vel "
         f"-p control_frame:=rslidar -p height_min:=-0.4 -p height_max:=1.8 "
         f"-p enable_low_band:=false "
         f"-p odom_topic:=/odom -p publish_scan_debug:=true -p active:=false"],
        stdout=open("/tmp/bench_node.log", "w"), stderr=subprocess.STDOUT)
    time.sleep(5)

    pid = node_pid()
    if pid is None:
        print("!! node did not start"); print(open("/tmp/bench_node.log").read()[-500:])
        return None

    pub = subprocess.Popen(
        ["bash", "-lc",
         f"{ROS_SETUP} && exec python3 $HOME/bench_cloud.py {npts} {secs} {hz}"],
        stdout=open("/tmp/bench_pub.log", "w"), stderr=subprocess.STDOUT)
    time.sleep(3)                      # let the first frames arrive

    j0 = proc_cpu_jiffies(pid)
    t0 = time.time()
    rss_max = 0
    while pub.poll() is None:
        r = rss_kb(pid)
        rss_max = max(rss_max, r or 0)
        time.sleep(0.25)
    t1 = time.time()
    j1 = proc_cpu_jiffies(pid)

    hz_clk = os.sysconf("SC_CLK_TCK")
    dt = t1 - t0
    cpu = (j1 - j0) / hz_clk / dt * 100.0
    print(f"  wall          {dt:.2f} s")
    print(f"  CPU           {cpu:.1f} % of one core")
    print(f"  RSS (peak)    {rss_max/1024:.1f} MB")
    print(f"  publisher     {open('/tmp/bench_pub.log').read().strip().splitlines()[-1]}")
    log = open("/tmp/bench_node.log").read().strip().splitlines()
    for l in log[-2:]:
        print(f"  node: {l}")
    node.send_signal(signal.SIGINT)
    try:
        node.wait(timeout=5)
    except subprocess.TimeoutExpired:
        node.kill()
    cleanup()
    return cpu, rss_max


def main():
    results = []
    for c in CASES:
        r = run_case(*c)
        if r:
            results.append((c[0], r[0], r[1]))
    print(f"\n{'='*70}")
    print("SUMMARY  (rs_follow_node alone, no simulator)")
    print("=" * 70)
    print(f"{'points':>8} {'CPU %core':>10} {'RSS MB':>8}")
    for n, cpu, rss in results:
        print(f"{n:>8} {cpu:>10.1f} {rss/1024:>8.1f}")


if __name__ == "__main__":
    main()
