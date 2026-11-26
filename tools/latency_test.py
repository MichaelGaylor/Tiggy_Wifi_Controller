#!/usr/bin/env python3
"""
How long the controller takes to answer over the network.

    python tools/latency_test.py 192.168.1.43 [count]

Sends "who is there?" packets one at a time, at uneven intervals, and times the answers.  What
matters for a machine is the worst case: a stop command waits just as long.
"""
import random
import socket
import statistics
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from protocol_conformance import Client, UDP_MOTION_PORT  # noqa: E402


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    ip = sys.argv[1]
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 200
    c = Client(ip)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1.0)
    times, lost = [], 0
    for n in range(count):
        time.sleep(random.uniform(0.02, 0.25))          # not in step with anything the radio does
        t0 = time.perf_counter()
        s.sendto(c.pkt(0x0B), (ip, UDP_MOTION_PORT))
        try:
            while True:
                data, _ = s.recvfrom(512)
                p = c.parse(data)
                if p and p[0] == 0x25:
                    break
            times.append((time.perf_counter() - t0) * 1000.0)
        except socket.timeout:
            lost += 1
    times.sort()
    if not times:
        sys.exit("no answers")
    print("%d answers, %d lost: median %.1f ms, 95 %% within %.1f ms, worst %.1f ms" % (
        len(times), lost, statistics.median(times), times[int(len(times) * 0.95) - 1], times[-1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
