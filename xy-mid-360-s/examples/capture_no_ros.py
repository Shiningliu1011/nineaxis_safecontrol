#!/usr/bin/env python3
"""No-ROS capture example: listen for pushed point clouds and print frame stats.

The device must already be pushing to this host:port.  Either configure it once
(``xy-mid-360-s configure --ip <device> --host-ip <local>``, a state-changing
command), or let the official ``livox_ros_driver2`` do it while this script is
*not* running alongside it — both bind the same UDP port.

Usage::

    python3 examples/capture_no_ros.py --host-ip 192.168.1.5 --seconds 10
"""

from __future__ import annotations

import argparse
import time

from xy_mid_360_s.protocol import PORT_PUSH_PCL
from xy_mid_360_s.stream import CloudReceiver


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-ip", default="", help="local address to bind ('' = all)")
    parser.add_argument("--port", type=int, default=PORT_PUSH_PCL)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    def on_frame(frame) -> None:
        print(f"frame: {frame.size} points, base_time_ns={frame.base_time_ns}, "
              f"src={frame.src_ip}")

    receiver = CloudReceiver(host_ip=args.host_ip, port=args.port, on_frame=on_frame)
    receiver.start()
    print(f"listening on {args.host_ip or '0.0.0.0'}:{receiver.port} "
          f"for {args.seconds:.0f}s (Ctrl-C to stop early)")
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()

    print("stats:", receiver.stats)
    return 0 if receiver.stats["frames"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
