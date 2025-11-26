#!/usr/bin/env python3
"""
Offline utilities for working with recorded LiDAR data.

Step 2 focus:
    - Load recorded .bin files into a list of LidarPacket objects.

Point cloud decoding and further processing will be added in later steps.
"""

from __future__ import annotations

from pathlib import Path
from typing import List

from lidar_reader import LidarPacket, LidarLogger


def load_packets(bin_path: str | Path) -> List[LidarPacket]:
    """
    Load all packets from a recorded .bin file.

    Args:
        bin_path: Path to the .bin file created by LidarLogger.

    Returns:
        List of LidarPacket objects in chronological order.
    """
    bin_path = Path(bin_path)
    header_struct = LidarLogger.HEADER
    packets: List[LidarPacket] = []

    with open(bin_path, "rb") as f:
        while True:
            header_bytes = f.read(header_struct.size)
            if not header_bytes or len(header_bytes) < header_struct.size:
                break

            ts, length = header_struct.unpack(header_bytes)
            data = f.read(length)
            if len(data) < length:
                break

            packets.append(LidarPacket(timestamp=ts, data=data))

    return packets


# Placeholder for later steps: decoding raw packets into XYZ points.
def decode_velodyne_packets_to_points(packets: List[LidarPacket]):
    """
    Convert raw Velodyne packets into a point cloud (x, y, z, intensity).

    This is intentionally left as a placeholder for later development
    (Step 4: preprocessing + projection to 2D).

    Args:
        packets: List of LidarPacket.

    Returns:
        To be defined: e.g. numpy array of shape (N, 4).
    """
    raise NotImplementedError(
        "Point cloud decoding will be implemented in the preprocessing step."
    )


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python lidar_processing.py <path_to_bin>")
        raise SystemExit(1)

    path = sys.argv[1]
    pkts = load_packets(path)
    if not pkts:
        print("No packets loaded. Check the file path.")
    else:
        print(f"Loaded {len(pkts)} packets from {path}")
        print(f"First packet timestamp: {pkts[0].timestamp}")
        print(f"First packet size    : {len(pkts[0].data)} bytes")
