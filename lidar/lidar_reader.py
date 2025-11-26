#!/usr/bin/env python3
"""
LiDAR reader, logger, and replayer for Velodyne-style sensors.

Usage examples:

    # Just listen and print packet counts
    python lidar_reader.py --mode listen

    # Record 60 seconds of data to logs/lidar/
    python lidar_reader.py --mode record --duration 60

    # Replay an existing .bin file in (approximate) real time
    python lidar_reader.py --mode replay --bin logs/lidar/vlp16_YYYYMMDD_HHMMSS.bin
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Iterator, BinaryIO

import yaml


# ----------------------------------------------------------------------
# Data structures and config helpers
# ----------------------------------------------------------------------


@dataclass
class LidarPacket:
    """Single LiDAR UDP packet with a host-side timestamp."""

    timestamp: float  # seconds since epoch
    data: bytes       # raw UDP payload


def load_lidar_config(path: str | Path = "lidar/config_lidar.yaml") -> dict:
    """
    Load LiDAR configuration from a YAML file.

    Expected structure:

    lidar:
      model: "VLP-16"
      sensor_ip: "192.168.1.201"
      host_ip: "0.0.0.0"
      data_port: 2368
      packet_size: 1206
      recv_buffer_size: 65535
      log_dir: "logs/lidar"
    """
    path = Path(path)
    with open(path, "r") as f:
        cfg_all = yaml.safe_load(f)
    return cfg_all["lidar"]


# ----------------------------------------------------------------------
# Live receiver
# ----------------------------------------------------------------------


class VelodyneReceiver:
    """Receive raw UDP packets from a Velodyne LiDAR."""

    def __init__(self, config_path: str | Path = "lidar/config_lidar.yaml"):
        self.cfg = load_lidar_config(config_path)
        self.sock: Optional[socket.socket] = None

    def connect(self) -> None:
        """Bind a UDP socket to receive LiDAR data."""
        host_ip = self.cfg.get("host_ip", "0.0.0.0")
        data_port = int(self.cfg.get("data_port", 2368))
        buf_size = int(self.cfg.get("recv_buffer_size", 65535))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buf_size)
        self.sock.bind((host_ip, data_port))

        print(f"[VelodyneReceiver] Listening on {host_ip}:{data_port}")

    def receive(self, timeout: Optional[float] = None) -> Optional[LidarPacket]:
        """
        Blocking receive for a single packet.

        Returns:
            LidarPacket on success, or None on timeout.
        """
        if self.sock is None:
            raise RuntimeError("Socket not initialized. Call connect() first.")

        if timeout is not None:
            self.sock.settimeout(timeout)

        try:
            data, _ = self.sock.recvfrom(65535)
        except socket.timeout:
            return None

        ts = time.time()
        return LidarPacket(timestamp=ts, data=data)


# ----------------------------------------------------------------------
# Logger: record to .bin + .json
# ----------------------------------------------------------------------


class LidarLogger:
    """
    Record raw LiDAR packets with timestamps to a binary file plus a JSON metadata file.

    Binary layout (repeated):
        [float64 timestamp][uint32 length][length bytes of data]
    """

    HEADER = struct.Struct("dI")  # double (8 bytes) + unsigned int (4 bytes)

    def __init__(self, config_path: str | Path = "lidar/config_lidar.yaml"):
        self.cfg = load_lidar_config(config_path)
        self.rx = VelodyneReceiver(config_path)
        self.log_dir = Path(self.cfg.get("log_dir", "logs/lidar"))
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def _create_files(self) -> tuple[Path, Path, BinaryIO]:
        ts_str = time.strftime("%Y%m%d_%H%M%S")
        base = f"vlp16_{ts_str}"

        bin_path = self.log_dir / f"{base}.bin"
        meta_path = self.log_dir / f"{base}.json"

        meta = {
            "start_time": time.time(),
            "binary_file": bin_path.name,
            "lidar_config": self.cfg,
            "format": {
                "timestamp": "float64 seconds since epoch",
                "length": "uint32 number of bytes",
                "data": "raw UDP payload",
            },
        }

        with open(meta_path, "w") as f_meta:
            json.dump(meta, f_meta, indent=2)

        fbin: BinaryIO = open(bin_path, "wb")

        print(f"[LidarLogger] Logging to {bin_path}")
        print(f"[LidarLogger] Metadata  {meta_path}")

        return bin_path, meta_path, fbin

    def record(self, duration_sec: Optional[float] = None) -> None:
        """
        Start recording packets.

        Args:
            duration_sec: If provided, stops after this many seconds.
                          If None, runs until Ctrl+C.
        """
        self.rx.connect()
        bin_path, meta_path, fbin = self._create_files()

        pkt_struct = self.HEADER
        count = 0
        t0 = time.time()

        try:
            while True:
                if duration_sec is not None and (time.time() - t0) >= duration_sec:
                    print("[LidarLogger] Reached requested duration. Stopping.")
                    break

                pkt = self.rx.receive(timeout=5.0)
                if pkt is None:
                    print("[LidarLogger] No packets received in last 5 seconds.")
                    continue

                header = pkt_struct.pack(pkt.timestamp, len(pkt.data))
                fbin.write(header)
                fbin.write(pkt.data)

                count += 1
                if count % 500 == 0:
                    dt = time.time() - t0
                    rate = count / dt if dt > 0 else 0.0
                    print(f"[LidarLogger] {count} packets recorded ({rate:.1f} pkts/s)")
        except KeyboardInterrupt:
            print("[LidarLogger] Stopped by user.")
        finally:
            fbin.close()
            print(f"[LidarLogger] Final packet count: {count}")
            print(f"[LidarLogger] Binary file: {bin_path}")
            print(f"[LidarLogger] Metadata file: {meta_path}")


# ----------------------------------------------------------------------
# Replayer: read .bin and yield packets in real time
# ----------------------------------------------------------------------


class LidarReplayer:
    """Replay a recorded .bin file as a stream of LidarPacket objects."""

    HEADER = LidarLogger.HEADER

    def __init__(self, bin_path: str | Path):
        self.bin_path = Path(bin_path)

    def _packet_generator(self) -> Iterator[LidarPacket]:
        header_struct = self.HEADER

        with open(self.bin_path, "rb") as f:
            while True:
                header_bytes = f.read(header_struct.size)
                if not header_bytes or len(header_bytes) < header_struct.size:
                    break

                ts, length = header_struct.unpack(header_bytes)
                data = f.read(length)
                if len(data) < length:
                    break

                yield LidarPacket(timestamp=ts, data=data)

    def packets_realtime(self) -> Iterator[LidarPacket]:
        """
        Yield packets, sleeping so that the timing roughly matches the original recording.
        """
        gen = self._packet_generator()
        first = next(gen, None)
        if first is None:
            return

        t0_original = first.timestamp
        t0_wall = time.time()
        yield first

        for pkt in gen:
            dt_original = pkt.timestamp - t0_original
            target_wall = t0_wall + dt_original
            now = time.time()
            if target_wall > now:
                time.sleep(target_wall - now)
            yield pkt


# ----------------------------------------------------------------------
# Command-line interface
# ----------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description="LiDAR reader / logger / replayer")
    parser.add_argument(
        "--mode",
        choices=["listen", "record", "replay"],
        required=True,
        help="Operation mode",
    )
    parser.add_argument(
        "--config",
        default="lidar/config_lidar.yaml",
        help="Path to LiDAR config YAML",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Recording duration in seconds (record mode only)",
    )
    parser.add_argument(
        "--bin",
        help="Path to recorded .bin file (replay mode only)",
    )

    args = parser.parse_args()

    if args.mode == "listen":
        rx = VelodyneReceiver(args.config)
        rx.connect()
        count = 0
        t0 = time.time()
        try:
            while True:
                pkt = rx.receive(timeout=5.0)
                if pkt is None:
                    print("[listen] No packets in last 5 seconds.")
                    continue
                count += 1
                if count % 100 == 0:
                    dt = time.time() - t0
                    rate = count / dt if dt > 0 else 0.0
                    print(f"[listen] Received {count} packets ({rate:.1f} pkts/s)")
        except KeyboardInterrupt:
            print("[listen] Stopped by user.")

    elif args.mode == "record":
        logger = LidarLogger(args.config)
        logger.record(duration_sec=args.duration)

    elif args.mode == "replay":
        if not args.bin:
            raise SystemExit("You must supply --bin for replay mode.")
        rep = LidarReplayer(args.bin)
        count = 0
        t0 = time.time()
        try:
            for pkt in rep.packets_realtime():
                count += 1
                if count % 500 == 0:
                    dt = time.time() - t0
                    rate = count / dt if dt > 0 else 0.0
                    print(f"[replay] Replayed {count} packets ({rate:.1f} pkts/s)")
        except KeyboardInterrupt:
            print("[replay] Stopped by user.")


if __name__ == "__main__":
    main()
