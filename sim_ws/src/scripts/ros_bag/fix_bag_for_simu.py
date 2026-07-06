#!/usr/bin/env python3

"""
Clean a ROS2 bag by removing map->odom TFs and fixing relative timestamps.

Usage:
    python3 fix_bag_time.py <input_bag> [output_bag]

If output_bag is not specified, it defaults to <input_bag>_clean.
"""

import argparse
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import get_typestore, Stores


EPOCH_THRESHOLD = 1_000_000_000
TF_TOPICS = {"/tf", "/tf_static"}
RESTAMP_TOPICS = {
    "/tf": "tf2_msgs/msg/TFMessage",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/imu/raw": "sensor_msgs/msg/Imu",
    "/odometry/filtered": "nav_msgs/msg/Odometry",
}


def is_map_to_odom(transform):
    return transform.header.frame_id == "map" and transform.child_frame_id == "odom"


def fix_stamp(stamp, offset_ns: int):
    stamp_ns = stamp.sec * 10**9 + stamp.nanosec
    if stamp.sec < EPOCH_THRESHOLD:
        stamp_ns += offset_ns
        stamp.sec = int(stamp_ns // 10**9)
        stamp.nanosec = int(stamp_ns % 10**9)
    return stamp


def restamp_any(value, offset_ns: int):
    restamped = 0

    if hasattr(value, "sec") and hasattr(value, "nanosec"):
        if value.sec < EPOCH_THRESHOLD:
            fix_stamp(value, offset_ns)
            restamped += 1
        return restamped

    if isinstance(value, list):
        for item in value:
            restamped += restamp_any(item, offset_ns)
        return restamped

    if isinstance(value, tuple):
        for item in value:
            restamped += restamp_any(item, offset_ns)
        return restamped

    if hasattr(value, "__dict__"):
        for key, item in vars(value).items():
            if key.startswith("_"):
                continue
            restamped += restamp_any(item, offset_ns)

    return restamped


def get_output_path(input_path: str, output_path: str | None):
    if output_path:
        return output_path.rstrip("/")
    return input_path.rstrip("/") + "_clean"


def clean_bag(input_path: str, output_path: str):
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # --- Pass 1: find offset ---
    min_relative_ns = None
    with Reader(input_path) as reader:
        bag_start_ns = reader.start_time
        for conn, timestamp, rawdata in reader.messages():
            if conn.topic in TF_TOPICS or conn.topic in RESTAMP_TOPICS:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                min_relative_ns = scan_for_earliest_relative_stamp(msg, min_relative_ns)

    if min_relative_ns is None:
        raise RuntimeError("No relative stamps found to anchor bag time")

    offset_ns = bag_start_ns - min_relative_ns
    print(f"Earliest relative stamp: {min_relative_ns / 1e9:.3f}s")
    print(f"Applying offset: {offset_ns / 1e9:.3f}s")

    # --- Pass 2: rewrite ---
    with Reader(input_path) as reader, Writer(output_path, version=8) as writer:
        conn_map = {}
        for conn in reader.connections:
            conn_map[conn.id] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                msgdef=conn.msgdef.data or conn.msgtype,
                rihs01=conn.digest or "0" * 32,
                serialization_format=conn.ext.serialization_format,
                offered_qos_profiles=conn.ext.offered_qos_profiles,
            )

        dropped_tf_transforms = 0
        restamped_stamps = 0

        for conn, timestamp, rawdata in reader.messages():
            if conn.topic in TF_TOPICS:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)

                filtered_transforms = [
                    transform for transform in msg.transforms if not is_map_to_odom(transform)
                ]
                dropped_tf_transforms += len(msg.transforms) - len(filtered_transforms)

                if filtered_transforms:
                    msg.transforms = filtered_transforms
                    restamped_stamps += restamp_any(msg, offset_ns)
                    rawdata = typestore.serialize_cdr(msg, conn.msgtype)
                else:
                    continue
            elif conn.topic in RESTAMP_TOPICS:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                restamped_stamps += restamp_any(msg, offset_ns)
                rawdata = typestore.serialize_cdr(msg, conn.msgtype)

            writer.write(conn_map[conn.id], timestamp, rawdata)

    print(f"Removed map->odom transforms: {dropped_tf_transforms}")
    print(f"Restamped stamps            : {restamped_stamps}")
    print(f"Done -> play {output_path}")


def scan_for_earliest_relative_stamp(value, current_min_ns):
    if hasattr(value, "sec") and hasattr(value, "nanosec"):
        if value.sec < EPOCH_THRESHOLD:
            stamp_ns = value.sec * 10**9 + value.nanosec
            if current_min_ns is None or stamp_ns < current_min_ns:
                return stamp_ns
        return current_min_ns

    if isinstance(value, list) or isinstance(value, tuple):
        for item in value:
            current_min_ns = scan_for_earliest_relative_stamp(item, current_min_ns)
        return current_min_ns

    if hasattr(value, "__dict__"):
        for key, item in vars(value).items():
            if key.startswith("_"):
                continue
            current_min_ns = scan_for_earliest_relative_stamp(item, current_min_ns)

    return current_min_ns


def main():
    parser = argparse.ArgumentParser(
        description="Clean a ROS2 bag by removing map->odom TFs and fixing timestamps."
    )
    parser.add_argument("input_bag", help="Path to the input .db3 bag directory")
    parser.add_argument(
        "output_bag",
        nargs="?",
        help="Path for the output bag (default: <input>_clean)",
    )
    args = parser.parse_args()

    input_path = args.input_bag.rstrip("/")
    output_path = get_output_path(input_path, args.output_bag)

    if not Path(input_path).exists():
        print(f"Error: input bag not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    if Path(output_path).exists():
        print(
            f"Error: output path already exists: {output_path}\n"
            "Remove it or choose a different name.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Input  : {input_path}")
    print(f"Output : {output_path}")
    print("Processing...")

    clean_bag(input_path, output_path)


if __name__ == "__main__":
    main()
