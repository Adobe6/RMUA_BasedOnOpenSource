#!/usr/bin/env python3
import argparse
import math
from bisect import bisect_left

import rosbag
from sensor_msgs import point_cloud2


def nearest(items, stamp):
    if not items:
        return None
    times = [item[0] for item in items]
    index = bisect_left(times, stamp)
    candidates = []
    if index < len(items):
        candidates.append(items[index])
    if index > 0:
        candidates.append(items[index - 1])
    return min(candidates, key=lambda item: abs(item[0] - stamp))


def distance3(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def pointcloud_distance_stats(msg, reference, box):
    rx, ry, rz = reference
    min_distance = float("inf")
    counts = {0.8: 0, 1.0: 0, 1.5: 0, 2.0: 0}
    in_box = 0

    for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        if abs(x - rx) > box or abs(y - ry) > box or abs(z - rz) > box:
            continue

        in_box += 1
        distance = math.sqrt((x - rx) ** 2 + (y - ry) ** 2 + (z - rz) ** 2)
        min_distance = min(min_distance, distance)
        for threshold in counts:
            if distance < threshold:
                counts[threshold] += 1

    if min_distance == float("inf"):
        min_distance = 999.0
    return min_distance, counts, in_box


def read_bag(path):
    odom = []
    command = []
    inflated = []

    with rosbag.Bag(path) as bag:
        start_time = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(
            topics=[
                "/gps_corrected_odometry",
                "/position_command",
                "/drone_0_ego_planner_node/grid_map/occupancy_inflate",
            ]
        ):
            bag_time = stamp.to_sec() - start_time
            if topic == "/gps_corrected_odometry":
                position = msg.pose.pose.position
                velocity = msg.twist.twist.linear
                odom.append(
                    (
                        bag_time,
                        (
                            position.x,
                            position.y,
                            position.z,
                            velocity.x,
                            velocity.y,
                            velocity.z,
                        ),
                    )
                )
            elif topic == "/position_command":
                position = msg.position
                velocity = msg.velocity
                command.append(
                    (
                        bag_time,
                        (
                            position.x,
                            position.y,
                            position.z,
                            velocity.x,
                            velocity.y,
                            velocity.z,
                        ),
                    )
                )
            elif topic == "/drone_0_ego_planner_node/grid_map/occupancy_inflate":
                inflated.append((bag_time, msg))

    return odom, command, inflated


def main():
    parser = argparse.ArgumentParser(
        description="Analyze drone and command clearance to EGO inflated occupancy in a bag."
    )
    parser.add_argument("bag", help="Path to rosbag")
    parser.add_argument(
        "--match-start",
        type=float,
        default=0.0,
        help="Competition/simulator time corresponding to bag start, in seconds.",
    )
    parser.add_argument(
        "--from-match",
        dest="from_match",
        type=float,
        default=None,
        help="Start analysis at this competition/simulator time. Omit to scan from bag start.",
    )
    parser.add_argument(
        "--to-match",
        dest="to_match",
        type=float,
        default=None,
        help="End analysis at this competition/simulator time. Omit to scan through bag end.",
    )
    parser.add_argument("--step", type=float, default=0.5, help="Sample period in bag seconds.")
    parser.add_argument("--box", type=float, default=5.0, help="Local search box half-size in meters.")
    parser.add_argument(
        "--tracking-error-max",
        type=float,
        default=2.0,
        help="Maximum odom-to-command error used for the pre-collision danger ranking.",
    )
    args = parser.parse_args()

    odom, command, inflated = read_bag(args.bag)
    if not odom or not command or not inflated:
        raise SystemExit(
            "Bag must contain /gps_corrected_odometry, /position_command, and "
            "/drone_0_ego_planner_node/grid_map/occupancy_inflate."
        )

    first_time = inflated[0][0]
    last_time = inflated[-1][0]
    if args.from_match is not None:
        first_time = max(first_time, args.from_match - args.match_start)
    if args.to_match is not None:
        last_time = min(last_time, args.to_match - args.match_start)

    rows = []
    sample_time = first_time
    while sample_time <= last_time + 1e-6:
        inflated_msg = nearest(inflated, sample_time)
        odom_msg = nearest(odom, sample_time)
        command_msg = nearest(command, sample_time)
        if inflated_msg and odom_msg and command_msg:
            odom_position = odom_msg[1][:3]
            command_position = command_msg[1][:3]
            odom_min, odom_counts, _ = pointcloud_distance_stats(inflated_msg[1], odom_position, args.box)
            command_min, command_counts, _ = pointcloud_distance_stats(
                inflated_msg[1], command_position, args.box
            )
            command_error = distance3(odom_position, command_position)
            speed = math.sqrt(sum(component**2 for component in odom_msg[1][3:6]))
            rows.append(
                {
                    "bag_time": inflated_msg[0],
                    "match_time": args.match_start + inflated_msg[0],
                    "odom_min": odom_min,
                    "cmd_min": command_min,
                    "odom_counts": odom_counts,
                    "cmd_counts": command_counts,
                    "cmd_error": command_error,
                    "speed": speed,
                    "odom": odom_position,
                    "cmd": command_position,
                }
            )
        sample_time += args.step

    print("match_t,bag_t,odom_min,cmd_min,odom_cnt<1,cmd_cnt<1,cmd_err,speed,odom,cmd")
    for row in rows:
        print(
            f"{row['match_time']:.2f},{row['bag_time']:.2f},"
            f"{row['odom_min']:.2f},{row['cmd_min']:.2f},"
            f"{row['odom_counts'][1.0]},{row['cmd_counts'][1.0]},"
            f"{row['cmd_error']:.2f},{row['speed']:.2f},"
            f"({row['odom'][0]:.2f} {row['odom'][1]:.2f} {row['odom'][2]:.2f}),"
            f"({row['cmd'][0]:.2f} {row['cmd'][1]:.2f} {row['cmd'][2]:.2f})"
        )

    if rows:
        print("\nMost dangerous by odom_min:")
        for row in sorted(rows, key=lambda item: item["odom_min"])[:8]:
            print(
                f"match={row['match_time']:.2f}s bag={row['bag_time']:.2f}s "
                f"odom_min={row['odom_min']:.2f} cmd_min={row['cmd_min']:.2f} "
                f"cmd_err={row['cmd_error']:.2f} speed={row['speed']:.2f}"
            )

        print("\nMost dangerous by cmd_min:")
        for row in sorted(rows, key=lambda item: item["cmd_min"])[:8]:
            print(
                f"match={row['match_time']:.2f}s bag={row['bag_time']:.2f}s "
                f"cmd_min={row['cmd_min']:.2f} odom_min={row['odom_min']:.2f} "
                f"cmd_err={row['cmd_error']:.2f} speed={row['speed']:.2f}"
            )

        tracking_rows = [
            row for row in rows if row["cmd_error"] <= args.tracking_error_max
        ]
        if tracking_rows:
            print(
                "\nMost dangerous while tracking "
                f"(cmd_err <= {args.tracking_error_max:.2f}m), by odom_min:"
            )
            for row in sorted(tracking_rows, key=lambda item: item["odom_min"])[:8]:
                print(
                    f"match={row['match_time']:.2f}s bag={row['bag_time']:.2f}s "
                    f"odom_min={row['odom_min']:.2f} cmd_min={row['cmd_min']:.2f} "
                    f"cmd_err={row['cmd_error']:.2f} speed={row['speed']:.2f}"
                )

            print(
                "\nMost dangerous while tracking "
                f"(cmd_err <= {args.tracking_error_max:.2f}m), by cmd_min:"
            )
            for row in sorted(tracking_rows, key=lambda item: item["cmd_min"])[:8]:
                print(
                    f"match={row['match_time']:.2f}s bag={row['bag_time']:.2f}s "
                    f"cmd_min={row['cmd_min']:.2f} odom_min={row['odom_min']:.2f} "
                    f"cmd_err={row['cmd_error']:.2f} speed={row['speed']:.2f}"
                )


if __name__ == "__main__":
    main()
