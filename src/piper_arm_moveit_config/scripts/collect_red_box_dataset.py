#!/usr/bin/env python3
"""Capture Gazebo RGB images and YOLO labels for the red target box.

Run while gazebo_moveit.launch.py is active. The red threshold is used only to
label training images; runtime object detection uses the trained YOLOv8 model.
"""

import argparse
import random
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def set_box_pose(x: float, y: float, z: float) -> None:
    request = (
        f'name: "red_target_box" position {{ x: {x} y: {y} z: {z} }} '
        'orientation { w: 1 }'
    )
    result = subprocess.run(
        [
            'gz', 'service', '-s', '/world/piper/set_pose',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', request,
        ],
        capture_output=True, text=True, timeout=5, check=True,
    )
    if 'data: true' not in result.stdout:
        raise RuntimeError(f'Gazebo rejected box pose: {result.stdout} {result.stderr}')


def red_box(image: np.ndarray):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(hsv, (0, 100, 65), (12, 255, 255))
    high = cv2.inRange(hsv, (170, 100, 65), (179, 255, 255))
    mask = cv2.bitwise_or(low, high)
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count < 2:
        return None
    best = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    x, y, w, h, area = map(int, stats[best])
    if area < 250 or w < 12 or h < 12:
        return None
    return x, y, w, h


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--positive', type=int, default=140)
    parser.add_argument('--negative', type=int, default=20)
    parser.add_argument('--seed', type=int, default=14)
    args = parser.parse_args()
    rng = random.Random(args.seed)
    rclpy.init()
    node = rclpy.create_node('red_box_dataset_capture')
    newest = None
    stamp = None

    def on_image(msg: Image) -> None:
        nonlocal newest, stamp
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        raw = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        newest = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else raw.copy()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    node.create_subscription(Image, '/left_camera/rgbd/image', on_image,
                             qos_profile_sensor_data)

    def next_image(previous_stamp):
        deadline = time.monotonic() + 5.0
        image = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if newest is not None and stamp is not None and (
                previous_stamp is None or stamp > previous_stamp + 0.13
            ):
                image = newest.copy()
                break
        if image is None:
            raise TimeoutError('No fresh left camera RGB image after moving target')
        return image, stamp

    for split in ('train', 'val'):
        (args.output / 'images' / split).mkdir(parents=True, exist_ok=True)
        (args.output / 'labels' / split).mkdir(parents=True, exist_ok=True)
    (args.output / 'dataset.yaml').write_text(
        f'path: {args.output.resolve()}\ntrain: images/train\nval: images/val\n'
        'names:\n  0: red_target_box\n', encoding='utf-8',
    )

    positives = len(list((args.output / 'images' / 'train').glob('red_*.jpg')))
    positives += len(list((args.output / 'images' / 'val').glob('red_*.jpg')))
    negatives = len(list((args.output / 'images' / 'train').glob('empty_*.jpg')))
    negatives += len(list((args.output / 'images' / 'val').glob('empty_*.jpg')))
    # Advance the deterministic pose sequence past samples already captured.
    for _ in range(positives):
        rng.uniform(0.52, 0.90)
        rng.uniform(0.14, 0.44)
        rng.uniform(0.055, 0.155)
    attempts = 0
    try:
        _, previous = next_image(None)
        while positives < args.positive and attempts < args.positive * 4:
            attempts += 1
            set_box_pose(rng.uniform(0.52, 0.90), rng.uniform(0.14, 0.44),
                         rng.uniform(0.055, 0.155))
            image, previous = next_image(previous)
            box = red_box(image)
            if box is None:
                continue
            x, y, w, h = box
            # Most training frames should contain a complete target face.
            if x < 2 or y < 2 or x + w > image.shape[1] - 2 or y + h > image.shape[0] - 2:
                continue
            split = 'val' if positives % 5 == 0 else 'train'
            name = f'red_{positives:04d}'
            cv2.imwrite(str(args.output / 'images' / split / f'{name}.jpg'), image)
            height, width = image.shape[:2]
            label = f'0 {(x + w / 2) / width:.6f} {(y + h / 2) / height:.6f} '
            label += f'{w / width:.6f} {h / height:.6f}\n'
            (args.output / 'labels' / split / f'{name}.txt').write_text(label)
            positives += 1
            if positives % 20 == 0:
                print(f'positive {positives}/{args.positive}', flush=True)

        set_box_pose(0.75, 1.3, 0.10)
        while negatives < args.negative:
            image, previous = next_image(previous)
            if red_box(image) is not None:
                continue
            split = 'val' if negatives % 5 == 0 else 'train'
            name = f'empty_{negatives:04d}'
            cv2.imwrite(str(args.output / 'images' / split / f'{name}.jpg'), image)
            (args.output / 'labels' / split / f'{name}.txt').write_text('')
            negatives += 1
        print(f'Captured {positives} positives, {negatives} negatives in {args.output}',
              flush=True)
        if positives < args.positive:
            raise RuntimeError('Too few visible target images; adjust pose sampling range')
    finally:
        set_box_pose(0.65, 0.29, 0.10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
