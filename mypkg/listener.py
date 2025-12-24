#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node("listener")


def cb(msg):
    node.get_logger().info("\n" + msg.data)


def main():
    node.create_subscription(String, "time", cb, 10)
    rclpy.spin(node)

