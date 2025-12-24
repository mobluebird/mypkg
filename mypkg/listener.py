#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('listener')

utc = ""
jd = ""
gmst = ""
lst = ""


def cb_utc(msg):
    global utc
    utc = msg.data


def cb_jd(msg):
    global jd
    jd = msg.data


def cb_gmst(msg):
    global gmst
    gmst = msg.data


def cb_lst(msg):
    global lst
    lst = msg.data


node.create_subscription(String, 'utc_time', cb_utc, 10)
node.create_subscription(String, 'julian_day', cb_jd, 10)
node.create_subscription(String, 'gmst', cb_gmst, 10)
node.create_subscription(String, 'lst', cb_lst, 10)


def print_time():
    if utc and jd and gmst and lst:
        node.get_logger().info(
            f"\nUTC : {utc}\n"
            f"JD  : {jd}\n"
            f"GMST: {gmst}\n"
            f"LST (Tokyo): {lst}"
        )


node.create_timer(1.0, print_time)
rclpy.spin(node)

