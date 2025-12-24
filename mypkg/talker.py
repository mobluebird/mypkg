#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timezone
from mypkg.time_utils import (
    utc_to_jd,
    jd_to_gmst,
    gmst_to_lst,
    hours_to_hms
)

rclpy.init()
node = Node("talker")
pub = node.create_publisher(String, "time", 10)

TOKYO_LONGITUDE = 139.6917


def cb():
    now_utc = datetime.now(timezone.utc)

    jd = utc_to_jd(now_utc)
    gmst = jd_to_gmst(jd)
    lst = gmst_to_lst(gmst, TOKYO_LONGITUDE)

    msg = String()
    msg.data = (
        f"UTC : {now_utc.isoformat()}\n"
        f"JD  : {jd:.5f}\n"
        f"GMST: {hours_to_hms(gmst)}\n"
        f"LST (Tokyo): {hours_to_hms(lst)}"
    )

    pub.publish(msg)


def main():
    node.create_timer(1.0, cb)
    rclpy.spin(node)

