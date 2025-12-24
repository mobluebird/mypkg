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

pub_utc = node.create_publisher(String, "utc_time", 10)
pub_jd = node.create_publisher(String, "julian_day", 10)
pub_gmst = node.create_publisher(String, "gmst", 10)
pub_lst = node.create_publisher(String, "lst", 10)

TOKYO_LONGITUDE = 139.6917


def cb():
    now_utc = datetime.now(timezone.utc)
    jd = utc_to_jd(now_utc)
    gmst = jd_to_gmst(jd)
    lst = gmst_to_lst(gmst, TOKYO_LONGITUDE)

    msg_utc = String()
    msg_utc.data = now_utc.isoformat()
    pub_utc.publish(msg_utc)

    msg_jd = String()
    msg_jd.data = f"{jd:.5f}"
    pub_jd.publish(msg_jd)

    msg_gmst = String()
    msg_gmst.data = hours_to_hms(gmst)
    pub_gmst.publish(msg_gmst)

    msg_lst = String()
    msg_lst.data = hours_to_hms(lst)
    pub_lst.publish(msg_lst)


def main():
    node.create_timer(1.0, cb)
    rclpy.spin(node)


