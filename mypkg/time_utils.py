#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Motona Shigehisa
# SPDX-License-Identifier: BSD-3-Clause

from datetime import datetime, timezone
import math


def utc_to_jd(dt: datetime) -> float:
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)

    year = dt.year
    month = dt.month
    day = dt.day + (dt.hour + dt.minute / 60 + dt.second / 3600) / 24

    if month <= 2:
        year -= 1
        month += 12

    A = year // 100
    B = 2 - A + A // 4

    jd = int(365.25 * (year + 4716)) \
       + int(30.6001 * (month + 1)) \
       + day + B - 1524.5
    return jd


def jd_to_gmst(jd: float) -> float:
    T = (jd - 2451545.0) / 36525.0
    gmst = 6.697374558 \
         + 2400.051336 * T \
         + 0.000025862 * T * T
    return gmst % 24


def gmst_to_lst(gmst: float, longitude_deg: float) -> float:
    return (gmst + longitude_deg / 15.0) % 24


def hours_to_hms(hours: float) -> str:
    h = int(hours)
    m = int((hours - h) * 60)
    s = ((hours - h) * 60 - m) * 60
    return f"{h:02d}:{m:02d}:{s:05.2f}"

