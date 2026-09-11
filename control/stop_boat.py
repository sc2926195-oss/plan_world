#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键停船：把左右推进器推力清零（多次发送，确保生效）。

用法:  python3 control/stop_boat.py
"""
import subprocess
import time

for _ in range(5):
    for side in ("left", "right"):
        subprocess.Popen(["gz", "topic", "-t", f"/wamv/thrusters/{side}/thrust",
                          "-m", "gz.msgs.Double", "-p", "data: 0"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.15)
print("已发送 L=0 R=0 (5次)")
