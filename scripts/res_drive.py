#!/usr/bin/env python3
"""Drive rs_follow during the load measurement: bind a target, enable, hold."""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

rclpy.init()
n = Node("res_drive")
bp = n.create_publisher(PointStamped, "/rs_follow/bind_target", 10)
ep = n.create_publisher(Bool, "/rs_follow/enable", 10)
time.sleep(1)

p = PointStamped()
p.header.frame_id = "rslidar"
p.point.x, p.point.y = 3.0, 0.0
for _ in range(5):
    p.header.stamp = n.get_clock().now().to_msg()
    bp.publish(p)
    time.sleep(0.1)

b = Bool()
b.data = True
for _ in range(5):
    ep.publish(b)
    time.sleep(0.1)

time.sleep(150)

b.data = False
for _ in range(3):
    ep.publish(b)
    time.sleep(0.1)
rclpy.shutdown()
