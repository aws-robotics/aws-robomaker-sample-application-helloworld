#!/usr/bin/env python
"""
 Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.

 Permission is hereby granted, free of charge, to any person obtaining a copy of this
 software and associated documentation files (the "Software"), to deal in the Software
 without restriction, including without limitation the rights to use, copy, modify,
 merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist

class Rotator(Node):
    def __init__(self):
        super().__init__('rotate')
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel')

    def rotate_forever(self):
        twist = Twist()
        while rclpy.ok():
            twist.angular.z = 0.1
            self._cmd_pub.publish(twist)
            self.get_logger().info('Rotating robot: {}'.format(twist))
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    rotator = Rotator()
    rotator.rotate_forever()

if __name__ == '__main__':
    main()
