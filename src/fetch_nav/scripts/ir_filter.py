#!/usr/bin/env python
"""Removes the RealSense projector's dot pattern from an infrared image for
RTAB-Map's visual features: a 3x3 median blur drops the 1-pixel dots and
keeps edges and print.  image (mono8) -> image_filtered (mono8, same header)

  rosrun fetch_nav ir_filter.py image:=/cam_front/infra1/image_rect_raw \
      image_filtered:=/cam_front/infra1/image_filtered
"""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


def main():
    rospy.init_node('ir_filter')
    ksize = rospy.get_param('~ksize', 3)
    pub = rospy.Publisher('image_filtered', Image, queue_size=2)

    def on_image(msg):
        if msg.encoding != 'mono8':
            rospy.logwarn_throttle(5.0, 'ir_filter: expected mono8, got %s', msg.encoding)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        out = Image(header=msg.header, height=msg.height, width=msg.width, encoding='mono8',
                    is_bigendian=0, step=msg.width)
        out.data = cv2.medianBlur(np.ascontiguousarray(img), ksize).tobytes()
        pub.publish(out)

    rospy.Subscriber('image', Image, on_image, queue_size=2, buff_size=2 ** 22)
    rospy.spin()


if __name__ == '__main__':
    main()
