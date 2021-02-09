#!/usr/bin/env python3

import rospy
from .main import setup, loop


CONTROL_RATE = 10  # Hz


def main():
    rospy.init_node("robot_controller")
    rospy.loginfo(f"Node iniciado de controle{rospy.get_time()}")
    rate = rospy.Rate(CONTROL_RATE)

    setup()

    while not rospy.is_shutdown():
        loop()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
