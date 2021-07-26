#! /usr/bin/env python
import rospy
from swarm_utils.utils_api import utils_api

rospy.init_node('takeoff')

if __name__ == '__main__':
    try:
        drone = utils_api()
        drone.set_mode('GUIDED')
        drone.takeoff(3)
        rospy.sleep(10)
        drone.land()
    except rospy.ROSInterruptException:
        pass
