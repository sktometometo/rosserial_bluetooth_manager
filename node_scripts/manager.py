#!/usr/bin/env python

import rospy
from rosserial_bluetooth_manager.manager_node import RosserialBluetoothManager


if __name__ == '__main__':
    rospy.init_node('discover_bluetooth_devices')
    node = RosserialBluetoothManager()
    rospy.loginfo('Initialized')
    node.spin()
