#!/usr/bin/env python

import rospy
import bluetooth
from rosserial_bluetooth_manager.msg import BluetoothDevice
from rosserial_bluetooth_manager.msg import BluetoothDeviceArray


if __name__ == '__main__':
    rospy.init_node('discover_bluetooth_devices')
    discover_duration = rospy.get_param('~discover_duration', 30)
    publisher = rospy.Publisher(
        '/bluetooth_devices',
        BluetoothDeviceArray,
        queue_size=1,
        latch=True)
    while not rospy.is_shutdown():
        try:
            devices = bluetooth.discover_devices(
                duration=discover_duration,
                lookup_names=True,
                flush_cache=True)
        except bluetooth.btcommon.BluetoothError as e:
            rospy.logerr(e)
            continue
        msg = BluetoothDeviceArray()
        msg.array = [BluetoothDevice(name=name, address=address)
                     for (address, name) in devices]
        publisher.publish(msg)
        rospy.loginfo('published {} devices'.format(len(devices)))
