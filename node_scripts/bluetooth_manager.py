#!/usr/bin/env python

import rospy
import subprocess
import pexpect
from rosserial_bluetooth_manager.msg import BluetoothDevice
from rosserial_bluetooth_manager.msg import BluetoothDeviceArray


class BluetoothInterface(object):

    def __init__(self):

        self.child = pexpect.spawn('bluetoothctl', echo=False)
        self.send_and_get('\n')
        self.bt_addrs = {}

    def send_and_get(self, string):

        self.child.send(string + '\n')
        self.child.expect(['bluetooth', pexpect.EOF])
        return self.child.before.split('\r\n')

    def connect_devices(self, bt_addr, rfcomm_port):

        self.child.send('pair {}\n'.format(bt_addr))
        self.child.send('yes\n')
        self.child.expect(['#', '[bluetooth]# ', pexpect.EOF])

        self.child.send('trust {}\n'.format(bt_addr))
        self.child.expect(['#', '[bluetooth]# ', pexpect.EOF])

        self.child.send('trust {}\n'.format(bt_addr))
        self.child.expect(['#', '[bluetooth]# ', pexpect.EOF])

        subprocess.run("sudo rfcomm bind {} {}".format(rfcomm_port, bt_addr), shell=True)
        subprocess.run("sudo stty -F /dev/rfcomm{} 57600 cs8".format(rfcomm_port), shell=True)

    def disconnect_devices(self, bt_addr, rfcomm_port):

        subprocess.run("sudo rfcomm release {}".format(rfcomm_port), shell=True)

        self.child.send('remove {}\n'.format(bt_addr))
        self.child.expect(['#', '[bluetooth]# ', pexpect.EOF])

    def scan_devices(self):

        command_result = subprocess.Popen(
                'hcitool scan',
                stdout=subprocess.PIPE,
                shell=True).communicate()
        out_str = command_result[0].decode('utf-8')
        lines = out_str.split('\n')[1:-1]
        result = []
        for line in lines:
            bt_addr = line.split('\t')[1]
            bt_name = line.split('\t')[2]
            result.append([bt_addr, bt_name])

        return result


if __name__ == '__main__':
    rospy.init_node('discover_bluetooth_devices')
    discover_duration = rospy.get_param('~discover_duration', 10)
    interface = BluetoothInterface()
    publisher = rospy.Publisher(
        '/bluetooth_devices',
        BluetoothDeviceArray,
        queue_size=1,
        latch=True)
    rospy.loginfo('Initialized')
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        rate.sleep()
        devices = interface.scan_devices()
        msg = BluetoothDeviceArray()
        msg.array = [BluetoothDevice(name=name, address=address)
                     for (address, name) in devices]
        publisher.publish(msg)
        rospy.loginfo('published {} devices'.format(len(devices)))
