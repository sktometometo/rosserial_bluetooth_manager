import subprocess
import threading

import pexpect

import rospy

from rosserial_bluetooth_manager.msg import BluetoothDevice
from rosserial_bluetooth_manager.msg import BluetoothDeviceArray
from rosserial_bluetooth_manager.msg import ConnectedDevice
from rosserial_bluetooth_manager.msg import ConnectedDeviceArray
from rosserial_bluetooth_manager.srv import ConnectDevice
from rosserial_bluetooth_manager.srv import ConnectDeviceResponse
from rosserial_bluetooth_manager.srv import DisconnectDevice
from rosserial_bluetooth_manager.srv import DisconnectDeviceResponse


class BluetoothInterface(object):

    def __init__(self):

        self.child = pexpect.spawn('bluetoothctl', echo=False)
        self.send_and_get('\n')

    def send_and_get(self, string):

        self.child.send(string + '\n')
        self.child.expect(['#', '[bluetooth]# ', pexpect.EOF])
        return self.child.before.split('\r\n')

    def connect_device(self, bt_addr, rfcomm_port):

        self.child.send('pair {}\n'.format(bt_addr))
        self.child.send('yes\n')
        self.child.expect(['#', '[bluetooth]# ', pexpect.EOF])

        self.child.send('trust {}\n'.format(bt_addr))
        self.child.expect(['#', '[bluetooth]# ', pexpect.EOF])

        self.child.send('trust {}\n'.format(bt_addr))
        self.child.expect(['#', '[bluetooth]# ', pexpect.EOF])

        subprocess.call("sudo rfcomm bind {} {}".format(rfcomm_port, bt_addr), shell=True)
        subprocess.call("sudo stty -F /dev/rfcomm{} 57600 cs8".format(rfcomm_port), shell=True)

    def disconnect_device(self, bt_addr, rfcomm_port):

        subprocess.call("sudo rfcomm release {}".format(rfcomm_port), shell=True)

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


class RosserialBluetoothManager(object):

    def __init__(self):

        self.interface = BluetoothInterface()

        # dict from bt_addr to rfcomm port
        self.connected_devices = {}
        self.lock = threading.Lock()

        self.srv_connect_device = rospy.Service(
                '~connect_device',
                ConnectDevice,
                self.handler_connect_device)
        self.srv_disconnect_device = rospy.Service(
                '~disconnect_device',
                DisconnectDevice,
                self.handler_disconnect_device)
        self.publisher = rospy.Publisher(
                '/bluetooth_devices',
                BluetoothDeviceArray,
                queue_size=1)

    def _get_unused_index(self):

        rfcomm_ports = [x['rfcomm_port'] for x in self.connected_devices.values()]
        for i in range(128):
            if i not in rfcomm_ports:
                return i
        return None

    def _launch_serial_node(self, bt_addr):

        rospy.loginfo('launching serial node for {}'.format(bt_addr))
        rfcomm_port = self.connected_devices[bt_addr]['rfcomm_port']
        p = subprocess.Popen(
                ['rosrun',
                    'rosserial_python',
                    'serial_node.py',
                    '_port:=/dev/rfcomm{}'.format(rfcomm_port),
                    '_baud:=57600',
                    '__name:=serial_node_for_{}'.format(bt_addr.replace(':','_'))]
                )
        self.connected_devices[bt_addr]['serial_node'] = p

    def _kill_serial_node(self, bt_addr):

        rospy.loginfo('killing serial node for {}'.format(bt_addr))
        self.connected_devices[bt_addr]['serial_node'].terminate()
        self.connected_devices[bt_addr]['serial_node'] = None

    def _publish_connected_devices(self):

        msg = ConnectedDeviceArray()

    def handler_connect_device(self, req):

        with self.lock:
            if req.address in self.connected_devices.keys():
                res = ConnectDeviceResponse()
                res.success = False
                res.message = 'Device {} is already connected.'.format(req.address)
                return res

            index = self._get_unused_index()
            self.interface.connect_device(req.address, index)
            self.connected_devices[req.address] = {'rfcomm_port': index,
                                                   'serial_node': None}
            self._launch_serial_node(req.address)

            res = ConnectDeviceResponse()
            res.success = True
            return res

    def handler_disconnect_device(self, req):

        with self.lock:
            if req.address not in self.connected_devices.keys():
                res = DisconnectDeviceResponse()
                res.success = False
                res.message = 'Device {} is not connected.'.format(req.address)
                return res

            self.interface.disconnect_device(req.address, self.connected_devices[req.address]['rfcomm_port'])
            self._kill_serial_node(req.address)
            self.connected_devices.pop(req.address)

            res = DisconnectDeviceResponse()
            res.success = True
            return res

    def spin(self, hz=0.1):

        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                devices = self.interface.scan_devices()
                msg = BluetoothDeviceArray()
                msg.array = [BluetoothDevice(name=name, address=address)
                             for (address, name) in devices]
                self.publisher.publish(msg)
                rospy.loginfo('published {} devices'.format(len(devices)))
            except UnicodeDecodeError as e:
                rospy.loginfo('Error: {}'.format(e))
