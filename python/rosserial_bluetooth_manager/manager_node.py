import threading
import subprocess

import rospy
from rosserial_bluetooth_manager.msg import BluetoothDevice
from rosserial_bluetooth_manager.msg import BluetoothDeviceArray
from rosserial_bluetooth_manager.msg import ConnectedDevice
from rosserial_bluetooth_manager.msg import ConnectedDeviceArray
from rosserial_bluetooth_manager.srv import ConnectDevice
from rosserial_bluetooth_manager.srv import ConnectDeviceResponse
from rosserial_bluetooth_manager.srv import DisconnectDevice
from rosserial_bluetooth_manager.srv import DisconnectDeviceResponse

from rosserial_bluetooth_manager import RFCOMMInterface
from rosserial_bluetooth_manager import BluetoothInterface


class RosserialBluetoothManager(object):

    def __init__(self):

        self.bt_interface = BluetoothInterface()
        self.lock_for_bt_interface = threading.Lock()
        self.rfcomm_interface = RFCOMMInterface()
        self.lock_for_rfcomm_interface = threading.Lock()

        # bt_address -> { 'rfcomm_port': rfcomm_port, 'serial_node': process }
        self.managed_devices = {}
        self.lock_for_managed_devices = threading.Lock()

        self.srv_connect_device = rospy.Service(
                '~connect_device',
                ConnectDevice,
                self.handler_connect_device)
        self.srv_disconnect_device = rospy.Service(
                '~disconnect_device',
                DisconnectDevice,
                self.handler_disconnect_device)
        self.publisher_devices = rospy.Publisher(
                '~bluetooth_devices',
                BluetoothDeviceArray,
                queue_size=1)
        self.publisher_managed = rospy.Publisher(
                '~managed_devices',
                ConnectedDeviceArray,
                queue_size=1)

    def _get_unused_index(self):

        used_rfcomm_ports = self.rfcomm_interface.get_current_rfcomm_ports().keys()
        for i in range(32):
            if i not in used_rfcomm_ports:
                return i
        return None

    def _launch_serial_node(self, bt_address, rfcomm_port):

        rospy.loginfo('launching serial node for {}'.format(bt_address))
        p = subprocess.Popen(
                ['rosrun',
                    'rosserial_python',
                    'serial_node.py',
                    '_port:=/dev/rfcomm{}'.format(rfcomm_port),
                    '_baud:=57600',
                    '__name:=serial_node_for_{}'.format(bt_address.replace(':','_'))]
                )
        return p, 'serial_node_for_{}'.format(bt_address.replace(':','_'))

    def _connect_device(self, bt_address):

        with self.lock_for_bt_interface:
            success, message = self.bt_interface.pair_device(bt_address)
            if not success:
                self.bt_interface.remove_device(bt_address)
                return success, message
            rospy.loginfo('Connect to {}'.format(bt_address))

        with self.lock_for_rfcomm_interface:
            target_port = self._get_unused_index()
            if target_port is None:
                self.bt_interface.remove_device(bt_address)
                return False, 'No rfcomm port available'
            success, message = self.rfcomm_interface.bind(target_port, bt_address)
            if not success:
                self.bt_interface.remove_device(bt_address)
                self.rfcomm_interface.release(target_port)
                return success, message
            rospy.loginfo('Bind to {}'.format(bt_address))

        with self.lock_for_managed_devices:
            p, node_name = self._launch_serial_node(bt_address, target_port)
            self.managed_devices[bt_address] = {
                    'rfcomm_port': target_port,
                    'serial_node': p,
                    'node_name': node_name}

        return True, ''

    def _disconnect_device(self, bt_address):

        with self.lock_for_managed_devices:
            self.managed_devices[bt_address]['serial_node'].terminate()
            target_port = self.managed_devices[bt_address]['rfcomm_port']
            self.managed_devices.pop(bt_address)

        with self.lock_for_rfcomm_interface:
            success, message = self.rfcomm_interface.release(target_port)

        with self.lock_for_bt_interface:
            success, message = self.bt_interface.remove_device(bt_address)

        return True, ''

    def handler_connect_device(self, req):

        rospy.loginfo('Called')
        with self.lock_for_managed_devices:
            if req.address in self.managed_devices.keys():
                res = ConnectDeviceResponse()
                res.success = False
                res.message = 'Device {} is already connected.'.format(req.address)
                return res

        success, message = self._connect_device(req.address)

        res = ConnectDeviceResponse()
        res.success = success
        res.message = message
        return res

    def handler_disconnect_device(self, req):

        rospy.loginfo('Called')
        with self.lock_for_managed_devices:
            if req.address not in self.managed_devices.keys():
                res = DisconnectDeviceResponse()
                res.success = False
                res.message = 'Device {} is not connected.'.format(req.address)
                return res

        success, message = self._disconnect_device(req.address)

        res = DisconnectDeviceResponse()
        res.success = success
        res.message = message

        return res

    def spin(self, hz=0.1):

        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                rospy.loginfo('loop started.')
                devices = self.bt_interface.scan_devices()
                msg = BluetoothDeviceArray()
                msg.array = [BluetoothDevice(name=name, address=address)
                             for (address, name) in devices]
                self.publisher_devices.publish(msg)
                with self.lock_for_managed_devices:
                    msg = ConnectedDeviceArray()
                    msg.array = [ConnectedDevice(
                                    address=k,
                                    rfcomm_port=v['rfcomm_port'],
                                    node_name=v['node_name'],
                                    )
                                 for k, v in self.managed_devices.items()]
                    self.publisher_managed.publish(msg)
                rospy.loginfo('loop finished.')
            except UnicodeDecodeError as e:
                rospy.logerr('Error: {}'.format(e))
