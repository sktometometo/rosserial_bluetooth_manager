import subprocess
import pexpect


class RFCOMMInterface(object):

    def get_current_rfcomm_ports(self):

        ret = subprocess.check_output('sudo rfcomm -a', shell=True)
        ans = {}
        for x in map(lambda x: x.split(' '), ret.decode('utf-8').split('\n')[:-1]):
            rfcomm_port = int(x[0].strip('rfcomm').strip(':'))
            bt_address = x[1]
            ans[rfcomm_port] = bt_address
        return ans

    def bind(self, rfcomm_port, bt_address):

        ret = subprocess.check_output(
                'sudo rfcomm bind {} {}'.format(rfcomm_port, bt_address),
                shell=True,
                stderr=subprocess.STDOUT
                )
        ret = ret.decode('utf-8')
        if 'Can\'t create device' in ret:
            return False, ret
        else:
            return True, ''

    def release(self, rfcomm_port):

        ret = subprocess.check_output(
                'sudo rfcomm release {}'.format(rfcomm_port),
                shell=True,
                stderr=subprocess.STDOUT
                )
        ret = ret.decode('utf-8')
        if 'Can\'t release device' in ret:
            return False, ret
        else:
            return True, ''


class BluetoothInterface(object):

    def __init__(self, debug=False, timeout=30):

        self.timeout = timeout
        self.debug = debug
        if self.debug:
            import sys
            self.logfile = sys.stdout
        else:
            self.logfile = None

    def remove_device(self, bt_address):

        if self.debug:
            child = pexpect.spawn(
                'sudo bluetoothctl',
                encoding='utf-8',
                echo=False
                )
            child.logfile = self.logfile
        else:
            child = pexpect.spawn(
                'sudo bluetoothctl',
                echo=False
                )
        child.sendline('')
        child.expect(['Agent registered', pexpect.TIMEOUT], timeout=1)
        child.expect(['#', pexpect.TIMEOUT], timeout=1)

        child.sendline('remove {}'.format(bt_address))
        ret = child.expect([
            'Device has been removed',
            'Device {} not available'.format(bt_address),
            pexpect.TIMEOUT
            ],
            timeout=self.timeout)

        if ret == 0:
            child.terminate()
            return True, 'Device has been removed'
        elif ret == 1:
            return False, 'Device {} not available'.format(bt_address)
        else:
            return False, 'Unknown Error'

    def pair_device(self, bt_address):

        if self.debug:
            child = pexpect.spawn(
                'bluetoothctl',
                encoding='utf-8',
                echo=False
                )
            child.logfile = self.logfile
        else:
            child = pexpect.spawn(
                'bluetoothctl',
                echo=False
                )
        child.sendline('')
        child.expect(['Agent registered', pexpect.TIMEOUT], timeout=1)
        child.expect(['#', pexpect.TIMEOUT], timeout=1)

        child.sendline('scan on')
        child.expect([pexpect.TIMEOUT], timeout=1)
        child.sendline('scan off')
        child.expect(['#', pexpect.TIMEOUT], timeout=1)
        child.expect(['#', pexpect.TIMEOUT], timeout=1)

        child.sendline('pair {}'.format(bt_address))
        ret_type_to_pairing_trial = child.expect([
            '#',
            'Attempting to pair with {}'.format(bt_address),
            'Device {} not available'.format(bt_address),
            ],
            timeout=self.timeout)
        if ret_type_to_pairing_trial == 0:
            print('ret_type_to_pairing_trial: {}'.format(ret_type_to_pairing_trial))
            child.terminate()
            return False, ''
        if ret_type_to_pairing_trial == 2:
            child.terminate()
            return False, 'Device {} not available'.format(bt_address)

        ret_type_to_after_trial = child.expect([
            'AlreadyExists',
            'Request confirmation',
            pexpect.TIMEOUT
            ],
            timeout=self.timeout)

        if ret_type_to_after_trial == 0:
            child.terminate()
            return False, 'org.bluez.Error.AlreadyExists'
        if ret_type_to_after_trial == 1:
            child.sendline('yes')
        elif ret_type_to_after_trial == 2:
            child.terminate()
            return False, 'Unknown Error'

        ret_pairing = child.expect([
            'Pairing successful',
            pexpect.TIMEOUT
            ],
            timeout=self.timeout)

        if ret_pairing == 0:
            child.terminate()
            return True, ''
        else:
            child.terminate()
            return False, 'Unknown Error'

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
