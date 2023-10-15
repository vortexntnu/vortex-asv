import freya_bms
# import usb

import re
import subprocess

device_re = re.compile(b"Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
df = subprocess.check_output("lsusb")
devices = []
for i in df.split(b'\n'):
    if i:
        info = device_re.match(i)
        if info:
            dinfo = info.groupdict()
            dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
            devices.append(dinfo)

for device in devices:
    print(device)

# print(devices)

test = freya_bms.BMS("ttyUSB0")
test.parse_bms_data(test.get_bms_data())

print(test._cells)

string = "hei                på"
print(string.split())




# [17908.264303] usb 1-1.1: USB disconnect, device number 6
# [17908.265921] ftdi_sio ttyUSB0: FTDI USB Serial Device converter now disconnected from ttyUSB0
# [17908.266006] ftdi_sio 1-1.1:1.0: device disconnected