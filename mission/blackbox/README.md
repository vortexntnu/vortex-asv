## Blackbox

Logs data of all the important topics such as:
/asv/power_sense_module/current
/asv/power_sense_module/voltage
/asv/temperature/ESC1
/asv/temperature/ESC2
/asv/temperature/ESC3
/asv/temperature/ESC4
/asv/temperature/ambient1
/asv/temperature/ambient2
internal/status/bms0
internal/status/bms1
/thrust/thruster_forces
/pwm


## Service file boot up

To start the blackbox logging automatically every time on boot up just run this command:
```
./vortex-asv/add_service_files_to_bootup_sequence.sh
```

Enjoy :)
