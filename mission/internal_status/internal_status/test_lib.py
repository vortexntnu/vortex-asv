import power_sense_module_lib

bm = power_sense_module_lib.BatteryMonitor()


print(bm.get_current())
print(bm.get_voltage())