import subprocess

class BMS:
    """ Class containing Freya's BMS system. 

    Note: 
    -----------
    If no USB directory is passed in the constructor, the program queries all active 
    ports to find out where battery pack is connected. Only pass in a USB directory 
    if you are completely sure that it will be correct.

    Attributes:
    -----------
        voltage (float): Voltage being delivered by the BMS
        current (float): Current being delivered by the BMS
        design_capacity (float): Capacity of the BMS
        remaining_capacity (float): Remaining capacity of the BMS
        percent_capacity (float): Remaining capacity as a float from 0-1
        cycle_count (int): idk
        probes (int): idk
        strings (int): idk
        temps (Tuple[float, float, float]): Temperatures of the cell arrays
        cells (Tuple[float, ...]): Voltages of individual cells (6 elements long)
        balance (str): idk
        cell_total (float): Sum of cell voltages
        cell_min (float): Smallest cell voltage
        cell_max (float): Largest cell voltage
        cell_diff (float): Difference between largest and smallest cell voltage
        cell_avg (float): Average of cell voltages
        device_name (str): Device name
        manufacture_date (str): Date of manufacturing
        version (str): Version
        FET (str): idk

    Methods:
    --------
        get_bms_data() -> str | None:
            returns pure BMS data string, or None if exception is thrown 
        change_usb_port(usb_port: str) -> None:
            changes the usb port for the BMS
    """

    def __init__(self, usb_port: str = None) -> None:
        """
            Parameters:
                usb_port (str): USB port to connect to, E.G. 'ttyUSB0'. If none is 
                supplied, the program automatically tries to find the correct USB 
                port 

            Returns:
                None

            Note: Private members are denoted by _variable_name            
        """     
        if usb_port:
            self._usb_port = usb_port
            self._command = ["jbdtool", "-t", f"serial:/dev/{self._usb_port}"]
        else:
            print("Looking for USB devices...")
            devices = subprocess.check_output(["ls", "/dev"], text=True).split("\n")
            usb_devices = [device for device in devices if device[:6] == "ttyUSB"]

            for i, device in enumerate(usb_devices):
                self._usb_port = device
                self._command = ["jbdtool", "-t", f"serial:/dev/{self._usb_port}"]
                resp = self.get_bms_data()
                if resp != "":
                    print(f"Found device {self._usb_port}")
                    break

                if i == len(usb_devices) - 1:
                    raise Exception("No USB device was found. Ensure that battery pack is connected to Raspberry Pi")

        self._voltage = 0
        self._current = 0
        self._design_capacity = 0
        self._remaining_capacity = 0
        self._percent_capacity = 0
        self._cycle_count = 0
        self._probes = 0
        self._strings = 0
        self._temps = 0
        self._cells = 0
        self._balance = ""
        self._cell_total = 0
        self._cell_min = 0
        self._cell_max = 0
        self._cell_avg = 0
        self._device_name = ""
        self._manufacture_date = ""
        self._version = ""
        self._FET = ""

    def get_bms_data(self) -> str | None:
        """
            Function for getting data from the BMS

            Parameters: 

            Returns: 
                if the jbdtool call works, it returns the BMS data as a string, 
                otherwise it prints the error and returns None

            Example output:
                Voltage                   22.050
                Current                   -2.000
                DesignCapacity            62.000
                RemainingCapacity         8.560
                PercentCapacity           14
                CycleCount                1
                Probes                    3
                Strings                   6
                Temps                     24.9,23.3,23.2
                Cells                     3.648,3.681,3.682,3.678,3.680,3.680
                Balance                   000000
                CellTotal                 22.049
                CellMin                   3.648
                CellMax                   3.682
                CellDiff                  0.034
                CellAvg                   3.675
                DeviceName                JBD-AP21S001-L21S-200A-B
                ManufactureDate           20221206
                Version                   6.8
                FET                       Charge,Discharge
        """

        try: 
            response = subprocess.run(self._command,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE,
                                  check=True)
            
            return response.stdout.decode()
        except subprocess.CalledProcessError as e:
            print("An error occured when getting BMS data")
            print(f"Error: {e.stderr.decode()}")
            print("Please check that USBs are connected to Raspberry Pi, and that the _usb_port variable is set correctly")

            return None

    def parse_bms_data(self, bms_data: str) -> None:
        """
            Parses BMS data and updates class members accordingly

            Parameters:
                bms_data (str): string containing result of the jbdtool command

            Returns: None
        """
        
        if bms_data == "":
            print("Warning: No data was found.")
            return

        data = [entry.split() for entry in bms_data.split("\n")][:-1]       # [:-1] is only there because there is a empty list at the end for some reason

        self._voltage = float(data[0][1])
        self._current = float(data[1][1])
        self._design_capacity = float(data[2][1])
        self._remaining_capacity = float(data[3][1])
        self._percent_capacity = float(data[4][1]) / 100
        self._cycle_count = int(data[5][1])
        self._probes = int(data[6][1])
        self._strings = int(data[7][1])
        self._temps = [float(temp) for temp in data[8][1].split(",")]
        self._cells = [float(cell) for cell in data[9][1].split(",")]
        self._balance = data[10][1]
        self._cell_total = float(data[11][1])
        self._cell_min = float(data[12][1])
        self._cell_max = float(data[13][1])
        self._cell_avg = float(data[14][1])
        self._device_name = data[15][1]
        self._manufacture_date = data[16][1]
        self._version = data[17][1]
        self._FET = data[18][1]


    def change_usb_port(self, usb_port: str) -> None:
        """
            Changes the usb port.

            Parameters: 
                usb_port (str): The name of the port to change to

            Returns: 
                None
        """

        self._usb_port = usb_port
        self._command = ["jbdtool", "-t", f"serial:/dev/{usb_port}"]

#region getters
    @property 
    def voltage(self):
        return self._voltage

    @property 
    def current(self):
        return self._current
    
    @property
    def design_capacity(self):    
        return self._design_capacity

    @property 
    def remaining_capacity(self):
        return self._remaining_capacity

    @property 
    def percent_capacity(self):    
        return self._percent_capacity

    @property 
    def cycle_count(self):    
        return self._cycle_count

    @property 
    def probes(self):    
        return self._probes

    @property 
    def strings(self):
        return self._strings

    @property 
    def temps(self):
        return self._temps
    
    @property 
    def cells(self):    
        return self._cells
    
    @property 
    def balance(self):    
        return self._balance
    
    @property 
    def cell_total(self):    
        return self._cell_total
    
    @property 
    def cell_min(self):    
        return self._cell_min
    
    @property 
    def cell_max(self):    
        return self._cell_max
    
    @property 
    def cell_avg(self):    
        return self._cell_avg
    
    @property 
    def device_name(self):    
        return self._device_name
    
    @property 
    def manufacture_date(self):    
        return self._manufacture_date
    
    @property 
    def version(self):    
        return self._version
    
    @property 
    def FET(self):    
        return self._FET
#endregion