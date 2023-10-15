import subprocess

# TODO: Fix so that it automatically detects which USB port to use
# TODO: Check that data is received on initialization (find out if batteries are 
# connected)

class BMS:
    """ Class containing Freya's BMS system

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

    def __init__(self, usb_port: str) -> None:
        """
            Parameters:
                usb_port (str): USB port to connect to, either ttyUSB0 or ttyUSB1

            Returns:
                None

            Note: Private members are denoted by _variable_name            
        """
        self.usb_port = usb_port

        self.command = ["jbdtool", "-t", f"serial:/dev/{usb_port}"]
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
            response = subprocess.run(self.command,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE,
                                  check=True)
            
            return response
        except subprocess.CalledProcessError as e:
            print("An error occured when getting BMS data")
            print(f"Error: {e.stderr.decode()}")
            print("Please check that USBs are connected to Raspberry PI")
            return None

    def parse_bms_data(self, bms_data: subprocess.CompletedProcess) -> None:
        """
            Parses BMS data and updates class members accordingly

            Parameters:
                bms_data (subprocess.CompletedProcess): object containing result 
                of the jbdtool command

            Returns: None
        """
        
        data = bms_data.stdout.decode().split("\n")
        
        for element in data:

            element = element.split()
            print(element)

        self._voltage = float(data[0])
        self._current = float(data[1])
        self._design_capacity = float(data[2])
        self._remaining_capacity = float(data[3])
        self._percent_capacity = float(data[4]) / 100
        self._cycle_count = int(data[5])
        self._probes = int(data[6])
        self._strings = int(data[7])
        self._temps = int(data[8].split(","))
        self._cells = int(data[9].split(","))
        self._balance = data[10]
        self._cell_total = float(data[11])
        self._cell_min = float(data[12])
        self._cell_max = float(data[13])
        self._cell_avg = float(data[14])
        self._device_name = data[15]
        self._manufacture_date = data[16]
        self._version = data[17]
        self._FET = data[18]


    def change_usb_port(self, usb_port: str) -> None:
        """
            Changes the usb port.

            Parameters: 
                usb_port (str): The name of the port to change to

            Returns: 
                None
        """

        self.usb_port = usb_port
        self.command = ["jbdtool", "-t", f"serial:/dev/{usb_port}"]

test = BMS
test.change_usb_port