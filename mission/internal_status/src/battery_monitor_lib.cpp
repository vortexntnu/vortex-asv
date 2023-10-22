#include <iostream>
using namespace std; // do i need this?

//#include <linus/i2c-dev.h>
//#include <sys/ioctl.h>
#include "smbus.h"
#include <time.h>
#include "MCP342x.h"

int main(){
    cout << "Hello world" << endl;
    cout << "smbus included" << endl;
    cout << "MCP342x included" << endl;
    return 0;
}

// include doesn't work well

// how to know the types of the values?

//

/*
class BatteryMonitor{

public:
    BatteryMonitor()  //default constructor to initialize the class
    {
        //***************I2C protocole*******************************
        i2c_adress = 0x69;
        bus = smbus.SMBus(1);
        channel_voltage = MCP342x(bus,i2c_adress,channel=0,resolution=18);
        channel_current = MCP342x(bus,i2c_adress,channel=1,resolution=18);
        //TIME SLEEP? -> only for rospy?

        //**********convertion ratio taken from PSM datasheet**************
        psm_to_battery_voltage = 11.0;
        psm_to_battery_current_scale_factor =37.8788;
        psm_to_battery_current_offset = 0.330;


        system_voltage = 0.0;
        system_current = 0.0;

        system_voltage_state = "No receive";
        system_current_state = "No receive";

        I2C_error_counter_current = 0;
        I2C_error_counter_voltage = 0;

        cout << "Iniatialization done" << endl;
    }

    float get_PSM_current(){

        try {
            system_current = ((channel_current.convert_and_read()
                            - psm_to_battery_current_offset)
                            * psm_to_battery_current_scale_factor);

            if (system_current_state != "Received") {
                system_current_state = "Received";
            }
        }

        catch (const std::ios_base::failure& e) {
            I2C_error_counter_current += 1;
            system_current_state = "Error";
            cerr << "Error: " << e.what() << endl;
        }

        return system_current;
    }

    float get_PSM_voltage(){

        try {
            system_voltage = (channel_voltage.convert_and_read()
                            * psm_to_battery_voltage);

            if (system_voltage_state != "Received") {
                system_voltage_state = "Received";
            }
        }

        catch (const std::ios_base::failure& e) {
            I2C_error_counter_voltage += 1;
            system_current_state = "Error";
            cerr << "Error: " << e.what() << endl;
        }

        return system_voltage;
    }

    void shutdown(){

        bus.close();
    }

private:
    int16_t i2c_adress;
    int bus;
    float channel_voltage; //what's the type of these value?
    float channel_current;  //what's the type of these value?
    float psm_to_battery_voltage;
    float psm_to_battery_current_scale_factor;
    float psm_to_battery_current_offset;
    float system_voltage;
    float system_current;
    string system_voltage_state;
    string system_current_state;
    int I2C_error_counter_current;
    int I2C_error_counter_voltage;

};

*/

/* TRY CATCH BLOCK


try {
    system_current = (channel_current.convert_and_read()
                    - psm_to_battery_current_offset)
                    * psm_to_battery_current_scale_factor;

    if (system_current_state != "Received") {
        system_current_state = "Received";
    }
}
catch (const std::ios_base::failure& e) {
    I2C_error_counter_current += 1;
    system_current_state = "Error";
}


*/