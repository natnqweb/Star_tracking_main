#include "StarTrackV1.h"
//13.10.2021 version 1 all rights reserved Natan Lisowski
void setup()
{

    initialize_();
    safety_motor_position_control();
    RTC_calibration();
    laser(off);
    LOG("program started");
}
void loop()
{
    safety_motor_position_control();
    RTC_calibration();
    switch (mode)
    {

    case getting_star_location:
        readGPS();
        read_compass();
        updateAccel();
        calculate_starposition();
        updateDisplay();
        decodeIR();

        break;
    case pointing_to_star:
        allign_with_star();

        break;

    case settings:
        //  print_debug_message(0, 0, 1);
        IRremote_callback(updateDisplay, one);

        break;
    case init_procedure:
        boot_init_procedure();
        break;

    default:

        //debug_rtc();
        boot_init_procedure();

        while (DEBUG)
        {

            // if (decodeIRfun() != no_command)
            // {
            //   LOG(decodeIRfun());
            // }
            updateAccel();
        }

        break;
    }
}