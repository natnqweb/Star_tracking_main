#include "StarTrackV1.h"
//all rights reserved by Natan Lisowski
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

    case GETTING_STAR_LOCATION:
        readGPS();
        read_compass();
        updateAccel();
        calculate_starposition();
        updateDisplay();
        decodeIR();

        break;
    case POINTING_TO_STAR:
        allign_with_star();

        break;

    case SETTINGS:
        //  print_debug_message(0, 0, 1);
        IRremote_callback(updateDisplay, one);

        break;
    case INIT_PROCEDURE:
        boot_init_procedure();
        break;
    case SELECT_OFFSET:
        offset_select();
        break;
    case OFFSET_EDIT:
        input_offsets();
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