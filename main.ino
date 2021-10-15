#include "StarTrackV1.h"
//all rights reserved by Natan Lisowski
#define test 1
void setup()
{

    initialize_();
    safety_motor_position_control();
    RTC_calibration();
    laser(off);
    LOG("program started");
    if (test)
        motor1.turn_on();
}
void loop()
{
    safety_motor_position_control();
    RTC_calibration();
    switch (mode)
    {

    case modes::GETTING_STAR_LOCATION:
        readGPS();
        read_compass();
        updateAccel();
        calculate_starposition();
        updateDisplay();
        decodeIR();

        break;
    case modes::POINTING_TO_STAR:
        allign_with_star();

        break;

    case modes::SETTINGS:
        //  print_debug_message(0, 0, 1);
        IRremote_callback(updateDisplay, one);

        break;
    case modes::INIT_PROCEDURE:
        boot_init_procedure();
        break;
    case modes::SELECT_OFFSET:
        offset_select();
        break;
    case modes::OFFSET_EDIT:
        input_offsets();
        break;

    default:

        //debug_rtc();
        while (test)
        {
            motor1.set_target(50);
            motor1.start();
        }
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