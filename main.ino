#include "StarTrackV1.h"
//all rights reserved by Natan Lisowski

void setup()
{

    initialize_();

    RTC_calibration();
    laser(off);
    LOG("program started");
}
void loop()
{

    if (!DEBUG)
    {
        safety_motor_position_control();
    }
    RTC_calibration();
    if (DEBUG)
        motor1.turn_off();
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
        while (DEBUG)
        {

            motor2.start();
            motor2.set_target(50);
            motor2.limit(76, 255);

            LOG(motor2.get_position());
        }
        boot_init_procedure();

        //while (DEBUG)
        // {

        // if (decodeIRfun() != no_command)
        // {
        //   LOG(decodeIRfun());
        // }
        // updateAccel();
        // }

        break;
    }
}