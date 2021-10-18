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

    switch (mode)
    {

    case modes::GETTING_STAR_LOCATION:
        readGPS();
        //read_compass();
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

        while (DEBUG) // loop for debbuging purposes
        {
            //readGPS();
            // read_compass();
            // updateAccel();
            //calculate_starposition();
            //updateDisplay();
            static String msss;
            remote_input_handler_str(print_exiting, msss, play);
            LOG(msss);
        }

        mode = modes::INIT_PROCEDURE;
        break;
    }
}
void print_exiting()
{
}