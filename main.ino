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
    RTC_calibration();

    switch (mode)
    {
    case modes::edit_RA:
        edit_ra();
        break;
    case modes::edit_dec:
        edit_dec();
        break;

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
        edit_Ra_Dec();

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
            RTC_calibration();
            readGPS();
            read_compass();
            updateAccel();
            calculate_starposition();
            updateDisplay();
            decodeIR();
            //static String msss;
            //remote_input_handler_str(print_exiting, msss, play);
            //LOG(msss);
        }

        mode = modes::INIT_PROCEDURE;
        break;
    }
}
void print_exiting()
{
}