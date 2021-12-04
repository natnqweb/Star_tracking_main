#include "StarTrackV1.h"
/**
All rights reserved by Natan Lisowski
GIT: @natnqweb 
Email: natanlisowski@gmail.com 


*/
/**
 * @author @b Natan @b Lisowski @github: @b @natnqweb   @email: @c pythonboardsbeta@gmail.com
 * 
 * */
void setup()
{

    initialize_();

    RTC_calibration();
    laser(off);
    LOG("program started");
}
void loop()
{

    if (mode == GETTING_STAR_LOCATION || mode == DISPLAY_RESULTS)
    {
        decodeIR_remote();
    }
    switch (mode)
    {
    case CALIBRATE_POSITION:
        startup = false;
        position_calibration_display();
        break;

    case MOVEMOTOR1:

        Az_engine();

        break;
    case MOVEMOTOR2:

        Alt_engine();

        break;
    case EDIT_LAT:
        edit_lat();

        break;
    case EDIT_LONG:
        edit_long();
        break;
    case EDIT_RA:
        edit_ra();
        break;
    case EDIT_DEC:
        edit_dec();
        break;

    case GETTING_STAR_LOCATION:

        read_compass();
        accel_callback_timer.run(refresh::accel_refresh_rate);
        calculate_starposition();

        display_callback_timer.run(refresh::TFT_refresh_rate);

        startup = false;

        break;

    case SETTINGS:
        edit_Ra_Dec();

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
    case MANUAL_CALIBRATION:
        manual_calibration_screen();
        break;

    default:
#if DEBUG
        while (DEBUG)
        {
            read_compass();
        }
#endif
        mode = INIT_PROCEDURE;
        break;
    }
}
