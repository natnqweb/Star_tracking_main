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

    if (mode == GETTING_STAR_LOCATION)
    {
        decodeIR();
    }
    switch (mode)
    {
    case CALIBRATE_POSITION:
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

        readGPS();
        read_compass();
        updateAccel();
        updateDisplay();
        calculate_starposition();

        break;
    case DISPLAY_RESULTS:
        updateDisplay();
        decodeIR();

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

    default:
#if DEBUG
        while (DEBUG)
        {
            TFTscreen.println("herehrhearjhaslrdasjkldujaskn");
        }
#endif
        mode = INIT_PROCEDURE;
        break;
    }
}
