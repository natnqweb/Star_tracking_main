#include "StarTrackV1.h"
/* todo ::
        1)try to move motors to correct position
        allign with north
        and allign with measured star position
        first of all i need to estimate how many encoder pulses it takes to move 360
        2)get user input from ir remote  //currently working on
        - user need to input few things for example // done partialy
        -right Ascension of star and declination of star // done
        - second thing is offsets for example magnetic declination ( only if we keep using magnetometer) currently im not convinced it will work
        3) add function to handle ir remote input 1 function that will replace all switch cases -- cuurrently working on ---- done! 
            done  18/10/2021

        current work progress: currently we have user input specificly we get some offsets and user now can input star Ra and Declination values from
        he cab take them from skymap cli app that is avaliable on github natnqweb 
        work on day : 22.10.2021 :
            working on adding debug modes for motors to estimate number of pulses for 360 degrees turn and then calculating gear ratio to adjust
            "allign " with accel and magnetometer
            deleted sirius pointer and replaced it with struct star that was unnecessary pointer
            finally detected parameters for both motors and inserted them into code 
            also added manual latitude and longitude input by user
            23.10.2021 :
            added language support english and polish currently are supported
            added exit from displayfunction and clear screen and empty buffer to structure to free existing buffer
            26.10.2021 :
             addded notification about star visibility to  displayupdate mainsreen
             also updated skymap with new function called IsVisible() that retruns ture if start is visible at positive value altitude
             27.10.2021:
             added starting angle diplay on mainscreen display for both motors
             added functions to allign with star and move both motors to correct position




        
        
 */
#pragma region constructor_definitions
bool simpletimer::timer(unsigned long _time)
{
    if (micros() - before >= _time * 1000)
    {
        before = micros();
        return true;
    }
    else
    {
        return false;
    }
}
Myposition::Myposition(degs latitude, degs longitude, degs azymuth)
{
    this->latitude = latitude;
    this->longitude = longitude;
    this->azymuth = azymuth;
}
Star::Star(degs azymuth, degs altitude, degs right_ascension, degs declination)
{
    this->azymuth = azymuth;
    this->altitude = altitude;
    this->right_ascension = right_ascension;
    this->declination = declination;
}
#pragma endregion constructor_definitions
#pragma region constructors
TinyGPSPlus gps;
Time t;
DS3231 rtc(SDA, SCL);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1132); //added custon id  1132
Adafruit_MPU6050 mpu;
simpletimer displaytimer;
simpletimer compass_timer;
simpletimer accel_timer;
simpletimer starposition_timer;
Adafruit_HX8357 TFTscreen = Adafruit_HX8357(cs, dc, rst);
SkyMap startracker;
motor motor1(ENCA, ENCB, IN1, IN2);
motor motor2(ENCA2, ENCB2, IN1_2, IN2_2);
simpletimer loadingscreen;

Myposition my_location(50.03, 21.01); //location for tarnów
Star star(0, 0, 101.52, -16.7424);    //Sirius ra and dec at start

#pragma endregion constructors
#pragma region functions
void laser(bool on_off)
{
    digitalWrite(Laser_pin, on_off);
    laser_state = on_off;
}
void allign_with_star()
{
    if (ready_to_move == true && (motor1.target_reached() == false || motor2.target_reached() == false))
    {
        float dAzymuth = abs(my_location.azymuth - star.azymuth);
        LOG("dAzymuth");
        LOG(dAzymuth);
        dAzymuth *= constants::motor1_gear_ratio; //numcer of pulses to move
                                                  // dAzymuth *= constants::motor1_gear_ratio
        //todo move to the star position
        // if (ready_to_move)
        // {
        //   while (ready_to_move && motor1.motor_state && dAzymuth > 3)
        //    {
        //  decodeIR();
        //  Az_engine(dAzymuth);
        // }
        // while (ready_to_move && motor2.motor_state)
        //  {
        // }

        //
        // }
        mode = modes::GETTING_STAR_LOCATION;
    }
    else
    {
        mode = modes::GETTING_STAR_LOCATION;
    }
}
void read_compass()
{

    if (compass_timer.timer(refresh::compass_refresh_rate))
    {

        float magnetic_x = 0;
        float magnetic_y = 0;
        for (int i = 0; i < constants::number_of_measurements; i++)
        {
            mag.getEvent(&compass_event);
            magnetic_x += compass_event.magnetic.x;
            magnetic_y += compass_event.magnetic.y;
        }
        magnetic_x /= constants::number_of_measurements;
        magnetic_y /= constants::number_of_measurements;

        //magnetic_x += compass_event.magnetic.x;
        // magnetic_y += compass_event.magnetic.y;
        float heading = atan2(magnetic_y, magnetic_x);
        heading += startracker.deg2rad(offsets::magnetic_variation);

        if (heading < 0)
            heading += 2 * PI;

        // Check for wrap due to addition of declination.
        if (heading > 2 * PI)
            heading -= 2 * PI;
        // Convert radians to degrees for readability.

        degs headingDegrees = heading * 180 / M_PI;
        headingDegrees += offsets::azymuth_offset;
        // headingDegrees += offsets::azymuth_offset + offsets::magnetic_variation;
        if (headingDegrees > 360)
        {
            headingDegrees -= 360;
        }
        else if (headingDegrees < 0)
        {
            headingDegrees += 360;
        }
        else if (headingDegrees == 360)
        {
            headingDegrees = 0;
        }

        LOG("Heading (degrees): ");
        LOG(headingDegrees);
        my_location.azymuth = headingDegrees;
    }
}
void RTC_calibration()
{
    if (calibration)
    {

        readGPS();

        // The following lines can be uncommented to set the date and time
        rtc.setDOW(MONDAY);                                                                            // Set Day-of-Week to SUNDAY or whatever day of week it is
        rtc.setTime(gps.time.hour() + offsets::timezone_offset, gps.time.minute(), gps.time.second()); // Set the time to 12:00:00 (24hr format)
        rtc.setDate(gps.date.day(), gps.date.month(), gps.date.year());                                // Set the date to January 1st, 2014
    }                                                                                                  // Initialize the rtc object
}

void init_accel()
{
    if (!mpu.begin(accel_address))
    {
        LOG("mpu connection failed");

        while (!mpu.begin(accel_address))
        {

            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}
void compass_init()
{
    if (!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        LOG("Ooops, no HMC5883 detected ... Check your wiring!");
        LOG("waiting for compass..");
        while (!mag.begin())
        {
            LOG(".");
            delay(100);
        }
    }
}
void initialize_()
{
    start_debuging(constants::Serial_Baud);
    motor1.init(constants::kp, constants::ki, constants::kd);
    motor2.init(constants::kp, constants::ki, constants::kd);
    motor1.limit(constants::motor1_lower_limit, constants::motor1_upper_limit);
    motor2.limit(constants::motor2_lower_limit, constants::motor2_upper_limit);
    pinMode(Laser_pin, OUTPUT);

    rtc.begin();
    Serial3.begin(constants::GPSBaud);

    TFTscreen.begin();
    TFTscreen.fillScreen(HX8357_BLACK);
    TFTscreen.setRotation(3);
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
    IrReceiver.decodeNEC();
    compass_init();
    init_accel();
}

void decodeIR()
{

    switch (decodeIRfun())
    {

    case plus:
        clearDisplay();
        mode = modes::INIT_PROCEDURE;
        break;
    case minus:
        //  mode == 0 ? mode = 3 : mode -= 1;
        break;
    case EQ:
        //calibration = !calibration;
        break;
    case zero:

        laser_mode = !laser_mode;
        laser_mode ? laser(on) : laser(off);
        break;
    case two:
        clearDisplay();
        reset_ready_to_move_markers();
        mode = modes::INIT_PROCEDURE;

        break;
    }
}
//void smartDelay()// no delays!
//{
//}

void readGPS()
{

    /* code */

    while (Serial3.available())
    {

        gps.encode(Serial3.read());
    }
}

void calculate_starposition()
{
    if (starposition_timer.timer(refresh::calculation_refresh_rate))
    {
        t = rtc.getTime();
        day = (float)t.date;
        month = (float)t.mon;
        year = (float)t.year;
        if (automatic_mode)
        {
            my_location.latitude = gps.location.lat();
            my_location.longitude = gps.location.lng();
        }

        LOG(my_location.longitude);

        MIN = (float)t.min;
        HOUR = (float)t.hour;
        SEKUNDA = (float)t.sec;
        TIME = startracker.Hh_mm_ss2UTC(HOUR,
                                        MIN,
                                        SEKUNDA,
                                        offsets::timezone_offset);
        // star.right_ascension = 101.52;
        // star.declination = -16.7424;
        if (my_location.latitude != 0 && my_location.longitude != 0)
        {

            startracker.update(my_location.latitude,
                               my_location.longitude,
                               star.declination,
                               star.right_ascension,
                               year,
                               month,
                               day,
                               TIME);
            GPS_status = true;

            star.azymuth = startracker.get_star_Azymuth();
            star.altitude = startracker.get_star_Altitude();
            azymuth_target = star.azymuth * constants::motor1_gear_ratio;
            altitude_target = star.altitude * constants::motor2_gear_ratio;

            ready_to_move = true;
            if (all_motors_ready_to_move())
            {
                mode = modes::MOVEMOTOR1;
            }
        }
        else
        {
            GPS_status = false;
            ready_to_move = false;
            mode = modes::GETTING_STAR_LOCATION;
        }
    }
}

void updateAccel()
{

    if (accel_timer.timer(refresh::accel_refresh_rate))
    {

        for (int i = 0; i < constants::number_of_measurements; i++)
        {

            mpu.getEvent(&a, &g, &temp);

            accelXsum += a.orientation.x;
            accelYsum += a.orientation.y;
            accelZsum += a.orientation.z;
        }
        accelXsum /= constants::number_of_measurements;
        accelYsum /= constants::number_of_measurements;
        accelZsum /= constants::number_of_measurements;

        // Calculate of roll and pitch in deg
        pointing_altitude = atan2(accelXsum, sqrt(square(accelYsum) + square(accelZsum))) / (constants::pi / 180);
        // degs angley = atan2(accelYsum, sqrt(square(accelXsum) + square(accelZsum))) / (constants::pi / 180);

        // pointing_altitude = anglex;
        LOG("angleX: ");
        LOG(pointing_altitude);
    }
}
void clearDisplay()
{
    mainscreen.reset_cursor();
    TFT_clear(un_long, mainscreen.column, mainscreen.row, mainscreen.textsize);
    TFT_clear(un_lat, mainscreen.column, mainscreen.row + 12 * 2, mainscreen.textsize);
    TFT_clear(un_second, mainscreen.column, mainscreen.row + 24 * 2, mainscreen.textsize);
    TFT_clear(un_azymuth, mainscreen.column, mainscreen.row + 36 * 2, mainscreen.textsize);
    TFT_clear(un_altitude, mainscreen.column, mainscreen.row + 48 * 2, mainscreen.textsize);
    TFT_clear(un_year, mainscreen.column, mainscreen.row + 60 * 2, mainscreen.textsize);
    TFT_clear(un_month, mainscreen.column, mainscreen.row + 72 * 2, mainscreen.textsize);
    TFT_clear(un_day, mainscreen.column, mainscreen.row + 84 * 2, mainscreen.textsize);
    TFT_clear(un_time_utc, mainscreen.column, mainscreen.row + 96 * 2, mainscreen.textsize);
    TFT_clear(un_calibration, mainscreen.column, mainscreen.row + 108 * 2, mainscreen.textsize);
    mainscreen.next_column(23);
    clear(un_laser_angle, mainscreen);
    mainscreen.next_column(18);

    clear(laser_angle_buff.disp, mainscreen);
    laser_angle_buff.clear_buffer();
    mainscreen.reset_cursor();
    mainscreen.next_row(2);
    mainscreen.next_column(23);
    clear(un_azymuth, mainscreen);
    mainscreen.next_column(18);
    clear(az_buff.disp, mainscreen);
    az_buff.clear_buffer();
    mainscreen.reset_cursor();
    mainscreen.next_row(4);
    mainscreen.next_column(23);
    clear(un_right_ascension, mainscreen);
    mainscreen.next_column(18);
    clear(ra_buff.disp, mainscreen);
    ra_buff.clear_buffer();
    mainscreen.reset_cursor();
    mainscreen.next_row(6);
    mainscreen.next_column(23);
    clear(un_declination, mainscreen);
    mainscreen.next_column(18);
    clear(dec_buff.disp, mainscreen);
    dec_buff.clear_buffer();
    mainscreen.reset_cursor();
    TFT_clear(bufferstr, mainscreen.column + (8 * 6) * 2, mainscreen.row, mainscreen.textsize);

    TFT_clear(bufferstr3, mainscreen.column + (8 * 6) * 2, mainscreen.row + 12 * 2, mainscreen.textsize);
    TFT_clear(bufferstr4, 8 * 7 * 2, mainscreen.row + 36 * 2, mainscreen.textsize);
    TFT_clear(bufferstr5, 8 * 7 * 2, mainscreen.row + 48 * 2, mainscreen.textsize);

    TFT_clear(bufferstr3, mainscreen.column + (8 * 6) * 2, mainscreen.row + 12 * 2, mainscreen.textsize);
    TFT_clear(bufferstr2, mainscreen.column + (8 * 6) * 2, mainscreen.row + 24 * 2, mainscreen.textsize);
    TFT_clear(bufferstr6, 8 * 7 * 2, mainscreen.row + 60 * 2, mainscreen.textsize);
    TFT_clear(bufferstr7, 8 * 7 * 2, mainscreen.row + 72 * 2, mainscreen.textsize);
    TFT_clear(bufferstr8, 8 * 7 * 2, mainscreen.row + 84 * 2, mainscreen.textsize);
    TFT_clear(bufferstr9, 8 * 7 * 2, mainscreen.row + 96 * 2, mainscreen.textsize);
    TFT_clear(bufferstr10, 8 * 10 * 2, mainscreen.row + 108 * 2, mainscreen.textsize);
    bufferstr = EMPTYSTRING;
    bufferstr3 = EMPTYSTRING;
    bufferstr4 = EMPTYSTRING;
    bufferstr5 = EMPTYSTRING;
    bufferstr2 = EMPTYSTRING;
    bufferstr6 = EMPTYSTRING;
    bufferstr7 = EMPTYSTRING;
    bufferstr8 = EMPTYSTRING;
    bufferstr9 = EMPTYSTRING;
    bufferstr10 = EMPTYSTRING;
    mainscreen.next_row(8);
    mainscreen.next_column(23);
    clear(un_star_visibility, mainscreen);
    mainscreen.next_row(2);
    clear(visibility_buffer.disp, mainscreen);
    visibility_buffer.clear_buffer();
    mainscreen.reset_cursor();
    mainscreen.next_row(29);
    clear(un_motor1, mainscreen);
    mainscreen.next_row(2); //row 14
    clear(un_degree, mainscreen);
    mainscreen.next_column(10); // column 10
    clear(motor1_ang_buff.disp, mainscreen);
    motor1_ang_buff.clear_buffer();

    mainscreen.set_cursor(33, 0);
    clear(un_motor2, mainscreen);
    mainscreen.next_row(2);
    clear(un_degree, mainscreen); //
    mainscreen.next_column(10);
    clear(motor2_ang_buff.disp, mainscreen);
    motor2_ang_buff.clear_buffer();
    mainscreen.reset_cursor();
}
void updateDisplay()
{
    if (displaytimer.timer(refresh::TFT_refresh_rate))
    {
        LOG("display updated");
        mainscreen.reset_cursor();
        TFT_dispStr(un_long, mainscreen.column, mainscreen.row, mainscreen.textsize);
        TFT_dispStr(un_lat, mainscreen.column, mainscreen.row + 12 * 2, mainscreen.textsize);
        TFT_dispStr(un_second, mainscreen.column, mainscreen.row + 24 * 2, mainscreen.textsize);
        TFT_dispStr(un_azymuth, mainscreen.column, mainscreen.row + 36 * 2, mainscreen.textsize);
        TFT_dispStr(un_altitude, mainscreen.column, mainscreen.row + 48 * 2, mainscreen.textsize);
        TFT_dispStr(un_year, mainscreen.column, mainscreen.row + 60 * 2, mainscreen.textsize);
        TFT_dispStr(un_month, mainscreen.column, mainscreen.row + 72 * 2, mainscreen.textsize);
        TFT_dispStr(un_day, mainscreen.column, mainscreen.row + 84 * 2, mainscreen.textsize);
        TFT_dispStr(un_time_utc, mainscreen.column, mainscreen.row + 96 * 2, mainscreen.textsize);
        TFT_dispStr(un_calibration, mainscreen.column, mainscreen.row + 108 * 2, mainscreen.textsize);
        //other method
        mainscreen.next_column(23);
        print(un_laser_angle, mainscreen);
        mainscreen.next_column(18);
        laser_angle_buff.disp = String(pointing_altitude);
        dynamic_print(mainscreen, laser_angle_buff);

        mainscreen.reset_cursor();
        mainscreen.next_row(2);
        mainscreen.next_column(23);
        print(un_azymuth, mainscreen);
        mainscreen.next_column(18);
        az_buff.disp = String(my_location.azymuth);
        dynamic_print(mainscreen, az_buff);

        mainscreen.reset_cursor();
        mainscreen.next_row(4);
        mainscreen.next_column(23);
        print(un_right_ascension, mainscreen);
        mainscreen.next_column(18);
        ra_buff.disp = (String)star.right_ascension;
        dynamic_print(mainscreen, ra_buff);

        mainscreen.reset_cursor();
        mainscreen.next_row(6);
        mainscreen.next_column(23);
        print(un_declination, mainscreen);
        mainscreen.next_column(18);
        dec_buff.disp = (String)star.declination;
        dynamic_print(mainscreen, dec_buff);

        mainscreen.reset_cursor();
        mainscreen.next_row(29);
        print(un_motor1, mainscreen); // row 29 column 0
        mainscreen.next_row(2);       //row 31
        print(un_degree, mainscreen); // row 31 column 0
        mainscreen.next_column(10);   // row 31 column 10
        motor1_ang_buff.disp = (String)(motor1.get_position() / constants::motor1_gear_ratio);
        dynamic_print(mainscreen, motor1_ang_buff); // row 31 column 10
        mainscreen.reset_cursor();
        mainscreen.set_cursor(33, 0);
        print(un_motor2, mainscreen);
        mainscreen.next_row(2);       // row 35 column 0
        print(un_degree, mainscreen); // row 35 column 0
        mainscreen.next_column(10);   //row 35 column 10
        motor2_ang_buff.disp = (String)(motor2.get_position() / constants::motor2_gear_ratio);
        dynamic_print(mainscreen, motor2_ang_buff); //row 35 column 10
        mainscreen.reset_cursor();                  //row 0 column 0

        //other method of printing to tft

        if (GPS_status)
        {
            mainscreen.next_row(8);
            mainscreen.next_column(23);
            print(un_star_visibility, mainscreen);
            mainscreen.next_row(2);
            startracker.IsVisible() ? visibility_buffer.disp = un_visible : visibility_buffer.disp = un_unvisible;
            dynamic_print(mainscreen, visibility_buffer);
            mainscreen.reset_cursor();

            String val1 = String(my_location.longitude);
            if (!val1.equals(bufferstr))
            {
                TFT_clear(bufferstr, mainscreen.column + (8 * 6) * 2, mainscreen.row, mainscreen.textsize);
                bufferstr = val1;
                TFT_dispStr(bufferstr, mainscreen.column + (8 * 6) * 2, mainscreen.row, mainscreen.textsize);
            }
            String val3 = String(my_location.latitude);
            if (!val3.equals(bufferstr3))
            {
                TFT_clear(bufferstr3, mainscreen.column + (8 * 6) * 2, mainscreen.row + 12 * 2, mainscreen.textsize);
                bufferstr3 = val3;
                TFT_dispStr(bufferstr3, mainscreen.column + (8 * 6) * 2, mainscreen.row + 12 * 2, mainscreen.textsize);
            }
            String val4 = String(star.azymuth);
            if (!val4.equals(bufferstr4))
            {
                TFT_clear(bufferstr4, 8 * 7 * 2, mainscreen.row + 36 * 2, mainscreen.textsize);
                bufferstr4 = val4;
                TFT_dispStr(bufferstr4, 8 * 7 * 2, mainscreen.row + 36 * 2, mainscreen.textsize);
            }
            String val5 = String(star.altitude);
            if (!val5.equals(bufferstr5))
            {
                TFT_clear(bufferstr5, 8 * 7 * 2, mainscreen.row + 48 * 2, mainscreen.textsize);
                bufferstr5 = val5;
                TFT_dispStr(bufferstr5, 8 * 7 * 2, mainscreen.row + 48 * 2, mainscreen.textsize);
            }
        }
        else
        {
            String val1 = "no gps";
            if (!val1.equals(bufferstr))
            {
                TFT_clear(bufferstr, mainscreen.column + (8 * 6) * 2, mainscreen.row, mainscreen.textsize);
                bufferstr = val1;
                TFT_dispStr(bufferstr, mainscreen.column + (8 * 6) * 2, mainscreen.row, mainscreen.textsize);
            }
            String val3 = "no gps";
            if (!val3.equals(bufferstr3))
            {
                TFT_clear(bufferstr3, mainscreen.column + (8 * 6) * 2, mainscreen.row + 12 * 2, mainscreen.textsize);
                bufferstr3 = val3;
                TFT_dispStr(bufferstr3, mainscreen.column + (8 * 6) * 2, mainscreen.row + 12 * 2, mainscreen.textsize);
            }
        }
        String val2 = String(int(SEKUNDA));
        if (!val2.equals(bufferstr2))
        {
            TFT_clear(bufferstr2, mainscreen.column + (8 * 6) * 2, mainscreen.row + 24 * 2, mainscreen.textsize);
            bufferstr2 = val2;
            TFT_dispStr(bufferstr2, mainscreen.column + (8 * 6) * 2, mainscreen.row + 24 * 2, mainscreen.textsize);
        }
        String val6 = String((int)year);
        if (!val6.equals(bufferstr6))
        {
            TFT_clear(bufferstr6, 8 * 7 * 2, mainscreen.row + 60 * 2, mainscreen.textsize);
            bufferstr6 = val6;
            TFT_dispStr(bufferstr6, 8 * 7 * 2, mainscreen.row + 60 * 2, mainscreen.textsize);
        }
        String val7 = String((int)month);
        if (!val7.equals(bufferstr7))
        {
            TFT_clear(bufferstr7, 8 * 7 * 2, mainscreen.row + 72 * 2, mainscreen.textsize);
            bufferstr7 = val7;
            TFT_dispStr(bufferstr7, 8 * 7 * 2, mainscreen.row + 72 * 2, mainscreen.textsize);
        }
        String val8 = String((int)day);
        if (!val8.equals(bufferstr8))
        {
            TFT_clear(bufferstr8, 8 * 7 * 2, mainscreen.row + 84 * 2, mainscreen.textsize);
            bufferstr8 = val8;
            TFT_dispStr(bufferstr8, 8 * 7 * 2, mainscreen.row + 84 * 2, mainscreen.textsize);
        }
        String val9 = String(TIME);
        if (!val9.equals(bufferstr9))
        {
            TFT_clear(bufferstr9, 8 * 7 * 2, mainscreen.row + 96 * 2, mainscreen.textsize);
            bufferstr9 = val9;
            TFT_dispStr(bufferstr9, 8 * 7 * 2, mainscreen.row + 96 * 2, mainscreen.textsize);
        }
        String val10 = String(calibration);
        if (!val10.equals(bufferstr10))
        {
            TFT_clear(bufferstr10, 8 * 10 * 2, mainscreen.row + 108 * 2, mainscreen.textsize);
            bufferstr10 = val10;
            TFT_dispStr(bufferstr10, 8 * 10 * 2, mainscreen.row + 108 * 2, mainscreen.textsize);
        }
    }
}
void TFT_dispStr(String str, int column, int row, uint8_t textsize)
{

    str.toCharArray(printout1, str.length() + 2);

    TFTscreen.setTextSize(textsize);
    TFTscreen.setTextColor(HX8357_WHITE);
    TFTscreen.setCursor(column, row);
    TFTscreen.print(printout1);
}
void TFT_clear(String strr, int column, int row, uint8_t textsize)
{

    strr.toCharArray(printout1, strr.length() + 2);
    TFTscreen.setTextSize(textsize);
    TFTscreen.setTextColor(HX8357_BLACK);
    TFTscreen.setCursor(column, row);
    TFTscreen.print(printout1);
}

#pragma region init_procedure
void boot_init_exit_func1()
{
    mode = modes::SELECT_OFFSET;
    TFT_clear(un_instruction, boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
    TFT_clear("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
    TFT_clear("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
    TFT_clear("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
    TFT_clear("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
    TFT_clear("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    TFT_clear(un_set_mag_declination, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
    TFT_clear(un_your_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
    TFT_clear(un_device_position_calibration, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
    TFT_clear(un_submit_continue, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
    TFT_clear(un_star_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    setmode = true;
}
void set_true_confirm()
{
    confirm = true;
}
void boot_init_exit_func2()
{
    mode = modes::SETTINGS;
    TFT_clear(un_instruction, boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
    TFT_clear("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
    TFT_clear("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
    TFT_clear("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
    TFT_clear("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
    TFT_clear("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    TFT_clear(un_set_mag_declination, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
    TFT_clear(un_your_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
    TFT_clear(un_device_position_calibration, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
    TFT_clear(un_submit_continue, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
    TFT_clear(un_star_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    setmode = true;
}
void boot_init_exit_func3()
{
    mode = modes::EDIT_LAT;
    TFT_clear(un_instruction, boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
    TFT_clear("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
    TFT_clear("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
    TFT_clear("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
    TFT_clear("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
    TFT_clear("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    TFT_clear(un_set_mag_declination, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
    TFT_clear(un_your_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
    TFT_clear(un_device_position_calibration, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
    TFT_clear(un_submit_continue, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
    TFT_clear(un_star_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    setmode = true;
}
void boot_init_exit_func4()
{
    mode = modes::CALIBRATE_POSITION;
    TFT_clear(un_instruction, boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
    TFT_clear("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
    TFT_clear("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
    TFT_clear("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
    TFT_clear("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
    TFT_clear("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    TFT_clear(un_set_mag_declination, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
    TFT_clear(un_your_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
    TFT_clear(un_device_position_calibration, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
    TFT_clear(un_submit_continue, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
    TFT_clear(un_star_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    setmode = true;
}
void boot_init_procedure()
{
    boot_init_disp.reset_cursor();

    confirm = false;
    setmode = false;
    remote_input_handler_selector(set_true_confirm, play, boot_init_exit_func1, EQ, boot_init_exit_func2, zero, boot_init_exit_func3, plus, boot_init_exit_func4, minus);
    if (confirm || setmode)
    {

        TFT_clear(un_instruction, boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
        TFT_clear("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_clear("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_clear("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_clear("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_clear("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
        TFT_clear(un_set_mag_declination, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_clear(un_your_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_clear(un_device_position_calibration, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_clear(un_submit_continue, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_clear(un_star_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    }
    else
    {

        TFT_dispStr(un_instruction, boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
        TFT_dispStr("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_dispStr("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_dispStr("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_dispStr("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_dispStr("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
        TFT_dispStr(un_set_mag_declination, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_dispStr(un_your_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_dispStr(un_device_position_calibration, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_dispStr(un_submit_continue, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_dispStr(un_star_location, boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    }

    static int mess_row = 0;
    static int mess_col = 0;
    if (confirm)
    {
        read_compass();
        updateAccel();
        readGPS();
        if (!manual_calibration)
        {
            new_starting_position();
        }
        confirm = false;
        mess_row = 0;
        mess_col = 0;
        mode = modes::GETTING_STAR_LOCATION;
    }
}
#pragma endregion init_procedure
void new_starting_position()
{
    //todo : define this constatns for motors they may differ significantly
    starting_position_az = my_location.azymuth * constants::motor1_gear_ratio;
    starting_position_alt = pointing_altitude * constants::motor2_gear_ratio;
    motor1.set_position(starting_position_az);
    motor2.set_position(starting_position_alt);
}
uint8_t decodeIRfun()
{
    bool command_flag = false;
    if (IrReceiver.decode())
    {

        for (int i = 0; i < sizeof(pilot_commands); i++)
        {
            if (IrReceiver.decodedIRData.command == pilot_commands[i])
            {

                IrReceiver.resume();
                command_flag = true;
                IrReceiver.decodedIRData.command = no_command;
                return pilot_commands[i];
            }
        }
        IrReceiver.resume();
    }
    if (command_flag == false)
    {

        return no_command;
    }
}
#pragma region editing_ra_dec
void entering_dec_exit_handle()
{

    TFT_clear(un_enter_star_dec, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.row += 30;
    TFT_clear(input_DEC, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.reset_cursor();
    star.declination = input_DEC.toFloat();
    entering_DEC = true;
    entering_RA ? mode = modes::INIT_PROCEDURE : mode = modes::edit_RA;
}
void entering_ra_exit_handle()
{

    TFT_clear(un_enter_star_ra, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.row += 30;
    TFT_clear(input_RA, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.reset_cursor();
    star.right_ascension = input_RA.toFloat();
    entering_RA = true;
    entering_DEC ? mode = modes::INIT_PROCEDURE : mode = modes::edit_dec;
}
void edit_Ra_Dec() // todo : make interface for entering Ra and Dec after booting
{
    boot_disp.reset_cursor();

    TFT_dispStr(un_setting_1_RA, boot_disp.column, boot_disp.row, boot_disp.textsize);
    TFT_dispStr(un_setting_2_DEC, boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
    TFT_dispStr(un_setting_play, boot_disp.column, 40, boot_disp.textsize);

    if (decodeIRfun() == one)
    {
        TFT_clear(un_setting_1_RA, boot_disp.column, boot_disp.row, boot_disp.textsize);
        TFT_clear(un_setting_2_DEC, boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
        TFT_clear(un_setting_play, boot_disp.column, boot_disp.row + 40, boot_disp.textsize);

        mode = modes::edit_RA;
    }
    else if (decodeIRfun() == two)
    {
        TFT_clear(un_setting_1_RA, boot_disp.column, boot_disp.row, boot_disp.textsize);
        TFT_clear(un_setting_2_DEC, boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
        TFT_clear(un_setting_play, boot_disp.column, boot_disp.row + 40, boot_disp.textsize);

        mode = modes::edit_dec;
    }
}
void edit_ra()
{
    TFT_dispStr(un_enter_star_ra, boot_disp.column, boot_disp.row, boot_disp.textsize);

    boot_disp.row += 30;
    TFT_dispStr(input_RA, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.reset_cursor();
    remote_input_handler_str(entering_ra_exit_handle, input_RA, play, deleteallinput);
    boot_disp.reset_cursor();
}
void edit_dec()
{
    TFT_dispStr(un_enter_star_dec, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.row += 30;
    deleteallinput = boot_disp;
    TFT_dispStr(input_DEC, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.reset_cursor();
    remote_input_handler_str(entering_dec_exit_handle, input_DEC, play, deleteallinput);
    boot_disp.reset_cursor();
}
#pragma endregion editing_ra_dec
bool check_if_calibrated() // run before calculating to ensure that its worth wasting time for calculations
{
    bool star_ready = false;
    (star.right_ascension != 0 && star.declination != 0) ? star_ready = true : star_ready = false;
    bool iamready = false;
    (my_location.latitude != 0 && my_location.longitude != 0 && (day && month && year && HOUR && MIN != 0)) ? iamready = true : iamready = false;
    if (star_ready && iamready)
        return true;
    else
        return false;
}

#pragma region offset_selectscrn
void offset_select_remote_exit_play()
{
    clear("1-", offsets_screen);
    offsets_screen.next_row(2);
    clear("2-", offsets_screen);
    offsets_screen.reset_cursor();
    offsets_screen.next_column(3);
    clear(un_enter_accel_offset, offsets_screen);
    offsets_screen.next_row(2);
    clear(un_enter_az_offset, offsets_screen);
    mode = modes::GETTING_STAR_LOCATION;
}
void offset_select_remote_exit_one()
{
    clear("1-", offsets_screen);
    offsets_screen.next_row(2);
    clear("2-", offsets_screen);
    offsets_screen.reset_cursor();
    offsets_screen.next_column(3);
    clear(un_enter_accel_offset, offsets_screen);
    offsets_screen.next_row(2);
    clear(un_enter_az_offset, offsets_screen);
    mode = modes::OFFSET_EDIT;
    offset_edit_mode = offset_editing::MAGNETIC;
}
void offset_select_remote_exit_two()
{
    clear("1-", offsets_screen);
    offsets_screen.next_row(2);
    clear("2-", offsets_screen);
    offsets_screen.reset_cursor();
    offsets_screen.next_column(3);
    clear(un_enter_accel_offset, offsets_screen);
    offsets_screen.next_row(2);
    clear(un_enter_az_offset, offsets_screen);
}

void offset_select() // todo: let user enter all offsets independently from this set in program
{
    offsets_screen.reset_cursor();
    print("1-", offsets_screen);
    offsets_screen.next_row(2);
    print("2-", offsets_screen);
    offsets_screen.reset_cursor();
    offsets_screen.next_column(3);
    print(un_enter_accel_offset, offsets_screen);
    offsets_screen.next_row(2);
    print(un_enter_az_offset, offsets_screen);
    offsets_screen.reset_cursor();
    remote_input_handler_selector(offset_select_remote_exit_one, one, offset_select_remote_exit_one, two, offset_select_remote_exit_play, play);
}

#pragma endregion offset_selectscrn
#pragma region display_functions
void clear(String sentence, displayconfig &cnfg)
{
    TFT_clear(sentence, cnfg.column, cnfg.row, cnfg.textsize);
}
void print(String sentence, displayconfig &cnfg)
{
    TFT_dispStr(sentence, cnfg.column, cnfg.row, cnfg.textsize);
}
void dynamic_print(displayconfig &cnfg, buffers &buffs)
{
    if (!buffs.disp.equals(buffs.buff))
    {
        clear(buffs.buff, cnfg);
        buffs.buff = buffs.disp;
        print(buffs.buff, cnfg);
    }
}
void clear_all()
{
    TFTscreen.fillScreen(HX8357_BLACK);
}

#pragma endregion display_function
#pragma region motor_control_functions
bool all_motors_ready_to_move()
{
    if (ready_to_move && (az_motor_target_reached == false) && (alt_motor_target_reached == false) && startracker.IsVisible())
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool reset_ready_to_move_markers()
{
    alt_motor_target_reached = false;
    az_motor_target_reached = false;

    motor1.target_reached(true);
    motor1.target_reached(true);
}
void safety_motor_position_control() // turn off motor if laser is to far up or down
{
    if (pointing_altitude > 90 || pointing_altitude < -10 || star.altitude > 90 || star.altitude < -10)
    {
        motor2.turn_off();
    }
    else
        motor2.turn_on();
}
bool reached_target_function(motor &engine)
{
    if (engine.target_reached())
    {

        return true;
    }
    else
    {

        return false;
    }
}

void Az_engine(float &target) //need to be in some standalone function cuz it is not attached to pin interuppt
{
    motor1.set_target(target);
    if (!az_motor_target_reached)
    {
        motor1.start();
        if (motor1.target_reached())
        {
            az_motor_target_reached = true;
            alt_motor_target_reached ? mode = modes::DISPLAY_RESULTS : mode = modes::MOVEMOTOR2;
        }
    }
}
void Alt_engine(float &target)
{

    motor2.set_target(target);

    if (!alt_motor_target_reached)
    {
        motor2.start();
        if (motor2.target_reached())
        {
            alt_motor_target_reached = true;
            az_motor_target_reached ? mode = modes::DISPLAY_RESULTS : mode = modes::MOVEMOTOR1;
        }
    }
}

#pragma endregion motor_control_functions
void input_offsets()
{

    switch (offset_edit_mode)
    {

    case offset_editing::MAGNETIC:
        edit_magnetic_var.reset_cursor();

        print(un_set_mag_declination, edit_magnetic_var);
        edit_magnetic_var.next_row(2);
        print(un_magnetic_declination, edit_magnetic_var);
        edit_magnetic_var.next_row(2);
        deleteallinput = edit_magnetic_var;
        print(input_MAG_DEC, edit_magnetic_var);

        remote_input_handler_str(offset_disp_exit_procedure, input_MAG_DEC, play, deleteallinput);
    case offset_editing::TIME:
        displayconfig edit_time;

        break;
    case offset_editing::ACCELEROMETER:
        displayconfig edit_accel_offset;

        break;
    case offset_editing::LOCATION:
        displayconfig edit_location;

        break;
    default:
        break;
    }
}
void offset_disp_exit_procedure()
{
    offset_edit_mode = offset_editing::TIME;
    offsets::magnetic_variation = input_MAG_DEC.toFloat();
    edit_magnetic_var.reset_cursor();
    clear(un_set_mag_declination, edit_magnetic_var);
    edit_magnetic_var.next_row(2);
    clear(un_magnetic_declination, edit_magnetic_var);
    edit_magnetic_var.next_row(2);
    clear(input_MAG_DEC, edit_magnetic_var);
    mode = modes::GETTING_STAR_LOCATION;
}
#pragma region edit_lat_long_functions
void exit_lat()
{
    lat_long_disp.reset_cursor();
    clear(un_enter_latitude, lat_long_disp);
    lat_long_disp.next_row(3);
    clear(input_lat, lat_long_disp);
    my_location.latitude = input_lat.toFloat();
    lat_long_disp.reset_cursor();
    mode = modes::EDIT_LONG;
}
void exit_long()
{
    lat_long_disp.reset_cursor();
    clear(un_enter_longitude, lat_long_disp);
    lat_long_disp.next_row(3);
    clear(input_long, lat_long_disp);
    my_location.longitude = input_long.toFloat();
    lat_long_disp.reset_cursor();
    automatic_mode = false;
    mode = modes::INIT_PROCEDURE;
}
void edit_lat()
{

    print(un_enter_latitude, lat_long_disp);
    lat_long_disp.next_row(3);
    deleteallinput = lat_long_disp;
    print(input_lat, lat_long_disp);
    lat_long_disp.reset_cursor();
    remote_input_handler_str(exit_lat, input_lat, play, deleteallinput);
}
void edit_long()
{

    print(un_enter_longitude, lat_long_disp);
    lat_long_disp.next_row(3);
    deleteallinput = lat_long_disp;
    print(input_long, lat_long_disp);
    lat_long_disp.reset_cursor();
    remote_input_handler_str(exit_long, input_long, play, deleteallinput);
}
#pragma endregion edit_lat_long_functions
#pragma region Remote_control_functions
void remote_input_handler_str(void_func exitprint, String &result, uint8_t number, displayconfig &cnfg, void_func exitprint2, uint8_t number2, void_func exitprint3, uint8_t number3, void_func exitprint4, uint8_t number4)
{
    switch (decodeIRfun())
    {
    case zero:

        result += "0";
        if (number == zero)
            exitprint();
        else if (number2 == zero)
            exitprint2();
        else if (number3 == zero)
            exitprint3();
        else if (number4 == zero)
            exitprint4();
        break;
    case one:

        result += "1";
        if (number == one)
            exitprint();
        else if (number2 == one)
            exitprint2();
        else if (number3 == one)
            exitprint3();
        else if (number4 == one)
            exitprint4();
        break;
    case two:

        result += "2";
        if (number == two)
            exitprint();
        else if (number2 == two)
            exitprint2();
        else if (number3 == two)
            exitprint3();
        else if (number4 == two)
            exitprint4();
        break;
    case three:

        result += "3";
        if (number == three)
            exitprint();
        else if (number2 == three)
            exitprint2();
        else if (number3 == three)
            exitprint3();
        else if (number4 == three)
            exitprint4();
        break;
    case four:

        result += "4";
        if (number == four)
            exitprint();
        else if (number2 == four)
            exitprint2();
        else if (number3 == four)
            exitprint3();
        else if (number4 == four)
            exitprint4();
        break;
    case five:

        result += "5";
        if (number == five)
            exitprint();
        else if (number2 == five)
            exitprint2();
        else if (number3 == five)
            exitprint3();
        else if (number4 == five)
            exitprint4();
        break;
    case six:

        result += "6";
        if (number == six)
            exitprint();
        else if (number2 == six)
            exitprint2();
        else if (number3 == six)
            exitprint3();
        else if (number4 == six)
            exitprint4();
        break;
    case seven:

        result += "7";
        if (number == seven)
            exitprint();
        else if (number2 == seven)
            exitprint2();
        else if (number3 == seven)
            exitprint3();
        else if (number4 == seven)
            exitprint4();
        break;
    case eight:

        result += "8";
        if (number == eight)
            exitprint();
        else if (number2 == eight)
            exitprint2();
        else if (number3 == eight)
            exitprint3();
        else if (number4 == eight)
            exitprint4();
        break;
    case nine:

        result += "9";
        if (number == nine)
            exitprint();
        else if (number2 == nine)
            exitprint2();
        else if (number3 == nine)
            exitprint3();
        else if (number4 == nine)
            exitprint4();
        break;
    case EQ:

        result += ".";
        if (number == EQ)
            exitprint();
        else if (number2 == EQ)
            exitprint2();
        else if (number3 == EQ)
            exitprint3();
        else if (number4 == EQ)
            exitprint4();
        break;
    case play:
        if (number == play)
            exitprint();
        else if (number2 == play)
            exitprint2();
        else if (number3 == play)
            exitprint3();
        else if (number4 == play)
            exitprint4();

        break;
    case plus: // plus clears input line and input string
        clear(result, cnfg);
        result = EMPTYSTRING;
        if (number == plus)
            exitprint();
        else if (number2 == plus)
            exitprint2();
        else if (number3 == plus)
            exitprint3();
        else if (number4 == plus)
            exitprint4();

        break;
    case minus:
        result += "-";
        if (number == minus)
            exitprint();
        else if (number2 == minus)
            exitprint2();
        else if (number3 == minus)
            exitprint3();
        else if (number4 == minus)
            exitprint4();

        break;
    }
}
void remote_input_handler_selector(void_func exitprint, uint8_t number, void_func exitprint2, uint8_t number2, void_func exitprint3, uint8_t number3, void_func exitprint4, uint8_t number4, void_func exitprint5, uint8_t number5)
{
    switch (decodeIRfun())
    {
    case zero:

        if (number == zero)
            exitprint();
        else if (number2 == zero)
            exitprint2();
        else if (number3 == zero)
            exitprint3();
        else if (number4 == zero)
            exitprint4();
        else if (number5 == zero)
            exitprint5();
        break;
    case one:

        if (number == one)
            exitprint();
        else if (number2 == one)
            exitprint2();
        else if (number3 == one)
            exitprint3();
        else if (number4 == one)
            exitprint4();
        else if (number5 == one)
            exitprint5();
        break;
    case two:

        if (number == two)
            exitprint();
        else if (number2 == two)
            exitprint2();
        else if (number3 == two)
            exitprint3();
        else if (number4 == two)
            exitprint4();
        else if (number5 == two)
            exitprint5();
        break;
    case three:

        if (number == three)
            exitprint();
        else if (number2 == three)
            exitprint2();
        else if (number3 == three)
            exitprint3();
        else if (number4 == three)
            exitprint4();
        else if (number5 == three)
            exitprint5();
        break;
    case four:

        if (number == four)
            exitprint();
        else if (number2 == four)
            exitprint2();
        else if (number3 == four)
            exitprint3();
        else if (number4 == four)
            exitprint4();
        else if (number5 == four)
            exitprint5();
        break;
    case five:

        if (number == five)
            exitprint();
        else if (number2 == five)
            exitprint2();
        else if (number3 == five)
            exitprint3();
        else if (number4 == five)
            exitprint4();
        else if (number5 == five)
            exitprint5();
        break;
    case six:

        if (number == six)
            exitprint();
        else if (number2 == six)
            exitprint2();
        else if (number3 == six)
            exitprint3();
        else if (number4 == six)
            exitprint4();
        else if (number5 == six)
            exitprint5();
        break;
    case seven:

        if (number == seven)
            exitprint();
        else if (number2 == seven)
            exitprint2();
        else if (number3 == seven)
            exitprint3();
        else if (number4 == seven)
            exitprint4();
        else if (number5 == seven)
            exitprint5();
        break;
    case eight:

        if (number == eight)
            exitprint();
        else if (number2 == eight)
            exitprint2();
        else if (number3 == eight)
            exitprint3();
        else if (number4 == eight)
            exitprint4();
        else if (number5 == eight)
            exitprint5();
        break;
    case nine:

        if (number == nine)
            exitprint();
        else if (number2 == nine)
            exitprint2();
        else if (number3 == nine)
            exitprint3();
        else if (number4 == nine)
            exitprint4();
        else if (number5 == nine)
            exitprint5();
        break;
    case EQ:

        if (number == EQ)
            exitprint();
        else if (number2 == EQ)
            exitprint2();
        else if (number3 == EQ)
            exitprint3();
        else if (number4 == EQ)
            exitprint4();
        else if (number5 == EQ)
            exitprint5();
        break;
    case play:
        if (number == play)
            exitprint();
        else if (number2 == play)
            exitprint2();
        else if (number3 == play)
            exitprint3();
        else if (number4 == play)
            exitprint4();
        else if (number5 == play)
            exitprint5();

        break;
    case plus:
        if (number == plus)
            exitprint();
        else if (number2 == plus)
            exitprint2();
        else if (number3 == plus)
            exitprint3();
        else if (number4 == plus)
            exitprint4();
        else if (number5 == plus)
            exitprint5();

        break;
    case minus:
        if (number == minus)
            exitprint();
        else if (number2 == minus)
            exitprint2();
        else if (number3 == minus)
            exitprint3();
        else if (number4 == minus)
            exitprint4();
        else if (number5 == minus)
            exitprint5();

        break;
    }
}
#pragma endregion Remote_control_functions
#pragma region Position_calibration
void check_gps_accel_compass()
{
    updateAccel();
    readGPS();
    read_compass();
}
bool check_if_pointing_at_north()
{
    float compassoutput = my_location.azymuth;

    if (compassoutput > 357)
    {
        if (abs(round(compassoutput - 357)) < 2)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        if (compassoutput < 5)
        {
            if (abs(round(compassoutput)) < 3)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }
}
void position_calibration_exit_func1()
{
    calibration_disp.set_cursor(0, 0);
    clear(un_device_position_calibration, calibration_disp);
    calibration_disp.set_cursor(4, 0);
    clear(un_laser_angle, calibration_disp);
    calibration_disp.set_cursor(6, 0);

    clear(laser_angle_buff.disp, calibration_disp);
    laser_angle_buff.clear_buffer();
    calibration_disp.set_cursor(8, 0);
    clear(un_azymuth, calibration_disp);
    calibration_disp.set_cursor(10, 0);
    clear(az_buff.disp, calibration_disp);
    az_buff.clear_buffer();
    calibration_disp.set_cursor(14, 0);
    clear(un_azymuth, calibration_disp);
    calibration_disp.set_cursor(14, 10);

    clear(ra_buff.disp, calibration_disp);
    ra_buff.clear_buffer();
    calibration_disp.reset_cursor();
    new_starting_position();
    manual_calibration = true;
    mode = modes::INIT_PROCEDURE;
}
void position_calibration_exit_cancel()
{
    calibration_disp.set_cursor(0, 0);
    clear(un_device_position_calibration, calibration_disp);
    calibration_disp.set_cursor(4, 0);
    clear(un_laser_angle, calibration_disp);
    calibration_disp.set_cursor(6, 0);

    clear(laser_angle_buff.disp, calibration_disp);
    laser_angle_buff.clear_buffer();
    calibration_disp.set_cursor(8, 0);
    clear(un_azymuth, calibration_disp);
    calibration_disp.set_cursor(10, 0);
    clear(az_buff.disp, calibration_disp);
    az_buff.clear_buffer();
    calibration_disp.set_cursor(14, 0);
    clear(un_azymuth, calibration_disp);
    calibration_disp.set_cursor(14, 10);

    clear(ra_buff.disp, calibration_disp);
    ra_buff.clear_buffer();
    calibration_disp.reset_cursor();
    manual_calibration = false;
    mode = modes::INIT_PROCEDURE;
}
void position_calibration_display()

{
    check_gps_accel_compass();
    calibration_disp.set_cursor(0, 0);
    print(un_device_position_calibration, calibration_disp);
    calibration_disp.set_cursor(4, 0);
    print(un_laser_angle, calibration_disp);
    calibration_disp.set_cursor(6, 0);
    laser_angle_buff.disp = String(pointing_altitude);
    dynamic_print(calibration_disp, laser_angle_buff);
    calibration_disp.set_cursor(8, 0);
    print(un_azymuth, calibration_disp);
    calibration_disp.set_cursor(10, 0);
    if (check_if_pointing_at_north())
    {
        az_buff.disp = un_pointing_at_north;
    }
    else
    {
        az_buff.disp = un_not_pointing_at_north;
    }
    dynamic_print(calibration_disp, az_buff);
    calibration_disp.set_cursor(14, 0);
    print(un_azymuth, calibration_disp);
    calibration_disp.set_cursor(14, 10);
    ra_buff.disp = String(my_location.azymuth);
    dynamic_print(calibration_disp, ra_buff);
    calibration_disp.reset_cursor();
    remote_input_handler_selector(position_calibration_exit_func1, play, position_calibration_exit_cancel, zero);
}

#pragma endregion Position_calibration
#if DEBUG
void print_debug_message(int col, int row, uint8_t size)
{
    TFT_dispStr("debug", col, row, size);
}
void debug_motors()
{

    /*   motor2.set_target(710);
    motor2.limit(120, 255);
    motor2.start(); */
    motor1.set_target(-900);
    motor1.limit(0, 120);
    motor1.start();
    /* MOTOR1
    during debugging i found settings for second motor witch are 
    same kp kd and ki as motor2 and limitits upper 120 lower 0 
    exactly 900 impulses per 360 degrees
    0.4 degrees per pulse
    */

    /* MOTOR2
    setting chosen during debuging for upper laser motor 
    is 
    kp 10 
    kd 0.1 
    ki 0.01
    limit lower :120 upper :255
    710-impulses for 90 degrees 
    that gives 2840 impulses per 360 degrees
    that gives 0.127degrees per pulse
    */

    //debug_motor_display.reset_cursor();
    // print("position:", debug_motor_display);
    // debug_motor_display.next_column(15);
    // debugbuffer.disp = String(motor2.get_position());
    // dynamic_print(debug_motor_display, debugbuffer);
}
void debug_rtc()
{
    t = rtc.getTime();
    displayconfig debugrtc;
    print(String(t.hour), debugrtc);
    debugrtc.next_row(3);
    print(String(t.min), debugrtc);
    debugrtc.next_row(3);
    print(String(t.sec), debugrtc);
    debugrtc.next_row(3);
    print(String(t.year), debugrtc);
    debugrtc.next_row(3);
    print(String(t.mon), debugrtc);
    debugrtc.next_row(3);
    print(String(t.date), debugrtc);

    debugrtc.reset_cursor();
}
#endif
#pragma endregion functions