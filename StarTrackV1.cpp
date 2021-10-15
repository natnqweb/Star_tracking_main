#include "StarTrackV1.h"
/* todo ::1)try to move motors to correct position
    allign with north
        and allign with measured star position
        first of all i need to estimate how many encoder pulses it takes to move 360
        2)get user input from ir remote 
        - user need to input few things for example 
        -right Ascension of star and declination of star 
        - second thing is offsets for example magnetic declination ( only if we keep using magnetometer) currently im not convinced it will work
        3) add function to handle ir remote input 1 function that will replace all switch cases -- cuurrently working on 

        
        
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
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1132);
Adafruit_MPU6050 mpu;
simpletimer compass_timer;
simpletimer accel_timer;
simpletimer starposition_timer;
Adafruit_HX8357 TFTscreen = Adafruit_HX8357(cs, dc, rst);
SkyMap startracker;
Encoder_Motor_PID motor1(ENCA, ENCB, IN1, IN2);
Encoder_Motor_PID motor2(ENCA2, ENCB2, IN1_2, IN2_2);
simpletimer loadingscreen;

Myposition my_location(50.03, 21.01);
Star star(0, 0, 101.52, -16.7424);
Star *sirius = &star;
#pragma endregion constructors
#pragma region functions
void laser(bool on_off)
{
    digitalWrite(Laser_pin, on_off);
    laser_state = on_off;
}
void allign_with_star()
{
    if (ready_to_move == true && sirius->azymuth != my_location.azymuth)
    {
        float dAzymuth = abs(my_location.azymuth - sirius->azymuth);
        LOG("dAzymuth");
        LOG(dAzymuth);
        dAzymuth *= constants::gear_constant; //numcer of pulses to move
                                              // dAzymuth *= constants::gear_constant
        //todo move to the star position
        if (ready_to_move)
        {
            while (ready_to_move && motor1.motor_state && dAzymuth > 3)
            {
                decodeIR();
                Az_engine(dAzymuth);
            }
            while (ready_to_move && motor2.motor_state)
            {
            }

            mode = modes::GETTING_STAR_LOCATION;
        }
    }
}
void read_compass()
{

    if (compass_timer.timer(refresh::compass_refresh_rate))
    {

        // float magnetic_x = 0;
        // float magnetic_y = 0;
        // for (int i = 0; i < constants::number_of_measurements; i++)
        // {
        // mag.getEvent(&compass_event);
        // magnetic_x += compass_event.magnetic.x;
        // magnetic_y += compass_event.magnetic.y;
        // }
        // magnetic_x /= constants::number_of_measurements;
        //magnetic_y /= constants::number_of_measurements;
        mag.getEvent(&compass_event);
        //magnetic_x += compass_event.magnetic.x;
        // magnetic_y += compass_event.magnetic.y;
        float heading = atan2(compass_event.magnetic.x, compass_event.magnetic.y);

        if (heading < 0)
            heading += 2 * PI;

        // Check for wrap due to addition of declination.
        if (heading > 2 * PI)
            heading -= 2 * PI;
        // Convert radians to degrees for readability.
        degs headingDegrees = heading * 180 / M_PI;
        headingDegrees += offsets::azymuth_offset + offsets::magnetic_variation;

        LOG("Heading (degrees): ");
        LOG(headingDegrees);
        my_location.azymuth = headingDegrees;
    }
}
void RTC_calibration()
{
    if (calibration)
    {

        smartDelay(1000);

        // The following lines can be uncommented to set the date and time
        rtc.setDOW(SUNDAY);                                                                            // Set Day-of-Week to SUNDAY or whatever day of week it is
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
    pinMode(Laser_pin, OUTPUT);

    rtc.begin();
    ss->begin(constants::GPSBaud);

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
        //  mode == 3 ? mode = 0 : mode += 1;
        break;
    case minus:
        //  mode == 0 ? mode = 3 : mode -= 1;
        break;
    case EQ:
        calibration = !calibration;
        break;
    case zero:

        static bool laser_mode = false;
        laser_mode ? laser(on) : laser(off);
        laser_mode = !laser_mode;
        break;
    }
}
static void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (ss->available())
        {

            gps.encode(ss->read());
        }
    } while (millis() - start < ms);
}

void readGPS()
{
    smartDelay(refresh::gps_refresh_rate);
    t = rtc.getTime();
}

void calculate_starposition()
{
    if (starposition_timer.timer(refresh::calculation_refresh_rate))
    {

        day = (float)t.date;
        month = (float)t.mon;
        year = (float)t.year;
        my_location.latitude = gps.location.lat();
        my_location.longitude = gps.location.lng();
        LOG(my_location.longitude);

        MIN = (float)t.min;
        HOUR = (float)t.hour;
        SEKUNDA = (float)t.sec;
        TIME = startracker.Hh_mm_ss2UTC(HOUR,
                                        MIN,
                                        SEKUNDA,
                                        offsets::timezone_offset);
        // sirius->right_ascension = 101.52;
        // sirius->declination = -16.7424;
        if (my_location.latitude != 0 && my_location.longitude != 0)
        {

            startracker.update(my_location.latitude,
                               my_location.longitude,
                               sirius->declination,
                               sirius->right_ascension,
                               year,
                               month,
                               day,
                               TIME);
            GPS_status = true;

            sirius->azymuth = startracker.get_star_Azymuth();
            sirius->altitude = startracker.get_star_Altitude();
            mode = modes::POINTING_TO_STAR;
            ready_to_move = true;
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
        float accelXsum = 0;
        float accelYsum = 0;
        float accelZsum = 0;

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
        degs anglex = atan2(accelXsum, sqrt(square(accelYsum) + square(accelZsum))) / (constants::pi / 180);
        degs angley = atan2(accelYsum, sqrt(square(accelXsum) + square(accelZsum))) / (constants::pi / 180);

        pointing_altitude = anglex;
        LOG("angleX: ");
        LOG(anglex);
    }
}

void updateDisplay()
{
    displayconfig mainscreen;

    TFT_dispStr("long:", mainscreen.column, mainscreen.row, mainscreen.textsize);
    TFT_dispStr("lat:", mainscreen.column, mainscreen.row + 12 * 2, mainscreen.textsize);
    TFT_dispStr("seconds", mainscreen.column, mainscreen.row + 24 * 2, mainscreen.textsize);
    TFT_dispStr("azymuth:", mainscreen.column, mainscreen.row + 36 * 2, mainscreen.textsize);
    TFT_dispStr("altitude:", mainscreen.column, mainscreen.row + 48 * 2, mainscreen.textsize);
    TFT_dispStr("year:", mainscreen.column, mainscreen.row + 60 * 2, mainscreen.textsize);
    TFT_dispStr("month:", mainscreen.column, mainscreen.row + 72 * 2, mainscreen.textsize);
    TFT_dispStr("day:", mainscreen.column, mainscreen.row + 84 * 2, mainscreen.textsize);
    TFT_dispStr("time:", mainscreen.column, mainscreen.row + 96 * 2, mainscreen.textsize);
    TFT_dispStr("calibration:", mainscreen.column, mainscreen.row + 108 * 2, mainscreen.textsize);
    //other method
    mainscreen.next_column(23);
    print("laser angle", mainscreen);
    mainscreen.next_column(18);
    String l_ang = String(pointing_altitude);
    if (!l_ang.equals(bfstr11))
    {
        clear(bfstr11, mainscreen);
        bfstr11 = l_ang;
        print(l_ang, mainscreen);
    }
    mainscreen.reset_cursor();
    mainscreen.next_row(2);
    mainscreen.next_column(23);
    print("Az_angle", mainscreen);
    mainscreen.next_column(18);
    String az_ang = String(my_location.azymuth);
    if (!az_ang.equals(bfstr12))
    {
        clear(bfstr12, mainscreen);
        bfstr12 = az_ang;
        print(az_ang, mainscreen);
    }
    mainscreen.reset_cursor();

    //other method of printing to tft

    if (GPS_status)
    {
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
        String val4 = String(sirius->azymuth);
        if (!val4.equals(bufferstr4))
        {
            TFT_clear(bufferstr4, 8 * 7 * 2, mainscreen.row + 36 * 2, mainscreen.textsize);
            bufferstr4 = val4;
            TFT_dispStr(bufferstr4, 8 * 7 * 2, mainscreen.row + 36 * 2, mainscreen.textsize);
        }
        String val5 = String(sirius->altitude);
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

void Az_engine(float &target) //need to be in some standalone function cuz it is not attached to pin interuppt
{

    target = long(target);
    motor1.set_target(target);
    motor1.start();

    if (motor1.Input == target)
    {
        motor1.turn_off();
    }
    else
    {
        motor1.turn_on();
    }
}
void Alt_engine(float &target)
{
    target = long(target);
}
void boot_init_procedure()
{
    displayconfig boot_init_disp;

    uint8_t decoded_procedure = decodeIRfun();

    bool confirm = false;
    bool setmode = false;
    switch (decoded_procedure)
    {

    case play: // button on remote 'play' //confirm

        confirm = true;
        break;
    case plus: // button on remote '+'

        break;
    case minus: // button on remote '-'

        break;
    case EQ: // button on remote 'EQ' input magnetic variation

        mode = modes::SELECT_OFFSET;
        TFT_clear("instrukcja:", boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
        TFT_clear("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_clear("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_clear("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_clear("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_clear("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
        TFT_clear("Ustw.mag.deklinacje", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_clear("wartosc++", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_clear("wartosc--", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_clear("potwierdz/kontynuuj", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_clear("wsp. gwiazdy", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
        setmode = true;
        break;
    case zero: // button on remote '0'
        break;
    }
    if (confirm || setmode)
    {

        TFT_clear("instrukcja:", boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
        TFT_clear("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_clear("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_clear("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_clear("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_clear("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
        TFT_clear("Ustw.mag.deklinacje", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_clear("wartosc++", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_clear("wartosc--", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_clear("potwierdz/kontynuuj", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_clear("wsp. gwiazdy", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    }
    else
    {

        TFT_dispStr("instrukcja:", boot_init_disp.column, boot_init_disp.row, boot_init_disp.textsize);
        TFT_dispStr("EQ-", boot_init_disp.column, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_dispStr("+", boot_init_disp.column, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_dispStr("-", boot_init_disp.column, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_dispStr("play", boot_init_disp.column, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_dispStr("0", boot_init_disp.row, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
        TFT_dispStr("Ustw.mag.deklinacje", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 12 * 2, boot_init_disp.textsize);
        TFT_dispStr("wartosc++", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 24 * 2, boot_init_disp.textsize);
        TFT_dispStr("wartosc--", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 36 * 2, boot_init_disp.textsize);
        TFT_dispStr("potwierdz/kontynuuj", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 48 * 2, boot_init_disp.textsize);
        TFT_dispStr("wsp. gwiazdy", boot_init_disp.column + (8 * 6) * 2, boot_init_disp.row + 60 * 2, boot_init_disp.textsize);
    }

    static int mess_row = 0;
    static int mess_col = 0;
    while (confirm)
    {
        read_compass();
        updateAccel();
        readGPS();
        new_starting_position();

        motor1.Input = my_location.azymuth * 2.5;
        delay(100);

        confirm = false;
        mess_row = 0;
        mess_col = 0;
        mode = modes::GETTING_STAR_LOCATION;
    }
}
void new_starting_position()
{
    //todo : define this constatns for motors they may differ significantly
    starting_position_az = my_location.azymuth * constants::gear_constant;
    starting_position_alt = pointing_altitude * constants::gear_constant;
    motor1.set_starting_position(long(starting_position_az));
    motor2.set_starting_position(long(starting_position_alt));
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

degs edit_Ra_Dec() // todo : make interface for entering Ra and Dec after booting
{
    displayconfig boot_disp;

    TFT_dispStr("1- RA", boot_disp.column, boot_disp.row, boot_disp.textsize);
    TFT_dispStr("2- DEC", boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
    TFT_dispStr("play- finish", boot_disp.column, 40, boot_disp.textsize);

    if (decoded_command == one)
    {
        TFT_clear("1-RA", boot_disp.column, boot_disp.row, boot_disp.textsize);
        TFT_clear("2-DEC", boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
        TFT_clear("play-finish", boot_disp.column, boot_disp.row + 40, boot_disp.textsize);
        TFT_dispStr("enter Star RA", boot_disp.column, boot_disp.row, boot_disp.textsize);
        bool entering_RA = true;
        String input_RA;
        while (entering_RA)
        {

            decoded_command = decodeIRfun();
            switch (decoded_command)
            {
            case zero:

                input_RA += "0";
                break;
            case one:

                input_RA += "1";
                break;
            case two:

                input_RA += "2";
                break;
            case three:

                input_RA += "3";
                break;
            case four:

                input_RA += "4";
                break;
            case five:

                input_RA += "5";
                break;
            case six:

                input_RA += "6";
                break;
            case seven:

                input_RA += "7";
                break;
            case eight:

                input_RA += "8";
                break;
            case nine:

                input_RA += "9";
                break;
            case EQ:

                input_RA += ".";
                break;
            case play:

                star.right_ascension = input_RA.toFloat();
                TFT_clear("enter Star RA", boot_disp.column, boot_disp.row, boot_disp.textsize);
                TFT_clear(input_RA, boot_disp.column, boot_disp.row + 30, boot_disp.textsize);
                entering_RA = false;
                break;
            }
            TFT_dispStr(input_RA, boot_disp.column, boot_disp.row + 30, boot_disp.textsize);
        }
    }
    else if (decoded_command == two)
    {
        TFT_clear("1-RA", boot_disp.column, boot_disp.row, boot_disp.textsize);
        TFT_clear("2-DEC", boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
        TFT_dispStr("enter Star DEC", boot_disp.column, boot_disp.row, boot_disp.textsize);
        bool entering_DEC = true;
        String input_DEC;
        while (entering_DEC)
        {
            switch (decodeIRfun())
            {
            case zero:

                input_DEC += "0";
                break;
            case one:

                input_DEC += "1";
                break;
            case two:

                input_DEC += "2";
                break;
            case three:

                input_DEC += "3";
                break;
            case four:

                input_DEC += "4";
                break;
            case five:

                input_DEC += "5";
                break;
            case six:

                input_DEC += "6";
                break;
            case seven:

                input_DEC += "7";
                break;
            case eight:

                input_DEC += "8";
                break;
            case nine:

                input_DEC += "9";
                break;
            case EQ:

                input_DEC += ".";
                break;
            case play:

                star.right_ascension = input_DEC.toFloat();
                TFT_clear("enter Star Dec", boot_disp.column, boot_disp.row, boot_disp.textsize);
                TFT_clear(input_DEC, boot_disp.column, boot_disp.row + 30, boot_disp.textsize);
                entering_DEC = false;
                break;
            }
            TFT_dispStr(input_DEC, boot_disp.column, boot_disp.row + 30, boot_disp.textsize);
        }
    }
}

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
void mode_selection() // currently useless may consider deleting this
{
    switch (decodeIRfun())
    {
    case one:
        mode = modes::INIT_PROCEDURE;
        break;

    case two:
        break;
    case three:
        break;
    case four:
        break;
    case five:
        break;
    case six:
        break;
    case seven:
        break;
    case eight:
        break;
    case nine:
        break;
    case zero:
        break;
    case no_command:
        break;
    }
}
void IRremote_callback(void_func fun, uint8_t _command) // currently not implemented
{

    if (decodeIRfun() == _command)
    {
        fun();
    }
}
void offset_select() // todo: let user enter all offsets independently from this set in program
{

    displayconfig offsets_screen;
    print("1-", offsets_screen);
    offsets_screen.next_row(2);
    print("2-", offsets_screen);
    offsets_screen.reset_cursor();
    offsets_screen.next_column(3);
    print("enter accel_offset", offsets_screen);
    offsets_screen.next_row(2);
    print("enter azymuth offset", offsets_screen);
    offsets_screen.reset_cursor();

    switch (decodeIRfun())
    {
    case play:
        clear("1-", offsets_screen);
        offsets_screen.next_row(2);
        clear("2-", offsets_screen);
        offsets_screen.reset_cursor();
        offsets_screen.next_column(3);
        clear("enter accel_offset", offsets_screen);
        offsets_screen.next_row(2);
        clear("enter azymuth offset", offsets_screen);
        mode = modes::GETTING_STAR_LOCATION;
        //clear_all();
        break;
    case one:
        clear("1-", offsets_screen);
        offsets_screen.next_row(2);
        clear("2-", offsets_screen);
        offsets_screen.reset_cursor();
        offsets_screen.next_column(3);
        clear("enter accel_offset", offsets_screen);
        offsets_screen.next_row(2);
        clear("enter azymuth offset", offsets_screen);
        mode = modes::OFFSET_EDIT;
        offset_edit_mode = offset_editing::MAGNETIC;
        //clear_all();

        break;

    case two:
        clear("1-", offsets_screen);
        offsets_screen.next_row(2);
        clear("2-", offsets_screen);
        offsets_screen.reset_cursor();
        offsets_screen.next_column(3);
        clear("enter accel_offset", offsets_screen);
        offsets_screen.next_row(2);
        clear("enter azymuth offset", offsets_screen);
        //clear_all();
        break;
    }
}
void clear(String sentence, displayconfig &cnfg)
{
    TFT_clear(sentence, cnfg.column, cnfg.row, cnfg.textsize);
}
void print(String sentence, displayconfig &cnfg)
{
    TFT_dispStr(sentence, cnfg.column, cnfg.row, cnfg.textsize);
}
void clear_all()
{
    TFTscreen.fillScreen(HX8357_BLACK);
}

void safety_motor_position_control() // turn off motor if laser is to far up or down
{
    if (pointing_altitude > 90 || pointing_altitude < -10)
    {
        motor2.turn_off();
    }
}
void input_offsets()
{

    switch (offset_edit_mode)
    {

    case offset_editing::MAGNETIC:
        displayconfig edit_magnetic_var;
        print("EDIT MAGNETIC DECLINATION", edit_magnetic_var);
        edit_magnetic_var.next_row(2);
        print("magnetic declination =", edit_magnetic_var);
        edit_magnetic_var.next_row(2);
        print(input_MAG_DEC, edit_magnetic_var);

        switch (decodeIRfun())
        {
        case zero:

            input_MAG_DEC += "0";
            break;
        case one:

            input_MAG_DEC += "1";
            break;
        case two:

            input_MAG_DEC += "2";
            break;
        case three:

            input_MAG_DEC += "3";
            break;
        case four:

            input_MAG_DEC += "4";
            break;
        case five:

            input_MAG_DEC += "5";
            break;
        case six:

            input_MAG_DEC += "6";
            break;
        case seven:

            input_MAG_DEC += "7";
            break;
        case eight:

            input_MAG_DEC += "8";
            break;
        case nine:

            input_MAG_DEC += "9";
            break;
        case EQ:

            input_MAG_DEC += ".";
            break;
        case play:

            offset_edit_mode = offset_editing::TIME;
            offsets::magnetic_variation = input_MAG_DEC.toFloat();
            edit_magnetic_var.reset_cursor();
            clear("EDIT MAGNETIC DECLINATION", edit_magnetic_var);
            edit_magnetic_var.next_row(2);
            clear("magnetic declination =", edit_magnetic_var);
            edit_magnetic_var.next_row(2);
            clear(input_MAG_DEC, edit_magnetic_var);
            mode = modes::GETTING_STAR_LOCATION;

            break;
        }
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
void remote_input_handler(void_func functionexit, String &result)
{
    switch (decodeIRfun())
    {
    case zero:

        result += "0";
        break;
    case one:

        result += "1";
        break;
    case two:

        result += "2";
        break;
    case three:

        result += "3";
        break;
    case four:

        result += "4";
        break;
    case five:

        result += "5";
        break;
    case six:

        result += "6";
        break;
    case seven:

        result += "7";
        break;
    case eight:

        result += "8";
        break;
    case nine:

        result += "9";
        break;
    case EQ:

        result += ".";
        break;
    case play:

        functionexit();

        break;
    }
}
#if DEBUG
void print_debug_message(int col, int row, uint8_t size)
{
    TFT_dispStr("debug", col, row, size);
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