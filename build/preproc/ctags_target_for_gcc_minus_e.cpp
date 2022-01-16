# 1 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
# 2 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 2
/**

 * @author @b Natan @b Lisowski @github: @b @natnqweb   @email: @c pythonboardsbeta@gmail.com

 * 

 * */
# 6 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
#pragma region constructor_definitions
void displayconfig::next_row(int how_many_rows_further, uint8_t pixels)
{

    this->row += (pixels * how_many_rows_further);
}
void displayconfig::next_column(int how_many_columns, uint8_t pixels)
{
    this->column += (pixels * how_many_columns);
}
void displayconfig::reset_cursor()
{
    /* reset cursor moves cursor to position 0,0 */
    this->column = 0;
    this->row = 0;
}
void displayconfig::set_cursor(int row, int column, uint8_t pixels)
{

    this->row = (pixels * row);
    this->column = (pixels * column);
}
template <>
void buffers<String>::clear_buffer()
{

    buff = "";
}
template <>
void buffers<float>::clear_buffer()
{

    buff = 0;
}
template <>
void buffers<const char *>::clear_buffer()
{

    buff = "";
}
template <>
void buffers<int>::clear_buffer()
{

    buff = 0;
}

Myposition::Myposition(degs latitude, degs longitude, degs azimuth)
{
    this->latitude = latitude;
    this->longitude = longitude;
    this->azimuth = azimuth;
}
Star::Star(degs azimuth, degs altitude, degs right_ascension, degs declination)
{
    this->azimuth = azimuth;
    this->altitude = altitude;
    this->right_ascension = right_ascension;
    this->declination = declination;
}

#pragma endregion constructor_definitions
#pragma region constructors
TinyGPSPlus gps;
Time t;
DS3231 rtc(SDA, SCL);
uEEPROMLib eeprom(0x57 /*eeprom  i2c_address*/);

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1132); //added custon id  1132
Adafruit_MPU6050 mpu;

Adafruit_HX8357 *TFTscreen = new Adafruit_HX8357(cs, dc, rst);
SkyMap *startracker = new SkyMap;

motor motor1 = motor(ENCA, ENCB, IN1, IN2);

motor motor2 = motor(ENCA2, ENCB2, IN1_2, IN2_2);

IRrecv *IR = new IRrecv(IR_RECEIVE_PIN);
//location for Tarnów 50.03 longitude 21.01 latitude
Myposition my_location(50.03, 21.01);
//Sirius ra and dec at start
Star star(0, 0, 101.5333, -16.7478);

#pragma endregion constructors
#pragma region eeprom
template <class T>
void EEPROM::write(unsigned int address, T value)
{ /* if eeprom write failed and debug is set to true program will display that error */
    if (!eeprom.eeprom_write(address, value))
        ;
}
template <class T>
T EEPROM::read(unsigned int address)
{
    T temprorary_storage = 0;
    eeprom.eeprom_read<T>(address, &temprorary_storage);
    return temprorary_storage;
}

#pragma endregion eeprom
#pragma region functions

#pragma region calculations_and_sensors
void laser(bool on_off)
{
    digitalWrite(Laser_pin, on_off);
}

void read_compass()
{

    float magnetic_x = 0;
    float magnetic_y = 0;

    mag.getEvent(&compass_event);
    magnetic_x = compass_event.magnetic.x;
    magnetic_y = compass_event.magnetic.y;

    //magnetic_x += compass_event.magnetic.x;
    // magnetic_y += compass_event.magnetic.y;
    float heading = atan2(magnetic_y, magnetic_x);
    heading += startracker->deg2rad(offsets::magnetic_variation);

    if (heading < 0)
        heading += 2 * 3.1415926535897932384626433832795;

    // Check for wrap due to addition of declination.
    if (heading > 2 * 3.1415926535897932384626433832795)
        heading -= 2 * 3.1415926535897932384626433832795;
    // Convert radians to degrees for readability.
    degs headingDegrees = heading * 180 / 
# 137 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                         3.14159265358979323846 /* pi */
# 137 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                             ;
    /* 

    if (headingDegrees >= 1 && headingDegrees < 240)

    {

        headingDegrees = mapf(headingDegrees, 0, 239, 0, 179);

    }

    else if (headingDegrees >= 240)

    {

        headingDegrees = mapf(headingDegrees, 240, 360, 180, 360);

    }

    smoothHeadingDegrees = round(headingDegrees);



    if (smoothHeadingDegrees < (previousDegree + 2) && smoothHeadingDegrees > (previousDegree - 2))

    {

        smoothHeadingDegrees = previousDegree;

    }

    if (smoothHeadingDegrees <= 2 || smoothHeadingDegrees >= 358)

    {

        smoothHeadingDegrees = 0;

    }



    previousDegree = smoothHeadingDegrees; */
# 159 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    smoothHeadingDegrees = headingDegrees;
    ;
    ;
    my_location.azimuth = headingDegrees;
}
void RTC_calibration()
{
    readGPS();
    if (calibration && gps.time.isValid())
    {

        // The following lines can be uncommented to set the date and time
        // Set Day-of-Week to SUNDAY or whatever day of week it is
        rtc.setTime(gps.time.hour() + offsets::timezone_offset, gps.time.minute(), gps.time.second()); // Set the time to 12:00:00 (24hr format)
        rtc.setDate(gps.date.day(), gps.date.month(), gps.date.year()); // Set the date to January 1st, 2014
    } // Initialize the rtc object
}

void init_accel()
{
    if (!mpu.begin(0x69))
    {
        ;

        while (!mpu.begin(0x69))
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
        ;
        ;
        while (!mag.begin())
        {
            ;
            delay(100);
        }
    }
}
void initialize_()
{
    ;
    motor1.init(constants::kp1, constants::ki1, constants::kd1);
    motor2.init(constants::kp2, constants::ki2, constants::kd2);
    motor1.limit(constants::motor1_lower_limit, constants::motor1_upper_limit);
    motor2.limit(constants::motor2_lower_limit, constants::motor2_upper_limit);
    pinMode(Laser_pin, 0x1);

    rtc.begin();
    IR->begin(IR_RECEIVE_PIN, false);
    Serial3.begin(constants::GPSBaud);
    IR->start(50);
    TFTscreen->begin();
    TFTscreen->fillScreen(0x0000 /*|< BLACK color for drawing graphics*/);
    TFTscreen->setRotation(3);

    init_accel();

    compass_init();
    accel_callback_timer.register_callback(updateAccel);
    display_callback_timer.register_callback(updateDisplay);
}

void switch_laser()
{
    laser_mode = !laser_mode;
    laser_mode ? laser(on) : laser(off);
}
void reset_all_go_to_main()
{
    clearDisplay();
    clear_all_buffers();
    reset_ready_to_move_markers();

    mode = INIT_PROCEDURE;
}
void go_to_main()
{
    clearDisplay();
    clear_all_buffers();
    mode = INIT_PROCEDURE;
}

void readGPS()
{

    while (Serial3.available())
    {

        gps.encode(Serial3.read());
    }
}

void calculate_starposition()
{
    readGPS();

    t = rtc.getTime();
    day = (float)t.date;
    month = (float)t.mon;
    year = (float)t.year;
    if (gps.location.isValid())
    {
        if (automatic_mode)
        {
            my_location.latitude = gps.location.lat();
            my_location.longitude = gps.location.lng();
            EEPROM::write(EEPROM::addresses::lat, my_location.latitude);
            EEPROM::write(EEPROM::addresses::longitude, my_location.longitude);
        }
        GPS_status = true;
    }
    ;
    ;
    MIN = (float)t.min;
    HOUR = (float)t.hour;
    SEKUNDA = (float)t.sec;
    TIME = startracker->Hh_mm_ss2UTC(HOUR,
                                     MIN,
                                     SEKUNDA,
                                     offsets::timezone_offset);
    // star.right_ascension = 101.52;
    // star.declination = -16.7424;
    if (GPS_status)
    {

        startracker->update(my_location.latitude,
                            my_location.longitude,
                            star.declination,
                            star.right_ascension,
                            year,
                            month,
                            day,
                            TIME);

        star.azimuth = startracker->get_star_Azimuth();
        star.altitude = startracker->get_star_Altitude();
        azimuth_target = star.azimuth * constants::motor1_gear_ratio;
        altitude_target = star.altitude * constants::motor2_gear_ratio;
        ready_to_move = true;
        //  motor position detection system
        if (all_motors_ready_to_move())
        {

            motor1.set_target(azimuth_target);
            motor1.limit(constants::motor1_lower_limit, constants::motor1_upper_limit);
            motor2.set_target(altitude_target);
            motor2.limit(constants::motor2_lower_limit, constants::motor2_upper_limit);

            if (continous_tracking)
            {

                if (az_motor_target_reached || alt_motor_target_reached)
                {

                    if (az_motor_target_reached)
                        mode = MOVEMOTOR2;
                    else
                        mode = MOVEMOTOR1;
                }
                else
                    mode = MOVEMOTOR1;
            }
            else
                mode = MOVEMOTOR1;
            laser(on);
        }
    }
    else
    {

        ready_to_move = false;
    }
}

void updateAccel()
{

    //if (accel_timer.timer(refresh::accel_refresh_rate))
    //{

    mpu.getEvent(&a, &g, &temp);

    accelXsum = a.orientation.x;
    accelYsum = a.orientation.y;
    accelZsum = a.orientation.z;

    // Calculate of roll and pitch in deg
    pointing_altitude = atan2(accelXsum, sqrt(square(accelYsum) + square(accelZsum))) / (constants::pi / 180);
    // degs angley = atan2(accelYsum, sqrt(square(accelXsum) + square(accelZsum))) / (constants::pi / 180);

    // pointing_altitude = anglex;
    ;
    ;
    //}
}
#pragma endregion calculations_and_sensors
#pragma region mainscreen
void clearDisplay()
{
    displayconfig mainscreen;

    mainscreen.reset_cursor();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 374 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 374 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "dlug."
# 374 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 374 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.set_cursor(2, 0);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 376 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 376 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "szer."
# 376 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 376 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 378 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 378 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "sekunda"
# 378 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 378 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 380 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 380 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 380 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 380 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, mainscreen);
    mainscreen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 382 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 382 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wysokosc"
# 382 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 382 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 384 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 384 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "rok"
# 384 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 384 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 386 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 386 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "miesiac"
# 386 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 386 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 388 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 388 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "dzien"
# 388 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 388 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 390 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 390 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "czas UTC"
# 390 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 390 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    /*   

      ------------------------------------------------display current mode ---------------------------------------------------------------------------

    */
# 394 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.set_cursor(31, 23);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 395 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 395 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "aktualny tryb"
# 395 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 395 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    continous_tracking ? clear((reinterpret_cast<const __FlashStringHelper *>(
# 397 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 397 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                              "tryb ciaglego sledzenia"
# 397 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                              ); &__c[0];}))
# 397 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                              )), mainscreen) : clear((reinterpret_cast<const __FlashStringHelper *>(
# 397 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                          (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 397 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                          "tryb pojedynczy"
# 397 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                          ); &__c[0];}))
# 397 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                          )), mainscreen);
    mainscreen.reset_cursor();
    /*   

      ------------------------------------------------ end of display current mode ---------------------------------------------------------------------------

    */
# 402 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    //other method
    mainscreen.next_column(31);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 404 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 404 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kat_lasera"
# 404 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 404 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_column(18);

    //clear(laser_angle_buff.disp, mainscreen);
    clear(laser_ang_buff.disp, mainscreen);

    mainscreen.reset_cursor();
    mainscreen.next_row();
    mainscreen.next_column(31);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 413 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 413 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 413 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 413 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, mainscreen);
    mainscreen.next_column(18);
    clear(az_buff.disp, mainscreen);

    mainscreen.reset_cursor();
    mainscreen.next_row(4);
    mainscreen.next_column(31);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 420 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 420 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "rektascensja"
# 420 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 420 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_column(18);
    clear(ra_buff.disp, mainscreen);

    mainscreen.reset_cursor();
    mainscreen.next_row(6);
    mainscreen.next_column(31);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 427 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 427 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "deklinacja"
# 427 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 427 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_column(18);
    clear(dec_buff.disp, mainscreen);

    mainscreen.reset_cursor();
    mainscreen.next_row(29);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 433 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 433 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "silnik 1"
# 433 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 433 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen); // row 29 column 0
    mainscreen.next_row(); //row 31
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 435 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 435 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kat"
# 435 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 435 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen); // row 31 column 0
    mainscreen.next_column(10); // row 31 column 10
    clear(motor1_ang_buff.disp, mainscreen); // row 31 column 10
    mainscreen.reset_cursor();
    mainscreen.set_cursor(33, 0);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 440 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 440 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "silnik 2"
# 440 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 440 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row(); // row 35 column 0
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 442 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 442 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kat"
# 442 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 442 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen); // row 35 column 0
    mainscreen.next_column(10); //row 35 column 10
    clear(motor2_ang_buff.disp, mainscreen); //row 35 column 10
    mainscreen.reset_cursor(); //row 0 column 0
    mainscreen.next_row(8);
    mainscreen.next_column(31);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 448 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 448 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "widocznosc gwiazdy:"
# 448 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 448 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();

    clear(visibility_buffer.disp, mainscreen);
    mainscreen.reset_cursor();
    // results
    mainscreen.set_cursor(0, 15);
    clear(_long_buff.disp, mainscreen);
    mainscreen.next_row();
    clear(_lat_buff.disp, mainscreen);
    mainscreen.next_row();
    clear(seconds_buff.disp, mainscreen);
    //clear((EEPROM::read<int>(EEPROM::addresses::second)), mainscreen);
    mainscreen.next_row();

    clear(_star_az_buff.disp, mainscreen);
    mainscreen.next_row();
    clear(_star_alt_buff.disp, mainscreen);
    mainscreen.next_row();

    clear((t.year), mainscreen);
    mainscreen.next_row();
    clear((t.mon), mainscreen);
    mainscreen.next_row();
    clear((t.date), mainscreen);
    mainscreen.next_row();
    clear(UTC_time_buffer.disp, mainscreen);
    // dodanaj wyświetlanie czasu
    int previous_column = mainscreen.column; //
    mainscreen.next_row();
    mainscreen.column = 0;

    clear((reinterpret_cast<const __FlashStringHelper *>(
# 480 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 480 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "czas lokalny"
# 480 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 480 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.column = previous_column;
    mainscreen.next_column(5);

    clear(_local_time_buff.disp, mainscreen);
    //

    //gps time
    mainscreen.next_row();
    mainscreen.column = 0;

    clear(_calibrate_buff.disp, mainscreen);
    //
    mainscreen.reset_cursor();
    clear_all_buffers();
}
void updateDisplay()
{
    ;
    /*   

      ------------------------------------------------display variable name---------------------------------------------------------------------------

    */
# 502 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    displayconfig mainscreen;
    mainscreen.reset_cursor();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 504 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 504 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "dlug."
# 504 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 504 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.set_cursor(2, 0);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 506 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 506 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "szer."
# 506 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 506 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 508 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 508 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "sekunda"
# 508 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 508 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 510 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 510 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 510 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 510 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, mainscreen);
    mainscreen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 512 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 512 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wysokosc"
# 512 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 512 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 514 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 514 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "rok"
# 514 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 514 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 516 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 516 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "miesiac"
# 516 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 516 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 518 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 518 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "dzien"
# 518 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 518 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 520 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 520 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "czas UTC"
# 520 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 520 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    /*   

      ------------------------------------------------end of display variable name---------------------------------------------------------------------------

    */
# 524 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display current mode ---------------------------------------------------------------------------

    */
# 527 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.set_cursor(31, 23);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 528 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 528 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "aktualny tryb"
# 528 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 528 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    continous_tracking ? print((reinterpret_cast<const __FlashStringHelper *>(
# 530 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 530 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                              "tryb ciaglego sledzenia"
# 530 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                              ); &__c[0];}))
# 530 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                              )), mainscreen) : print((reinterpret_cast<const __FlashStringHelper *>(
# 530 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                          (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 530 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                          "tryb pojedynczy"
# 530 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                          ); &__c[0];}))
# 530 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                          )), mainscreen);
    mainscreen.reset_cursor();
    /*   

      ------------------------------------------------ end of display current mode ---------------------------------------------------------------------------

    */
# 536 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------print accelerometer information cursor set on column 31 row 0 -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 539 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_column(31);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 540 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 540 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kat_lasera"
# 540 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 540 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    //go to column 49 row 0
    mainscreen.next_column(18);
    laser_ang_buff.disp = pointing_altitude;
    dynamic_print(mainscreen, laser_ang_buff);
    //EEPROM::dynamic_print_eeprom(mainscreen, pointing_altitude, EEPROM::addresses::laser_angle);
    /*   

      ------------------------------------------------end of print accelerometer information  -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 549 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display  magnetometer information cursor set on column 31, row 2 -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 552 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.reset_cursor();
    mainscreen.next_row();
    mainscreen.next_column(31);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 555 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 555 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 555 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 555 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, mainscreen);
    mainscreen.next_column(18);
    az_buff.disp = String(my_location.azimuth);

    dynamic_print(mainscreen, az_buff);
    /*   

      ------------------------------------------------end of display  magnetometer -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 563 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    //cursor set to 0,0
    mainscreen.reset_cursor();
    /*   

      ------------------------------------------------display stars right ascension on row 4 and column 31 and its value on row 4 column 49 -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 568 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row(4);
    mainscreen.next_column(31);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 570 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 570 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "rektascensja"
# 570 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 570 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_column(18);
    ra_buff.disp = star.right_ascension;
    dynamic_print(mainscreen, ra_buff);
    /*   

      ------------------------------------------------ end of display stars right ascension  -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 578 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display stars declination row 6 column 31 and its value on row 6 column 49 -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 581 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.reset_cursor();
    mainscreen.next_row(6);
    mainscreen.next_column(31);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 584 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 584 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "deklinacja"
# 584 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 584 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_column(18);
    dec_buff.disp = star.declination;
    dynamic_print(mainscreen, dec_buff);
    /*   

      ------------------------------------------------end of display stars declination row 6 column 31 and its value on row 6 column 49 -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 591 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.reset_cursor();
    /*   

      ------------------------------------------------display motors data row 29 column 0 -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 595 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row(29);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 596 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 596 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "silnik 1"
# 596 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 596 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen); // row 29 column 0
    mainscreen.next_row(); //row 31
    print((reinterpret_cast<const __FlashStringHelper *>(
# 598 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 598 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kat"
# 598 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 598 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen); // row 31 column 0
    mainscreen.next_column(10); // row 31 column 10
    motor1_ang_buff.disp = (motor1.get_position() / constants::motor1_gear_ratio);
    dynamic_print(mainscreen, motor1_ang_buff); // row 31 column 10
    mainscreen.reset_cursor();
    mainscreen.set_cursor(33, 0);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 604 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 604 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "silnik 2"
# 604 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 604 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row(); // row 35 column 0
    print((reinterpret_cast<const __FlashStringHelper *>(
# 606 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 606 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kat"
# 606 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 606 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen); // row 35 column 0
    mainscreen.next_column(10); //row 35 column 10
    motor2_ang_buff.disp = (motor2.get_position() / constants::motor2_gear_ratio);
    dynamic_print(mainscreen, motor2_ang_buff); //row 35 column 10
                                                /*   

      ------------------------------------------------end of display motors data  -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 613 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.reset_cursor(); //row 0 column 0
                                                /*   

      ------------------------------------------------display if star is visible or not row 8 column 31-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 617 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row(8);
    mainscreen.next_column(31);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 619 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 619 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "widocznosc gwiazdy:"
# 619 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 619 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    mainscreen.next_row();
    if (GPS_status)

        startracker->IsVisible() ? visibility_buffer.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 623 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                           (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 623 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                           "widoczna"
# 623 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                           ); &__c[0];}))
# 623 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                           )) : visibility_buffer.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 623 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                                                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 623 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                                                 "nie widoczna"
# 623 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                                                 ); &__c[0];}))
# 623 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                                                 ));

    else
        visibility_buffer.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 626 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 626 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                "brak satelity"
# 626 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                ); &__c[0];}))
# 626 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                ));

    dynamic_print(mainscreen, visibility_buffer);
    /*   

      ------------------------------------------------ end of display if star is visible or not row 8 column 31-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 632 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.reset_cursor();
    /*   

      ------------------------------------------------display GPS data row 0 column 15-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 636 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    // if gps location is valid then display location if else display no gps info
    gps.location.isValid() ? _long_buff.disp = String(my_location.longitude) : _long_buff.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 637 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                                                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 637 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                                                "brak gps"
# 637 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                                                ); &__c[0];}))
# 637 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                                                ));
    // results
    mainscreen.set_cursor(0, 15);
    dynamic_print(mainscreen, _long_buff);
    mainscreen.next_row();
    gps.location.isValid() ? _lat_buff.disp = String(my_location.latitude) : _lat_buff.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 642 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                                             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 642 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                                             "brak gps"
# 642 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                                             ); &__c[0];}))
# 642 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                                             ));
    dynamic_print(mainscreen, _lat_buff);
    /*   

      ------------------------------------------------end of display GPS data -------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 647 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display seconds value on column 15 row 4-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 650 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row();
    /*   _sec_buff.disp = String(int(SEKUNDA));

    dynamic_print(mainscreen, _sec_buff); */
# 653 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    seconds_buff.disp = int(SEKUNDA);
    dynamic_print(mainscreen, seconds_buff);
    //EEPROM::dynamic_print_eeprom(mainscreen, (int)SEKUNDA, EEPROM::addresses::second);
    /*   

      ------------------------------------------------end of display seconds value on column 15 row 4-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 659 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display star.azimuth value on column 15 row 6-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 662 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row();
    GPS_status ? _star_az_buff.disp = String(star.azimuth) : _star_az_buff.disp = "...";
    dynamic_print(mainscreen, _star_az_buff);
    /*   

      ------------------------------------------------end of display star.azimuth value on column 15 row 6-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 668 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display star.altitude value on column 15 row 8-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 671 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row();
    GPS_status ? _star_alt_buff.disp = String(star.altitude) : _star_alt_buff.disp = "...";
    dynamic_print(mainscreen, _star_alt_buff);
    /*   

      ------------------------------------------------end of display star.altitude value on column 15 row 8-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 677 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display year value on column 15 row 10-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 680 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row();

    print((t.year), mainscreen);
    /*   

      ------------------------------------------------end of display year value on column 15 row 10-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 686 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display month value on column 15 row 12-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 689 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row();
    print((t.mon), mainscreen);
    /*   

      ------------------------------------------------end of display month value on column 15 row 12-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 694 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display day value on column 15 row 14-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 697 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row();
    print((t.date), mainscreen);
    /*   

      ------------------------------------------------end of display day value on column 15 row 14-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 702 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display time_utc value on column 15 row 16-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 705 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row();
    /*     _time_buff.disp = (String)TIME;

    dynamic_print(mainscreen, _time_buff); */
# 708 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    UTC_time_buffer.disp = TIME;
    dynamic_print(mainscreen, UTC_time_buffer);
    //EEPROM::dynamic_print_eeprom(mainscreen, TIME, EEPROM::addresses::time_utc);

    /*   

      ------------------------------------------------end of display time_utc value on column 15 row 16-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 715 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display local time name on column 0 row 18-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 718 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    // save previous cursor column
    int previous_column = mainscreen.column; //
    mainscreen.next_row();
    mainscreen.column = 0;

    print((reinterpret_cast<const __FlashStringHelper *>(
# 723 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 723 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "czas lokalny"
# 723 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 723 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), mainscreen);
    /*   

      ------------------------------------------------end of display local time name on column 0 row 18-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 727 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display local time value on column 20 row 18-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 730 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.column = previous_column;
    mainscreen.next_column(5);

    char disps[6];

    t.min < 10 ? snprintf(disps, sizeof(disps), "%d:0%d", t.hour, t.min) : snprintf(disps, sizeof(disps), "%d:%d", t.hour, t.min);

    _local_time_buff.disp = String(disps);
    dynamic_print(mainscreen, _local_time_buff);
    /*   

      ------------------------------------------------end of display local time value on column 20 row 18-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 742 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    /*   

      ------------------------------------------------display if ready to calibrate data on column 0 row 20-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 745 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    mainscreen.next_row();
    mainscreen.column = 0;
    gps.time.isValid() ? _calibrate_buff.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 747 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                               (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 747 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                               "{1}"
# 747 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                               ); &__c[0];}))
# 747 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                               )) : _calibrate_buff.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 747 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                                         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 747 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                                         "{0}"
# 747 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                                                                         ); &__c[0];}))
# 747 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                                                                         ));
    dynamic_print(mainscreen, _calibrate_buff);
    /*   

      ------------------------------------------------end of display if ready to calibrate data on column 0 row 20-------------------------------------------------------------------------------------------------------------------------------------------------------------

    */
# 752 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
    //
    // set cursor to 0.0
    mainscreen.reset_cursor();

    //}
}
#pragma endregion mainscreen
void clear_all_buffers()
{

    az_buff.clear_buffer();
    ra_buff.clear_buffer();
    //laser_angle_buff.clear_buffer();
    dec_buff.clear_buffer();
    visibility_buffer.clear_buffer();
    motor1_ang_buff.clear_buffer();
    motor2_ang_buff.clear_buffer();
    // _time_buff.clear_buffer();
    _lat_buff.clear_buffer();

    _long_buff.clear_buffer();
    _star_alt_buff.clear_buffer();
    _star_az_buff.clear_buffer();

    // _sec_buff.clear_buffer();
    _calibrate_buff.clear_buffer();
    // dodanaj wyświetlanie czasu
    seconds_buff.clear_buffer();
    UTC_time_buffer.clear_buffer();

    _local_time_buff.clear_buffer();

    //
}
/** 

* @brief function displays  data on TFT display:

* @tparam T it can be any type

* @param message - this is a string message to clear from TFT display

* @param column - column on tft its x vector

* @param row - row on tft  translate to y vector

* @param textsize 

* @return nothing

*/
# 795 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
template <class T>
void TFT_dispStr(T message, int column, int row, uint8_t textsize)
{

    TFTscreen->setTextSize(textsize);
    TFTscreen->setTextColor(0xFFFF /*|< WHITE color for drawing graphics*/);
    TFTscreen->setCursor(column, row);
    TFTscreen->print(message);
}
/** 

 * @brief function used to clear previously displayed string default values:

*  @tparam T it can be any type

* @param message - this is a string message to clear from TFT display

* @param column - column on tft its x vector

* @param row - row on tft  translate to y vector

* @param textsize 

* @return nothing

*/
# 813 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
template <class T>
void TFT_clear(T message, int column, int row, uint8_t textsize)
{

    TFTscreen->setTextSize(textsize);
    TFTscreen->setTextColor(0x0000 /*|< BLACK color for drawing graphics*/);
    TFTscreen->setCursor(column, row);
    TFTscreen->print(message);
}
#pragma region init_procedure
void clear_exit_disp()
{
    displayconfig boot_init_disp;
    boot_init_disp.reset_cursor();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 827 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 827 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "instrukcja:"
# 827 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 827 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.next_row();
    clear("EQ-", boot_init_disp);
    boot_init_disp.next_row();
    clear("+", boot_init_disp);
    boot_init_disp.next_row();
    clear("-", boot_init_disp);
    boot_init_disp.next_row();
    clear("play", boot_init_disp);
    boot_init_disp.next_row();
    clear("0", boot_init_disp);
    boot_init_disp.next_row();
    int prev_column = boot_init_disp.column;
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 840 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 840 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "ostatnia lokalizacja"
# 840 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 840 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.next_column(34);
    clear((EEPROM::read<float>(EEPROM::addresses::lat)), boot_init_disp);
    boot_init_disp.next_column(12);
    clear((EEPROM::read<float>(EEPROM::addresses::longitude)), boot_init_disp);
    boot_init_disp.column = prev_column;
    boot_init_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 847 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 847 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "ostatnia gwiazda"
# 847 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 847 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.next_column(34);
    clear((EEPROM::read<float>(EEPROM::addresses::ra)), boot_init_disp);
    boot_init_disp.next_column(12);
    clear((EEPROM::read<float>(EEPROM::addresses::dec)), boot_init_disp);
    boot_init_disp.set_cursor(36, 0);
    clear("1-", boot_init_disp);
    boot_init_disp.set_cursor(36, 4);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 855 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 855 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "zacznij sledzic gwiazde"
# 855 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 855 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.reset_cursor();
    boot_init_disp.set_cursor(0, 8);
    boot_init_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 859 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 859 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "Ustw.mag.deklinacje"
# 859 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 859 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 861 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 861 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "twoja lok."
# 861 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 861 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 863 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 863 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kalibracja pozycji urzadzenia"
# 863 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 863 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 865 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 865 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "potwierdz/kontynuuj"
# 865 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 865 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 867 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 867 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wsp. gwiazdy"
# 867 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 867 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), boot_init_disp);
    boot_init_disp.reset_cursor();
    print_boot_init_once.Reset();
}
void boot_init_exit_tracking_mode()
{
    clear_exit_disp();
    read_compass();
    updateAccel();
    readGPS();

    new_starting_position();

    continous_tracking = true;
    mode = GETTING_STAR_LOCATION;
}
void boot_init_exit_func1()
{
    mode = SELECT_OFFSET;
    clear_exit_disp();
}
void set_true_confirm()
{
    manual_calibration = false;
    continous_tracking = false;
    read_compass();
    updateAccel();
    readGPS();
    if (!manual_calibration)
    {
        new_starting_position();
    }
    clear_exit_disp();
    mode = GETTING_STAR_LOCATION;
}
void boot_init_exit_func2()
{
    mode = SETTINGS;
    clear_exit_disp();
}
void boot_init_exit_func3()
{
    mode = EDIT_LAT;
    clear_exit_disp();
}
void boot_init_exit_func4()
{
    mode = CALIBRATE_POSITION;
    clear_exit_disp();
}
void boot_init_procedure()
{
    displayconfig boot_init_disp;
    boot_init_disp.reset_cursor();
    load_data_from_eeprom.Run([&]()
                              {
                                  offsets::minimal_deg_alt_to_move = EEPROM::read<float>(EEPROM::addresses::min_Alt_diff_to_move);
                                  offsets::minimal_deg_az_to_move = EEPROM::read<float>(EEPROM::addresses::min_Az_diff_to_move);
                              });
    void_func exit_functions[6] = {set_true_confirm, boot_init_exit_func1, boot_init_exit_func2, boot_init_exit_func3, boot_init_exit_func4, boot_init_exit_tracking_mode};
    uint8_t commands[6] = {0x43, 0x9, 0x16, 0x15, 0x7, 0xC};
    size_t number_of_functions = sizeof(commands);
    print_boot_init_once.Run([&]()
                             {
                                 boot_init_disp.reset_cursor();
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 932 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 932 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "instrukcja:"
# 932 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 932 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 boot_init_disp.next_row();
                                 print("EQ-", boot_init_disp);
                                 boot_init_disp.next_row();
                                 print("+", boot_init_disp);
                                 boot_init_disp.next_row();
                                 print("-", boot_init_disp);
                                 boot_init_disp.next_row();
                                 print("play", boot_init_disp);
                                 boot_init_disp.next_row();
                                 print("0", boot_init_disp);
                                 //display recent search
                                 boot_init_disp.next_row();
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 945 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 945 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "ostatnia lokalizacja"
# 945 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 945 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 int prev_column = boot_init_disp.column;
                                 boot_init_disp.next_column(34);
                                 print((EEPROM::read<float>(EEPROM::addresses::lat)), boot_init_disp);
                                 boot_init_disp.next_column(12);
                                 print((EEPROM::read<float>(EEPROM::addresses::longitude)), boot_init_disp);
                                 boot_init_disp.column = prev_column;
                                 boot_init_disp.next_row();
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 953 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 953 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "ostatnia gwiazda"
# 953 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 953 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 boot_init_disp.next_column(34);
                                 print((EEPROM::read<float>(EEPROM::addresses::ra)), boot_init_disp);
                                 boot_init_disp.next_column(12);
                                 print((EEPROM::read<float>(EEPROM::addresses::dec)), boot_init_disp);

                                 //display recent search

                                 boot_init_disp.set_cursor(36, 0);
                                 print("1-", boot_init_disp);
                                 boot_init_disp.set_cursor(36, 4);
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 964 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 964 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "zacznij sledzic gwiazde"
# 964 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 964 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 boot_init_disp.reset_cursor();
                                 boot_init_disp.set_cursor(0, 8);
                                 boot_init_disp.next_row();
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 968 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 968 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "Ustw.mag.deklinacje"
# 968 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 968 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 boot_init_disp.next_row();
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 970 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 970 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "twoja lok."
# 970 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 970 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 boot_init_disp.next_row();
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 972 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 972 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "kalibracja pozycji urzadzenia"
# 972 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 972 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 boot_init_disp.next_row();
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 974 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 974 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "potwierdz/kontynuuj"
# 974 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 974 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 boot_init_disp.next_row();
                                 print((reinterpret_cast<const __FlashStringHelper *>(
# 976 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 976 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      "wsp. gwiazdy"
# 976 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                      ); &__c[0];}))
# 976 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                      )), boot_init_disp);
                                 boot_init_disp.reset_cursor();
                             });
    remote_input_handler_selector(exit_functions, commands, number_of_functions);
}
#pragma endregion init_procedure
void new_starting_position()
{
    //todo : define this constatns for motors they may differ significantly // done
    if (automatic_calibration)
    {
        starting_position_az = my_location.azimuth * constants::motor1_gear_ratio;
        starting_position_alt = pointing_altitude * constants::motor2_gear_ratio;
        motor1.set_position(starting_position_az);
        motor2.set_position(starting_position_alt);
    }
    else
    {
        starting_position_az = offsets::magnetic_declination * constants::motor1_gear_ratio;
        starting_position_alt = pointing_altitude * constants::motor2_gear_ratio;
        motor1.set_position(starting_position_az);
        motor2.set_position(starting_position_alt);
    }
}
uint8_t *decodeIRfun()
{
    bool command_flag = false;

    if (IR->decode())
    {

        for (auto &command : pilot_commands)
        {
            if (IR->decodedIRData.command == command)
            {

                IR->resume();
                command_flag = true;
                IR->decodedIRData.command = 0x00;

                return &command;
            }
        }

        IR->resume();
    }
    if (command_flag == false)
    {

        return &_no_command_decoded;
    }
}
#pragma region editing_ra_dec
void entering_dec_exit_handle()
{
    displayconfig boot_disp;
    TFT_clear((reinterpret_cast<const __FlashStringHelper *>(
# 1032 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1032 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             "wprowadz deklinacje gwiazdy"
# 1032 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             ); &__c[0];}))
# 1032 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             )), boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.row += 30;
    TFT_clear(input_DEC, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.reset_cursor();
    star.declination = input_DEC.toFloat();
    EEPROM::write(EEPROM::addresses::dec, star.declination);
    entering_DEC = true;
    entering_RA ? mode = INIT_PROCEDURE : mode = EDIT_RA;
    if (mode == INIT_PROCEDURE)
        clear_all_buffers();
}
void entering_ra_exit_handle()
{
    displayconfig boot_disp;
    TFT_clear((reinterpret_cast<const __FlashStringHelper *>(
# 1046 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1046 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             "wprowadz rektascensje gwiazdy"
# 1046 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             ); &__c[0];}))
# 1046 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             )), boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.row += 30;
    TFT_clear(input_RA, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.reset_cursor();
    star.right_ascension = input_RA.toFloat();
    EEPROM::write(EEPROM::addresses::ra, star.right_ascension);
    entering_RA = true;
    entering_DEC ? mode = INIT_PROCEDURE : mode = EDIT_DEC;
    if (mode == INIT_PROCEDURE)
        clear_all_buffers();
}
void edit_Ra_Dec() // todo : make interface for entering Ra and Dec after booting *done
{
    displayconfig boot_disp;
    boot_disp.reset_cursor();

    TFT_dispStr((reinterpret_cast<const __FlashStringHelper *>(
# 1062 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1062 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               "1- RA"
# 1062 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               ); &__c[0];}))
# 1062 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               )), boot_disp.column, boot_disp.row, boot_disp.textsize);
    TFT_dispStr((reinterpret_cast<const __FlashStringHelper *>(
# 1063 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1063 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               "2- DEC"
# 1063 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               ); &__c[0];}))
# 1063 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               )), boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
    TFT_dispStr((reinterpret_cast<const __FlashStringHelper *>(
# 1064 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1064 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               "play- zakoncz"
# 1064 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               ); &__c[0];}))
# 1064 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               )), boot_disp.column, 40, boot_disp.textsize);

    if (*decodeIRfun() == 0xC)
    {
        TFT_clear((reinterpret_cast<const __FlashStringHelper *>(
# 1068 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1068 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "1- RA"
# 1068 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1068 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), boot_disp.column, boot_disp.row, boot_disp.textsize);
        TFT_clear((reinterpret_cast<const __FlashStringHelper *>(
# 1069 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1069 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "2- DEC"
# 1069 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1069 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
        TFT_clear((reinterpret_cast<const __FlashStringHelper *>(
# 1070 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1070 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "play- zakoncz"
# 1070 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1070 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), boot_disp.column, boot_disp.row + 40, boot_disp.textsize);

        mode = EDIT_RA;
    }
    else if (*decodeIRfun() == 0x18)
    {
        TFT_clear((reinterpret_cast<const __FlashStringHelper *>(
# 1076 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1076 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "1- RA"
# 1076 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1076 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), boot_disp.column, boot_disp.row, boot_disp.textsize);
        TFT_clear((reinterpret_cast<const __FlashStringHelper *>(
# 1077 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1077 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "2- DEC"
# 1077 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1077 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), boot_disp.column, boot_disp.row + 20, boot_disp.textsize);
        TFT_clear((reinterpret_cast<const __FlashStringHelper *>(
# 1078 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1078 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "play- zakoncz"
# 1078 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1078 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), boot_disp.column, boot_disp.row + 40, boot_disp.textsize);

        mode = EDIT_DEC;
    }
}
void edit_ra()
{
    displayconfig boot_disp;
    TFT_dispStr((reinterpret_cast<const __FlashStringHelper *>(
# 1086 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1086 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               "wprowadz rektascensje gwiazdy"
# 1086 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               ); &__c[0];}))
# 1086 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               )), boot_disp.column, boot_disp.row, boot_disp.textsize);

    boot_disp.row += 30;
    TFT_dispStr(input_RA, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.reset_cursor();
    uint8_t exit_commands[1] = {0x43};
    void_func exit_functions[1] = {entering_ra_exit_handle};
    size_t number_of_functions = sizeof(exit_commands);
    remote_input_handler_str(exit_functions, input_RA, exit_commands, deleteallinput, number_of_functions);
    boot_disp.reset_cursor();
}
void edit_dec()
{
    displayconfig boot_disp;
    TFT_dispStr((reinterpret_cast<const __FlashStringHelper *>(
# 1100 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1100 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               "wprowadz deklinacje gwiazdy"
# 1100 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
               ); &__c[0];}))
# 1100 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
               )), boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.row += 30;
    deleteallinput = boot_disp;
    TFT_dispStr(input_DEC, boot_disp.column, boot_disp.row, boot_disp.textsize);
    boot_disp.reset_cursor();
    uint8_t exit_commands[1] = {0x43};
    void_func exit_functions[1] = {entering_dec_exit_handle};
    size_t number_of_functions = sizeof(exit_commands);
    remote_input_handler_str(exit_functions, input_DEC, exit_commands, deleteallinput, number_of_functions);
    boot_disp.reset_cursor();
}
#pragma endregion editing_ra_dec

#pragma region offset_selectscrn
void offset_select_clear_screen()
{
    offsets_screen.reset_cursor();
    clear("1-", offsets_screen);
    offsets_screen.next_row();
    clear("2-", offsets_screen);
    offsets_screen.next_row();
    clear("3-", offsets_screen);
    offsets_screen.reset_cursor();
    offsets_screen.next_column(3);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1124 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1124 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz offset akcelerometru"
# 1124 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1124 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), offsets_screen);
    offsets_screen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1126 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1126 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz offset azymutu"
# 1126 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1126 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), offsets_screen);
    offsets_screen.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1128 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1128 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "edytuj reakcje silnikow"
# 1128 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1128 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), offsets_screen);
    offsets_screen.reset_cursor();
}
void offset_select_remote_exit_play()
{
    offset_select_clear_screen();
    mode = GETTING_STAR_LOCATION;
}
void offset_select_remote_exit_one()
{
    offset_select_clear_screen();
    mode = OFFSET_EDIT;
    offset_edit_mode = offset_editing::MAGNETIC;
}
void offset_select_remote_exit_tracking_conditions()
{
    offset_select_clear_screen();
    mode = OFFSET_EDIT;
    run_tr_scr_once.Reset();
    offset_edit_mode = offset_editing::TRACKING_CONDITIONS;
}

void offset_select() // todo: let user enter all offsets independently from this set in program
{
    offsets_screen.reset_cursor();
    print("1-", offsets_screen);
    offsets_screen.next_row();
    print("2-", offsets_screen);
    offsets_screen.next_row();
    print("3-", offsets_screen);
    offsets_screen.reset_cursor();
    offsets_screen.next_column(3);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1160 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1160 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz offset akcelerometru"
# 1160 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1160 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), offsets_screen);
    offsets_screen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1162 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1162 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz offset azymutu"
# 1162 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1162 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), offsets_screen);
    offsets_screen.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1164 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1164 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "edytuj reakcje silnikow"
# 1164 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1164 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), offsets_screen);
    offsets_screen.reset_cursor();
    void_func exit_func[4] = {offset_select_remote_exit_one, offset_select_remote_exit_one, offset_select_remote_exit_tracking_conditions, offset_select_remote_exit_play};
    uint8_t exit_commands[4] = {0xC, 0x18, 0x5E, 0x43};
    size_t number_of_functions = sizeof(exit_commands);
    remote_input_handler_selector(exit_func, exit_commands, number_of_functions);
}

#pragma endregion offset_selectscrn
#pragma region display_functions
template <class T>
void clear(T sentence, displayconfig &cnfg)
{
    TFT_clear(sentence, cnfg.column, cnfg.row, cnfg.textsize);
}
template <class T>
void print(T sentence, displayconfig &cnfg)
{
    TFT_dispStr(sentence, cnfg.column, cnfg.row, cnfg.textsize);
}
template <class T>
void dynamic_print(displayconfig &cnfg, buffers<T> &buffs)
{
    if (buffs.disp != buffs.buff)
    {
        clear(buffs.buff, cnfg);
        buffs.buff = buffs.disp;
        print(buffs.buff, cnfg);
    }

    /*    if (!buffs.disp.equals(buffs.buff))

    {

        clear(buffs.buff, cnfg);

        buffs.buff = buffs.disp;

        print(buffs.buff, cnfg);

    } */
# 1200 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
}
/* void dynamic_print_eeprom_int(displayconfig &cnfg, int val, unsigned int address)

{

    if (startup)

    {

        print(String(val), cnfg);

        EEPROM::write(address, val);

    }

    else if (!(EEPROM::read<int>(address) == val))

    {



        clear(String(EEPROM::read<int>(address)), cnfg);

        EEPROM::write(address, val);

        print(String(val), cnfg);

    }

}



void dynamic_print_eeprom_float(displayconfig &cnfg, float val, unsigned int address)

{

    if (startup)

    {

        print(String(val), cnfg);

        EEPROM::write(address, val);

    }

    else if (!(EEPROM::read<float>(address) == val))

    {



        clear(String(EEPROM::read<float>(address)), cnfg);

        EEPROM::write(address, val);

        print(String(val), cnfg);

    }

} */
# 1232 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
template <class T>
void EEPROM::dynamic_print_eeprom(displayconfig &cnfg, T val, unsigned int address)
{
    if (startup)
    {
        print((val), cnfg);
        EEPROM::write(address, val);
    }
    else if (EEPROM::read<T>(address) != val)
    {

        clear((EEPROM::read<T>(address)), cnfg);
        EEPROM::write(address, val);
        print((val), cnfg);
    }
}

#pragma endregion display_function
#pragma region motor_control_functions
bool all_motors_ready_to_move()
{

    if ((continous_tracking != true) && (az_motor_target_reached == false) && (alt_motor_target_reached == false) && (startracker->IsVisible() == true) && (tracking_finished == false))
    {
        return true;
    }
    else if (continous_tracking == true)
    {
        if (startracker->IsVisible() && ((((((azimuth_target - motor1.get_position())>0?(azimuth_target - motor1.get_position()):-(azimuth_target - motor1.get_position())))>=0?(long)((((azimuth_target - motor1.get_position())>0?(azimuth_target - motor1.get_position()):-(azimuth_target - motor1.get_position())))+0.5):(long)((((azimuth_target - motor1.get_position())>0?(azimuth_target - motor1.get_position()):-(azimuth_target - motor1.get_position())))-0.5)) >= offsets::minimal_deg_az_to_move) || (((((altitude_target - motor2.get_position())>0?(altitude_target - motor2.get_position()):-(altitude_target - motor2.get_position())))>=0?(long)((((altitude_target - motor2.get_position())>0?(altitude_target - motor2.get_position()):-(altitude_target - motor2.get_position())))+0.5):(long)((((altitude_target - motor2.get_position())>0?(altitude_target - motor2.get_position()):-(altitude_target - motor2.get_position())))-0.5)) >= offsets::minimal_deg_alt_to_move)))
        {
            if ((((((azimuth_target - motor1.get_position())>0?(azimuth_target - motor1.get_position()):-(azimuth_target - motor1.get_position())))>=0?(long)((((azimuth_target - motor1.get_position())>0?(azimuth_target - motor1.get_position()):-(azimuth_target - motor1.get_position())))+0.5):(long)((((azimuth_target - motor1.get_position())>0?(azimuth_target - motor1.get_position()):-(azimuth_target - motor1.get_position())))-0.5)) >= offsets::minimal_deg_az_to_move))
                az_motor_target_reached = false;
            if ((((((altitude_target - motor2.get_position())>0?(altitude_target - motor2.get_position()):-(altitude_target - motor2.get_position())))>=0?(long)((((altitude_target - motor2.get_position())>0?(altitude_target - motor2.get_position()):-(altitude_target - motor2.get_position())))+0.5):(long)((((altitude_target - motor2.get_position())>0?(altitude_target - motor2.get_position()):-(altitude_target - motor2.get_position())))-0.5)) >= offsets::minimal_deg_alt_to_move))
                alt_motor_target_reached = false;
            return true;
        }
        else
            return false;
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
    tracking_finished = false;
    entering_RA = false;
    entering_DEC = false;
    motor1.target_reached(true);
    motor2.target_reached(true);
}

void Az_engine() //need to be in some standalone function cuz it is not attached to pin interuppt
{
    az_motor_target_reached = false;

    motor1.start();

    if (motor1.target_reached())
    {
        az_motor_target_reached = true;

        alt_motor_target_reached ? mode = GETTING_STAR_LOCATION : mode = MOVEMOTOR2;
    }
}
void Alt_engine()
{
    alt_motor_target_reached = false;
    motor2.start();

    if (motor2.target_reached())
    {

        alt_motor_target_reached = true;

        az_motor_target_reached ? mode = GETTING_STAR_LOCATION : mode = MOVEMOTOR1;
        if (mode == GETTING_STAR_LOCATION && continous_tracking == false)
        {
            displayconfig *newcursor = new displayconfig;
            delay(200);
            //clear main disp
            clearDisplay();
            clear_all_buffers();
            delay(100);
            // print star found sccessfully
            newcursor->reset_cursor();
            print((reinterpret_cast<const __FlashStringHelper *>(
# 1322 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1322 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "znaleziono gwiazde"
# 1322 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1322 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), *newcursor);
            newcursor->next_row(15);
            print((reinterpret_cast<const __FlashStringHelper *>(
# 1324 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1324 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "azymut"
# 1324 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1324 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )) /*short from universal azimuth*/, *newcursor);
            newcursor->next_column(15);
            print((reinterpret_cast<const __FlashStringHelper *>(
# 1326 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1326 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "wysokosc"
# 1326 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1326 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), *newcursor);
            newcursor->column = 0;
            newcursor->next_row();
            print(startracker->get_star_Azimuth(), *newcursor);
            newcursor->next_column(15);
            print(startracker->get_star_Altitude(), *newcursor);

            delay(5000);
            //clear message
            newcursor->reset_cursor();
            clear((reinterpret_cast<const __FlashStringHelper *>(
# 1336 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1336 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "znaleziono gwiazde"
# 1336 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1336 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), *newcursor);
            newcursor->next_row(15);
            clear((reinterpret_cast<const __FlashStringHelper *>(
# 1338 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1338 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "azymut"
# 1338 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1338 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )) /*short from universal azimuth*/, *newcursor);
            newcursor->next_column(15);
            clear((reinterpret_cast<const __FlashStringHelper *>(
# 1340 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1340 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 "wysokosc"
# 1340 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                 ); &__c[0];}))
# 1340 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                 )), *newcursor);
            newcursor->column = 0;
            newcursor->next_row();
            clear(startracker->get_star_Azimuth(), *newcursor);
            newcursor->next_column(15);
            clear(startracker->get_star_Altitude(), *newcursor);
            delay(1000);
            // delete pointer
            delete newcursor;
        }
        tracking_finished = true;
    }
}

#pragma endregion motor_control_functions
void offset_disp_exit_procedure()
{
    displayconfig edit_magnetic_var;
    offset_edit_mode = offset_editing::TIME;
    offsets::magnetic_variation = input_MAG_DEC.toFloat();
    offsets::magnetic_declination = input_MAG_DEC.toFloat();
    edit_magnetic_var.reset_cursor();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1362 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1362 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "Ustw.mag.deklinacje"
# 1362 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1362 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), edit_magnetic_var);
    edit_magnetic_var.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1364 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1364 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "magnetyczna deklinacja"
# 1364 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1364 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), edit_magnetic_var);
    edit_magnetic_var.next_row();
    clear(input_MAG_DEC, edit_magnetic_var);

    automatic_calibration ? mode = GETTING_STAR_LOCATION : mode = MANUAL_CALIBRATION;
}
#pragma region exit_procedures_tracking_conditions_screen

void inputscreenclear()
{
    if (Az_conditions)
    {
        displayconfig tracking_conditions_disp;
        tracking_conditions_disp.reset_cursor();
        print((reinterpret_cast<const __FlashStringHelper *>(
# 1378 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1378 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             "wprowadz w stopniach"
# 1378 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             ); &__c[0];}))
# 1378 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             )), tracking_conditions_disp);
        tracking_conditions_disp.next_row();
        clear((reinterpret_cast<const __FlashStringHelper *>(
# 1380 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1380 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             "azymut"
# 1380 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             ); &__c[0];}))
# 1380 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             )) /*short from universal azimuth*/, tracking_conditions_disp);
        tracking_conditions_disp.next_row();
        clear(stringbufferTC.disp, tracking_conditions_disp);
        tracking_conditions_disp.reset_cursor();
    }
    else if (Alt_conditions)
    {
        displayconfig tracking_conditions_disp;
        tracking_conditions_disp.reset_cursor();
        clear((reinterpret_cast<const __FlashStringHelper *>(
# 1389 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1389 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             "wprowadz w stopniach"
# 1389 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             ); &__c[0];}))
# 1389 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             )), tracking_conditions_disp);
        tracking_conditions_disp.next_row();
        clear((reinterpret_cast<const __FlashStringHelper *>(
# 1391 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1391 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             "wysokosc"
# 1391 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             ); &__c[0];}))
# 1391 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             )), tracking_conditions_disp);
        tracking_conditions_disp.next_row();
        clear(stringbufferTC.disp, tracking_conditions_disp);
        tracking_conditions_disp.reset_cursor();
    }
}
void clear_not_selected_screen()
{
    displayconfig tracking_conditions_disp;
    tracking_conditions_disp.reset_cursor();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1401 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1401 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "co chcesz edytowac?"
# 1401 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1401 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), tracking_conditions_disp);
    tracking_conditions_disp.next_row();
    //option 1 select azimuth press 1
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1404 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1404 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 1404 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1404 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, tracking_conditions_disp);
    int previous_cursorp_col = tracking_conditions_disp.column;
    int previous_cursorp_row = tracking_conditions_disp.row;
    tracking_conditions_disp.next_column(10);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1408 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1408 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wcisnij 1"
# 1408 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1408 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), tracking_conditions_disp);
    //option 2 select altitude press 2
    tracking_conditions_disp.set_cursor(previous_cursorp_row, previous_cursorp_col);
    tracking_conditions_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1412 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1412 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wysokosc"
# 1412 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1412 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), tracking_conditions_disp);
    tracking_conditions_disp.next_column(15);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1414 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1414 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wcisnij 2"
# 1414 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1414 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), tracking_conditions_disp);
    tracking_conditions_disp.reset_cursor();
    //display current eeprom memory data
    tracking_conditions_disp.set_cursor(25, 0);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1418 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1418 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 1418 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1418 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, tracking_conditions_disp);
    tracking_conditions_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1420 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1420 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wysokosc"
# 1420 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1420 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), tracking_conditions_disp);
    tracking_conditions_disp.set_cursor(25, 15);
    clear(EEPROM::read<float>(EEPROM::min_Az_diff_to_move), tracking_conditions_disp);
    tracking_conditions_disp.next_row();
    clear(EEPROM::read<float>(EEPROM::min_Alt_diff_to_move), tracking_conditions_disp);
    tracking_conditions_disp.reset_cursor();
    run_tr_scr_once.Reset();
}
void tracking_conditions_confirm()
{
    tracking_not_selected = true;
    Az_conditions = false;
    Alt_conditions = false;
    clear_not_selected_screen();
    mode = INIT_PROCEDURE;
}
void tracking_conditions_select_azimuth()
{
    tracking_not_selected = false;
    Az_conditions = true;
    Alt_conditions = false;
    clear_not_selected_screen();
}
void tracking_conditions_select_altitude()
{
    tracking_not_selected = false;
    Alt_conditions = true;
    Az_conditions = false;
    clear_not_selected_screen();
}
void exit_conditions()
{
    //transfer input string to int
    if (Alt_conditions)
    {

        if ((input_str_tracking_conditions.toFloat() * constants::motor2_gear_ratio) >= 1)
        {
            EEPROM::write(EEPROM::addresses::min_Alt_diff_to_move, (input_str_tracking_conditions.toFloat() * constants::motor2_gear_ratio));
        }
        else
        {
            EEPROM::write(EEPROM::addresses::min_Alt_diff_to_move, (float)1);
        }
    }
    else if (Az_conditions)
    {
        if ((input_str_tracking_conditions.toFloat() * constants::motor1_gear_ratio) >= 1)
        {
            EEPROM::write(EEPROM::addresses::min_Az_diff_to_move, (input_str_tracking_conditions.toFloat() * constants::motor1_gear_ratio));
        }
        else
        {
            EEPROM::write(EEPROM::addresses::min_Az_diff_to_move, (float)1);
        }
    }
    //reset all markers and go back to selectionscreen
    inputscreenclear();
    tracking_not_selected = true;
    Az_conditions = false;
    Alt_conditions = false;
    stringbufferTC.clear_buffer();
    load_data_from_eeprom.Reset();
    input_str_tracking_conditions = "";
}
#pragma endregion exit_procedures_tracking_conditions_screen
void edit_minimum_Az_deg()
{

    //print current az input
    tracking_conditions_disp.reset_cursor();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1491 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1491 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz w stopniach"
# 1491 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1491 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), tracking_conditions_disp);
    tracking_conditions_disp.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1493 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1493 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 1493 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1493 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, tracking_conditions_disp);
    tracking_conditions_disp.next_row();
    stringbufferTC.disp = input_str_tracking_conditions;
    dynamic_print(tracking_conditions_disp, stringbufferTC);
    tracking_conditions_disp.reset_cursor();
    void_func functions[1] = {exit_conditions};
    uint8_t commandseletc[1] = {0x43};
    remote_input_handler_str(functions, input_str_tracking_conditions, commandseletc, tracking_conditions_disp, 1);
}
void edit_minimum_Alt_deg()
{
    tracking_conditions_disp.reset_cursor();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1505 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1505 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz w stopniach"
# 1505 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1505 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), tracking_conditions_disp);
    tracking_conditions_disp.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1507 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1507 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wysokosc"
# 1507 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1507 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), tracking_conditions_disp);
    tracking_conditions_disp.next_row();
    stringbufferTC.disp = input_str_tracking_conditions;
    dynamic_print(tracking_conditions_disp, stringbufferTC);
    tracking_conditions_disp.reset_cursor();
    void_func functions[1] = {exit_conditions};
    uint8_t commandseletc[1] = {0x43};
    remote_input_handler_str(functions, input_str_tracking_conditions, commandseletc, tracking_conditions_disp, 1);
}
void input_offsets()
{

    if (offset_edit_mode == offset_editing::MAGNETIC)
    {
        displayconfig edit_magnetic_var;
        edit_magnetic_var.reset_cursor();

        print((reinterpret_cast<const __FlashStringHelper *>(
# 1524 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1524 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             "Ustw.mag.deklinacje"
# 1524 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             ); &__c[0];}))
# 1524 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             )), edit_magnetic_var);
        edit_magnetic_var.next_row();
        print((reinterpret_cast<const __FlashStringHelper *>(
# 1526 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1526 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             "magnetyczna deklinacja"
# 1526 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
             ); &__c[0];}))
# 1526 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
             )), edit_magnetic_var);
        edit_magnetic_var.next_row();
        deleteallinput = edit_magnetic_var;
        print(input_MAG_DEC, edit_magnetic_var);
        uint8_t exit_commands[1] = {0x43};
        void_func exit_functions[1] = {offset_disp_exit_procedure};
        size_t number_of_functions = sizeof(exit_commands);
        remote_input_handler_str(exit_functions, input_MAG_DEC, exit_commands, deleteallinput, number_of_functions);
    }
    else if (offset_edit_mode == offset_editing::TRACKING_CONDITIONS)
    {
        run_tr_scr_once.Run([&]()
                            {
                                tracking_conditions_disp.reset_cursor();
                                print((reinterpret_cast<const __FlashStringHelper *>(
# 1540 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1540 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     "co chcesz edytowac?"
# 1540 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     ); &__c[0];}))
# 1540 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     )), tracking_conditions_disp);
                                tracking_conditions_disp.next_row();
                                //option 1 select azimuth press 1
                                print((reinterpret_cast<const __FlashStringHelper *>(
# 1543 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1543 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     "azymut"
# 1543 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     ); &__c[0];}))
# 1543 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     )) /*short from universal azimuth*/, tracking_conditions_disp);
                                int previous_cursorp_col = tracking_conditions_disp.column;
                                int previous_cursorp_row = tracking_conditions_disp.row;
                                tracking_conditions_disp.next_column(10);
                                print((reinterpret_cast<const __FlashStringHelper *>(
# 1547 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1547 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     "wcisnij 1"
# 1547 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     ); &__c[0];}))
# 1547 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     )), tracking_conditions_disp);
                                //option 2 select altitude press 2
                                tracking_conditions_disp.set_cursor(previous_cursorp_row, previous_cursorp_col);
                                tracking_conditions_disp.next_row();
                                print((reinterpret_cast<const __FlashStringHelper *>(
# 1551 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1551 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     "wysokosc"
# 1551 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     ); &__c[0];}))
# 1551 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     )), tracking_conditions_disp);
                                tracking_conditions_disp.next_column(15);
                                print((reinterpret_cast<const __FlashStringHelper *>(
# 1553 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1553 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     "wcisnij 2"
# 1553 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     ); &__c[0];}))
# 1553 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     )), tracking_conditions_disp);
                                tracking_conditions_disp.reset_cursor();
                                //display current eeprom memory data
                                tracking_conditions_disp.set_cursor(25, 0);
                                print((reinterpret_cast<const __FlashStringHelper *>(
# 1557 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1557 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     "azymut"
# 1557 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     ); &__c[0];}))
# 1557 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     )) /*short from universal azimuth*/, tracking_conditions_disp);
                                tracking_conditions_disp.next_row();
                                print((reinterpret_cast<const __FlashStringHelper *>(
# 1559 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1559 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     "wysokosc"
# 1559 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                                     ); &__c[0];}))
# 1559 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                                     )), tracking_conditions_disp);
                                tracking_conditions_disp.set_cursor(25, 15);
                                print(EEPROM::read<float>(EEPROM::addresses::min_Az_diff_to_move), tracking_conditions_disp);
                                tracking_conditions_disp.next_row();
                                print(EEPROM::read<float>(EEPROM::addresses::min_Alt_diff_to_move), tracking_conditions_disp);
                                tracking_conditions_disp.reset_cursor();
                            });
        void_func options[3] = {tracking_conditions_select_azimuth, tracking_conditions_select_altitude, tracking_conditions_confirm};
        uint8_t command_select[3] = {0xC, 0x18, 0x43};
        remote_input_handler_selector(options, command_select, 3);
    }
}

#pragma region edit_lat_long_functions
// function is called when coresponding command is decoded and then mode exits and performs this task
void exit_lat()
{
    displayconfig lat_long_disp;
    lat_long_disp.reset_cursor();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1578 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1578 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz szerokosc geograficzna"
# 1578 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1578 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), lat_long_disp);
    lat_long_disp.next_row(3);
    clear(input_lat, lat_long_disp);
    my_location.latitude = input_lat.toFloat();
    EEPROM::write(EEPROM::addresses::lat, my_location.latitude);
    lat_long_disp.reset_cursor();
    mode = EDIT_LONG;
}
// function is called when coresponding command is decoded and then mode exits and performs this task
void exit_long()
{
    displayconfig lat_long_disp;
    lat_long_disp.reset_cursor();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 1591 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1591 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz dlugosc geograficzna"
# 1591 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1591 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), lat_long_disp);
    lat_long_disp.next_row(3);
    clear(input_long, lat_long_disp);
    my_location.longitude = input_long.toFloat();
    EEPROM::write(EEPROM::addresses::longitude, my_location.longitude);
    lat_long_disp.reset_cursor();
    automatic_mode = false;
    GPS_status = true;
    clear_all_buffers();
    mode = INIT_PROCEDURE;
}
/* edit_latitude screen in here you input your actuall latitude when gps is not available */
void edit_lat()
{
    displayconfig lat_long_disp;
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1606 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1606 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz szerokosc geograficzna"
# 1606 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1606 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), lat_long_disp);
    lat_long_disp.next_row(3);
    deleteallinput = lat_long_disp;
    print(input_lat, lat_long_disp);
    lat_long_disp.reset_cursor();
    uint8_t exit_commands[1] = {0x43};
    void_func exit_functions[1] = {exit_lat};
    size_t number_of_functions = sizeof(exit_commands);
    remote_input_handler_str(exit_functions, input_lat, exit_commands, deleteallinput, number_of_functions);
}
/* edit_latitude screen in here you input your actuall longitude when gps is not available */
void edit_long()
{
    displayconfig lat_long_disp;
    print((reinterpret_cast<const __FlashStringHelper *>(
# 1620 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1620 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "wprowadz dlugosc geograficzna"
# 1620 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 1620 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), lat_long_disp);
    lat_long_disp.next_row(3);
    deleteallinput = lat_long_disp;
    print(input_long, lat_long_disp);
    lat_long_disp.reset_cursor();
    uint8_t exit_commands[1] = {0x43};
    void_func exit_functions[1] = {exit_long};
    size_t number_of_functions = sizeof(exit_commands);
    remote_input_handler_str(exit_functions, input_long, exit_commands, deleteallinput, number_of_functions);
}
#pragma endregion edit_lat_long_functions
#pragma region Remote_control_functions
const char *command_decoder(uint8_t command)
{
    const char *result = "";
    switch (command)
    {
    case 0x16:

        result = "0";

        break;
    case 0xC:

        result = "1";

        break;
    case 0x18:

        result = "2";

        break;
    case 0x5E:

        result = "3";

        break;
    case 0x8:

        result = "4";

        break;
    case 0x1C:

        result = "5";

        break;
    case 0x5A:

        result = "6";

        break;
    case 0x42:

        result = "7";

        break;
    case 0x52:

        result = "8";

        break;
    case 0x4A:

        result = "9";
        break;

    case 0x9:

        result = ".";
        break;

    case 0x43:

        break;
    case 0x15: // plus clears input line and input string
        result = "clear";

        break;
    case 0x7:
        result = "-";

        break;
    }
    return result;
}
void remote_input_handler_str(void_func *exitprint, String &result, uint8_t *number, displayconfig &cnfg, size_t size)
{
    uint8_t data = *decodeIRfun();
    if (data != 0x00)
    {
        result += command_decoder(data);
        if (command_decoder(data) == "clear")
        {

            clear(result, cnfg);
            result = "";
        }
        for (size_t i = 0; i < size; i++)
        {
            if (number[i] == data)
            {

                exitprint[i]();
                break;
            }
        }
    }

    /*   switch (decodeIRfun())

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

    } */
# 1900 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
}
void remote_input_handler_selector(void_func *exitprint, uint8_t *number, size_t size)
{
    uint8_t data2 = *decodeIRfun();
    if (data2 != 0x00)
    {

        for (size_t i = 0; i < size; i++)
        {
            if (number[i] == data2)
            {

                exitprint[i]();
                break;
            }
        }
    }

    /*  switch (decodeIRfun())

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

        else if (number6 == zero)

            exitprint6();

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

        else if (number6 == one)

            exitprint6();

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

        else if (number6 == two)

            exitprint6();

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

        else if (number6 == three)

            exitprint6();

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

        else if (number6 == four)

            exitprint6();

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

        else if (number6 == five)

            exitprint6();

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

        else if (number6 == six)

            exitprint6();

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

        else if (number6 == seven)

            exitprint6();

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

        else if (number6 == eight)

            exitprint6();

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

        else if (number6 == nine)

            exitprint6();

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

        else if (number6 == EQ)

            exitprint6();

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

        else if (number6 == play)

            exitprint6();



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

        else if (number6 == plus)

            exitprint6();



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

        else if (number6 == minus)

            exitprint6();



        break;

    } */
# 2131 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
}
#pragma endregion Remote_control_functions
#pragma region Position_calibration
void check_gps_accel_compass()
{
    updateAccel();
    readGPS();
    read_compass();
}

void clear_calibration_screen()
{
    calibration_disp.set_cursor(0, 0);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 2144 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2144 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kalibracja pozycji urzadzenia"
# 2144 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2144 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    calibration_disp.set_cursor(4, 0);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 2146 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2146 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kat_lasera"
# 2146 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2146 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    calibration_disp.set_cursor(6, 0);

    clear(laser_ang_buff.disp, calibration_disp);

    calibration_disp.set_cursor(8, 0);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 2152 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2152 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 2152 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2152 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, calibration_disp);
    calibration_disp.set_cursor(10, 0);
    clear(az_buff.disp, calibration_disp);
    az_buff.clear_buffer();
    calibration_disp.set_cursor(14, 0);
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 2157 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2157 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 2157 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2157 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, calibration_disp);
    calibration_disp.set_cursor(14, 10);

    clear(ra_buff.disp, calibration_disp);

    ra_buff.clear_buffer();
    calibration_disp.column = 0;
    calibration_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 2165 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2165 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "1- ustaw urzadzenie recznie"
# 2165 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2165 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);

    calibration_disp.reset_cursor();
}
void position_calibration_exit_func1()
{
    clear_calibration_screen();
    new_starting_position();
    manual_calibration = true;
    clear_all_buffers();
    mode = INIT_PROCEDURE;
}
void position_calibration_exit_cancel()
{
    clear_calibration_screen();
    manual_calibration = false;
    clear_all_buffers();
    mode = INIT_PROCEDURE;
}
void turn_on_off_calibration()
{
    calibration = !calibration;

    RTC_calibration();
}
void position_calibration_exit_manual()
{
    clear_calibration_screen();
    clear_all_buffers();
    automatic_calibration = false;
    mode = OFFSET_EDIT;
    offset_edit_mode = offset_editing::MAGNETIC;
}
void position_calibration_display()

{
    check_gps_accel_compass();
    calibration_disp.set_cursor(0, 0);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 2203 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2203 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kalibracja pozycji urzadzenia"
# 2203 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2203 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    calibration_disp.set_cursor(4, 0);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 2205 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2205 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "kat_lasera"
# 2205 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2205 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    calibration_disp.set_cursor(6, 0);
    laser_ang_buff.disp = pointing_altitude;
    dynamic_print(calibration_disp, laser_ang_buff);
    // EEPROM::dynamic_print_eeprom(calibration_disp, pointing_altitude, EEPROM::addresses::laser_angle);
    calibration_disp.set_cursor(8, 0);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 2211 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2211 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 2211 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2211 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, calibration_disp);
    calibration_disp.set_cursor(10, 0);
    if (smoothHeadingDegrees == 0)
    {
        az_buff.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 2215 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2215 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                      "gotowe, wskazujesz polnoc"
# 2215 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                      ); &__c[0];}))
# 2215 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                      ));
    }
    else
    {
        az_buff.disp = (reinterpret_cast<const __FlashStringHelper *>(
# 2219 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2219 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                      "nie wskazujesz polnocy"
# 2219 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
                      ); &__c[0];}))
# 2219 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
                      ));
    }
    dynamic_print(calibration_disp, az_buff);
    calibration_disp.set_cursor(14, 0);
    print((reinterpret_cast<const __FlashStringHelper *>(
# 2223 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2223 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "azymut"
# 2223 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2223 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )) /*short from universal azimuth*/, calibration_disp);
    calibration_disp.set_cursor(14, 10);
    ra_buff.disp = (smoothHeadingDegrees);
    dynamic_print(calibration_disp, ra_buff);
    calibration_disp.column = 0;
    calibration_disp.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 2229 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2229 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "1- ustaw urzadzenie recznie"
# 2229 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2229 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);

    calibration_disp.reset_cursor();
    void_func exit_func[3] = {position_calibration_exit_func1, position_calibration_exit_cancel, position_calibration_exit_manual};
    uint8_t exit_commands[3] = {0x43, 0x16, 0xC};
    size_t number_of_functions = sizeof(exit_commands);
    remote_input_handler_selector(exit_func, exit_commands, number_of_functions);
}
void clear_manual_calibration_disp()
{
    calibration_disp.reset_cursor();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 2240 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2240 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "uzyj busoli i wskaz polnoc"
# 2240 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2240 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    calibration_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 2242 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2242 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "aby kalibrowac wcisnij play"
# 2242 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2242 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    calibration_disp.next_row();
    clear((reinterpret_cast<const __FlashStringHelper *>(
# 2244 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2244 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "aby wyjsc wcisnij 0"
# 2244 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2244 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
}
void manual_calibration_exit_confirm()
{
    clear_manual_calibration_disp();
    new_starting_position();
    manual_calibration = true;
    automatic_calibration = false; // set to false to enable fully manual calibration
    mode = GETTING_STAR_LOCATION;
}
void manual_calibration_exit_leave()
{
    clear_manual_calibration_disp();
    manual_calibration = false;
    automatic_calibration = true; // set to false to enable fully manual calibration
    clear_all_buffers();

    mode = INIT_PROCEDURE;
}

void manual_calibration_screen()
{
    calibration_disp.reset_cursor();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 2267 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2267 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "uzyj busoli i wskaz polnoc"
# 2267 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2267 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    calibration_disp.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 2269 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2269 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "aby kalibrowac wcisnij play"
# 2269 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2269 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    calibration_disp.next_row();
    print((reinterpret_cast<const __FlashStringHelper *>(
# 2271 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 2271 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         "aby wyjsc wcisnij 0"
# 2271 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp" 3
         ); &__c[0];}))
# 2271 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
         )), calibration_disp);
    void_func exit_func[2] = {manual_calibration_exit_confirm, manual_calibration_exit_leave};
    uint8_t exit_commands[2] = {0x43, 0x16};
    size_t number_of_functions = sizeof(exit_commands);
    remote_input_handler_selector(exit_func, exit_commands, number_of_functions);
}
void decodeIR_remote()
{

    static Simpletimer::callback IRremote_exit_functions[4] = {go_to_main, reset_all_go_to_main, switch_laser, turn_on_off_calibration};
    static uint8_t IRremote_exit_commands[4] = {0x15, 0x7, 0x16, 0x18};

    remote_input_handler_selector(IRremote_exit_functions, IRremote_exit_commands, (size_t)4);
}
#pragma endregion Position_calibration

#pragma region debugging_mode
# 2339 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\StarTrackV1.cpp"
#pragma endregion debugging_mode
#pragma endregion functions
# 1 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\main.ino"

#pragma region author_note
/**

 * @author @b Natan @b Lisowski @github: @b @natnqweb   @email: @c pythonboardsbeta@gmail.com

 * 

 * */
# 7 "c:\\Users\\Admin\\Documents\\Arduino\\Star_tracking_main\\main.ino"
#pragma endregion author_note

void setup()
{

    initialize_();

    RTC_calibration();
    laser(off);
    ;
}
void loop()
{

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
        decodeIR_remote();

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
        while (Alt_conditions)
            edit_minimum_Alt_deg();
        while (Az_conditions)
            edit_minimum_Az_deg();

        break;
    case MANUAL_CALIBRATION:
        manual_calibration_screen();
        break;

    default:






        mode = INIT_PROCEDURE;
        break;
    }
}
