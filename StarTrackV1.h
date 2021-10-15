#pragma once
#ifndef StarTrackV1_h
#define StarTrackV1_h
#pragma region includes
#include <Arduino.h>
#include <DS3231.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <IRremote.h>
#include <Adafruit_HMC5883_U.h>
#include <SkyMap.h>
#include <Encoder_Motor_PID.h>
#pragma endregion includes
#pragma region definitions
#pragma region macros_debg
#define DEBUG false // enable or disable debug messages
#ifndef DEBUG
#define DEBUG false
#endif
#if DEBUG == true
#define LOG(x) Serial.println(x)          //send debug message to serial port
#define start_debuging(y) Serial.begin(y) //enable serial port with y baud
#else
#define LOG(x)
#define start_debuging(y)
#endif
#define ENABLE_LED_FEEDBACK 0          //feedback from IRDECODER 0 off 1on
#define USE_DEFAULT_FEEDBACK_LED_PIN 1 //feedback from IRDECODER
#pragma endregion macros_debg
#pragma region i2c_adress
#define accel_address 0x69
#define compass_adress 0x1E
#define other_address 0x57 //was detected in scanning
#define rtc_address 0x68
#pragma endregion i2c_adress

#pragma endregion definitions

#pragma region enumerations
enum class offset_editing //enumeration for editing offset screen
{
    NOT_SET = 0,
    MAGNETIC = 1,
    ACCELEROMETER = 2,
    TIME = 3,
    LOCATION = 4

};
enum pins : const uint8_t
{

    IR_RECEIVE_PIN = 2,
    cs = 28,
    dc = 27,
    rst = 26,
    Laser_pin = 23,
    ENCA = 31,  // YELLOW from polulu encoder
    ENCB = 32,  // WHITE from polulu encoder
    ENCA2 = 33, // YELLOW from polulu encoder
    ENCB2 = 34, // WHITE from polulu encoder
    IN1 = 8,    //A1-A
    IN2 = 9,    //B1-A
    IN1_2 = 10, //A2-A
    IN2_2 = 11  //B2-A

};
enum class modes
{ // program modes

    SETTINGS = 1,
    MAIN = 0,
    GETTING_STAR_LOCATION = 2,
    POINTING_TO_STAR = 3,
    INIT_PROCEDURE = 4,
    OFFSET_EDIT = 5,
    SELECT_OFFSET = 6

};
enum remote_commands : unsigned char // all commands from ir remote
{
    plus = 0x15,
    minus = 0x7,
    play = 0x43,
    EQ = 0x9,
    zero = 0x16,
    one = 0xC,
    two = 0x18,
    three = 0x5E,
    four = 0x8,
    five = 0x1C,
    six = 0x5A,
    seven = 0x42,
    eight = 0x52,
    nine = 0x4A,
    no_command = 0x00

};
enum states : bool
{

    on = 1,
    off = 0

};
#pragma endregion enumerations
#pragma region structures
struct Myposition //struct to store location specific information
{
    float latitude;
    float longitude;
    float azymuth;
    Myposition(degs latitude = 0, degs longitude = 0, degs azymuth = 0);
};

struct Star
{
    degs azymuth;
    degs altitude;
    degs right_ascension;                                                                      //must be in degrees
    degs declination;                                                                          //must be in degrees
    Star(degs azymuth = 0, degs altitude = 0, degs right_ascension = 0, degs declination = 0); //must be in degrees
};
struct simpletimer //struct to manage time dependent tasks
{

    unsigned long before;
    bool timer(unsigned long _time);
};
struct displayconfig
{
    int row = 0;
    int column = 0;
    uint8_t textsize = 2;
    void next_row(int how_many_rows_further = 1, uint8_t pixels = 8)
    {
        this->row += (pixels * how_many_rows_further);
    }
    void next_column(int how_many_columns = 1, uint8_t pixels = 8)
    {
        this->column += (pixels * how_many_columns);
    }
    void reset_cursor()
    {
        this->column = 0;
        this->row = 0;
    }
};
#pragma endregion structures
#pragma region namespaces
namespace offsets
{
    const hrs timezone_offset = 2;     //UTC +2
    const degs magnetic_variation = 6; // 6 degrees due to magnetic declination
    const degs azymuth_offset = -65;   // offset from uneven attachment
    //degs altitude_offset = 0;
    // degs motor_position_offset = 0;
};

namespace refresh // all timer refresh rates here
{
    const unsigned int gps_refresh_rate = 990;         //ms
    const unsigned int calculation_refresh_rate = 950; //ms
    const unsigned int accel_refresh_rate = 50;        //now its only print refresh
    const unsigned int compass_refresh_rate = 1000;    //ms
    const unsigned int ir_refresh_rate = 70;           //ms
    const unsigned int TFT_refresh_rate = 200;         // frequency of reading the IR data in ms
    const unsigned int loading_messenge_refresh = 200; // frequency of reading the IR data in ms
};

namespace constants //some usefull constants to for calibration and configuration
{
    const uint8_t number_of_measurements = 100;
    const double pi = 3.1415926536;
    const uint8_t motor_gear_ratio = 65;
    const uint8_t gear_constant = 5;
    const unsigned int GPSBaud = 9600;
    const unsigned long Serial_Baud = 115200;
    //const unsigned int HALFSTEPS = 4096; // Number of half-steps for a full rotation
    const float kp = 1;
    const float kd = 0.1;
    const float ki = 0.02;

};
#pragma endregion namespaces
#pragma region variables
String input_MAG_DEC;
auto offset_edit_mode = offset_editing::NOT_SET;
auto mode = modes::MAIN;
auto *ss = &Serial3;
unsigned char pilot_commands[] = {plus, minus, play, EQ, zero, one, two, three, four, five, six, seven, eight, nine, no_command};
bool calibration = false;
float day, month, year, TIME, MIN, HOUR, SEKUNDA; //datetime
bool ready_to_move = false;
char printout1[4];                                                                                                                               //uint buffer
String bufferstr, bufferstr2, bufferstr3, bufferstr4, bufferstr5, bufferstr6, bufferstr7, bufferstr8, bufferstr9, bufferstr10, bfstr11, bfstr12; //string buffer
degs pointing_altitude;                                                                                                                          //data from accel
bool laser_state;
degs starting_position_az, starting_position_alt; // calibration starting point for encoder so you dont need to level it every time manually
uint8_t decoded_command = 0x00U;
bool GPS_status = false;
sensors_event_t a, g, temp;
sensors_event_t compass_event;
sensor_t compass_hmcl;
#pragma endregion variables
#pragma region custom_typedefs
typedef void (*void_func)(void);
#pragma endregion custom_typedefs
#pragma region function_prototypes
void movemotors();
void read_compass();
void init_accel();
void initialize_();
void decodeIR();
void RTC_calibration();
void readGPS();
void updateAccel();
void updateDisplay();
void laser(bool on_off);
void TFT_dispStr(String str, int column, int row, uint8_t textsize = 1);
void TFT_clear(String strr, int column, int row, uint8_t textsize = 1);
static void smartDelay(unsigned long ms = 0);
void calculate_starposition();
void submit_data();
void input_offsets();
void Az_engine(float &target);  // function take target to follow and getting it by reference . for azymuth motor
void Alt_engine(float &target); // function take target to follow and getting it by reference . for altitude motor
void boot_init_procedure();
void allign_with_star();
uint8_t decodeIRfun();
bool check_if_calibrated();
degs edit_Ra_Dec();
void mode_selection();
void IRremote_callback(void_func, uint8_t); //function will run only if specific command was detected
void print(String, displayconfig &);
void clear(String, displayconfig &);
void compass_init();
void new_starting_position();
void safety_motor_position_control();
void offset_select();
#if DEBUG
void print_debug_message(int col = 0, int row = 0, uint8_t size = 1);
void debug_rtc();
#endif
#pragma endregion function_prototypes
#endif