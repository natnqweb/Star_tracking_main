#pragma once
#ifndef StarTrackV1_h
#define StarTrackV1_h
#pragma region includes
#include <Arduino.h>
#include "language.h"
#include <DS3231.h>
#include <Adafruit_HX8357.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <IRremote.h>
#include <Adafruit_HMC5883_U.h>
#include <SkyMap.h>
#include <Mapf.h>
#include <Simpletimer.h>

#include <Motor_PID.h>

#pragma endregion includes
#pragma region definitions
#define right 1
#define left -1
#define up 1
#define down -1
#define EMPTYSTRING ""
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
    IN1_2 = 10, //A2-b
    IN2_2 = 11  //B2-b

};
enum modes : const uint8_t
{ // program modes

    SETTINGS = 1,
    MAIN = 0,
    GETTING_STAR_LOCATION = 2,
    TRACKING_MODE = 3,
    INIT_PROCEDURE = 4,
    OFFSET_EDIT = 5,
    SELECT_OFFSET = 6,
    EDIT_RA = 7,
    EDIT_DEC = 8,
    EDIT_LAT = 9,
    EDIT_LONG = 10,
    MOVEMOTOR1 = 11,
    MOVEMOTOR2 = 12,
    DISPLAY_RESULTS = 13,
    CALIBRATE_POSITION = 14

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
struct buffers
{
    String buff;
    String disp;
    void clear_buffer();
};
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

struct displayconfig
{
    int row = 0;
    int column = 0;
    uint8_t textsize = 2;
    void next_row(int how_many_rows_further = 2, uint8_t pixels = 8);

    void next_column(int how_many_columns = 1, uint8_t pixels = 8);
    void reset_cursor();
    void set_cursor(int row, int column, uint8_t pixels = 8);
};
#pragma endregion structures
#pragma region namespaces
namespace offsets
{
    hrs timezone_offset = 2;        //UTC +2
    degs magnetic_variation = -6.5; // 7 degrees due to magnetic declination
    degs azymuth_offset = 0;        // offset from uneven attachment
    //degs altitude_offset = 0;
    // degs motor_position_offset = 0;
};

namespace refresh // all timer refresh rates here
{
    // const unsigned int gps_refresh_rate = 1;         //ms
    // const unsigned int calculation_refresh_rate = 1; //ms
    //  const unsigned int accel_refresh_rate = 1;       //now its only print refresh
    //   const unsigned int compass_refresh_rate = 1;     //ms
    //const unsigned int ir_refresh_rate = 100;        //ms
    // const unsigned int TFT_refresh_rate = 1;         // frequency of reading the IR data in ms
    // const unsigned int loading_messenge_refresh = 1; // frequency of reading the IR data in ms
};

namespace constants //some usefull constants to for calibration and configuration
{
    const float number_of_measurements = 64;
    const double pi = 3.1415926536;
    const float motor2_gear_ratio = 7.874;
    const float motor1_gear_ratio = 2.5;
    const unsigned int GPSBaud = 9600;
    const unsigned long Serial_Baud = 115200;
    //const unsigned int HALFSTEPS = 4096; // Number of half-steps for a full rotation
    const float kp1 = 0.84;
    const float kd1 = 1;
    const float ki1 = 0.3;
    const float kp2 = 13;
    const float kd2 = 0.1;
    const float ki2 = 0.01;
    const int motor1_lower_limit = 0;
    const int motor1_upper_limit = 130;
    const int motor2_lower_limit = 136;
    const int motor2_upper_limit = 255;
    const float minimal_deg_diff_to_move = 3;
};
#pragma endregion namespaces
#pragma region variables

auto offset_edit_mode = offset_editing::NOT_SET;
auto mode = modes::MAIN;
unsigned char pilot_commands[] = {plus, minus, play, EQ, zero, one, two, three, four, five, six, seven, eight, nine, no_command};
float day, month, year, TIME, MIN, HOUR, SEKUNDA; //datetime
degs pointing_altitude;                           //data from accel
degs starting_position_az, starting_position_alt; // calibration starting point for encoder so you dont need to level it every time manually
uint8_t decoded_command = 0x00U;
float accelXsum = 0;
float accelYsum = 0;
float accelZsum = 0;
int previousDegree, smoothHeadingDegrees;
#if DEBUG
Simpletimer logtimer;
#endif
#pragma region buffers
//buffers
float previous_azymuth, previous_altitude;
String input_MAG_DEC;
String input_RA, input_DEC, input_lat, input_long;
float azymuth_target = 0, altitude_target = 0;
char printout1[4]; //uint buffer
buffers ra_buff, dec_buff, az_buff, laser_angle_buff, visibility_buffer, motor1_ang_buff, motor2_ang_buff, _long_buff, _lat_buff, _day_buff, _year_buff, _time_buff, _star_az_buff, _star_alt_buff, _sec_buff, _local_time_buff;
//string buffer
#pragma endregion buffers
#pragma region booleans
//booleans and markers
bool tracking_finished = false;
bool laser_state;
bool manual_calibration = false;
bool laser_mode = false;
bool setmode, confirm;
bool calibration = false;
bool az_motor_target_reached = false, alt_motor_target_reached = false;
bool ready_to_move = false;
bool GPS_status = false;
bool entering_DEC = false, entering_RA = false, automatic_mode = true;
bool continous_tracking = false;

#pragma endregion booleans

#pragma region sensors
//sensor events Adafruit_MPU6050 and hmc5883l
sensors_event_t a, g, temp;
sensors_event_t compass_event;
sensor_t compass_hmcl;
#pragma endregion sensors
#pragma region displayconfig
//displayconfiguration structure it contains information about cursor position
displayconfig mainscreen;
displayconfig boot_init_disp;
displayconfig boot_disp;
displayconfig edit_magnetic_var;
displayconfig offsets_screen;
displayconfig lat_long_disp;
displayconfig deleteallinput;
displayconfig star_visibility_disp;
displayconfig calibration_disp;
#pragma endregion displayconfig

#pragma endregion variables
#pragma region custom_typedefs
typedef void (*void_func)(void);
typedef void (*exit_print)(String, displayconfig &);
#pragma endregion custom_typedefs
#pragma region function_prototypes
// its just an empty function that do nothing
void empty_function()
{
}
#pragma region main_functions
// get hmc5883l readings and save them
void read_compass();
// init procedure called at setup
void initialize_();
// custom pilot procedure can be called anywhere in program
void decodeIR_remote();
// that function is called when you need to calibrate rtc when calibrated you dont need it
void RTC_calibration();
// this function contains while loop for reading data from serial port 3 on mega to reed neo6m data
void readGPS();
// updates accelerometer and changes values to degrees to get laser angle
void updateAccel();
// main display that displays all information about star location etc.
void updateDisplay();
// turns on or off laser
void laser(bool on_off);
// main functions that handles calculations and decide whenever startracking is posible
void calculate_starposition();
void Az_engine();  // function take target to follow and getting it by reference . for azymuth motor
void Alt_engine(); // function take target to follow and getting it by reference . for altitude motor
// procedure at the beginning of program it takes place in loop not in setup
void boot_init_procedure();
// when user decide to change offsets and click edit offset he enters input_offsets mode
void input_offsets();
void edit_Ra_Dec();
void edit_dec();
void edit_ra();
//offset selection screen
void offset_select();
void edit_lat();
void edit_long();
// position calibration screen you need to confirm when you are facing north to set calibration data
void position_calibration_display();
#pragma endregion main_functions
void movemotors();
void init_accel();
void clearDisplay();
void TFT_dispStr(String str, int column, int row, uint8_t textsize = 1);
void TFT_clear(String strr, int column, int row, uint8_t textsize = 1);
//main function that preforms astronomy calculations based on current time and location
void check_gps_accel_compass();

uint8_t decodeIRfun();

// when value changes refresh display clear previous displayed value and print new one
void dynamic_print(displayconfig &, buffers &);
void print(String, displayconfig &); //this custom function for printing it takes dislayconfig as a parameter to control where the disp cursor is
void clear(String, displayconfig &);
void compass_init();
void new_starting_position();
void safety_motor_position_control();

// this functions saves in string every clicked button and performs exitfnct when irremote input matches expected command can take up to 3 functions
void remote_input_handler_str(void_func, String &, uint8_t, displayconfig &, void_func exitprint2 = empty_function, uint8_t number2 = 0, void_func exitprint3 = empty_function, uint8_t number3 = 0, void_func exitprint4 = empty_function, uint8_t number4 = 0);
// function that takes void functions as parameters and performs whats inside them only if ir reemote decodes given command can take up to 3 functions
void remote_input_handler_selector(void_func, uint8_t, void_func exitprint2 = empty_function, uint8_t number2 = 0, void_func exitprint3 = empty_function, uint8_t number3 = 0, void_func exitprint4 = empty_function, uint8_t number4 = 0, void_func exitprint5 = empty_function, uint8_t number5 = 0, void_func exitprint6 = empty_function, uint8_t number6 = 0);
//code specific for debuging purposes only if debug not true this code is not visible for compiler
// // true if target  position is reached and false if not

bool all_motors_ready_to_move();
bool reset_ready_to_move_markers();
bool check_if_pointing_at_north();
void clear_all_buffers();

#if DEBUG
void print_debug_message(int col = 0, int row = 0, uint8_t size = 1);
void debug_rtc();
void debug_motors();
displayconfig debug_motor_display;
buffers debugbuffer;
#endif
#pragma endregion function_prototypes
#endif