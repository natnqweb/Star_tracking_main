
#pragma once

/* 
@natnqweb
*/
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
#include <uEEPROMLib.h>
#include <Motor_PID.h>

#pragma endregion includes
#pragma region definitions

#define right 1
#define left -1
#define up 1
#define down -1
#define EMPTYSTRING ""
#define buffersize 30
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
#pragma region i2c_address
#define accel_address 0x69
#define compass_address 0x1E
#define eeprom_address 0x57 //eeprom  i2c_address
#define rtc_address 0x68
#pragma endregion i2c_address
// under this region all IR_remote commands are stored
#pragma region remote_commands
#define plus 0x15
#define minus 0x7
#define play 0x43
#define EQ 0x9
#define zero 0x16
#define one 0xC
#define two 0x18
#define three 0x5E
#define four 0x8
#define five 0x1C
#define six 0x5A
#define seven 0x42
#define eight 0x52
#define nine 0x4A
#define no_command 0x00
#pragma endregion remote_commands
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
    /* MODE : SETTINGS
    this mode when selected takes user to edditing Right ascension and declination interface where user inputs
    star data
    */
    SETTINGS = 1,
    /* MODE : MAIN
    this mode when selected takes user to edditing Right ascension and declination interface where user inputs
    star data
    */
    MAIN = 0,
    /* MODE : GETTING_STAR_LOCATION
    this mode is responsible for performing all astronomical calculations 
    */
    GETTING_STAR_LOCATION = 2,
    TRACKING_MODE = 3,
    /* MODE : INIT_PROCEDURE
    this mode contains initialization procedure for startracker, 
    it performs operations like:
    -instruction display
    -boot_procedure 
    -go to offset_edit screen
    -previous search information
    -etc.
    */
    INIT_PROCEDURE = 4,
    OFFSET_EDIT = 5,
    SELECT_OFFSET = 6,
    EDIT_RA = 7,
    EDIT_DEC = 8,
    EDIT_LAT = 9,
    EDIT_LONG = 10,
    MOVEMOTOR1 = 11,
    MOVEMOTOR2 = 12,
    /* MODE : DISPLAY_RESULTS
    after moving to motors to destination position display results mode is activated
    */
    DISPLAY_RESULTS = 13,
    /* MODE : CALIBRATE_POSITION 
    this is calibration procedure it is neccesary to calibrate your position at start
    */
    CALIBRATE_POSITION = 14,
    /* MODE : MANUAL_CALIBRATION 
    mode displays information about manual calibration and let you chose magnetic declinatiojn offset
    */
    MANUAL_CALIBRATION = 15

};

enum states : bool
{

    on = 1,
    off = 0

};
#pragma endregion enumerations
#pragma region structures

/* structure created so buffers can be easly erased and stored information for TFT display  */
struct buffers
{
    String buff;
    String disp;
    void clear_buffer();
};
//struct to store location specific information
struct Myposition
{
    float latitude;
    float longitude;
    float azymuth;
    /* constructor */
    Myposition(degs latitude = 0, degs longitude = 0, degs azymuth = 0);
};
//struct to store Star specific information
struct Star
{
    degs azymuth;
    degs altitude;
    degs right_ascension; //must be in degrees
    degs declination;     //must be in degrees
    /* constructor */
    Star(degs azymuth = 0, degs altitude = 0, degs right_ascension = 0, degs declination = 0); //must be in degrees
};
/* display configuration structure its job is to store information about cursor in specific screen 
it contains data about cursor and textsize, it makes navigation on screen easier*/
struct displayconfig
{
    int row = 0;
    int column = 0;
    //default textsize is 2
    uint8_t textsize = 2;
    /*  changes row to row directly under the current cursor postion 
   only if its empty, if user provides data it moves cursor number of given rows*/
    void next_row(int how_many_rows_further = 2, uint8_t pixels = 8);
    /* go to column one char away from current cursor position to right */
    void next_column(int how_many_columns = 1, uint8_t pixels = 8);
    /* sets cursor to 0,0 */
    void reset_cursor();
    /* set row and collumn to given location*/
    void set_cursor(int row, int column, uint8_t pixels = 8);
};
#pragma endregion structures
#pragma region namespaces
/* most important offsets */
namespace offsets
{
    hrs timezone_offset = 1;        //UTC +2
    degs magnetic_variation = -6.5; // 7 degrees due to magnetic declination
    degs magnetic_declination = 6.1;

};
// all timer refresh rates here
namespace refresh
{
    //refreshrate of accel reading
    const unsigned int accel_refresh_rate = 500;
    //   const unsigned int compass_refresh_rate = 1;     //ms
    //const unsigned int ir_refresh_rate = 100;        //ms
    const unsigned int TFT_refresh_rate = 1000; // frequency of reading the IR data in ms

};
/* some usefull constants to for calibration and configuration */
namespace constants
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
char printout1[30]; //uint buffer 240bits 30 bytes

buffers ra_buff, dec_buff, az_buff, visibility_buffer, motor1_ang_buff, motor2_ang_buff, _long_buff, _lat_buff, _day_buff, _year_buff, _star_az_buff, _star_alt_buff, _local_time_buff, _calibrate_buff;
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
bool startup = true;
bool print_boot_init_once = true;
bool automatic_calibration = true;
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
void manual_calibration_screen();
/* read data from compass take measurment and then calculate mean of it 
after that this function create another type of data called smooth reading for not precise but smooth data about current heading */
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
// function take target to follow and getting it by reference . for azymuth motor
void Az_engine();
// function take target to follow and getting it by reference . for altitude motor
void Alt_engine();
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
void turn_on_off_calibration();
//accelerometer initialization function
void init_accel();
//this function clears everything what's inside the updateDisplay function
void clearDisplay();
/* this function displays any string data on TFT display 
default values:
TFT_dispStr(String String_to_display, int cursor_column, int cursor_row, uint8_t textsize = 1);
*/
void TFT_dispStr(String str, int column, int row, uint8_t textsize = 1);
/* function used to clear previously displayed string
default values:
TFT_clear(String String_to_display, int cursor_column, int cursor_row, uint8_t textsize = 1);
*/
void TFT_clear(String strr, int column, int row, uint8_t textsize = 1);
//main function that preforms astronomy calculations based on current time and location
void check_gps_accel_compass();
/*  function that makes user input from remote possible
this function returns recognized command if command is not recognized it returns no_command
no_command is defined as 0 
*/
uint8_t decodeIRfun();

// when value changes refresh display clear previous displayed value and print new one
void dynamic_print(displayconfig &, buffers &);
void print(String, displayconfig &); //this custom function for printing it takes dislayconfig as a parameter to control where the disp cursor is
void clear(String, displayconfig &);
void compass_init();
void new_starting_position();
void safety_motor_position_control();
//memory allocation and access
/* all eeprom functions and adresses are stored under namespace EEPROM */
namespace EEPROM
{ //addresses of variables stored in eeprom
    enum addresses : const uint8_t
    {
        lat = 10,
        longitude = 15,
        ra = 20,
        dec = 25,
        second = 30,
        laser_angle = 34,
        time_utc = 39

    };
    template <class T>
    //returns data from eeprom it can be any type
    T read(unsigned int);
    template <class T>
    //writes any data type to eeprom
    void write(unsigned int, T);
    template <class T>
    //this function is using eeprom as buffer for printing non constant data
    void dynamic_print_eeprom(displayconfig &, T, unsigned int);
};
/* this function handles input data from IR remote,
it is important to recive data in edit modes for example offsets and this function handles not only input strings but 
also exit functions that should be feeded as [] with the given size
example: 
uint8_t exit_command[3]={one,two,three};// command decoded from ir reciever
size_t number_of_commands=sizeof(exit_command);
void_func exit_functions[3]={first_exit_function,second_exit_function,third_exit_function};
remote_input_handler_str(exit_functions,input_string,exit_command,displayconfigbuffer,number_of_commands);
under this circumstances first_exit_function will be performed wehn command one was decoded,
or second_exit_function will be performed if command "two" was decoded and so on...
*/
void remote_input_handler_str(void_func *, String &, uint8_t *, displayconfig &, size_t);
void remote_input_handler_selector(void_func *, uint8_t *, size_t);
//command  decoder transforms decoded command to equivalent const char*
const char *command_decoder(uint8_t);
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
void init_compass_test();
float read_compass_test();
#endif
#pragma endregion function_prototypes
#endif