#pragma once
/**
 * @author @b Natan @b Lisowski @github: @b @natnqweb   @email: @c pythonboardsbeta@gmail.com
 * 
 * */
#define pl F("polish")
//#define eng "english" //uncomment this line and comment line 2 to change language
#pragma region polish
#ifdef pl
#define un_azymuth F("azymut") //short from universal azymuth
#define un_altitude F("wysokosc")
#define un_declination F("deklinacja")
#define un_right_ascension F("rektascensja")
#define un_year F("rok")
#define un_month F("miesiac")
#define un_time_utc F("czas UTC")
#define un_day F("dzien")
#define un_calibration F("kalibracja")
#define un_laser_angle F("kat_lasera")
#define un_second F("sekunda")
#define un_dev_AZ F("Azym.urz.")
#define un_eq F("EQ-")
#define un_lat F("szer.")
#define un_long F("dlug.")
#define un_set_mag_declination F("Ustw.mag.deklinacje")
#define un_magnetic_declination F("magnetyczna deklinacja")
#define un_your_location F("twoja lok.")
#define un_submit_continue F("potwierdz/kontynuuj")
#define un_star_location F("wsp. gwiazdy")
#define un_instruction F("instrukcja:")
#define un_setting_1_RA F("1- RA")
#define un_setting_2_DEC F("2- DEC")
#define un_setting_play F("play- zakoncz")
#define un_enter_star_ra F("wprowadz rektascensje gwiazdy")
#define un_enter_star_dec F("wprowadz deklinacje gwiazdy")
#define un_enter_latitude F("wprowadz szerokosc geograficzna")
#define un_enter_longitude F("wprowadz dlugosc geograficzna")
#define un_enter_accel_offset F("wprowadz offset akcelerometru")
#define un_enter_az_offset F("wprowadz offset azymutu")
#define un_star_visibility F("widocznosc gwiazdy:")
#define un_visible F("widoczna")
#define un_unvisible F("nie widoczna")
#define un_motor1 F("silnik 1")
#define un_motor2 F("silnik 2")
#define un_degree F("kat")
#define un_device_position_calibration F("kalibracja pozycji urzadzenia")
#define un_pointing_at_north F("gotowe, wskazujesz polnoc")
#define un_not_pointing_at_north F("nie wskazujesz polnocy")
#define un_start_tracking_continously F("zacznij sledzic gwiazde")
#define un_star_found F("znaleziono gwiazde")
#define un_no_gps F("brak gps")
#define un_no_satelites F("brak satelit")
#define un_local_time F("czas lokalny")
#define un_time_from_gps F("czas gps")
#define un_can_calibrate F("{1}")
#define un_cant_calibrate F("{0}")
#define un_recent_location F("ostatnia lokalizacja")
#define un_recently_tracked_star F("ostatnia gwiazda")
#define un_set_position_manually F("1- ustaw urzadzenie recznie")
#define un_if_you_want_to_calibrate_play F("aby kalibrowac wcisnij play")
#define un_exit_press F("aby wyjsc wcisnij 0")
#define un_use_compass_to_find_north F("uzyj busoli i wskaz polnoc")
#endif
#pragma endregion polish
#pragma region english
#ifdef eng
#define un_azymuth "azymuth" //short from universal azymuth
#define un_altitude "altitude"
#define un_declination "declination"
#define un_right_ascension "right asc."
#define un_year "year"
#define un_time_utc "UTC"
#define un_day "day"
#define un_calibration "calibration"
#define un_laser_angle "laser angle"
#define un_second "second"
#define un_dev_AZ "dev Az."
#define un_eq "EQ-"
#define un_month "month"
#define un_lat "lat."
#define un_long "long."
#define un_set_mag_declination "set mag. declination"
#define un_your_location "location"
#define un_submit_continue "confirm/continue"
#define un_star_location "star.loc."
#define un_instruction "instruction:"
#define un_setting_1_RA "1- RA"
#define un_setting_2_DEC "2- DEC"
#define un_setting_play "play- finish"
#define un_enter_star_ra "enter Star RA"
#define un_enter_star_dec "enter Star Dec"
#define un_enter_latitude "enter your latitude"
#define un_enter_longitude "enter your longitude"
#define un_enter_accel_offset "enter accel offset"
#define un_enter_az_offset "enter azymuth offset"
#define un_magnetic_declination "magnetic declination"
#define un_star_visibility "star visibility"
#define un_visible "visible"
#define un_unvisible "not visible"
#define un_motor1 "motor 1"
#define un_motor2 "motor 2"
#define un_degree "degree"
#define un_device_position_calibration "device position calibration"
#define un_pointing_at_north "ready, poining at north"
#define un_not_pointing_at_north "not pointing at north"
#define un_start_tracking_continously "start star tracking"
#define un_star_found "star found"
#define un_no_gps "no gps"
#define un_no_satelites "no satelites"
#define un_local_time "local time"
#define un_time_from_gps "gps time"
#define un_can_calibrate "{1}"
#define un_cant_calibrate "{0}"
#define un_recent_location "recent location"
#define un_recently_tracked_star "recent star"
#define un_set_position_manually "1- set device position manually"
#define un_use_compass_to_find_north "use compass to find north"
#define un_if_you_want_to_calibrate_play "to continue press play"
#define un_exit_press "to exit plress 0"
#endif

#pragma endregion english