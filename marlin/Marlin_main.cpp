/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#include "ultralcd.h"
#include "OledShow.h"
#include "planner.h"
#include "stepper.h"
#include "laser.h"
//#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"

#if NUM_SERVOS > 0
#include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#define VERSION_STRING  "1.0.0"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G29 - Calibrate print surface with automatic Z probe
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

// M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M32  - Select file and start SD print (Can be used when printing from SD card)
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M250 - Set LCD contrast C<contrast value> (value 0..63)
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beepsound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
//float z_probe_offset[] = Z_PROBE_OFFSET;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
//int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

uint8_t active_extruder = 0;

#ifdef ULTIPANEL
	bool powersupply = true;
#endif

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
//static float offset[3] = {0.0, 0.0, 0.0};
//static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN = 0, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE+2];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

bool Stopped=false;

bool target_direction;

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void process_commands(char* cmd=cmdbuffer[bufindr]);

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

extern "C"{
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
#if FASTLOG
    fromsd[bufindw] == true;
#endif
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
#if FASTLOG
    fromsd[bufindw] == true;
#endif
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void setup_killpin()
{
  #if defined(KILL_PIN) && KILL_PIN > -1
    pinMode(KILL_PIN,INPUT_PU);
//    WRITE(KILL_PIN,HIGH);
  #endif
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif
}

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

#define _delay_ms(ms) delay(ms)
void setup()
{
//    for(int i = 2; i; i--)
//       delay(1000);

  setup_killpin();
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
#ifndef ARDUINO_ARCH_STM32
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;
#else
#endif

//  SERIAL_ECHOPGM(MSG_MARLIN);
//  SERIAL_ECHOLNPGM(VERSION_STRING);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif
  #endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  MSerial.println("retrievesettings");

  MSerial.println("lcd_init");
//oled_init();//lcd_init();
//dect: while(READ(12)){0;};
//  delay(1);
//  if(READ(12)==0){
//      LED_STATUS = 1;
//      display();
//  }else
//      goto dect;

  MSerial.println("plan_init");
  plan_init();  // Initialize planner;
//MSerial.println("watchdog_init");
  watchdog_init();
  MSerial.println("laser_init");
  laser_init();

  action(POINT, 10, 30);//while(1);

  MSerial.println("st_init");
  st_init();    // Initialize stepper, this enables interrupts!
//MSerial.println("photpin_init");
//  setup_photpin();
//MSerial.println("tp_init");
//  tp_init();    // Initialize temperature loop
//  _delay_ms(500);	// wait 1sec to display the splash screen

  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif
}

void loop()
{
  if(buflen < (BUFSIZE-1))
    get_command();
  #ifdef SDSUPPORT
  card.checkautostart(false);
  #endif
  if(buflen)
  {
    #ifdef SDSUPPORT
      if(card.saving) {
        #if FASTLOG
          if(card.fastlogging == 1)
          {
              if((card.getfilesize() >= 1024*10))//&& ((starttime + 5000) < millis()))
              {
                  card.fastlogging = 2;
    //                  card.openFile(card.filename, true);//M23 open read file!!
                  card.startFileprint();
                  //starttime=millis();
              }
          }
        #endif
        if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
        {
          card.write_command(cmdbuffer[bufindr]);
          if(card.logging)
          {
            process_commands();
          }
          else
          {
            SERIAL_PROTOCOLLNPGM(MSG_OK);
          }
        }
        else
        {
          card.closefile();
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
        }

        #if FASTLOG
          if(card.sdprinting)//<==lead to (card.fastlogging == 2)
          {
              buflen = (buflen-1);
              bufindr = (bufindr + 1)%BUFSIZE;
              manage_sdprint();
              goto manage;
          }
        #endif
      }
      else
      {
        process_commands();
      }
    #else
      process_commands();
    #endif //SDSUPPORT
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
manage:
  //check heater every n milliseconds
  //manage_heater();
  manage_inactivity();
//  manage_fastlog();
  //checkHitEndstops();
  LED_STATUS = IN(12);//lcd_update();
}

void get_command()
{
//  if(buflen >= BUFSIZE)
//      return;
//  else
  while( MYSERIAL.available() > 0) {//  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();
    //MSerial.print(serial_char, BYTE);
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE) )
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        continue;//return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      		MSerial.print(cmdbuffer[bufindw]);
      {//if(!comment_mode || serial_count){//if(!comment_mode){
        //comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strchr_pointer = strchr(cmdbuffer[bufindw], 'N'))//(strchr(cmdbuffer[bufindw], 'N') != NULL)
        {
          //strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = strtol(strchr_pointer+1, NULL, 10);//(strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            SERIAL_ERRORLN(gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strchr_pointer = strchr(cmdbuffer[bufindw], '*'))//(strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            //strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if(strtod(strchr_pointer+1, NULL) != checksum) {//( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              SERIAL_ERRORLN(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            else
                *strchr_pointer = '\0';
            //if no errors, continue parsing
          }
          else
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        if(strchr_pointer = strchr(cmdbuffer[bufindw], 'G')) {//((strchr(cmdbuffer[bufindw], 'G') != NULL)){
          //strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)strtod(strchr_pointer+1, NULL)) {//((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
          #ifdef SDSUPPORT
              if(card.saving)
                break;
          #endif //SDSUPPORT
              SERIAL_PROTOCOLLNPGM(MSG_OK);
            }
            else {
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
            }
            break;
          default:
            break;
          }
        }
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer
      if(buflen >= BUFSIZE)
          break;
    }
    else if(comment_mode == false)
    {
      if(serial_char == ';')
          comment_mode = true;
      else
          cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

  #ifdef SDSUPPORT
  if(card.saving || !card.sdprinting || serial_count!=0 || buflen >= BUFSIZE) {//if(!card.sdprinting || serial_count!=0){
    return;
  }
  while( !card.eof()) {//  && buflen < BUFSIZE) {
//    int16_t n=card.get();
    serial_char = card.get();//(char)n;
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1)||serial_char == -1)//n==-1)
    {
      if(card.eof()){
        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
        stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        int hours, minutes;
        minutes=(t/60)%60;
        hours=t/60/60;
        sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(time);
        lcd_setstatus(time);
        card.printingHasFinished();
        card.checkautostart(true);

        SetLedRunDefault();
      }
      if(!serial_count)
      {
        comment_mode = false; //for new command
        continue;//return; //if empty line
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      if(!comment_mode){
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
//      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
//      if(buflen >= BUFSIZE) !the sd file don't need to be cached too much for responding uart and running commands quickly!
          break;
    }
    else if(comment_mode == false)
    {
      if(serial_char == ';')
          comment_mode = true;
      else
          cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

  #endif //SDSUPPORT

}

#if FASTLOG
void manage_fastlog(void)
{
    if(card.fastlogging) {
        if(buflen == 0)
        {
            while(MYSERIAL.available() > 0) {
                serial_char = MYSERIAL.read();
                if(serial_char == '\n' ||
                   serial_char == '\r' ||
                   (serial_char == ':' && comment_mode == false) ||
                   serial_count >= (MAX_CMD_SIZE-1))
                {
                    if(!serial_count)
                    {
                        comment_mode = false;//for new command
                        return;
                    }
                    cmdbuffer[bufindw][serial_count] = '\0';
                    comment_mode = false;//for new command
                    fromsd[bufindw] = false;
                    serial_count = 0;//clear buffer

                    if(strchr_pointer = strchr(cmdbuffer[bufindw], 'N'))
                    {
                        gcode_N = strtol(strchr_pointer + 1, NULL, 10);
                        if((gcode_N != gcode_LastN+1) && (strstr(cmdbuffer[bufindw], "M110")==NULL))
                        {
                            SERIAL_ERROR_START;
                            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
                            SERIAL_ERRORLN(gcode_LastN);
                            FlushSerialRequestResend();
                            return;
                        }
                        if(strchr_pointer = strchr(cmdbuffer[bufindw], '*'))
                        {
                            byte checksum = 0, count = 0;
                            while(cmdbuffer[bufindw][count] != '*') checksum ^= cmdbuffer[bufindw][count++];

                            if(strtod(strchr_pointer+1, NULL) != checksum)
                            {
                                SERIAL_ERROR_START;
                                SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
                                SERIAL_ERRORLN(gcode_LastN);
                                FlushSerialRequestResend();
                                return;
                            }
                            else
                                *(uint16_t*)strchr_pointer = ';';// flag that writed by manage_fastlog //*strchr_pointer = '\0'
                            //if no error, contuine parsing
                        }
                        else
                        {
                            SERIAL_ERROR_START;
                            SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
                            SERIAL_ERRORLN(gcode_LastN);
                            FlushSerialRequestResend();
                            return;
                        }

                        gcode_LastN = gcode_N;
                        //if no error, contuine parsing
                    }
                    else if(strchr(cmdbuffer[bufindw], '*'))//if we don't receive 'N', but still see '*'
                    {
                        SERIAL_ERROR_START;
                        SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
                        SERIAL_ERRORLN(gcode_LastN);
                        return;
                    }

                    bufindw = (bufindw + 1) % BUFSIZE;
                    buflen++;

                    break;//write command to sd
                }
                else if(comment_mode == false)
                {
                    if(serial_char == ';')
                        comment_mode = true;
                    else
                        cmdbuffer[bufindw][serial_count++] = serial_char;
                }
            }
        }

        if(buflen > 0)
        {
            if(strstr(cmdbuffer[bufindr], "M29") == NULL)
            {
                card.write_command(cmdbuffer[bufindr]);
                SERIAL_PROTOCOLLNPGM(MSG_OK);
            }
            else
            {
                card.closefile();
                SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
            }

            bufindr = (bufindr + 1) % BUFSIZE;
            buflen--;
        }
    }
}

void manage_sdprint(void)
{
    static char cmdsd[MAX_CMD_SIZE + 2], sd_char;
    static uint32_t sd_count;

    while(!card.eof()) {
        sd_char = card.get();
        if(sd_char == '\r' ||
           sd_char == '\n' ||
           (sd_char == ':' && comment_mode == false) ||
           sd_count >= (MAX_CMD_SIZE) ||
           sd_char == -1)
        {
//            if(card.eof()) {
//                this must not happen when fastlog, or the transmission wrong and check it..
//            }
            if(!sd_count)
            {
                comment_mode = false;//for new command
                continue;//return;
            }

            cmdsd[sd_count] = '\0';//terminate string
            comment_mode = false;//for new command
            sd_count = 0;//clear buffer

            process_commands(cmdsd);
            break;
        }
        else
        {
            if(sd_char == ';') comment_mode = true;
            if(!comment_mode)
                cmdsd[sd_count++] = sd_char;
        }
    }
}
#endif

float code_value()
{
  return (strtod(strchr_pointer+1, NULL));//(strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(strchr_pointer+1, NULL, 10));//(strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}
static char* code_pointer;
char* code_seen(char code, char* cmd = code_pointer)
{
  strchr_pointer = strchr(cmd, code);//strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer);//(strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);


void process_commands(char* cmd)
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
    code_pointer = cmd;
  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
        //ClearToSend();
        return;
      }
      break;
//    case 2: // G2  - CW ARC
//    case 3: // G3  - CCW ARC
//    case 4: // G4 dwell
    case 28: //G28 Home all Axis one at a time
        previous_millis_cmd = millis();

        st_synchronize();
        enable_endstops(true);

        point(0, 0);
        delayMicroseconds(600);
        for(int i = 0; i < 5; i++)
        {
            LASER = 1;
            delay(300);
            LASER = 0;
            delay(300);
        }
        current_position[X_AXIS] = 0;
        current_position[Y_AXIS] = 0;
        current_position[Z_AXIS] = 0;//!!z axis is STEPPER, has not process.....
        plan_set_position(0, 0, 0, 0);

        enable_endstops(false);
        break;
//    case 29: // G29 Calibrate print surface with automatic Z probe.
    case 30:
        mode mode;
        unsigned short g;
        unsigned int num;

        if(code_seen('P'))
            mode = POINT;
        else if(code_seen('L'))
            mode = LINE;
        else if(code_seen('A'))
            mode = AREA;
        num = code_value();
        if(code_seen('D'))
            g = code_value();
        else
            g = 1;

        action(mode, g, num);

        break;
    case 31:
        if(code_seen('L'))
            LASER = code_value();
        else if(code_seen('Z'))
        {
            if(code_value())
                {enable_z();}
            else
                {disable_z();}
        }
        break;
    case 32:
        extern unsigned short laser_t;
        if(code_seen('T'))
            laser_t = code_value();
        break;
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
        if(!code_seen(axis_codes[E_AXIS]))
          st_synchronize();
        for(int8_t i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) {
             if(i == E_AXIS) {
               current_position[i] = code_value();
               plan_set_e_position(current_position[E_AXIS]);
             }
             else {
               current_position[i] = code_value()+add_homeing[i];
               plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
             }
          }
        }
      break;
    }
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {
    case 19:
        display();
        break;
#ifdef SDSUPPORT
    case 20: // M20 - list SD card
      SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      card.ls();
      SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      break;
    case 21: // M21 - init SD card
      card.initsd();
      break;
    case 22: //M22 - release SD card
      card.release();
      break;
    case 23: //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;
    case 24: //M24 - Start SD print
      card.startFileprint();
      starttime=millis();
      break;
    case 25: //M25 - Pause SD print
      card.pauseSDPrint();
      break;
    case 26: //M26 - Set SD index
      if(card.cardOK && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
    case 28: //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //card,saving = false;
      break;
    case 30: //M30 <filename> Delete File
      if (card.cardOK){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos-1) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
    case 32: //M32 - Select file and start SD print
      if(card.sdprinting) {
        st_synchronize();
        card.closefile();
        card.sdprinting = false;
      }
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      card.startFileprint();
      starttime=millis();
      break;
    case 928: //M928 - Start SD write
      starpos = (strchr(strchr_pointer + 5,'*'));
#if !FASTLOG
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
#else
      if(starpos) {
          *starpos = '\0';
      }
      if(*(strchr_pointer+4) == 'F'){
          card.openfastlogfile(strchr_pointer+5);
          starttime = millis();
      }
      else
          card.openLogFile(strchr_pointer+5);
#endif
      break;
#endif //SDSUPPORT

    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      lcd_setstatus(time);
      //autotempShutdown();
      }
      break;
//    case 42: //M42 -Change pin status via gcode
//     break;
    case 105 : // M105
//      if(setTargetedHotend(105)){
//        break;
//        }
//      #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
//        SERIAL_PROTOCOLPGM("ok T:");
//        SERIAL_PROTOCOL_F(50.0, 1);//(degHotend(tmp_extruder),1);
//        SERIAL_PROTOCOLPGM(" /");
//        SERIAL_PROTOCOL_F(50.0, 1);//(degTargetHotend(tmp_extruder),1);
//        #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
//          SERIAL_PROTOCOLPGM(" B:");
//          SERIAL_PROTOCOL_F(30.0, 1);//(degBed(),1);
//          SERIAL_PROTOCOLPGM(" /");
//          SERIAL_PROTOCOL_F(30.0, 1);//(degTargetBed(),1);
//        #endif //TEMP_BED_PIN
//        for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
//          SERIAL_PROTOCOLPGM(" T");
//          SERIAL_PROTOCOL(cur_extruder);
//          SERIAL_PROTOCOLPGM(":");
//          SERIAL_PROTOCOL_F(50.0, 1);//(degHotend(cur_extruder),1);
//          SERIAL_PROTOCOLPGM(" /");
//          SERIAL_PROTOCOL_F(50.0, 1);//(degTargetHotend(cur_extruder),1);
//        }
//      #else
//        SERIAL_ERROR_START;
//        SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
//      #endif
//
//        SERIAL_PROTOCOLPGM(" @:");
//        SERIAL_PROTOCOL(50);//(getHeaterPower(tmp_extruder));
//
//        SERIAL_PROTOCOLPGM(" B@:");
//        SERIAL_PROTOCOL(50);//(getHeaterPower(-1));
//
//        SERIAL_PROTOCOLLN("");
//      return;
      break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
              disable_e1();
              disable_e2();
            }
          #endif
        }
      }
      break;
    case 85: // M85
      code_seen('S');
      max_inactive_time = code_value() * 1000;
      break;
    case 92: // M92
      if(!code_seen(axis_codes[E_AXIS]))
          st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      break;
    case 117: // M117 display message
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      lcd_setstatus(strchr_pointer + 5);
      break;
    case 114: // M114
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM("E:");
      SERIAL_PROTOCOL(current_position[E_AXIS]);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      SERIAL_PROTOCOLLN("");
      break;
    case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
        if(code_seen('C'))
            oled.Set_Contrast_Control(4.04*code_value());
        if(code_seen('R')){
            oled.Set_Segment_Remap(0xA1);
            oled.Set_Common_Remap(0xC8);
        }
        else if(code_seen('N')){
            oled.Set_Segment_Remap(0xA0);
            oled.Set_Common_Remap(0xC0);
        }
    break;
    case 400: // M400 finish all moves
    {
      st_synchronize();
    }
    break;
    case 500: // M500 Store settings in EEPROM
    {
        Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
        Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;
    case 999: // M999: Restart after being stopped
      Stopped = false;
      lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
    break;
    }
  }

  else if(code_seen('T'))
  {
  }

  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
  }

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
#if FASTLOG
  if(card.fastlogging)
      return;
#endif
  #endif //SDSUPPORT
  SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
//  if(code_seen('F')) {
//    next_feedrate = code_value();
//    if(next_feedrate > 0.0) feedrate = next_feedrate;
//  }
}

//void get_arc_coordinates()
//{
//#ifdef SF_ARC_FIX
//   bool relative_mode_backup = relative_mode;
//   relative_mode = true;
//#endif
//   get_coordinates();
//#ifdef SF_ARC_FIX
//   relative_mode=relative_mode_backup;
//#endif
//
//   if(code_seen('I')) {
//     offset[0] = code_value();
//   }
//   else {
//     offset[0] = 0.0;
//   }
//   if(code_seen('J')) {
//     offset[1] = code_value();
//   }
//   else {
//     offset[1] = 0.0;
//   }
//}

void clamp_to_software_endstops(float target[3])
{
//  if (min_software_endstops)
	{
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

//  if (max_software_endstops)
	{
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

//#ifdef DELTA
//void calculate_delta(float cartesian[3])
//{
//  delta[X_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
//                       - sq(DELTA_TOWER1_X-cartesian[X_AXIS])
//                       - sq(DELTA_TOWER1_Y-cartesian[Y_AXIS])
//                       ) + cartesian[Z_AXIS];
//  delta[Y_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
//                       - sq(DELTA_TOWER2_X-cartesian[X_AXIS])
//                       - sq(DELTA_TOWER2_Y-cartesian[Y_AXIS])
//                       ) + cartesian[Z_AXIS];
//  delta[Z_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
//                       - sq(DELTA_TOWER3_X-cartesian[X_AXIS])
//                       - sq(DELTA_TOWER3_Y-cartesian[Y_AXIS])
//                       ) + cartesian[Z_AXIS];
//  /*
//  SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
//  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
//  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);
//
//  SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
//  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
//  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
//  */
//}
//
//// Adjust print surface height by linear interpolation over the bed_level array.
//void adjust_delta(float cartesian[3])
//{
//  float grid_x = max(-2.999, min(2.999, cartesian[X_AXIS] / AUTOLEVEL_GRID));
//  float grid_y = max(-2.999, min(2.999, cartesian[Y_AXIS] / AUTOLEVEL_GRID));
//  int floor_x = floor(grid_x);
//  int floor_y = floor(grid_y);
//  float ratio_x = grid_x - floor_x;
//  float ratio_y = grid_y - floor_y;
//  float z1 = bed_level[floor_x+3][floor_y+3];
//  float z2 = bed_level[floor_x+3][floor_y+4];
//  float z3 = bed_level[floor_x+4][floor_y+3];
//  float z4 = bed_level[floor_x+4][floor_y+4];
//  float left = (1-ratio_y)*z1 + ratio_y*z2;
//  float right = (1-ratio_y)*z3 + ratio_y*z4;
//  float offset = (1-ratio_x)*left + ratio_x*right;
//
//  delta[X_AXIS] += offset;
//  delta[Y_AXIS] += offset;
//  delta[Z_AXIS] += offset;
//
//  /*
//  SERIAL_ECHOPGM("grid_x="); SERIAL_ECHO(grid_x);
//  SERIAL_ECHOPGM(" grid_y="); SERIAL_ECHO(grid_y);
//  SERIAL_ECHOPGM(" floor_x="); SERIAL_ECHO(floor_x);
//  SERIAL_ECHOPGM(" floor_y="); SERIAL_ECHO(floor_y);
//  SERIAL_ECHOPGM(" ratio_x="); SERIAL_ECHO(ratio_x);
//  SERIAL_ECHOPGM(" ratio_y="); SERIAL_ECHO(ratio_y);
//  SERIAL_ECHOPGM(" z1="); SERIAL_ECHO(z1);
//  SERIAL_ECHOPGM(" z2="); SERIAL_ECHO(z2);
//  SERIAL_ECHOPGM(" z3="); SERIAL_ECHO(z3);
//  SERIAL_ECHOPGM(" z4="); SERIAL_ECHO(z4);
//  SERIAL_ECHOPGM(" left="); SERIAL_ECHO(left);
//  SERIAL_ECHOPGM(" right="); SERIAL_ECHO(right);
//  SERIAL_ECHOPGM(" offset="); SERIAL_ECHOLN(offset);
//  */
//}
//void prepare_move_raw()
//{
//  previous_millis_cmd = millis();
//  calculate_delta(destination);
//  plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
//		   destination[E_AXIS], feedrate*feedmultiply/60/100.0,
//		   active_extruder);
//  for(int8_t i=0; i < NUM_AXIS; i++) {
//    current_position[i] = destination[i];
//  }
//}
//#else
//void calculate_delta(float cartesian[3]) {}
//void prepare_move_raw()
//{
//  previous_millis_cmd = millis();
//  calculate_delta(destination);
//  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
//		   destination[E_AXIS], feedrate*feedmultiply/60/100.0,
//		   active_extruder);
//  for(int8_t i=0; i < NUM_AXIS; i++) {
//    current_position[i] = destination[i];
//  }
//}
//#endif

void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_cmd = millis();
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

//void prepare_arc_move(char isclockwise) {
//  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc
//
//  // Trace the arc
//  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);
//
//  // As far as the parser is concerned, the position is now == target. In reality the
//  // motion control system might still be processing the action and the real tool position
//  // in any intermediate location.
//  for(int8_t i=0; i < NUM_AXIS; i++) {
//    current_position[i] = destination[i];
//  }
//  previous_millis_cmd = millis();
//}

void manage_inactivity()
{
  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      kill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
//        disable_x();
//        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }
  #if defined(KILL_PIN) && KILL_PIN > -1
    if( 0 == READ(KILL_PIN) )
      kill();
  #endif
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  check_axes_activity();
}

void kill()
{
  cli(); // Stop interrupts
//  disable_heater();

//  disable_x();
//  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
#endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM(MSG_KILLED);
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

//void Stop()
//{
////  disable_heater();
//  if(Stopped == false) {
//    Stopped = true;
//    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
//    SERIAL_ERROR_START;
//    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
//    LCD_MESSAGEPGM(MSG_STOPPED);
//  }
//}

//bool IsStopped() { return Stopped; };
