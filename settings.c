/*
  settings.c - eeprom configuration handling 
  Part of Grbl

  This is a modified version of the Grbl code written to account for a geometry 
  change for the Hangbot Project by Jack Boland, Ian Anderson and Paul Lorenz.

  Original Code Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <math.h>
#include "nuts_bolts.h"
#include "settings.h"
#include "eeprom.h"
#include "wiring_serial.h"
#include <avr/pgmspace.h>
#include "protocol.h"

settings_t settings;											

// Version 1 outdated settings record
typedef struct {
  double steps_per_mm[3];
  uint8_t microsteps;
  uint8_t pulse_microseconds;
  double default_feed_rate;
  double default_seek_rate;
  uint8_t invert_mask;
  double mm_per_arc_segment;
} settings_v1_t;

// Default settings (used when resetting eeprom-settings)
#define MICROSTEPS 8
#define DEFAULT_X_STEPS_PER_MM (MICROSTEPS)*2.5
#define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS)*2.5
#define DEFAULT_Z_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define DEFAULT_STEP_PULSE_MICROSECONDS 30
#define DEFAULT_MM_PER_ARC_SEGMENT 0.1
#define DEFAULT_RAPID_FEEDRATE 500.0 // in millimeters per minute
#define DEFAULT_FEEDRATE 500.0
#define DEFAULT_ACCELERATION (DEFAULT_FEEDRATE/10.0)
#define DEFAULT_MAX_JERK 300.0
#define DEFAULT_X_DISTANCE 940
#define DEFAULT_Y_DISTANCE 740
#define DEFAULT_STEPPING_INVERT_MASK 0x1C	//@grblshield/
//#define DEFAULT_STEPPING_INVERT_MASK 0

void settings_reset() {
  settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
  settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
  settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
  settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
  settings.default_feed_rate = DEFAULT_FEEDRATE;
  settings.default_seek_rate = DEFAULT_RAPID_FEEDRATE;
  settings.acceleration = DEFAULT_ACCELERATION;
  settings.mm_per_arc_segment = DEFAULT_MM_PER_ARC_SEGMENT;
  settings.invert_mask = DEFAULT_STEPPING_INVERT_MASK;
  settings.max_jerk = DEFAULT_MAX_JERK;
  settings.x_distance = DEFAULT_X_DISTANCE;
  settings.y_distance = DEFAULT_Y_DISTANCE;
}

void settings_dump() {
  printPgmString(PSTR("$0 = ")); printFloat(settings.steps_per_mm[X_AXIS]);											// $0 = steps per mm [X axis]
  printPgmString(PSTR(" (steps/mm x)\r\n$1 = ")); printFloat(settings.steps_per_mm[Y_AXIS]);						// $1 = steps per mm [Y axis]
  printPgmString(PSTR(" (steps/mm y)\r\n$2 = ")); printFloat(settings.steps_per_mm[Z_AXIS]);						// $2 = steps per mm [Z axis]
  printPgmString(PSTR(" (steps/mm z)\r\n$3 = ")); printInteger(settings.pulse_microseconds);						// $3 = pulses per microstep?
  printPgmString(PSTR(" (microseconds step pulse)\r\n$4 = ")); printFloat(settings.default_feed_rate);				// $4 = default feedrate
  printPgmString(PSTR(" (mm/min default feed rate)\r\n$5 = ")); printFloat(settings.default_seek_rate);				// $5 = seek rate
  printPgmString(PSTR(" (mm/min default seek rate)\r\n$6 = ")); printFloat(settings.mm_per_arc_segment);			// $6 = mm per arc segment
  printPgmString(PSTR(" (mm/arc segment)\r\n$7 = ")); printInteger(settings.invert_mask); 							// $7 = invert mask??
  printPgmString(PSTR(" (step port invert mask. binary = ")); printIntegerInBase(settings.invert_mask, 2); 			 
  printPgmString(PSTR(")\r\n$8 = ")); printFloat(settings.acceleration);											// $8 = acceleration
  printPgmString(PSTR(" (acceleration in mm/sec^2)\r\n$9 = ")); printFloat(settings.max_jerk);						// $9 = max jerk [max instant cornering speed change in delta mm/min]  
  printPgmString(PSTR(" (max instant cornering speed change in delta mm/min)\r\n$10 = "));
  printFloat(settings.x_distance);	
  printPgmString(PSTR(" (x distance between anchors)\r\n$11 = "));
  printFloat(settings.y_distance);
  printPgmString(PSTR(" (y distance of drawing surface)\r\n" )); 							
  printPgmString(PSTR("\r\n'$x=value' to set parameter or just '$' to dump current settings\r\n"));					// Prints instructions to write '$x=value' inorder to set a value or 
  																													// '$' to dump current settings
}

// Parameter lines are on the form '$4=374.3' or '$' to dump current settings
uint8_t settings_execute_line(char *line) {
  uint8_t char_counter = 1;
  double parameter, value;
  if(line[0] != '$') { 
    return(STATUS_UNSUPPORTED_STATEMENT); 
  }
  if(line[char_counter] == 0) { 
    settings_dump(); return(STATUS_OK); 
  }
  if(!read_double(line, &char_counter, &parameter)) {
    return(STATUS_BAD_NUMBER_FORMAT);
  };
  if(line[char_counter++] != '=') { 
    return(STATUS_UNSUPPORTED_STATEMENT); 
  }
  if(!read_double(line, &char_counter, &value)) {
    return(STATUS_BAD_NUMBER_FORMAT);
  }
  if(line[char_counter] != 0) { 
    return(STATUS_UNSUPPORTED_STATEMENT); 
  }
  settings_store_setting(parameter, value);
  return(STATUS_OK);
}

void write_settings() {
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(1, (char*)&settings, sizeof(settings_t));
}

int read_settings() {
  // Check version-byte of eeprom
  uint8_t version = eeprom_get_char(0);
  
  if (version == SETTINGS_VERSION) {
    // Read settings-record and check checksum
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_t)))) {
      return(false);
    }
  } else if (version == 1) {
    // Migrate from old settings version
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_v1_t)))) {
      return(false);
    }
    settings.acceleration = DEFAULT_ACCELERATION;
    settings.max_jerk = DEFAULT_MAX_JERK;
  } else {      
    return(false);
  }
  return(true);
}

// A helper method to set settings from command line
void settings_store_setting(int parameter, double value) {
  switch(parameter) {
    case 0: case 1: case 2:
    if (value <= 0.0) {
      printPgmString(PSTR("Steps/mm must be > 0.0\r\n"));
      return;
    }
    settings.steps_per_mm[parameter] = value; break;
    case 3: settings.pulse_microseconds = round(value); break;
    case 4: settings.default_feed_rate = value; break;
    case 5: settings.default_seek_rate = value; break;
    case 6: settings.mm_per_arc_segment = value; break;
    case 7: settings.invert_mask = trunc(value); break;
    case 8: settings.acceleration = value; break;
    case 9: settings.max_jerk = fabs(value); break;
	case 10: settings.x_distance = value; break;
	case 11: settings.y_distance = value; break;
    default: 
      printPgmString(PSTR("Unknown parameter\r\n"));
      return;
  }
  write_settings();
  printPgmString(PSTR("Stored new setting\r\n"));
}

// Initialize the config subsystem
void settings_init() {
  if(read_settings()) {
    printPgmString(PSTR("'$' to dump current settings\r\n"));
  } else {
    printPgmString(PSTR("Warning: Failed to read EEPROM settings. Using defaults.\r\n"));
    settings_reset();
    write_settings();
    settings_dump();
  }
}
