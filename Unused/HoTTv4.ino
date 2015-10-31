#ifdef HoTT
#include "HoTTv4.h"

#define HOTTV4_TX_DELAY 1000    // minimum value is 630us, lower and the HoTT will not decode the data

#define HOTTV4_BUTTON_DEC 0xEB
#define HOTTV4_BUTTON_INC 0xED
#define HOTTV4_BUTTON_SET 0xE9
#define HOTTV4_BUTTON_NIL 0x0F
#define HOTTV4_BUTTON_NEXT 0xEE
#define HOTTV4_BUTTON_PREV 0xE7

#define OFFSET_HEIGHT 500
#define OFFSET_M2S 120
#define OFFSET_M3S 120

static uint8_t outBuffer[173];

static uint8_t row = 2;
static uint8_t col = 0;

/** 
 * Common setup method for HoTTv4
 */
void hottV4Setup() {
  hottV4Serial.begin(19200);
  hottV4EnableReceiverMode();
}

/**
 * Enables RX and disables TX
 */
static inline void hottV4EnableReceiverMode() {
  DDRD &= ~(1 << hTX);   // direct portmanipulation, number 5 is port 5 for RX/TX output portD = 0 - 7
  PORTD |= (1 << hTX);
}

/**
 * Enabels TX and disables RX
 */
static inline void hottV4EnableTransmitterMode() {
  DDRD |= (1 << hTX);
}

/**
 * Writes out given byte to HoTT serial interface.
 * If in debug mode, data is also written to UART serial interface. 
 */
static void hottV4SerialWrite(uint8_t c) {
  hottV4Serial.write(c);
}

/* Protocol decoding and frame transmitting from here on.. */
 
/**
 * Expects an array of at least size bytes. All bytes till size will be transmitted
 * to the HoTT capable receiver. Last byte will always be treated as checksum and is
 * calculated on the fly.
 */
static void hottV4SendData(uint8_t *data, uint8_t size) {
  hottV4Serial.flush();
  
  // Protocoll specific waiting time
  // to avoid collisions
  delay(5);
  
  if (hottV4Serial.available() == 0) {
    hottV4EnableTransmitterMode();
    
    uint16_t crc = 0;

    for (uint8_t i = 0; i < (size - 1); i++) {
      crc += data[i];     
      hottV4SerialWrite(data[i]);
      
      // Protocoll specific delay between each transmitted byte
      delayMicroseconds(HOTTV4_TX_DELAY);
    }
    
    // Write package checksum
    hottV4SerialWrite(crc & 0xFF);
    hottV4EnableReceiverMode();  
  }
}

/**
 * Read the HoTTv4 request from the transmitter and send HoTTv4 capable data frames according to the
 * requested module.
 */
void hottV4SendTelemetry() {
  static enum _hottV4_state {
    IDLE,
    BINARY,
    TEXT,
  } hottV4_state = IDLE;
  
  if (hottV4Serial.available() > 1) {
    for (uint8_t i = 0; i < 2; i++) {     /* Read twice, first C is start byte (80) second c contains the address request */
      uint8_t c = hottV4Serial.read();
      if (IDLE == hottV4_state) {
        switch (c) {
          case 0x80:
            hottV4_state = BINARY;
            break;
          case 0x7F:
            hottV4_state = TEXT;
            break;
          default:
            hottV4_state = IDLE;
        }
      } else if (BINARY == hottV4_state) {
        switch (c) {
          case HOTTV4_ELECTRICAL_AIR_SENSOR_ID:
            hottV4SendEAM();
            hottV4_state = IDLE;
            break;

          case HOTTV4_GENERAL_AIR_MODULE:
            hottV4SendGAM();
            hottV4_state = IDLE;
            break;

          case HOTTV4_GPS_MODULE:
            hottV4SendGPS();
            hottV4_state = IDLE;
            break;

          case HOTTV4_VARIO_MODULE:
            hottV4SendVario();
            hottV4_state = IDLE;
            break;

          default:
            hottV4_state = IDLE;
        }
      } else if (TEXT == hottV4_state) {
        switch (c) {
          case HOTTV4_BUTTON_NEXT:
            break;
          case HOTTV4_BUTTON_PREV:
            break;
          case HOTTV4_BUTTON_DEC:
            if (col) {
              if (row == 2) {
                m2h_alarmDriveVoltage -= 1;
              } else if (row == 3) {
                m2h_alarmTemp1 -= 1;
              }
            } else {
              row = row > 2 ? row - 1 : row;
            }
            break;
          case HOTTV4_BUTTON_INC:
            if (col) {
              if (row == 2) {
                m2h_alarmDriveVoltage += 1;
              } else if (row == 3) {
                m2h_alarmTemp1 += 1;
              }
            } else {
              row = row < 3 ? row + 1 : row;
            }
            break;
          case HOTTV4_BUTTON_SET:
            col = col == 1 ? 0 : 1;
            //writeSettings();
            break;
        }
        
        hottV4SendEAMText(row, col);
        hottV4_state = IDLE;
        
      }
    }
  }
}


/**
 * Sends HoTTv4 capable EAM (Electric Air Module) telemetry frame.
 */
static void hottV4SendEAM() {
  /** Minimum data set for EAM */
  HoTTV4ElectricAirModule.startByte = 0x7C;
  HoTTV4ElectricAirModule.sensorID = HOTTV4_ELECTRICAL_AIR_SENSOR_ID;
  HoTTV4ElectricAirModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTV4ElectricAirModule.endByte = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4ElectricAirModule.alarmTone = 0x0;
  HoTTV4ElectricAirModule.alarmInverse = 0x0;
  //(int &)
  HoTTV4ElectricAirModule.battery1 = m2h_vbat_A;        /* bat1 gauge */
  HoTTV4ElectricAirModule.driveVoltage = m2h_vbat_A;    /* main batt icon */
  HoTTV4ElectricAirModule.temp1 = 20 + (m2h_temp/1E2);  /* pressure sensor temp from AutoQuad */
  HoTTV4ElectricAirModule.temp2 = 20;                   /* nothing yet */

  HoTTV4ElectricAirModule.current = 0/ 10; 
  HoTTV4ElectricAirModule.height = OFFSET_HEIGHT + (m2h_gps_alt/1E3);
  HoTTV4ElectricAirModule.climbm2s = OFFSET_M2S + (alt_diff_1s/1E1);
  HoTTV4ElectricAirModule.climbm3s = OFFSET_M3S + (alt_diff_3s/1E3);

  HoTTV4ElectricAirModule.minutes = minutes;
  HoTTV4ElectricAirModule.seconds = seconds;

  /* low voltage alarm: battery voltage / number of cells not lower then LOW_BATT value per cell. note: battery voltage is x10 hense the LOW_BATT * 10 */
  if ( (m2h_vbat_A / m2h_num_cells) <= (LOW_BATT * 10) ) {    /* can also be settable on fixed by using: HoTTModuleSettings.alarmDriveVoltage */
    HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationUndervoltage;  
    HoTTV4ElectricAirModule.alarmInverse |= 0x80;         /* Invert Voltage display to indicate alarm */
  } 
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4ElectricAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/**
 * Sends HoTTv4 capable GAM (General Air Module) telemetry frame.
 */
static void hottV4SendGAM() {
  /** Minimum data set for GAM */
  HoTTV4GeneralAirModule.startByte = 0x7C;
  HoTTV4GeneralAirModule.moduleID = HOTTV4_GENERAL_AIR_MODULE;
  HoTTV4GeneralAirModule.sensorID = HOTTV4_GENERAL_AIR_SENSOR_ID;
  HoTTV4GeneralAirModule.endByte = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4GeneralAirModule.alarmTone = 0x0;
  HoTTV4GeneralAirModule.alarmInverse = 0x0;

  HoTTV4GeneralAirModule.battery1 = m2h_vbat_A;
  HoTTV4GeneralAirModule.driveVoltage = m2h_vbat_A;
  HoTTV4GeneralAirModule.temp1 = 20 + (m2h_temp/1E2);      /* pressure sensor temp from AutoQuad */
  HoTTV4GeneralAirModule.temp2 = 20;                       /* nothing yet */
  HoTTV4GeneralAirModule.min_cell_volt_num = m2h_num_cells;/* nothing yet, for the moment the number of detected cells in the battery */
  
  HoTTV4GeneralAirModule.current = 0 / 10; 
  HoTTV4GeneralAirModule.height = OFFSET_HEIGHT + (m2h_gps_alt/1E3);
  HoTTV4GeneralAirModule.climbm2s = OFFSET_M2S + (alt_diff_1s/1E1); 
  HoTTV4GeneralAirModule.climbm3s = OFFSET_M3S + (alt_diff_3s/1E3);
  HoTTV4GeneralAirModule.fuel_procent = m2h_fuel_procent;
  
  HoTTV4GeneralAirModule.pressure = m2h_abs/1E2;

  /* low voltage alarm: battery voltage / number of cells not lower then LOW_BATT value per cell. note: battery voltage is x10 hense the LOW_BATT * 10 */
  if ( (m2h_vbat_A / m2h_num_cells) <= (LOW_BATT * 10) ) {    /* can also be settable on fixed by using: HoTTModuleSettings.alarmDriveVoltage */
    HoTTV4GeneralAirModule.alarmTone = HoTTv4NotificationUndervoltage;  
    HoTTV4GeneralAirModule.alarmInverse |= 0x80;         /* Invert Voltage display to indicate alarm */
  } 
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy GAM data to output buffer
  memcpy(&outBuffer, &HoTTV4GeneralAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/**
 * Sends HoTTv4 capable GPS telemetry frame.
 */
static void hottV4SendGPS() {
  /** Minimum data set for GPS */
  HoTTV4GPSModule.startByte = 0x7C;
  HoTTV4GPSModule.moduleID = HOTTV4_GPS_MODULE;
  HoTTV4GPSModule.sensorID = HOTTV4_GPS_SENSOR_ID;
  HoTTV4GPSModule.version = 0xFF;
  HoTTV4GPSModule.endByte = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4GPSModule.alarmTone = 0x0;
  HoTTV4GPSModule.alarmInverse = 0x0;

  HoTTV4GPSModule.flight_direction = ((uint16_t)(m2h_yaw*100)/100) >>1; /* in 2* steps HoTTModule.flight_direction, arrives as float *100/100 to loose the floating point and convert to int */
  HoTTV4GPSModule.height = OFFSET_HEIGHT + (m2h_gps_alt/1E3);
  HoTTV4GPSModule.msl_height = msl_height;
  HoTTV4GPSModule.climbm2s = OFFSET_M2S + (alt_diff_1s/1E1); 
  HoTTV4GPSModule.climbm3s = OFFSET_M3S + (alt_diff_3s/1E3);
  
  convertLat (m2h_gps_lat);                                /* coordinates arrives from mavlink *1e7, convert in dec min & dec sec */
  convertLon (m2h_gps_lon);
  
  HoTTV4GPSModule.gps_fix_char = 48+m2h_fix_type;          /* ASCII 48=0 fixtype can be 0,2(d) or 3(d) */
  HoTTV4GPSModule.gps_satelites = m2h_satellites_visible;
  HoTTV4GPSModule.pos_NS = pos_NS;              /* calculated by convertLat / convertLon */
  HoTTV4GPSModule.pos_NS_dm = pos_NS_dm;
  HoTTV4GPSModule.pos_NS_sec = pos_NS_sec;
  HoTTV4GPSModule.pos_EW_dm = pos_EW_dm;
  HoTTV4GPSModule.pos_EW_sec = pos_EW_sec;
  HoTTV4GPSModule.home_distance = home_distance_calc;
  HoTTV4GPSModule.gps_speed = m2h_gps_vel * 3.6;          /* gps speed in m/s, convert to km/h */

  HoTTV4GPSModule.angle_roll = 0;                         /* ?? where on the screens ?? */
  HoTTV4GPSModule.angle_heading = 0;                      /* ?? where on the screens ?? */
  HoTTV4GPSModule.home_direction = ((uint16_t)(bearing*100)/100) >>1;           /* bearing angle from MAV position to home position in 2 deg steps */
  HoTTV4GPSModule.vibration = 0;                          /* ?? where on the screens ?? */  
  
  HoTTV4GPSModule.free_char1 = free_char1;
  HoTTV4GPSModule.free_char2 = free_char2;
  //HoTTV4GPSModule.free_char3 = free_char3;
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy GPS data to output buffer
  memcpy(&outBuffer, &HoTTV4GPSModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/**
 * Sends HoTTv4 capable GPS telemetry frame.
 */
static void hottV4SendVario() {
  /** Minimum data set for Vario */
  HoTTV4VarioModule.startByte = 0x7C;
  HoTTV4VarioModule.moduleID = HOTTV4_VARIO_MODULE;
  HoTTV4VarioModule.sensorID = HOTTV4_VARIO_SENSOR_ID;
  HoTTV4VarioModule.version = 0xFF;
  HoTTV4VarioModule.endByte = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4VarioModule.alarmTone = 0x0;
  HoTTV4VarioModule.alarmInverse = 0x0;

  HoTTV4VarioModule.climbm2s = OFFSET_M3S + (alt_diff_1s/1E1);           /* in cm/s */
  HoTTV4VarioModule.climbm3s = OFFSET_M3S + (alt_diff_3s/1E1);           /* in cm/3s */
  HoTTV4VarioModule.climbm10s = OFFSET_M3S + (alt_diff_10s/1E1);         /* in cm/10s */
  HoTTV4VarioModule.height = OFFSET_HEIGHT + (m2h_gps_alt/1E3);
  HoTTV4VarioModule.height_min = OFFSET_HEIGHT;
  HoTTV4VarioModule.height_max = OFFSET_HEIGHT + height_max;   /* either Ceiling or maximum reached altitude (GPS Altitude) */
  
  HoTTV4VarioModule.free_char1 = mavlinkHB_char;                          /* Mavlink HeartBeat char next to Alt */
  //HoTTV4VarioModule.free_char2 = HoTTModule.free_char2;
  HoTTV4VarioModule.flight_direction =  ((uint16_t)(m2h_yaw*100)/100) >>1; /* in 2* steps HoTTModule.flight_direction, arrives as float *100/100 to loose the floating point and convert to int */

  /* Text message on bottom line in GPS, Vario, General Air module screen */
  snprintf(text, VARIO_ASCIIS+1, HOTTV4_VARIO_MSG[flMode]);               /* set text according to FlightMode */
  
  /* First, break down the message (max 21 char) and place the seperate characters into the index */
  uint8_t offset = (VARIO_ASCIIS - strlen(text)) / 2;
  for(uint8_t index = 0; (index + offset) < VARIO_ASCIIS; index++) {
    if (text[index] != 0x0) {
      HoTTV4VarioModule.text_msg[index+offset] = text[index];
    } else {
      break;
    }
  }  
 
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy Vario data to output buffer
  memcpy(&outBuffer, &HoTTV4VarioModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}  
/* ##################################################################### *
 *                HoTTv4 Text Mode                                       *
 * ##################################################################### */

static void hottV4ClearAllTextLines() {
  memset(&HoTTv4ElectricalAirTextModule.text[0], ' ', 8*21);
}

/**
 * Writes out a single text line of max. 21 chars into HoTTv4ElectricalAirTextModule
 */
static void hottV4WriteLine(uint8_t line, const char *text) {
  uint8_t writeText = 1;

  for (uint8_t index = 0; index < 21; index++) {
    if (0x0 != text[index] && writeText) {
       HoTTv4ElectricalAirTextModule.text[(line * 21) + index] = text[index];
    } else {
      writeText = 0;
      HoTTv4ElectricalAirTextModule.text[(line * 21) + index] = ' ';
    }
  }
}

/**
 * Writes out a single text line of max. 21 chars into HoTTv4ElectricalAirTextModule.
 * If row == line it gets a selection indicator and given row is also highlighted.
 */
static void hottV4WriteLine(uint8_t line, const char *text, uint8_t row, uint8_t col) {
  char lineText[21];
  uint8_t inCol = 0;

  enum {
    IDLE,
    COLON,
    SPACE,
    COL,
    DONE,
  } state = IDLE;

  const char selectionIndicator = (line == row) ? '>' : ' ';  
  snprintf(lineText, 21, "%c%s", selectionIndicator, text);  
  
  for (uint8_t index = 0 ; index < 21 ; index++) {
    uint8_t c = lineText[index];
    
    if (IDLE == state) {
      state = (':' == c) ? COLON : IDLE;
    } else if (COLON == state) {
      state = (' ' == c) ? SPACE : COLON; 
    } else if (SPACE == state) {
      if ('.' <= c) {
        inCol++;
        state = COL;
      } else {
        state = SPACE;
      }
    } else if (COL == state) {
      if (' ' == c) {
        state = SPACE;
      } else if (0x0 == c) {
        state = DONE;
      } else {
        state = COL;
      }
    } else if (DONE == c) {
      break;
    }
    
    if ((COL == state) && (inCol == col) && (line == row)) {
      lineText[index] += 128;
    } 
  }

  hottV4WriteLine(line, lineText);
}

/**
 * Sends HoTTv4 capable EAM text frame.
 */
static void hottV4SendEAMText(uint8_t row, uint8_t col) {
  /** Minimum data set for EAM Text mode */
  HoTTv4ElectricalAirTextModule.startByte = 0x7B;
  HoTTv4ElectricalAirTextModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTv4ElectricalAirTextModule.endByte = 0x7D;

  // Clear output buffer
  memset(&outBuffer, 0x0, sizeof(outBuffer));
  
  hottV4ClearAllTextLines();
  hottV4WriteLine(0, " AUTOQUAD SETTINGS");

  char text[21];
  snprintf(text, 21, "ALARM VOLT : %2i.%1iV", m2h_alarmDriveVoltage / 10, m2h_alarmDriveVoltage % 10);
  hottV4WriteLine(2, text, row, col);
  
  snprintf(text, 21, "ALARM TEMP1:  %3iC", m2h_alarmTemp1);
  hottV4WriteLine(3, text, row, col);
 
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTv4ElectricalAirTextModule, kHoTTv4TextPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4TextPacketSize);
}
#endif
