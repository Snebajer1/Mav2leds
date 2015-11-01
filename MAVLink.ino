#ifdef MAVLINK_COMM_NUM_BUFFERS
  #undef MAVLINK_COMM_NUM_BUFFERS
#endif
#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// this code was moved from libraries/GCS_MAVLink to allow compile
// time selection of MAVLink 1.0
BetterStream	*mavlink_comm_0_port;
BetterStream	*mavlink_comm_1_port;

mavlink_system_t mavlink_system = {12,1,0,0};

#include "Mavlink_compat.h"
#include "include/mavlink/v1.0/mavlink_types.h"
#include "include/mavlink/v1.0/common/mavlink.h" 

static int packet_drops = 0;
static int parse_error = 0;

void read_mavlink(){
  mavlink_message_t msg; 
  mavlink_status_t status;
  
  // grabbing data 
  while(Serial.available() > 0) { 
    uint8_t c = Serial.read();

    // trying to grab msg  
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      messageCounter = 0; 
      mavlink_active = 1;         /* Valid Mavlink packet received */
      switch(msg.msgid)           /* Handle msg */
      {
        /** HeartBeat
        * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        * @param custom_mode A bitfield for use for autopilot-specific flags.  - AutoQuad nav mode
        * @param system_status System status flag, see MAV_STATE ENUM
        */
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
            #ifdef DEBUGLED
              digitalWriteFast (DEBUGLED, HIGH);
            #endif
            
            mavbeat = 1;
	    apm_mav_system    = msg.sysid;
	    apm_mav_component = msg.compid;
            apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);

            m2h_mode = mavlink_msg_heartbeat_get_base_mode(&msg);         // Base mode, not very informative, only used for armed/unarmed
            m2h_nav_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);   // Here we should find the flight modes
            m2h_sysstat = mavlink_msg_heartbeat_get_system_status(&msg);  // Severity flag is in here

            /* Bit 7 tells whether motors are armed */
            isArmed = ((m2h_mode&128) == 128 ? 1 : 0);

            CheckFlightMode();
            
            #ifdef SERDB            
              dbSerial.print("System: ");
              dbSerial.print(apm_mav_system);
              dbSerial.print(" Type: ");
              dbSerial.print(apm_mav_type);
              dbSerial.print(" MAV: ");
              dbSerial.print((mavlink_msg_heartbeat_get_base_mode(&msg),DEC));
              dbSerial.print(" BaseMode: ");
              dbSerial.print(m2h_mode);
              dbSerial.print(" CustModes: ");
              dbSerial.print(m2h_nav_mode);
              dbSerial.print(" SysStat: ");
              dbSerial.print(m2h_sysstat);
              dbSerial.print(" SatFIX: ");
              dbSerial.print(m2h_fix_type);
              dbSerial.print(" flMode: ");
              dbSerial.print(flMode);
              dbSerial.print("  Armed: ");
              dbSerial.print(isArmed);
              dbSerial.println();
            #endif 
          }
          break;
          
        /* GPS Status
         * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
         */
        case MAVLINK_MSG_ID_GPS_RAW_INT:
          {      
            m2h_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
          }
          break;

        /* Severity (could be critical)
         */
        case MAVLINK_MSG_ID_STATUSTEXT:
          {   
           #ifdef SERDB            
             DPL(mavlink_msg_statustext_get_severity(&msg));
           #endif
          }  
          break;

//        case MAVLINK_MSG_ID_SET_MODE:
//          {
//           #ifdef SERDB            
//             DPL(newmode = mavlink_msg_set_mode_get_mode(&msg));
//           #endif
//          }
//          break;

        /* Most messages are just discarded */
        default:
        //Do nothing
//        #ifdef SERDB
//          {
//            dbSerial.println("...");
//          }
//        #endif 
        break;
      }
    }
    delayMicroseconds(138);
    //next one
  }
  // Update global packet drops counter
  packet_drops += status.packet_rx_drop_count;
  parse_error += status.parse_error;
}

