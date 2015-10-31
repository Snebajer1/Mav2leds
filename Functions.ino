void timerEvent() {  /* this event is called @ 10Hz */
  ioCounter++ ;     /* update counter to use in all blink loops */

  /* 10Hz calls */
  
  /* Only count from 0 to 10 */
  if (ioCounter >10) ioCounter =0;
  /* do the LED stuff */
  LEDControl();
  heartBeat();
  /* end 10Hz calls */

  /* 1Hz calls */
  if (ioCounter == 1){ /* Nothing done at this frequency */ }
  
  /* end timer event */
}

// Megapiratenr 2.8r3 flight modes, set in m2h_nav_mode
// ----------------
//#define STABILIZE 0			// hold level position
//#define ACRO 1			// rate control
//#define ALT_HOLD 2			// AUTO control
//#define AUTO 3			// AUTO control
//#define GUIDED 4			// AUTO control
//#define LOITER 5			// Hold a single location
//#define RTL 6				// AUTO control
//#define CIRCLE 7			// AUTO control
//#define POSITION 8			// AUTO control
//#define LAND 9			// AUTO control
//#define OF_LOITER 10                  // Hold a single location using optical flow sensor

// Set flightmode from heartbeat m2h_navn_mode and whether mavlink is active
void CheckFlightMode() {
  if((mavlink_active == 1))
  {
    flMode = m2h_nav_mode + isArmed + 1;                           // flMode 0 means no mavlink heartbeat received
    if(m2h_sysstat == MAV_STATE_CRITICAL) flMode = 12;   // Drone in crisis, handled specially
  }
  else
  {
    flMode = 0;                // disarmed - no mavlink data
  }
}

// Checking if BIT is active in PARAM, return true if it is, false if not
byte isBit(byte param, byte bitfield) {
 if((param & bitfield) == bitfield) return 1;
  else return 0;  
}

void heartBeat(){
   #ifdef HEARTBEAT
    digitalWriteFast(ledPin, mavlink_active);
    messageCounter++;
   #endif

   /* check if Mavlink is lost */   
   if(messageCounter >= 15 && mavlink_active) {
     #ifdef SERDB  
        DPL("!!! We lost MAVLink !!!");
     #endif
     mavlink_active = 0;
     CheckFlightMode();
     messageCounter = 0;
     mavlink_request = 1;
   }
   
   if(messageCounter >250) messageCounter = 0;  /* to prevent overflow */
   #ifdef SERDB  
   if(messageCounter == 250){
     DPL("??? (Still) no mavlink       ???");
   }
   #endif
}

/* Converts unsigned long representation of GPS coordinate
 * Mavlink transmits the coordinates as coordinate * 1E7
 */
//void convertLat (uint32_t in_coords){
//    if(in_coords >= 0) {
//    pos_NS = 0;
//  } else {
//    pos_NS = 1;
//  }
//  pos_NS_dm = in_coords/100000;                 /* Get the first digits by integer divide by 100.000 and thus removing the floating point */
//  pos_NS_sec = in_coords - (degMin*100000);      /* And deduct it from the original coords to get the sec */
//}
//  
//void convertLon (uint32_t in_coords){
//    if(in_coords >= 0) {
//    pos_EW = 0;
//  } else {
//    pos_EW = 1;
//  }
//  pos_EW_dm = in_coords/100000;                 /* Get the first digits by integer divide by 100.000 and thus removing the floating point */
//  pos_EW_sec = in_coords - (degMin*100000);     /* And deduct it from the original coords to get the sec*/
//}
//
/*************************************************************************
 * //Function to calculate the distance and bearing between two waypoints
 *************************************************************************/
//  void calcDist(float flat1, float flon1, float flat2, float flon2){
//  float x = ( (flon2 - flon1) * DEG_TO_RAD) * cos ( ( (flat1 * DEG_TO_RAD) +(flat2 * DEG_TO_RAD) )/2);
//  float y = ( (flat1 * DEG_TO_RAD) - (flat2 * DEG_TO_RAD) );
//  home_distance_calc = sqrt(x*x + y*y) * NAV_EQUATORIAL_RADIUS;
//  
//  flat2 = flat2 * DEG_TO_RAD;
//  flat1 = flat1 * DEG_TO_RAD;
//  flon2 = flon2 * DEG_TO_RAD;
//  flon1 = flon1 * DEG_TO_RAD;
//  
//  #ifdef BEARING_HOME2MAV
//    bearing = atan2(sin(flon2-flon1)*cos(flat2), cos(flat1)*sin(flat2)-sin(flat1)*cos(flat2)*cos(flon2-flon1)), 2*M_PI;
//    bearing = bearing * RAD_TO_DEG;  // convert from radians to degrees
//    int bearing = bearing; //make it a integer now
//    if(bearing<0) bearing+=360;   //if the heading is negative then add 360 to make it positive
//    if(bearing>360) bearing-=360;
//  #endif
//
//  #ifdef BEARING_MAV2HOME
//    bearing = atan2(sin(flon1-flon2)*cos(flat1), cos(flat2)*sin(flat1)-sin(flat2)*cos(flat1)*cos(flon1-flon2)), 2*M_PI;
//    bearing = bearing * RAD_TO_DEG;  // convert from radians to degrees
//    int bearing = bearing; //make it a integer now
//    if(bearing<0) bearing+=360;   //if the heading is negative then add 360 to make it positive
//    if(bearing>360) bearing-=360;
//  #endif
//
//  if (m2h_got_home) free_char1 = char ('.');
//}

