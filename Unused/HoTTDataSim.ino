/* data simulation for testing HoTT protocol 
 * not used in normal operation
 */

void simulationData() {
   /*  a lot of offline test values to display * * * */
  int GPS_FIX = 0;    // just a test
  int ceiling = 50;
  
  // Status , capacity and flighttime related
  m2h_fuel_procent = 70;

  // Altitude related
  height = 39;   // gps height in gps screen
  msl_height = 103.2; // height above sea level
  height_max= ceiling;
  
  // Text related
  msg_number = 4;   // armed message
  if (seconds >= 30) {
    free_char1 = 87;  // ascii value 'W'
    free_char2 = 80;  // ascii value 'P'
  }else{
    free_char1 = 0;  // ascii value 'W'
    free_char2 = 0;  // ascii value 'P'
  }
  
  // Position & GPS related
  home_distance_calc = 2009.73;
  
  if (GPS_FIX == 1) {
    m2h_satellites_visible = 32;  // just a fake number because AQ does not report satellites
    m2h_fix_type =  51;  // ascii value '3' for 3D fix
  }else {
    m2h_satellites_visible = 0;
    m2h_fix_type =  45; // ascii value '-' for no fix
  }

  convertLat (52.0835);      // test coordinate
  pos_NS = NSEW;
  pos_NS_dm = degMin;
  pos_NS_sec = sec;
    convertLon (4.5258);
  pos_EW = NSEW;    
  pos_EW_dm = degMin;
  pos_EW_sec = sec;
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}

