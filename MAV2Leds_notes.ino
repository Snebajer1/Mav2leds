/*

  JWR; 20151016, copied from:_

  File       : AQ_mav_to_hott_notes
  Version    : v0.83, 10.02.2013
  Author     : Menno de Gans (joebar.rc@googlemail.com)

  
  TODO:
  - Beeper functions for highpower output
  - Altitude screen inverted when reaching ceiling
  
  - low powermodus ??, lower LED intensity of even less LED's when set (how?, switch, jumper, mavlink command)
  - Pattern matrix solution instead of hard-coded patterns

  Instructions:

  ! ! place the included libraries in your default sketchfolder so Arduino can find them ! !
  Look under file-preferences to see what you sketchbook location is and place the included libraries directory in that location.
  
  Port 12 (default) is used for low speed telemetry output for mavlink debugging but you can set every port you want.
  Serial port 0 (ftdi) is used for connection to Mavlink msg sender (QUAD)
  
  User configuratbles:
  In Mav2Leds.ino:
  Set the mavlink telemetry speed (default 57600 for AutoQuad)
  #define TELEMETRY_SPEED  57600 
  
  Examples:
  Use as AutoQuad mavlink to LED only
  //#define SERDB         // uncomment to output debug mavlink information to SoftwareSerial @ 19200 baud

  The default setting is for a quad with 1 led strips on each arm. You can define the ports, as you like (nonconflicting with each other or RX/TX pins)

#define FR  Out[0]    // Front right    port D6
#define RR  Out[1]    // Rear Right     port B0
#define RL  Out[2]    // Rear left      port B1
#define FL  Out[3]    // Front left     port D7

#define ledPin 5     // Heartbeat LED if any, port D5

Pinout for the Arduino (Pro) Mini
  int Out[] = {6,7,8,9,5,13};  // Output I/O pin array 

#define hRXpin 12 // Port B4   // Pinout for serial debug, RX not used
#define hTXpin 13 // Port B5   // Pinout for serial debug, output pin
*/
