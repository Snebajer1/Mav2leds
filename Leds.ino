/* LED patterns are hard coded to the flightstatus number but use your imagination to write your own patterns */

void startLED(){
  digitalWriteFast(FR, (ioCounter == 0 || ioCounter == 4) ? HIGH : LOW);
  digitalWriteFast(RR, (ioCounter == 1 || ioCounter == 5) ? HIGH : LOW);    /* high on ioCounter 0 or 5 pass, low on all other */
  digitalWriteFast(RL, (ioCounter == 2 || ioCounter == 6) ? HIGH : LOW);        
  digitalWriteFast(FL, (ioCounter == 3 || ioCounter == 7) ? HIGH : LOW);    /* high on ioCounter 0 or 5 pass, low on all other */
}

// #undef SERDB
// ----------------
//#define UNITIALIZED 0
//#define STABILIZE 1			// autohold level 
//#define ACRO 2			// rate control
//#define ALT_HOLD 3			// AUTO control
//#define AUTO 4			// AUTO control
//#define GUIDED 5			// AUTO control
//#define LOITER 6			// Hold a single location
//#define RTL 7				// AUTO control
//#define CIRCLE 8			// AUTO control
//#define POSITION 9			// AUTO control
//#define LAND 10			// AUTO control
//#define OF_LOITER 11                  // Hold a single location using optical flow sensor
//#define CRITICAL 12  
void LEDControl() {

  // GPS Status pattern encoding; 0-1: no fix, 2: 2D fix, 3: 3D fix.
  switch (m2h_fix_type) {
    case 0:          // No GPS attached
      {
         digitalWriteFast(GPSLED, ((ioCounter & 1) == 1) ? HIGH : LOW);
      }
      break;
    case 1:          // No fix yet, Slow flash
      {
         digitalWriteFast(GPSLED, (ioCounter > 5) ? HIGH : LOW);
      }
      break;
    case 2:          // 2D fix, quick dual flash
      {
        digitalWriteFast(GPSLED, ((ioCounter & 4) == 4) ? HIGH : LOW);
      }
      break;    
    case 3:          // 3D fix, steady on
      {
        digitalWriteFast(GPSLED, HIGH);
      }
      break;    
  }
    
  // Flight mode led pattern encoding
  switch (flMode) {
  case 0:            // circle fast (uninitialized)
    {
      startLED();
//      AllOff();
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Uninitialized    ...");
      #endif
    }
    break;    

  case 1:            // front and read leds flash slow in sync (disarmed)
    {
      digitalWriteFast(FL, (ioCounter < 5) ? HIGH : LOW);
      digitalWriteFast(FR, (ioCounter < 5) ? HIGH : LOW);
      digitalWriteFast(RR, (ioCounter < 5) ? HIGH : LOW);
      digitalWriteFast(RL, (ioCounter < 5) ? HIGH : LOW);
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Disarmed         ...");
      #endif
    }
    break;    

  case 2:            // front led on, rear led on (armed & manual flight)
    {
      digitalWriteFast(FL, EN);
      digitalWriteFast(FR, EN);
      digitalWriteFast(RR, EN);
      digitalWriteFast(RL, EN);
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & Manual   ...");
      #endif

    }
    break;    

  case 3:           // front led on, rear leds 1 short flash, inverted (armed & alt hold (no GPS) )
    {
      digitalWriteFast(FL, EN);
      digitalWriteFast(FR, EN);
      digitalWriteFast(RR, (ioCounter == 0) ? LOW : HIGH);
      digitalWriteFast(RL, (ioCounter == 0) ? LOW : HIGH);  
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & AltHold  ...");
      #endif
    }
    break;

  case 4:           // front led on, rear led dual flash, inverted (armed & position hold)
    {
      digitalWriteFast(FL, EN);
      digitalWriteFast(FR, EN);
      digitalWriteFast(RR, (ioCounter == 0 || ioCounter == 4 ) ? LOW : HIGH);
      digitalWriteFast(RL, (ioCounter == 0 || ioCounter == 4 ) ? LOW : HIGH);
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & PosHold  ...");
      #endif
    }
    break;     

  case 5:         // front leds slow flash interchanging, rear leds slow flash interchanging (armed & mission mode)
    {
      digitalWriteFast(FL, (ioCounter < 5) ? HIGH : LOW);
      digitalWriteFast(FR, (ioCounter < 5) ? LOW : HIGH);
      digitalWriteFast(RR, (ioCounter > 4) ? HIGH : LOW);
      digitalWriteFast(RL, (ioCounter > 4) ? LOW : HIGH);  
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & Mission  ...");
      #endif
    }
    break;    

  case 6:         // front led on, rear led flash slow (armed & DVH )
    {
      digitalWriteFast(FL, EN);
      digitalWriteFast(FR, EN);
      digitalWriteFast(RR, (ioCounter == 0 || ioCounter ==1) ? HIGH : LOW); 
      digitalWriteFast(RL, (ioCounter == 0 || ioCounter ==1) ? HIGH : LOW);  
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & DVH      ...");
      #endif
    }
    break;    

  case 7:       // TBD (CareFree)
    {
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & CareFree ...");
      #endif

    }
    break; 

  case 8:           // front led on, rear leds triple flash, inverted (Return to home)
    {
      digitalWriteFast(FL, EN);
      digitalWriteFast(FR, EN);
      digitalWriteFast(RR, (ioCounter == 0 || ioCounter == 3 || ioCounter == 7 ) ? LOW : HIGH);
      digitalWriteFast(RL, (ioCounter == 0 || ioCounter == 3 || ioCounter == 7 ) ? LOW : HIGH);
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & RTH      ...");
      #endif

    }
    break; 

  case 9:       // TBD (circle)
    {
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & Circle   ...");
      #endif

    }
    break; 

  case 10:       // Circling leds, slow (land)
    {
      digitalWriteFast(RR, (ioCounter == 0 || ioCounter ==1) ? LOW : HIGH);
      digitalWriteFast(RL, (ioCounter == 3 || ioCounter ==4) ? LOW : HIGH);        
      digitalWriteFast(FR, (ioCounter == 5 || ioCounter ==6) ? LOW : HIGH);
      digitalWriteFast(FL, (ioCounter == 8 || ioCounter ==9) ? LOW : HIGH);
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & Land     ...");
      #endif
    }
    break; 

  case 11:       // Circling leds, slow (loiter)
    {
//      digitalWriteFast(RR, (ioCounter == 0 || ioCounter ==1) ? LOW : HIGH);
//      digitalWriteFast(RL, (ioCounter == 3 || ioCounter ==4) ? LOW : HIGH);        
//      digitalWriteFast(FR, (ioCounter == 5 || ioCounter ==6) ? LOW : HIGH);
//      digitalWriteFast(FL, (ioCounter == 8 || ioCounter ==9) ? LOW : HIGH);
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & Loiter     ...");
      #endif
    }
    break; 

  case 12:      // front leds PWM flash and panic blinking rear leds (CRITICAL ?)
    {
      digitalWriteFast(FR, (ioCounter == 5 || ioCounter ==6) ? HIGH : LOW);
      digitalWriteFast(FL, (ioCounter == 5 || ioCounter ==6) ? HIGH : LOW);        
      digitalWriteFast(RR, (ioCounter == 0 || ioCounter ==5) ? HIGH : LOW);
      digitalWriteFast(RL, (ioCounter == 3 || ioCounter ==8) ? HIGH : LOW);
      #ifdef SERDB
        if(ioCounter == 0) DPL("... F-Mode: Armed & Panic    ...");
      #endif
    }
    break;    

  default:
    AllOff();   // No valid signal, all leds off
    break;
  }
}
#define SERDB
// Switch all outputs ON
void AllOn() {
  for(int looper = 0; looper < OutSize; looper++) {
    digitalWriteFast(Out[looper], EN);
  }  
}

// Switch all outputs OFF
void AllOff() {
  for(int looper = 0; looper < OutSize; looper++) {
    digitalWriteFast(Out[looper], DI);
  }  
}

