/* This file contains defines and declarations for the Mavlink protocol, the mavlink decoder and LED sequencer */

/* Some basic defaults */
#define EN  1         // Enable value
#define DI  0         // Disable value

/* Pinout for serial debug */
#define hRXpin 11 // Not Used
#define hTXpin 12 // Serial out

/* PinOuts for Leds */
#define FR  Out[0]    // Rear right    port D6
#define RR  Out[1]    // Front left    port B0
#define RL  Out[2]    // Rear left     port B1
#define FL  Out[3]    // Front right   port D7
#define GPSLED Out[4] // GPS Status    port D4

#ifdef HEARTBEAT
  #define ledPin 5     // Port D5, Mavlink Heartbeat LED if any
#endif
/* direct port manupulation instead of digitalwrite. faster and smaller in code */
#define OutSize 6 /* number of output pins */
int Out[] = {6,7,8,9,5,4};  /* Output I/O pin array, order FR, RR, RL, FL and potential heartbeat */


//#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)
//
//#define LOW_BATT   3.35      /* low battery per cell for HoTT display */
//#define LOW_BATT_2 3.2       /* low battery per cell for LED warning */
//#define throttleMin 1100     /* Throttle min position for flighttimer */

///* Bearing calculation, define MAV to Home bearing (view from MAV use in OSD) or Home to MAV bearing (view from Home) */
//#define BEARING_MAV2HOME
////#define BEARING_HOME2MAV
//#define useMavHome           /* Set when using a special message in aq_mavlink to transmitt AQ's home coordinates. */

/* AutoQuad Mavlink status definitions, for other Mavlink flightcontrollers you can 'map' the correct status in this list */

//#define DISARMED 64                        /* MAV_STATE_STANDBY */
//#define ARMED 192                          /* mavlinkData.mode | MAV_MODE_FLAG_SAFETY_ARMED */
//#define STABILIZED 16                      /* MAV_MODE_FLAG_STABILIZE_ENABLED */
//#define ALT_HOLD ARMED + STABILIZED        /* mavlinkData.mode | MAV_MODE_FLAG_STABILIZE_ENABLED */
//#define POS_HOLD ARMED + STABILIZED + 1    /* mavlinkData.mode | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLE */
//#define MANUAL_MODE ARMED                  /* mavlinkData.mode | MAV_MODE_FLAG_SAFETY_ARMED */
//#define MISION_MODE ARMED + STABILIZED + 4 /* mavlinkData.mode | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED */
//#define DVH_MODE ARMED + STABILIZED + 8    /* mavlinkData.mode | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED */
//#define CARE_FREE 251           

//#define DISARMED     MAV_MODE_MANUAL_DISARMED
//#define ARMED        MAV_MODE_MANUAL_ARMED
//#define MANUAL_MODE  MAV_MODE_MANUAL_ARMED
//#define ALT_HOLD     MAV_MODE_MANUAL_ARMED + MAV_MODE_FLAG_STABILIZE_ENABLED
//#define POS_HOLD     MAV_MODE_MANUAL_ARMED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
//#define MISSION_MODE MAV_MODE_MANUAL_ARMED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_AUTO_ENABLED
//#define DVH_MODE     MAV_MODE_MANUAL_ARMED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED
//#define CARE_FREE    251 /* no definition in common.h yet */

#define MAV_STATE_STANDBY  3   /* System is grounded and on standby. It can be launched any time. | */
#define	MAV_STATE_ACTIVE   4   /* System is active and might be already airborne. Motors are engaged. | */
#define	MAV_STATE_CRITICAL 5   /* System is in a non-normal flight mode. It can however still navigate. | */

//#define WAYPOINT  16
//#define ORBIT     1
//#define RTH       20
//#define LAND      21
//#define TAKEOFF   22 

/* * * * * * * * * * * * * * * * * * * * * */

/* Global variables */
byte  r, g, b, r2, g2, b2;
byte ioCounter = 0;          /* Loop counter */
//static uint32_t p_cMillis = 0;
//byte variocounter3s = 0;     /* vario m/3s counter */
//byte variocounter10s = 0;    /* vario m/10s counter */
//static int counter = 0;      /* General counter */
//static int stripCounter =0;

/* MAVLink session control */
static boolean  mavbeat = 0;
static uint8_t  apm_mav_type;
static uint8_t  apm_mav_system; 
static uint8_t  apm_mav_component;
static boolean  mavlink_request = 0;
//static boolean  displayVersionDone = 0;

/* General states */
byte flMode;                                  /* Our current flight mode as defined */
byte isArmed = 0;
byte isArmedOld = 0;
byte isActive;

/* AutoQuad received values */
static uint16_t m2h_mode = 0;                   // Status mode (manual,stabilized etc)
static uint8_t  m2h_nav_mode = 0;               // Navigation mode
static uint8_t  m2h_sysstat = 0;                // system status (active,standby,critical)
static uint8_t  m2h_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D

// Unused for now:
//static float    m2h_vbat_A = 0;                 // Battery A voltage in milivolt
//static uint16_t m2h_battery_remaining_A = 0;    // 0 to 100 <=> 0 to 1000 calculated by the UAV (AutoQuad)
//static boolean  m2h_vbat_set = 0;               // first arrival correct voltage
//static uint8_t  m2h_num_cells = 0;              // number of cells
//static uint8_t  m2h_fuel_procent = 0;
//static uint8_t  version;
//static uint8_t  m2h_alarmDriveVoltage;
//static uint8_t  m2h_alarmTemp1;
//static uint8_t  m2h_sysstat_old = 0; 
//static uint16_t m2h_old_mode = 0;
//static int32_t  m2h_gps_lon = 0;
//static int32_t  m2h_gps_lat = 0;
//static int32_t  m2h_gps_alt = 0;
//static uint32_t m2h_gps_lon_home = 0;          // home position longitude
//static uint32_t m2h_gps_lat_home = 0;          // home position latitude
//static uint8_t  pos_NS;
//static uint8_t  pos_EW;
//static uint16_t pos_NS_dm;
//static uint16_t pos_NS_sec;
//static uint16_t pos_EW_dm;
//static uint16_t pos_EW_sec;
//static int      degMin;
//static float    home_distance_calc = 0;        // distance to home //
//static int8_t m2h_got_home = 0;              // tels if home position is set

//static uint16_t m2h_ceiling = 0;               // altitude ceiling from AQ = Max Altitude in HoTT
//static uint16_t alt_diff_1s = 0;
//static uint16_t alt_diff_3s = 0;
//static uint16_t alt_diff_10s = 0;
//static uint16_t alt_start_1s = 0;
//static uint16_t alt_start_3s = 0;
//static uint16_t alt_start_10s = 0;
//static uint16_t height_max = 0;
//
//static uint16_t throttle = 0;                  // throttle value

//static uint8_t free_char1;
//static uint8_t msg_number;  

/* time */
//static uint8_t minutes;
//static uint8_t seconds;


