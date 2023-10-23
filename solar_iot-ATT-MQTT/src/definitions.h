#define  VERSION_ID       F( "Solar IoT 2022 07 2.0" )

#define nobrownout true
#define lowtxpower true

// I/O definition
// ADC conversion ready
#define Ready           4

// MAX17043 Alert
#define Alert           27

// Inputs
#define DotSndData      33
#define DotWakeESP      35

// Outputs
#define ESPSentData     19
#define ESPLed          2
#define ESPRdy2Snd      18
//#define ESPWakeDot    26

#define DHT11Ext        14
#define DHT11Cab        23

// Other definitions
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300      /* Time ESP32 will go to sleep (in seconds) */
