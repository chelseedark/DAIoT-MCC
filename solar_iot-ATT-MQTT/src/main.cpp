#include <definitions.h>
//#include "Logging.h"

#include <ESP32Time.h>
#include "time.h"

#include <Arduino.h>
#include <Wire.h> // Needed for I2C

#include "MAX1704X.h"
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
//#include "ADS1X15.h"
#include "DHTesp.h"
#include <Ticker.h>

#include "soc/rtc.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <WiFi.h>
#include <BluetoothSerial.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_sleep.h>

// For UDP
#include <WiFiUdp.h>

char ssidf[] = "nombre_de_la_red";              // password for the WiFi network used
char passf[] = "clave_de_la_red";             

char deviceId[] = "BJkGwiTc0ROGhDYaQtcZlsW3"; 
char token[] = "maker:4dlokyHHBckwPe2kZTKg2ijKnkzotvHQnhYzqfs8"; 

String RSSIf;

// UDP variables
WiFiUDP UDP;
unsigned int localPort = 8888;
unsigned int remotePort = 8891;
const char* udp_server = "api.allthingstalk.io"; 
String UDPHead = "BJkGwiTc0ROGhDYaQtcZlsW3\nmaker:4dlokyHHBckwPe2kZTKg2ijKnkzotvHQnhYzqfs8\n";
uint8_t payload[26];

// NTP Server Sync
const char* ntpServer = "south-america.pool.ntp.org";
const long  gmtOffset_sec = -10800;
const int   daylightOffset_sec = 0;

// RTC
ESP32Time rtc(-10800);  // offset in seconds GMT-3
int16_t timeToSleep = 275;

// Array [H,M] to indicate timeframes of solar hours
// According to 
bool HourMonthMtx[24][12] = {
//     Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
/*0*/  {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
/*1*/  {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
/*2*/  {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
/*3*/  {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1},
/*4*/  {1,  1,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1},
/*5*/  {1,  1,  1,  1,  1,  0,  0,  1,  1,  1,  1,  1},
/*6*/  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*7*/  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*8*/  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*9*/  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*10*/ {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*11*/ {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*12*/ {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*13*/ {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*14*/ {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*15*/ {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*16*/ {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*17*/ {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
/*18*/ {1,  1,  1,  1,  0,  0,  0,  1,  1,  1,  1,  1},
/*19*/ {1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1},
/*20*/ {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
/*21*/ {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
/*22*/ {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
/*23*/ {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}
};

// ModbusClient TCP style
#include <ModbusClientTCPasync.h>

//char ssidm[] = "Ingeteam_0BM192510A85_5801"; // SSID and ...
//char passm[] = "ingeconsun"; // Password for the WiFi network used

char ssidm[] = "nombre_de_la_red";              // password for the WiFi network used
char passm[] = "clave_de_la_red";

//char ssidm[] = "CCCP";              // password for the WiFi network used
//char passm[] = "genkernela11";

//IPAddress ip = {169, 254, 1, 1};  // IP address of modbus server (Ingecon)
IPAddress ip = {192, 168, 0, 11};   // IP address of modbus (Mendeleyev)
//IPAddress ip = {192, 168, 175, 104};    // IP address of modbus server (AVV)
uint16_t port = 502;  // port of modbus server
String RSSIm;

bool noWiFi = false;                      

// Create a ModbusTCP client instance
ModbusClientTCPasync MB(ip, port);
RTC_DATA_ATTR uint8_t MBMsg[31];
/*uint8_t InvStatus[1];
uint8_t FVPower[2];
uint8_t FVVoltage[2];
uint8_t FVCurrent[2];*/
//uint8_t InvStatus;
//uint16_t FVPower;
//uint16_t FVVoltage;
//uint16_t FVCurrent;
bool MBTCPError = false;
bool DataReceived = false;
Error err;
uint32_t lastMillis;

// Temporary variables to save previous readings on error
//RTC_DATA_ATTR int8_t mbSts;
//RTC_DATA_ATTR int16_t mbP;
//RTC_DATA_ATTR int16_t mbV;
//RTC_DATA_ATTR int16_t mbI;
RTC_DATA_ATTR int16_t ADCRTC[3] = { 0, 0, 0 };
RTC_DATA_ATTR uint16_t Irr;
RTC_DATA_ATTR uint16_t Temp1;
RTC_DATA_ATTR uint16_t Temp2;
RTC_DATA_ATTR uint16_t TempC;
RTC_DATA_ATTR uint16_t HumC;
RTC_DATA_ATTR uint16_t Bat;
RTC_DATA_ATTR uint16_t VBat;
RTC_DATA_ATTR bool InitRTC = false;

// Error counters
RTC_DATA_ATTR int16_t cntMBStsErr;
RTC_DATA_ATTR int16_t cntADSErr;
RTC_DATA_ATTR int16_t cntMAXErr;

// I2C ADC for Irradiance and temperature
//ADS1115 ADS(0x48);
Adafruit_ADS1115 ADS;
volatile bool RDY = false;
//volatile bool conv = false;
uint8_t channel = 0;

// Data from ADC // ADC[0] Irradiance // ADC[1] temp1 // ADC[0] temp2
int16_t ADC[4] = { 0, 0, 0, 0 };
float Rad;
float Tp1;   
float Tp2;

// I2C battery level gauge (0x36)
MAX1704X Lipo = MAX1704X(1.25); //1.2491

// Auxiliary variables
bool DhtFail = false;
bool ADSFail = false;
RTC_DATA_ATTR bool MAXFail = false;
bool MBTCPFail = false;

// Initialize DHT sensor
DHTesp dhtCab;
TempAndHumidity THCabinet;// Data from sensor

// Define an onData handler function to receive the regular responses
// Arguments are Modbus server ID, the function code requested, the message data and length of it, 
// plus a user-supplied token to identify the causing request
void handleData(ModbusMessage response, uint32_t token) 
{
  //printf("Response --- FC:%02X Server:%d Length:%d\n", 
  //response.getFunctionCode(), 
  //response.getServerID(), 
  //response.size());
  //HEXDUMP_N("Data dump", response.data(), response.size());
  
  //Serial.println(F("!\n"));

  //Map bytes to an array
  int i = 0;
  for (auto& byte : response) {
    MBMsg[i] = byte;
    //Serial.printf("%02X ", byte);
    i++;
  }
  //Serial.println(F("!!\n"));
  MBTCPError = false;
  DataReceived = true;
}

// MBTCP connection error handler
void handleError(Error error, uint32_t token) 
{
  ModbusError me(err);
  //printf("Error response: %s (%02X)\n", (const char *)me, err);
  MBTCPError = true;
  DataReceived = false;
}

// Disable WiFi when entering Deep Sleep
void disableWiFi(void){
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
}

// Enable the connection to a WiFi AP 
void enableWiFi(void){
    WiFi.disconnect(false);   // Reconnect the network
    WiFi.mode(WIFI_STA);      // Switch WiFi 
    //Serial.println(F("Wi-Fi Enabled...\n"));
    //WiFi.enableLongRange(true); 
}

// Connect to a WiFi AP
bool connectWiFi(char* ssid, char* pass){
    //WiFi.disconnect(true);  // Disconnect from the network
    //WiFi.disconnect(false);   // Reconnect the network
    WiFi.begin(ssid, pass);
    //  Serial.println(F("Connecting to Wi-Fi network...\n"));
    
    bool Timeout = false;   
    int count = 0;
    unsigned long lastTime = millis();
    while ((WiFi.status() != WL_CONNECTED) and (!Timeout)) 
    {
        if (((millis()-lastTime) > 5000)) //Repeat the connection 3 times
        {
          count++;  
          WiFi.disconnect(true);
          WiFi.mode(WIFI_STA);      // Switch WiFi off
          WiFi.begin(ssid, pass);
          lastTime = millis();
          //  Serial.println(F("Connection FAILED\n"));
        } 

        if (count > 2)
        {
          Timeout = true;  
        }
    }    
    //Serial.println(F("Connected?...\n"));
    return Timeout;
}

// Disable BT to lower the consumption
void disableBluetooth(void){
    btStop();
    esp_bt_controller_disable();
    //delay(1000);
    //Serial.println("BT STOP");
}

// Join Bytes from the MBTCP response to a single register
uint16_t JoinBytes(uint8_t low, uint8_t high)
{
  uint16_t result = low;
  result = (result << 8) + high;
  return result;
}

bool ConnectUDP(void) {
	//Serial.println();
	//Serial.println("Starting UDP");

	// in UDP error, block execution
	if (UDP.begin(localPort) != 1) 
	{
	//	Serial.println("Connection failed");
    return true;
		//while (true) { delay(1000); } 
	} else {
	//  Serial.println("UDP successful");
    return false;
  }
}

void SendUDP_Packet(void)
{
  // Send ABCL Payload
  UDP.beginPacket(udp_server, remotePort);
  UDP.printf(UDPHead.c_str());
  UDP.write(payload,sizeof(payload));
  UDP.endPacket();
}

// =================================================================================
/* 
                    ______   _______   _______   __   __   _______ 
                  |  _____| |    ___| |_     _| |  | |  | |    _  |
                  | |_____  |   |___    |   |   |  | |  | |   |_| |
                  |_____  | |    ___|   |   |   |  |_|  | |    ___|
                   _____| | |   |___    |   |   |       | |   |    
                  |_______| |_______|   |___|   |_______| |___|         
*/    
// =================================================================================    

void setup() {
  if(nobrownout) WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  disableBluetooth();

  // Init Serial monitor for debug
  Serial.begin(115200);
  while (!Serial) {}

  // Sync RTC with NTP Server
  if (!InitRTC)
  {
    // Init and get the time
    //Serial.println(rtc.getTime("\n%A, %B %d %Y %H:%M:%S"));
    if (!connectWiFi(ssidf, passf))
    {
      struct tm timeinfo;
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      getLocalTime(&timeinfo);
      WiFi.disconnect(true);
      InitRTC = true;
      rtc.setTimeStruct(timeinfo);
      //Serial.println(F("RTC config successful!\n"));
    } else {
      //Serial.println(F("Wi-Fi connection for RTC Configuration FAILED\n"));
    } 
  } else {
    //Serial.println(F("RTC already configured\n"));
    // Check the month and hour to determine the sampling frequency
    // according to https://globalsolaratlas.info/ for the location
    int hour = rtc.getHour(true);
    if (!MAXFail)
    {
      if (Bat >= 20)
      {
        if (HourMonthMtx[hour][rtc.getMonth()])
        {
          timeToSleep = 280;
        } else {
          timeToSleep = 1780;
        }
      } else {
        if ((Bat < 20) & (Bat >= 10))
        {
          timeToSleep = 580;
        }

        if ((Bat < 10) & (Bat >= 50))
        {
          timeToSleep = 1180;
        }
        
        if ((Bat < 5))
        {
          timeToSleep = 3580;
        }
      }
    } else {
      if (HourMonthMtx[hour][rtc.getMonth()])
      {
        timeToSleep = 280;
      } else {
        timeToSleep = 1780;
      }
    }

    // If it's the first day of the week in the morning
    // Sync the RTC due to ESP32 internal oscillator inaccuracy
    if ((rtc.getDayofWeek() == 0) & (hour == 6))
    {
      InitRTC = false;
    }
  }

  // Enable wakeup source
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
  //esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  // Indicator LED
  pinMode(ESPLed, OUTPUT);
  digitalWrite(ESPLed, HIGH);

  // Set up ModbusTCP client.
  // - provide onData handler function
  MB.onDataHandler(&handleData);
  // - provide onError handler function
  MB.onErrorHandler(&handleError);
  // Set message timeout to 5000ms and interval between requests to the same host to 200ms
  MB.setTimeout(8000);
  // Start ModbusTCP background task
  MB.setIdleTimeout(30000);
  
  // Setup the MAX14043 fuel gauge
  int count = 0;
  bool MAXconnected = Lipo.begin();
  unsigned long lastTime = millis();

  // If there's a failure, retry 3 times
  while ((!MAXconnected) and (!MAXFail)) 
  {
      if (((millis()-lastTime) > 200)) //Repeat the connection 3 times
      {
        count++;  
        MAXconnected = Lipo.begin();
        lastTime = millis();
      } 

      if (count > 2)
      {
        MAXFail = true;  
          Serial.println(F("Connection to the Fuel Gauge FAILED\n"));
      } else {
        MAXFail = false;
      }
  }

  // Setup ADS1115 ADC
  count = 0;
  bool ADSconnected = ADS.begin();
  lastTime = millis();

  // If there's a failure, retry 3 times
  while ((!ADSconnected) and (!ADSFail)) 
  {
      if (((millis()-lastTime) > 200)) //Repeat the connection 3 times
      {
        count++;  
        ADSconnected = ADS.begin();
        lastTime = millis();
      } 

      if (count > 2)
      {
        ADSFail = true;  
          Serial.println(F("Connection to the ADS FAILED\n"));
      } else {
        ADSFail = false;  
      }
  }

  // Initialize DHT11 Cabinet and Exterior temperature sensors
  //  Serial.println(F("Set DHT cab.\n"));
  dhtCab.setup(DHT11Cab, DHTesp::DHT11);
    Serial.println(F("Set DHT ext.\n"));
  //dhtExt.setup(DHT11Ext, DHTesp::DHT11);

  // Enable WiFi to read modbus data
  enableWiFi();
  noWiFi = connectWiFi(ssidm, passm);
}

// ==================================================================
/*
                   ___      _______  _______  _______ 
                  |   |    |       ||       ||       |
                  |   |    |   _   ||   _   ||    _  |
                  |   |    |  | |  ||  | |  ||   |_| |
                  |   |___ |  |_|  ||  |_|  ||    ___|
                  |       ||       ||       ||   |    
                  |_______||_______||_______||___|    
*/
// ==================================================================
void loop() {  
  // Read data from the 1Play inverter
  if (!noWiFi){
    payload[9] = WiFi.RSSI();
    //Serial.printf("RSSI: %d\n", payload[9]);
    Error err = MB.addRequest(1234, 1, READ_INPUT_REGISTER, 12, 14);
    DataReceived = false;
    if (err != SUCCESS) {
      delay(500);
      err = MB.addRequest(2345, 1, READ_INPUT_REGISTER, 12, 14);

      //ModbusError e(err);
      //Serial.printf("Error creating request: %02X - %s\n", (int)e, (const char *)e);
      //Serial.printf("Millis: %d",(uint32_t)lastMillis);
    }

    if (err != SUCCESS) {
      delay(1000);
      err = MB.addRequest(3456, 1, READ_INPUT_REGISTER, 12, 14);
      //Serial.println(F("Error1\n"));
    }

    if (err != SUCCESS) {
      ModbusError e(err);
      payload[24] &= ~(1 << 1);
      //Serial.println(F("MBTCP reading failed\n"));
      cntMBStsErr++;
      if (cntMBStsErr > 4)
      {
        cntMBStsErr = 0;
        MBMsg[4] = 0;
        MBMsg[25] = 0;
        MBMsg[26] = 0;
        MBMsg[27] = 0;
        MBMsg[28] = 0;
        MBMsg[29] = 0;
        MBMsg[30] = 0;
      } 
      WiFi.disconnect();
    } else {
      // Process Status reading
      bool MBError = false;

      // Timeout on receiving status reading
      int lastTime = millis();
      while (!DataReceived and !MBError) {
        if (((millis()-lastTime) > 3000))
        {
          MBError = true;
          //  Serial.println(F("MBTCP Status reading failed\n"));
        }  
      }

      if (MBError){
          payload[24] &= ~(1 << 1);
          cntMBStsErr++;
          if (cntMBStsErr > 4)
          {
            cntMBStsErr = 0;
            MBMsg[4] = 0;
            MBMsg[25] = 0;
            MBMsg[26] = 0;
            MBMsg[27] = 0;
            MBMsg[28] = 0;
            MBMsg[29] = 0;
            MBMsg[30] = 0;
          } 
      } else {
        payload[24] |= (1 << 1);
        cntMBStsErr = 0;
      }
      DataReceived = false;
    }
      // Process readings
      //InvStatus = MBMsg[4];
      payload[0] = MBMsg[4];
      //FVCurrent = JoinBytes(MBMsg[25],MBMsg[26]);
      payload[1] = MBMsg[7];
      payload[2] = MBMsg[8];
      //FVCurrent = JoinBytes(MBMsg[25],MBMsg[26]);
      payload[3] = MBMsg[11];
      payload[4] = MBMsg[12];
      //FVVoltage = JoinBytes(MBMsg[27],MBMsg[28]);
      payload[5] = MBMsg[25];
      payload[6] = MBMsg[26];
      //FVPower = JoinBytes(MBMsg[29],MBMsg[30]);
      payload[7] = MBMsg[29];
      payload[8] = MBMsg[30];

      /*FVCurrent[1] = MBMsg[26];
      FVVoltage[0] = MBMsg[27];
      FVVoltage[1] = MBMsg[28];
      FVPower[0] = MBMsg[29];
      FVPower[1] = MBMsg[30];*/
      //Serial.printf("Power: %d",FVPower);
      WiFi.disconnect();

  } else {
    //Serial.println("Error connecting");
    payload[24] &= ~(1 << 1);
  }

  // Read Irradiance and temperature data
  if (!ADSFail)
  {
    // Indicate the status of the ADC In the Status Register
    payload[24] |= (1 << 2);

    //  Serial.println(F("ADS1115 found.\n"));
    ADS.setGain(GAIN_TWOTHIRDS);           // 6.144 volt
    ADS.setDataRate(RATE_ADS1115_16SPS);   // 16 samples per second
    cntADSErr = 0;

    // Dummy read ADC channel 3
    //Serial.printf("\nAIN3: %d\n", ADC[3]);
    
    // Convert PT1000 Temperature readings
    // Low sensor
    ADC[2] = ADS.readADC_SingleEnded(2);
    Tp2 = (ADC[2] - 9821)/1.886;
    Temp2 = round(Tp2);
    payload[15] = (Temp2 >> 8) & 0xFF;
    payload[16] = Temp2 & 0xFF;
    //Serial.printf("T1: %d\n", ADC[2]);

    // High sensor
    ADC[1] = ADS.readADC_SingleEnded(1);
    Tp1 = (ADC[1] - 9771)/1.886;
    Temp1 = round(Tp1);
    payload[13] = (Temp1 >> 8) & 0xFF;
    payload[14] = Temp1 & 0xFF;
    //Serial.printf("T1: %d\n", ADC[1]);
    
    // Read and process Irradiance
    ADC[0] = ADS.readADC_SingleEnded(0);
    delay(100);
    ADC[0] = ADS.readADC_SingleEnded(0);

    if (ADC[0] < 340){
      Irr = 0;
    } else {
      Rad = (ADC[0] - 340)/1.93871;
      Irr = round(Rad);
    }
    payload[11] = (Irr >> 8) & 0xFF;
    payload[12] = Irr & 0xFF;
    //Irradiance = ADC[0];
    //Serial.printf("Irradiance: %d\n", ADC[0]);
    //  Serial.printf("\nIrradiance: %d\n", Irradiance);
    // Split readings into bytes

  } else {
    //  Serial.println(F("ADS Failed\n"));
    payload[11] = (Irr >> 8) & 0xFF;
    payload[12] = Irr & 0xFF;
    payload[13] = (Temp1 >> 8) & 0xFF;
    payload[14] = Temp1 & 0xFF;
    payload[15] = (Temp2 >> 8) & 0xFF;
    payload[16] = Temp2 & 0xFF;
    // Indicate the status of the ADC In the Status Register
    payload[24] &= ~(1 << 2);
    //  Serial.println(F("ADS1115 NOT found.\n"));
    cntADSErr++;
    if (cntADSErr > 2)
    {
      Irr = 0;
      Temp1 = 0;
      Temp2 = 0;
    }
  }

  // Read battery charge
  if (!MAXFail)
  {
    //  Serial.println(F("MAX17043 found.\n"));
    Lipo.quickstart();

    if (Lipo.isSleeping())
    {
      Lipo.wake();
    }

    uint16_t VBatPrev = VBat;
    VBat = Lipo.voltage();
    payload[21] = (VBat >> 8) & 0xFF;
    payload[22] = VBat & 0xFF;
    //int PercentMAX = Lipo.percent();
    //BattPercent = round(Voltage*0.1-320); //For vmin = 3.2V
    Bat = round(VBat * 0.2-740);   //For vmin = 3.7V
    //  Serial.printf("Batería: %d\n", PercentMAX);
    //  Serial.printf("Tensión: %d\n", Voltage);
    
    if (VBat >= VBatPrev){
      payload[24] |= (1 << 0);
        //  Serial.println(F("% Batería cargando\n"));
        } else {
      payload[24] &= ~ (1 << 0);
        //  Serial.println(F("% Batería descargando\n"));
    }

    if (Bat > 100){
      payload[23] = 100;
    } else {
      payload[23] = Bat & 0xFF;
    }

    // Indicate the status of the Gauge In the Status Register
    payload[24] |= (1 << 3);
    cntMAXErr = 0;

    // Send the Fuel Gauge to sleep
    Lipo.sleep();
  } else {
    // Indicate the status of the Gauge In the Status Register
    payload[24] &= ~(1 << 3);
    payload[21] = (VBat >> 8) & 0xFF;
    payload[22] = VBat & 0xFF;
    payload[23] = Bat;
    //  Serial.println(F("MAX17043 NOT found.\n"));
    cntMAXErr++;
    if (cntMAXErr > 2)
    {
      Bat = 0;
      VBat = 0;
    }
  }
  
  // DHT11 sensor readings 
	THCabinet = dhtCab.getTempAndHumidity();	// Read values from DHT sensor
  TempC = round(THCabinet.temperature*100.0);
  HumC = round(THCabinet.humidity*100.0);

  //Serial.printf("T gabinete: %d\n", TempC);
  //Serial.printf("H gabinete: %d\n", HumC);

  if (TempC == 65535){
    TempC = 0;
    HumC = 0;
    payload[24] &= ~(1 << 4);
  } else {
    payload[24] |= (1 << 4);
  }
  payload[17] = (TempC >> 8) & 0xFF;
  payload[18] = TempC & 0xFF;
  payload[19] = (HumC >> 8) & 0xFF;
  payload[20] = HumC & 0xFF;
  
  // Send the packet twice because ATT may not receive one properly
  if (!connectWiFi(ssidf, passf))
  {
    payload[10] = WiFi.RSSI();
    if (!ConnectUDP()){
      SendUDP_Packet();
      //SendUDP_Packet();
    }
    delay(5000);
  }

  //Serial.println(rtc.getTime("\n%A, %B %d %Y %H:%M:%S"));
  disableWiFi();

  Serial.printf("Segundos transcurridos: %d\n", millis()/1000);
  //Serial.println(F("\nEntering Deep Sleep\n"));
  esp_deep_sleep_start();
}