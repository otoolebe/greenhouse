// Major Revision 2020-06-23: Add OTA Update Support
// Minor Revision 2020-06-23: OTA Support Debugging

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeLib.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Ezo_i2c.h>
#include <ADS1115.h>

#define Addr_MCP23008 0x20                                                                        // MCP23008_I2CR8G5LE I2C address is 0x20(32)

// Set enumerations for process devices
#define SV_WATER 7                                                                                // Water Dosing Solenoid
#define P_NUTDEL 2                                                                                // Nutrient Delivery Pump
#define P_NUTREC 1                                                                                // Nutrient Recirculation Pump
#define P_AIR 0                                                                                   // Aeration Pump

// Set enumerations for task periods
#define PERIOD_500MS 0
#define PERIOD_1000MS 1
#define PERIOD_2000MS 2
#define PERIOD_5000MS 3
#define PERIOD_30000MS 4
#define PERIOD_180000MS 5
#define PERIOD_1H 6

// Configure WiFi Client
WiFiClient espClient;
const char* ssid = "Ptarmigan";
const char* password =  "johnbryan";

// Configure NTP Client
WiFiUDP Udp;
unsigned int localPort = 8888;
IPAddress timeServer(192, 168, 10, 2);
int timeZone = -4;                                                                                // Eastern Daylight Time (USA)   
const int NTP_PACKET_SIZE = 48;                                                                   // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE];                                                               // Buffer to hold incoming & outgoing packets

// Configure MQTT Client
IPAddress mqtt_server(192, 168, 10, 184);
PubSubClient mqtt_client(espClient);

//Define Delivery Sequence varibles
enum DelStep {DISABLE, ANALYZE, IDLE, AERATE, DELIVER, DRAIN };                                   //Define nutrient delivery sequence steps
enum DelStep DelStep_Cur = DISABLE;                                      
enum DelStep DelStep_Prev = IDLE;
enum DelStep DelStep_Prop = DISABLE;
bool DelSeq_StepChange = false;
bool DelSeq_Abort = false;
bool DelSeq_Abort_OCmd = false;

unsigned long DelSeq_StepChange_TimeStamp = now();                                                // Last step change timestamp (Unix Time)
unsigned long DelSeq_Timer = 0;                                                                   // Delivery sequence timer (s)
unsigned long DelStepTime_Analyze = 300;                                                          // Analyze step duration                                             
unsigned long DelStepTime_Idle = 2365;                                                            // Idle step duration  
unsigned long DelStepTime_Aerate = 300;                                                           // Aerate step duration  
unsigned long DelStepTime_Deliver = 45;                                                           // Delivery step duration  
unsigned long DelStepTime_Drain = 300;                                                            // Drain step duration  
int DelStep_Countdown;                                                                            // Countdown until delivery step

// Define functions
int config(String command);
int Diag(String DiagData);

// Define Water Dosing Commands
bool WatDos_Init = false;
bool WatDos_Active = false;
float WatDos_SP_sec = 0;
float WatDos_SP_gal = 0;
float WatDos_Act_sec;
float WatDos_Act_gal;
unsigned long WatDos_StartTime;
unsigned long WatDos_EndTime;
unsigned long WatDos_OffTime;

// Define global relay-control variables
bool RelayAuto[8];
bool AutoCmd[8];
bool ManualCmd[8];
bool RelayOut[8];
int DO01_Cmd;
int DO01_Sts;
int DO01_Cmd_Prev;
int DO01_Sts_Prev = 0;
int DO01_Fault = false;

// Define level limit variables
float TomNutRes_LowLvl_SP = 1.8;
bool NutDel_LowLvlOvrd;

// Scan time and periodic task variables
int ScanDelaySP = 250;
unsigned int Scantime_prev = 0;
unsigned long micros_prev = 0;
unsigned int Periodic_set[7];
unsigned int Periodic_acc[7];
bool Periodic_flg[7];

// Atlas Sensor Objects and Variables
Ezo_board PH = Ezo_board(99, "PH");                                                               //create a pH circuit object, who's I2C address is 99 and name is "PH"
Ezo_board EC = Ezo_board(100, "EC");                                                              //create a EC circuit object, who's I2C address is 100 and name is "EC"
Ezo_board RTD = Ezo_board(102, "RTD");                                                            //create an RTD circuit object who's I2C address is 102 and name is "RTD"

enum reading_step {READ_LEVEL, REQUEST_TEMP, READ_TEMP_AND_REQUEST_PH_EC, READ_PH_EC };
//EZO readings are taken in 3 steps
//step 1: tell the temp sensor to take a reading
//step 2: consume the temp reading and tell the ph and EC to take a reading based on the temp reading we just received 
//step 3: consume the ph and EC readings

enum reading_step ezo_step_cur = REQUEST_TEMP;                                                    //the current step keeps track of where we are. lets set it to REQUEST_TEMP (step 1) on startup     

int ezo_return_code = 0;                                                                          //holds the return code sent back from thingSpeak after we upload the data 
unsigned long next_step_time = 0;                                                                 //holds the time in milliseconds. this is used so we know when to move between the 3 steps    

const unsigned long ezo_read_delay = 815;                                                         //how long we wait to receive a response, in milliseconds 
const unsigned long ezo_loop_delay = 10000;

double NutTemp;
double NutpH;
double NutEC;

// Analog Input Object and Variables
ADS1115 AI01;
int8_t AI01_address;
int16_t AI01_raw [4];
double AI01_filt[4];
double AI01_eu[4];
double AI01_pct[4];
double AI01_filtpct[4];
byte AI01_error;

// Declare first scan condition
bool FirstScan;

void setup_wifi() {
  delay(100);
  // We start by connecting to a WiFi network

  WiFi.begin(ssid, password);
  WiFi.setHostname("espNutrients");

  int connectdelays = 0;

  while (WiFi.status() != WL_CONNECTED) {
    connectdelays++;
    delay(500);
    if (connectdelays > 5){break;}
  }

}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {

  String strPayload = String();
  for (int i = 0; i < length; i++) {
    strPayload = strPayload + String((char)payload[i]);
  }

  Serial.println(String(topic));
  Serial.println(String(strPayload));

  if (String(topic).equalsIgnoreCase("nutrients/diagnostics")){
    diag(strPayload);
  }

    if (String(topic).equalsIgnoreCase("nutrients/config")){
    config(strPayload);
  }

}

void mqtt_reconnect() {
     // Loop until we're reconnected
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect("ESP32_Greenhouse")) {
      Serial.println("connected");
      // Subscribe to MQTT Topics
      mqtt_client.subscribe("nutrients/config");
      mqtt_client.subscribe("nutrients/diagnostics");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
    }
}

// NTP Functions
time_t getNtpTime(){
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address){
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

int config(String CfgCmd){
  // Configure tomato water dosing
  if(CfgCmd.startsWith("WatDos")){
    if(CfgCmd.substring(6).startsWith("Auto")){
      RelayAuto[SV_WATER] = true;
      mqtt_client.publish("nutrients/waterdosing","Auto");
      return 1;
    }if(CfgCmd.substring(6).startsWith("ManOn")){
      RelayAuto[SV_WATER] = false;
      ManualCmd[SV_WATER] = true;
      mqtt_client.publish("nutrients/waterdosing","Manual On");
      return 1;
    }if(CfgCmd.substring(6).startsWith("ManOff")){
      RelayAuto[SV_WATER] = false;
      ManualCmd[SV_WATER] = false;
      mqtt_client.publish("nutrients/waterdosing","Manual Off");
      return 1;
    }
  }
  
  // Configure tomato nutrient delivery
  if(CfgCmd.startsWith("NutDel")){
    if(CfgCmd.substring(6).startsWith("Auto")){
      RelayAuto[P_NUTDEL] = true;
      mqtt_client.publish("nutrients/nutrientdelivery","Auto");
      return 1;
    }if(CfgCmd.substring(6).startsWith("ManOn")){
      RelayAuto[P_NUTDEL] = false;
      ManualCmd[P_NUTDEL] = true;
      mqtt_client.publish("nutrients/nutrientdelivery","Manual On");
      return 1;
    }if(CfgCmd.substring(6).startsWith("ManOff")){
      RelayAuto[P_NUTDEL] = false;
      ManualCmd[P_NUTDEL] = false;
      mqtt_client.publish("nutrients/nutrientdelivery","Manual Off");
      return 1;
    }if(CfgCmd.substring(6).startsWith("StepTime")){
      DelStepTime_Deliver = (CfgCmd.substring(14,(CfgCmd.length()+1)).toInt());
      //Particle.publish("Nutrient Delivery Step Time Configured",String(DelStepTime_Deliver));
      return 1;
    }if(CfgCmd.substring(6).startsWith("IdleStepTime")){
      DelStepTime_Idle = (CfgCmd.substring(18,(CfgCmd.length()+1)).toInt());
      //Particle.publish("Nutrient Delivery Idle Step Time Configured",String(DelStepTime_Idle));
      return 1;
    }if(CfgCmd.substring(6).startsWith("LowLvlOvrdOn")){
      NutDel_LowLvlOvrd = true;
      return 1;
    }if(CfgCmd.substring(6).startsWith("LowLvlOvrdOff")){
      NutDel_LowLvlOvrd = false;
      return 1;
    }
  }
  
  // Configure tomato nutrient recirc
  if(CfgCmd.startsWith("NutRec")){
    if(CfgCmd.substring(6).startsWith("Auto")){
      RelayAuto[P_NUTREC] = true;
      mqtt_client.publish("nutrients/nutrientrecirc","Auto");
      return 1;
    }if(CfgCmd.substring(6).startsWith("ManOn")){
      RelayAuto[P_NUTREC] = false;
      ManualCmd[P_NUTREC] = true;
      mqtt_client.publish("nutrients/nutrientrecirc","Manual On");
      return 1;
    }if(CfgCmd.substring(6).startsWith("ManOff")){
      RelayAuto[P_NUTREC] = false;
      ManualCmd[P_NUTREC] = false;
      mqtt_client.publish("nutrients/nutrientrecirc","Manual Off");
      return 1;
    }if(CfgCmd.substring(6).startsWith("StepTime")){
      DelStepTime_Analyze = (CfgCmd.substring(14,(CfgCmd.length()+1)).toInt());
      //Particle.publish("Nutrient Analysis Step Time Configured",String(DelStepTime_Analyze));
      return 1;
    }
  }
  
  // Configure aeration 
  if(CfgCmd.startsWith("NutAer")){
    if(CfgCmd.substring(6).startsWith("Auto")){
      RelayAuto[P_AIR] = true;
      mqtt_client.publish("nutrients/nutrientaeration","Auto");
      return 1;
    }if(CfgCmd.substring(6).startsWith("ManOn")){
      RelayAuto[P_AIR] = false;
      ManualCmd[P_AIR] = true;
      mqtt_client.publish("nutrients/nutrientaeration","Manual On");
      return 1;
    }if(CfgCmd.substring(6).startsWith("ManOff")){
      RelayAuto[P_AIR] = false;
      ManualCmd[P_AIR] = false;
      mqtt_client.publish("nutrients/nutrientaeration","Manual Off");
      return 1;
    }if(CfgCmd.substring(6).startsWith("StepTime")){
      DelStepTime_Aerate = (CfgCmd.substring(14,(CfgCmd.length()+1)).toInt());
      //Particle.publish("Nutrient Aeration Step Time Configured",String(DelStepTime_Aerate));
      return 1;
    }
  }


    // Configure scan delay
  if(CfgCmd.startsWith("scandelay")){
      ScanDelaySP = CfgCmd.substring(9,(CfgCmd.length()+1)).toInt();
      return 1;
  }

  if(CfgCmd.startsWith("dose")){
      WatDos_SP_gal = (CfgCmd.substring(4,CfgCmd.length()+1)).toFloat();
      if (WatDos_SP_gal > 6.5){WatDos_SP_gal = 6.5;}
      if (WatDos_SP_gal < 0){WatDos_SP_gal = 0;}
      //Particle.publish("Dose SP",String(WatDos_SP_gal) + " gallons");
      WatDos_Init = true;
      return 1;
  }

  if(CfgCmd.startsWith("ai01filter")){
      AI01_filtpct[0] = (CfgCmd.substring(10,(CfgCmd.length()+1)).toFloat()/100);
      if (AI01_filtpct[0] > 1){AI01_filtpct[0] = 1;}
      if (AI01_filtpct[0] < 0){AI01_filtpct[0] = 0;}
      //Particle.publish("AI01 Filter",String(AI01_filtpct[0]));
      return 1;
  }

  if(CfgCmd.startsWith("DO01init")){
      // Configure MCP23008 GPIO to ALL OUTPUT
      Wire.beginTransmission(Addr_MCP23008);
      Wire.write(0x00);
      Wire.write(0xF0);
      Wire.endTransmission();
      return 1;
  }

    return -1;
}

int diag(String DiagData){
    if(DiagData.equalsIgnoreCase("localip")){
    mqtt_client.publish("nutrients/localip",WiFi.localIP().toString().c_str());
    return 1;
  }
  if(DiagData.equalsIgnoreCase("scantime")){
    mqtt_client.publish("nutrients/scantime",String(Scantime_prev).c_str());
    return 1;
  }
  if(DiagData.equalsIgnoreCase("synctime")){
    setSyncProvider(getNtpTime);
    return 1;
  }
    if(DiagData.equalsIgnoreCase("wifidb")){
    mqtt_client.publish("nutrients/wifidb",String(WiFi.RSSI()).c_str());
    return 1;
  }
  if(DiagData.equalsIgnoreCase("time")){
    mqtt_client.publish("nutrients/now",String(now()).c_str());
    return 1;
  }
  if(DiagData.equalsIgnoreCase("hour")){
    mqtt_client.publish("nutrients/hour",String(hour()).c_str());
    return 1;
  }
  if(DiagData.equalsIgnoreCase("reset")){
    ESP.restart();
    return 1;
  }
  return -1;
}

int DO01(int RelayCmd){
    int retrys = 0;
    bool initialized = false;
    setBankStatusRetry:
    Wire.beginTransmission(Addr_MCP23008);
    Wire.write(0x0A);
    Wire.write(RelayCmd);
    byte s = Wire.endTransmission();
    if(s != 0){
        if(retrys < 3){
            retrys++;
            delay(20);
            goto setBankStatusRetry;
        }else{
            initialized = false;
            retrys = 0;
        }
    }else{
        initialized = true;
        retrys = 0;
    }

    readBankOneRetry:
    Wire.beginTransmission(Addr_MCP23008);
    Wire.write(0x0A);
    byte status = Wire.endTransmission();
    if(status != 0){
        if(retrys < 3){
            retrys++;
            delay(20);
            goto readBankOneRetry;
        }else{
            initialized = false;
            retrys = 0;
        }
    }else{
        retrys = 0;
        initialized = true;
        Wire.requestFrom(Addr_MCP23008, 1);
        delay(20);
        int RelaySts = Wire.read();
        return RelaySts;
    }
    
    return -1;
}

bool ezo_read_success(Ezo_board &Sensor) {                                                 //this function makes sure that when we get a reading we know if it was valid or if we got an error 

  switch (Sensor.get_error()) {                                                             //switch case based on what the response code was
    case Ezo_board::SUCCESS:                                                                //if the reading was a success
      return true;                                                                          //return true, the reading succeeded 
      
    case Ezo_board::FAIL:                                                                   //if the reading faild
      //Serial.print("Failed ");                                                              //print "failed"
      return false;                                                                         //return false, the reading was not successful

    case Ezo_board::NOT_READY:                                                              //if the reading was taken to early, the command has not yet finished calculating
      //Serial.print("Pending ");                                                             //print "Pending"
      return false;                                                                         //return false, the reading was not successful

    case Ezo_board::NO_DATA:                                                                //the sensor has no data to send
      //Serial.print("No Data ");                                                             //print "no data"
      return false;                                                                         //return false, the reading was not successful
      
    default:                                                                                //if none of the above happened
     return false;                                                                          //return false, the reading was not successful
  }
}

void setup() {
    
    // Intitiate I2C and UART Communications
    Wire.begin();
    Serial.begin(115200);
    //Serial.println("Setup");  

    // Call Wifi Setup Subroutine
    setup_wifi();

    // Set time zone and DST
    if((month() >= 3) && (month() <= 10)){
      timeZone = -4;
    }else{
      timeZone = -5;
    }

    // Sync RTC with NTP Server
    Udp.begin(localPort);
    setSyncProvider(getNtpTime);

    // Configure MQTT server and callback function
    mqtt_client.setServer(mqtt_server,1883);
    mqtt_client.setCallback(mqtt_callback);

    // Configure MCP23008 GPIO to ALL OUTPUT
    Wire.beginTransmission(Addr_MCP23008);
    Wire.write(0x00);
    Wire.write(0x00);
    Serial.println(Wire.endTransmission());

    DO01(0);

    // Configure ADS1115 4-ch Analog Inout Module. See header for enumerations.
    AI01.getAddr_ADS1115(ADS1115_DEFAULT_ADDRESS);                                                // 0x48, 1001 000 (ADDR = GND)
    AI01.setGain(GAIN_TWO);                                                                       // 2x gain   +/- 2.048V  1 bit = 0.0625mV (default)
    AI01.setMode(MODE_CONTIN);                                                                    // Continuous conversion mode
    AI01.setRate(RATE_128);                                                                       // 128SPS (default)
    AI01.setOSMode(OSMODE_SINGLE);                                                                // Set to start a single-conversion   
    AI01_filtpct[0] = 0.93;                                                                       // Set Initial Filter Value to 90%

    // OTA firmware update support
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();

    // Initiate Periodic Timers
    Periodic_set[PERIOD_500MS] = 500000;
    Periodic_set[PERIOD_1000MS] = 1000000;
    Periodic_set[PERIOD_2000MS] = 2000000;
    Periodic_set[PERIOD_5000MS] = 5000000;
    Periodic_set[PERIOD_30000MS] = 30000000;
    Periodic_set[PERIOD_180000MS] = 180000000;
    Periodic_set[PERIOD_1H] = 3600000000;

    // Set first scan condition to true
    FirstScan = true;
}

void loop() {

    ArduinoOTA.handle();

    if (Periodic_flg[PERIOD_1H] && (WiFi.status() != WL_CONNECTED)){
        setup_wifi();
    }

    if (!mqtt_client.connected()) {
        mqtt_reconnect();                                                                         
    }
    mqtt_client.loop();                                                                           // Call MQTT Loop Function

    // First Scan Operations
    if (FirstScan){
      config("NutAerAuto");
      config("NutDelAuto");
      config("NutRecAuto");
      config("WatDosAuto");
    }

    // Scan time calculations for periodic operations
    Scantime_prev = (micros() - micros_prev);
    if (micros() > micros_prev){
       Scantime_prev = (micros() - micros_prev); 
    }
    micros_prev = micros();

    // Periodic flag calculations
    for (int i = 0; i <= 6; i++){
        Periodic_acc[i] = Periodic_acc[i] + Scantime_prev;
        if (Periodic_acc[i] > Periodic_set[i]){
            Periodic_flg[i] = true;
            Periodic_acc[i] = Periodic_acc[i] - Periodic_set[i];
        }else{
            Periodic_flg[i] = false;
        }
    }

    // Read Battery Voltage
    float battlvl = (analogRead(A13));

    if (Periodic_flg[PERIOD_30000MS]){
        mqtt_client.publish("nutrients/battlvl",String(battlvl).c_str());
        mqtt_client.publish("nutrients/scantime",String(Scantime_prev).c_str());
    }

    // Process timed water dose request
    if (WatDos_Init == true && WatDos_Active == false && millis() < 4294697296){
        WatDos_Init = false;
        WatDos_Active = true;
        WatDos_StartTime = millis();
        WatDos_SP_sec = WatDos_SP_gal * 40;
        WatDos_EndTime = WatDos_StartTime + (int) (WatDos_SP_sec*1000);


    }else if (WatDos_Active == true && (millis() >= WatDos_EndTime)){
        WatDos_Active = false;
    }else if (millis() >= 4294697296){
        mqtt_client.publish("nutrients/watdos/status","millis counter rollover, try again later");
    }

    AutoCmd[SV_WATER] = WatDos_Active;
  
    // Delivery Sequence Code

    if (DelSeq_Abort_OCmd){                                                     // Set abort sequence bit conditions
        DelSeq_Abort = true;
    }

    DelSeq_Timer = now() - DelSeq_StepChange_TimeStamp;
  
    switch(DelStep_Cur) {                                                       // Execute code based on current delivery sequence step

        //------------------------------
        case DISABLED:                                                          // Disabled Step: All outputs off, reset timers

          // Sequence Event Notifications
          if (DelSeq_StepChange){
            mqtt_client.publish("nutrients/delseq/curstp","Disabled");                   // Publish that sequence is in disabled step on step change
          }

          // Sequence Calculations
          if (!DelSeq_Abort) {                                                                                              
              DelStep_Prop = ANALYZE;                                           //Set Proposed Step to ANALYZE
          }

          // Miscellaneous Calculations
          DelSeq_Abort_OCmd = false;                                            // Reset Abort Operator Command  
          DelStep_Countdown = -1;

          // Output Calculations
          AutoCmd[P_NUTREC] = false;
          AutoCmd[P_NUTDEL] = false;
          AutoCmd[P_AIR] = false;

          break;  

        //------------------------------

        case ANALYZE:                                                          // Analyze Step: Run recirculation pump

          // Sequence Event Notifications
          if (DelSeq_StepChange){
            mqtt_client.publish("nutrients/delseq/curstp","Analyzing (Recirculating)");  // Publish that sequence is in disabled step on step change
          }

          // Sequence Calculations
          if (!DelSeq_Abort && (DelSeq_Timer > DelStepTime_Analyze)) {                                                                                              
              DelStep_Prop = IDLE;                                              //Set Proposed Step to IDLE
          }

          // Miscellaneous Calculations
          DelStep_Countdown = ((DelStepTime_Analyze - DelSeq_Timer) + DelStepTime_Idle + DelStepTime_Aerate);

          // Output Calculations
          AutoCmd[P_NUTREC] = true;                                             // Nutrient recirculation pump ON
          AutoCmd[P_NUTDEL] = false;                                            // Nutrient delivery pump OFF
          AutoCmd[P_AIR] = false;                                               // Aeration pump OFF

          break;  

        //------------------------------

        case IDLE:                                                          // Analyze Step: Run recirculation pump

          // Sequence Event Notifications
          if (DelSeq_StepChange){
            mqtt_client.publish("nutrients/delseq/curstp","Idle");  // Publish that sequence is in disabled step on step change
          }

          // Sequence Calculations
          if (!DelSeq_Abort && (DelSeq_Timer > DelStepTime_Idle)) {                                                                                              
              DelStep_Prop = AERATE;                                              //Set Proposed Step to IDLE
          }

          // Miscellaneous Calculations
          DelStep_Countdown = ((DelStepTime_Idle - DelSeq_Timer) + DelStepTime_Aerate);

          // Output Calculations
          AutoCmd[P_NUTREC] = false;                                             // Nutrient recirculation pump ON
          AutoCmd[P_NUTDEL] = false;                                            // Nutrient delivery pump OFF
          AutoCmd[P_AIR] = false;                                               // Aeration pump OFF

          break;  

        //------------------------------  

        case AERATE:                                                          // Analyze Step: Run recirculation pump

          // Sequence Event Notifications
          if (DelSeq_StepChange){
            mqtt_client.publish("nutrients/delseq/curstp","Aerating");  // Publish that sequence is in disabled step on step change
          }

          // Sequence Calculations
          if (!DelSeq_Abort && (DelSeq_Timer > DelStepTime_Aerate)) {                                                                                              
              DelStep_Prop = DELIVER;                                              //Set Proposed Step to IDLE
          }

          // Miscellaneous Calculations
          DelStep_Countdown = (DelStepTime_Aerate - DelSeq_Timer);

          // Output Calculations
          AutoCmd[P_NUTREC] = false;                                             // Nutrient recirculation pump ON
          AutoCmd[P_NUTDEL] = false;                                            // Nutrient delivery pump OFF
          AutoCmd[P_AIR] = true;                                               // Aeration pump OFF

          break;  

        //------------------------------  

        case DELIVER:                                                          // Analyze Step: Run recirculation pump

          // Sequence Event Notifications
          if (DelSeq_StepChange){
            mqtt_client.publish("nutrients/delseq/curstp","Delivering");  // Publish that sequence is in disabled step on step change
          }

          // Sequence Calculations
          if (!DelSeq_Abort && (DelSeq_Timer > DelStepTime_Deliver)) {                                                                                              
              DelStep_Prop = DRAIN;                                              //Set Proposed Step to IDLE
          }

          // Miscellaneous Calculations
          DelStep_Countdown = 0;

          // Output Calculations
          AutoCmd[P_NUTREC] = false;                                             // Nutrient recirculation pump ON
          AutoCmd[P_NUTDEL] = true;                                            // Nutrient delivery pump OFF
          AutoCmd[P_AIR] = false;                                               // Aeration pump OFF

          break;  

        //------------------------------  

        case DRAIN:                                                          // Analyze Step: Run recirculation pump

          // Sequence Event Notifications
          if (DelSeq_StepChange){
            mqtt_client.publish("nutrients/delseq/curstp","Draining");  // Publish that sequence is in disabled step on step change
          }

          // Sequence Calculations
          if (!DelSeq_Abort && (DelSeq_Timer > DelStepTime_Drain)) {                                                                                              
              DelStep_Prop = ANALYZE;                                              //Set Proposed Step to IDLE
          }

          // Miscellaneous Calculations
          DelStep_Countdown = ((DelStepTime_Drain - DelSeq_Timer) + DelStepTime_Analyze + DelStepTime_Idle + DelStepTime_Aerate);

          // Output Calculations
          AutoCmd[P_NUTREC] = false;                                             // Nutrient recirculation pump ON
          AutoCmd[P_NUTDEL] = false;                                            // Nutrient delivery pump OFF
          AutoCmd[P_AIR] = false;                                               // Aeration pump OFF

          break;  

        //------------------------------  

    } 

    // Administer Sequence Step Changes and Timer
    if (DelStep_Prop != DelStep_Cur){
      DelStep_Cur = DelStep_Prop;
      DelSeq_StepChange = true;
      DelSeq_StepChange_TimeStamp = now();
    }else{
      DelSeq_StepChange = false;
    }

    if (Periodic_flg[PERIOD_1000MS]){
      mqtt_client.publish("nutrients/delseq/countdown",String(DelStep_Countdown).c_str());
    }

    // Turn Aeration ON if water has been dosed in the past 10 minutes   
    
    if (now() <= (WatDos_OffTime + 600UL)){
      AutoCmd[P_AIR] = true;
    }

    // Turn Recirc Pump ON if water has been dosed in the past 2 minutes   
    
    if (now() <= (WatDos_OffTime + 120UL)){
      AutoCmd[P_NUTREC] = true;
    }  

    // Turn Delivery Pump OFF if level is below Low Level SP  
    
    if (AI01_eu[0] < TomNutRes_LowLvl_SP){
      AutoCmd[P_NUTDEL] = false;
      if (!NutDel_LowLvlOvrd){
        ManualCmd[P_NUTDEL] = false;
      }
    }  

    // Calculate Relay Out Commands from Mode/Commands
    DO01_Cmd = 0;
    for (int i = 0; i <= 7; i++) {
        if (RelayAuto[i] == true){
            ManualCmd[i] = AutoCmd[i];
        if (AutoCmd[i] == true){
            RelayOut[i] = true;
            }else{
                RelayOut[i] = false;
            }
        }else{
            if (ManualCmd[i] == true){
                RelayOut[i] = true;
            }else{
                RelayOut[i] = false;
            }
        }
        // Pack relay status bits into command int
        if (RelayOut[i] == true){
            DO01_Cmd = (DO01_Cmd | ((int) pow(2,i)));
        }
    }
    
    // Clear extra bits from int and send command to DO01
    DO01_Cmd = DO01_Cmd & 255;


    if (DO01_Cmd != DO01_Cmd_Prev || (Periodic_flg[PERIOD_2000MS] && !Periodic_flg[PERIOD_1H])){
        DO01_Sts = DO01(DO01_Cmd);
    } else if (Periodic_flg[PERIOD_1H] && DO01_Sts <= 0){
        // Configure MCP23008 GPIO to ALL OUTPUT
        Wire.beginTransmission(Addr_MCP23008);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();

    }
    
    if (DO01_Sts >= 0){
        DO01_Fault = false;
    }else{
        DO01_Fault = true;
        DO01_Sts = 0;
    }

    if (DO01_Sts != DO01_Sts_Prev || DO01_Cmd != DO01_Cmd_Prev || Periodic_flg[PERIOD_30000MS]){
      mqtt_client.publish("nutrients/OutCmd",String(DO01_Cmd).c_str());
      mqtt_client.publish("nutrients/OutSts",String(DO01_Sts).c_str());
      mqtt_client.publish("nutrients/DO01Flt",String(DO01_Fault).c_str());
    }


    
    DO01_Sts_Prev = DO01_Sts;
    DO01_Cmd_Prev = DO01_Cmd;

    // Record time when water dosing valve was last on
    if (DO01_Sts & P_NUTDEL){
      WatDos_OffTime = now();
    }



  switch(ezo_step_cur) {                                                                          //selects what to do based on what reading_step we are in

    case READ_LEVEL:                                                                              //when we are in the first step

        AI01_address = AI01.ads_i2cAddress;
        Wire.beginTransmission(AI01_address);
        AI01_error = Wire.endTransmission();

        if (AI01_error == 0){            
            for (int i = 0; i <= 0; i++) {
                AI01_raw[i] = AI01.Measure_SingleEnded(i);
                if (FirstScan){AI01_filt[i] = AI01_raw[i];}
                AI01_filt[i] = (AI01_filt[i]*AI01_filtpct[i] + AI01_raw[i]*(1-AI01_filtpct[i]));
                AI01_eu[i] = 0.0009856 * AI01_filt[i] - 6.49;
                AI01_pct[i] = AI01_filt[i] / 13481;
            }
        }
        if (Periodic_flg[PERIOD_2000MS]){
            mqtt_client.publish("nutrients/level",String(AI01_eu[0]).c_str());
        }
        
        if (millis() >= next_step_time) {                                                         //check to see if enough time has past, if it has 
            next_step_time = millis() + ezo_read_delay;                                           //advance the next step time by adding the delay we need for the sensor to take the reading
            ezo_step_cur = REQUEST_TEMP;                                                          //switch to step 2
      }
      break;                                                                                      //break out of this we are done
 
    case REQUEST_TEMP:                                                                            //when we are in the first step
      if (millis() >= next_step_time) {                                                           //check to see if enough time has past, if it has 
        RTD.send_read_cmd();                                                                      //request a temp reading
        next_step_time = millis() + ezo_read_delay;                                               //advance the next step time by adding the delay we need for the sensor to take the reading
        ezo_step_cur = READ_TEMP_AND_REQUEST_PH_EC;                                               //switch to step 2
      }
      break;                                                                                      //break out of this we are done

    case READ_TEMP_AND_REQUEST_PH_EC:                                                             //when we are in the second step
      if (millis() >= next_step_time) {                                                           //check to see if enough time has past, if it has                    
            
        RTD.receive_read_cmd();                                                                   //get the temp reading  
        
        if((ezo_read_success(RTD) == true) && (RTD.get_last_received_reading() > -1000.0)){      //if the temperature reading has been received and it is valid
          PH.send_read_with_temp_comp(RTD.get_last_received_reading());                           //send readings from temp sensor to pH sensor
          EC.send_read_with_temp_comp(RTD.get_last_received_reading());                           //send readings from temp sensor to EC sensor
        } 
        else                                                                                      //if the temperature reading is invalid
        {
          PH.send_read_with_temp_comp(25.0);                                                      //send default temp = 25 deg C to pH sensor
          EC.send_read_with_temp_comp(25.0);                                                      //send default temp = 25 deg C to EC sensor
        }
        
        if(RTD.get_error() == Ezo_board::SUCCESS){                                                //if the RTD reading was successful
            NutTemp = RTD.get_last_received_reading();
        }
        
        mqtt_client.publish("nutrients/temp",String(NutTemp).c_str());

        next_step_time = millis() + ezo_read_delay;                                               //advance the next step time by adding the delay we need for the sensor to take the reading
        ezo_step_cur = READ_PH_EC;                                                                //switch to step 3 
      }
      break;                                                                                      //break out of this we are done

    case READ_PH_EC:                                                                              //when we are in the third step
      if (millis() >= next_step_time) {                                                           //check to see if enough time has past, if it has
        
        PH.receive_read_cmd();                                                                    //get the PH reading 
     
        if(ezo_read_success(PH) == true){                                                         //if the pH reading has been received and it is valid
            NutpH = PH.get_last_received_reading();
        }

        mqtt_client.publish("nutrients/ph",String(NutpH).c_str());

        EC.receive_read_cmd();                                                                    //get the EC reading 
       
        if(ezo_read_success(EC) == true){                                                        //if the EC reading has been received and it is valid
            NutEC = EC.get_last_received_reading();
        }

        mqtt_client.publish("nutrients/ec",String(NutEC).c_str());

        if(RTD.get_error() == Ezo_board::SUCCESS){                                                //if the RTD reading was successful (back in step 1)
            ezo_return_code = 200;
        }

        next_step_time =  millis() + ezo_loop_delay;                                              //update the time for the next reading loop 
        ezo_step_cur = READ_LEVEL;                                                                //switch back to step 1 
      }
      break;                                                                                      //break out of this we are done
  }

    delay(ScanDelaySP);

    if (FirstScan){
      FirstScan = false;                                                                          // Set First Scan condition to false
    }

}
