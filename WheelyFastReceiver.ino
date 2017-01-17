#include "PS2X_lib.h"
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Ticker.h>
#include "MotorController.h"

extern "C" { 
   #include "user_interface.h" 
 } 

//////////////////////
// WiFi Definitions //
//////////////////////
const char *password = "12345678";      // This is the Wifi Password (only numbers and letters,  not . , |)
String AP_Name = "WheelyFast";             // This is the Wifi Name(SSID), some numbers will be added for clarity (mac address)
byte vibrate = 0;
PS2X ps2x; // create PS2 Controller Class

void setupWiFi(void);

/////////////////////
// Pin Definitions //
/////////////////////

// stepper with direction and speed pins, don't use D0 for speed
const int motorLeftDir  = D7;
const int motorLeftSpd  = D8;
const int motorRightDir = D1;
const int motorRightSpd = D2;
//const int motorLeftA  = D5;
//const int motorLeftB  = D6;
//const int motorRightA = D3;
//const int motorRightB = D2;
motorController motors(motorLeftDir,motorLeftSpd,motorRightDir,motorRightSpd); 
//motorController motors(motorLeftA,motorLeftB,motorRightA,motorRightB, true);

// UDP variables
unsigned int localPort = 8888;
WiFiUDP UDP;
boolean udpConnected = false;
IPAddress remoteIPAddress;
int remotePortAddress = 0;

Ticker HeartBeatTicker;

bool HeartBeatRcvd = false;

void Stop(void)
{
  motors.update(0,0);
}

void CheckHeartBeat(void)
{
  if (HeartBeatRcvd == true)
  {
    HeartBeatRcvd = false;
  }
  else
  {
    Stop();
  }
}

void setup()
{
  system_update_cpu_freq(80);        // set cpu to 160MHZ !
  Serial.begin(250000);
  delay(100);
  setupWiFi();
  HeartBeatTicker.attach_ms(500, CheckHeartBeat);
  Stop();
  //motors.setTrim(1.0,1.0);            // this setting is optional, it compensates for speed difference of motors eg (0.95,1.0), and it can reduce maximum speed of both eg (0.75,0.75);
  motors.setSteeringSensitivity(0.15);  // this setting is optional
  motors.setPWMFrequency(1000);           // this setting is optional, depending on power supply and H-Bridge this option will alter motor noise and torque.
  //motors.setMinimumSpeed(0.10);         // this setting is optional, default is 0.1(10%) to prevent motor from stalling at low speed
  }

void loop()
{
    if(!checkForIncomingData())return;
    if(!udpConnected || !remotePortAddress) return;
    processData();
}

void setupWiFi()
{
  WiFi.mode(WIFI_AP);
  char AP_NameChar[AP_Name.length() + 1];
  AP_Name.toCharArray(AP_NameChar,AP_Name.length() + 1);
  int channel = random(1,13 + 1);               // have to add 1 or will be 1 - 12
  IPAddress subnet(255, 255, 255, 0);
  IPAddress apIP(192, 168, 1, 1);
  WiFi.softAPConfig(apIP, apIP, subnet);
  WiFi.softAP(AP_NameChar, password , channel , 0 );
  Serial.println("");
  if(UDP.begin(localPort) == 1){
      Serial.println(F("Ready for Controller to connect"));
      udpConnected = true;
    }
  else{
      Serial.println(F("Connection failed"));
      }
}
boolean checkForIncomingData(){
  if(udpConnected){
    int packetSize = UDP.parsePacket();
    if (packetSize){// if there’s data available, read a packet
      char packetBuffer[packetSize];
      boolean firstPacketSkipped = remotePortAddress;
      if (!remotePortAddress || remotePortAddress != UDP.remotePort())Serial.println("Controller Connected, Port: " + String(UDP.remotePort()));
      if(packetSize) // have data
      {
        remoteIPAddress = UDP.remoteIP(); // update client details
        remotePortAddress = UDP.remotePort();
        // read the packet into packetBuffer
        UDP.read(packetBuffer,packetSize);
        //Serial.println(packetBuffer);
        // send to library
        for (int a = 0; a < 21; a++){
          ps2x.PS2data[a] = packetBuffer[a];
          }
        ps2x.last_buttons = ps2x.buttons; //store the previous buttons states
        ps2x.buttons = (ps2x.PS2data[4] << 8) + ps2x.PS2data[3]; //store as one value for multiple functions
        //Serial.print(F("Free Ram: "));
        //Serial.println(system_get_free_heap_size());
        if (!firstPacketSkipped)return false; // to set buttons to default skip first packet
        return true;
        }
    }
  }
  return false;
}
int translateStick(int value, int newRange){
  int newValue = 0;
  int deadZone = 70;
  int upperZone = 127 + (deadZone * 0.5);
  if (value > upperZone){
    newValue = map(value , upperZone, 255 , 1 , newRange);
  }
  int lowerZone = 127 - (deadZone * 0.5);
  if (value < lowerZone){
    newValue = map(value , lowerZone, 1 , -1 , -newRange);
  }
  return newValue;
}
void processData(){
    int range = 100;
    if (ps2x.Button(PSB_L2))range = 500; // slow drive mode
    int dY = translateStick(ps2x.Analog(PSS_LY) , range);
    if (ps2x.Button(PSB_R2))range = 500; // slow steering mode
    int dX = translateStick(ps2x.Analog(PSS_RX) , range);
    motors.update(dX,dY);
    HeartBeatRcvd = true;               // recieved data, must be connected
}

