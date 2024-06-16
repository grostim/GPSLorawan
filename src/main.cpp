
//
//  Based initially on the OTAA-LoRa-Seeed sketch, but I've
//  pretty much gutted it.
//
//  2024 Grostim.


// #define DEBUG_GPS 1
#include "Arduino.h"
#include "LoRaWan.h"

#include "TinyGPS++.h"
TinyGPSPlus gps;

unsigned char data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xA,};
char buffer[256];

unsigned long last_update = 0;
double last_lat;
double last_lng;
String toLog;
uint8_t txBuffer[9];
uint8_t sizePacket;
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
char c;

int interval_sec = 60;
int interval_meters = 20;

void checkDownlink();
void displayInfo();
void led_on();
void led_off();
void build_packet();

void setup(void)
{
    char c;
    
    SerialUSB.begin(115200); // open Serial to computer
    // make sure usb serial connection is available,
    // or after 10s go on anyway for 'headless' use of the
    // node.
    while((!SerialUSB)&& (millis() < 10000));;
    
    lora.init();
    lora.setDeviceReset();


    Serial.begin(9600);     // open Serial to the GPS

    // For S&G, let's get the GPS fix now, before we start running arbitary
    // delays for the LoRa section

    while (!gps.location.isValid()) {
      while (Serial.available() > 0) {
        if (gps.encode(c=Serial.read())) {
          displayInfo();
          if (gps.location.isValid()) {
              last_lat =gps.location.lat();
              last_lng =gps.location.lng();
            break;
          }
        }
        #ifdef DEBUG_GPS
        SerialUSB.print(c);
        #endif
      }
        
      if (millis() > 15000 && gps.charsProcessed() < 10)
      {
        SerialUSB.println(F("No GPS detected: check wiring."));
        SerialUSB.println(gps.charsProcessed());
        while(true);
      } 
      else if (millis() > 20000) {
        SerialUSB.println(F("Not able to get a fix in alloted time."));     
        break;
      }
    }
    
    memset(buffer, 0, 256);
    lora.getVersion(buffer, 256, 1);
    SerialUSB.print(buffer); 
    
    memset(buffer, 0, 256);
    lora.getId(buffer, 256, 1);
    SerialUSB.print(buffer);

    //lora.setId(NULL, NULL, "70b3d57ed0006fcc");      // set App key, from TTN device overview

    // ???
    lora.setKey("6670063DBBFA94C4774BF2CF726099C2",     // NetSKEY
                "6670063DBBFA94C4774BF2CF726099C2",     // AppSKEY
                "6670063DBBFA94C4774BF2CF726099C2");    // AppKey
    
    lora.setDeciveMode(LWOTAA);
    lora.setClassType(CLASS_A);
    lora.setDataRate(DR0, EU868);
    lora.setAdaptiveDataRate(false);
    lora.setChannel(0, 868.1);
    lora.setChannel(1, 868.3);
    lora.setChannel(2, 868.5);
    
    lora.setReceiceWindowFirst(0, 868.1);
    lora.setReceiceWindowSecond(868.5, DR3);
    
    lora.setDutyCycle(false);
    lora.setJoinDutyCycle(false);
    
    lora.setPower(20);
    
    while(!lora.setOTAAJoin(JOIN));
}

    
void loop(void)
{   
  while (Serial.available()){
    gps.encode(c=Serial.read());
        #ifdef DEBUG_GPS
        SerialUSB.print(c);
        #endif
  }

  double distanceM = TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      last_lat,
      last_lng);

  if (((millis() - last_update) >= interval_sec * 1000) ||(distanceM >= interval_meters)) {
  
    SerialUSB.print("Interval: ");
    SerialUSB.println(millis()-last_update);
    SerialUSB.print("Distance: ");
    SerialUSB.println(distanceM);
    displayInfo();
    build_packet();
    SerialUSB.println(toLog);

    if (distanceM <= 5000) {
      lora.transferPacket(txBuffer, sizePacket, 8);
      SerialUSB.println("TX done");
      checkDownlink();
    }
    else {
      SerialUSB.println("Distance >5000M . Fix is probably wrong");
    }

    last_update = millis();
    last_lat =gps.location.lat();
    last_lng =gps.location.lng();
    SerialUSB.println();
  }

}

void checkDownlink()
{
  short length = 0;
  short rssi = 0;

  memset(buffer, 0, 256);
  length = lora.receivePacket(buffer, 256,  &rssi);

  if(length)
    {
        SerialUSB.println("Downlink Received");
        SerialUSB.print("Length is: ");
        SerialUSB.println(length);
        SerialUSB.print("RSSI is: ");
        SerialUSB.println(rssi);
        SerialUSB.print("Data is: ");
        for(unsigned char i = 0; i < length; i ++)
        {
            SerialUSB.print("0x");
            SerialUSB.print(buffer[i], HEX);
            SerialUSB.print(" ");
        }
        SerialUSB.println();

        switch ( buffer[0] )
        {
          case 01: // Format: Command Code (0x01) followed by 3 bytes time value in seconds.
            SerialUSB.println("Command Code (0x01)-set the END Node's Transmit Interval");
            if (length != 4) {
              SerialUSB.println("Command improperly set");
              break;
            }
            interval_sec = (buffer[1]<<16) + (buffer[2]<<8) +  buffer[3];
            SerialUSB.print("Delay between updates (s):");
            SerialUSB.println(interval_sec, DEC);
            break;
          case 02: // Format: Command Code (0x02) followed by 3 bytes time value in meters.
            SerialUSB.println("Command Code (0x02)-set the END Node's Transmit Interval based on distance");
            if (length != 4) {
              SerialUSB.println("Command improperly set");
              break;
            }
            interval_meters = (buffer[1]<<16) + (buffer[2]<<8) +  buffer[3];
            SerialUSB.print("Distance between updates (m):");
            SerialUSB.println(interval_meters, DEC);
            break;
         default:
            SerialUSB.println("Command Code undefined");
        }
        SerialUSB.println();
    }
}

void displayInfo()
{
  SerialUSB.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    SerialUSB.print(gps.location.lat(), 6);
    SerialUSB.print(F(","));
    SerialUSB.print(gps.location.lng(), 6);
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    SerialUSB.print(gps.date.month());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.day());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.year());
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.hour());
    SerialUSB.print(F(":"));
    if (gps.time.minute() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.minute());
    SerialUSB.print(F(":"));
    if (gps.time.second() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.second());
    SerialUSB.print(F("."));
    if (gps.time.centisecond() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.centisecond());
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.println();
}

void build_packet()
{
  if ( gps.location.isValid() )
  {

    LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
    LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;

    txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    txBuffer[2] = LatitudeBinary & 0xFF;

    txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    txBuffer[5] = LongitudeBinary & 0xFF;

    altitudeGps = gps.altitude.meters();
    txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
    txBuffer[7] = altitudeGps & 0xFF;

    hdopGps = gps.hdop.value()/10;
    txBuffer[8] = hdopGps & 0xFF;
    sizePacket=9;
    led_on();
  } else {
    // 0xFF if no fix
    txBuffer[0] = 0xFF;
    sizePacket=1;
    led_off();
  }

  toLog = "";
  for(size_t i = 0; i<sizePacket; i++)
  {
    char buffer[3];
    sprintf(buffer, "%02x", txBuffer[i]);
    toLog = toLog + String(buffer);
  }
}

void led_on(){
  digitalWrite(13, 1);
}

void led_off(){
  digitalWrite(13, 0);
}