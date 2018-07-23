#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address! * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS.h>
//#include "DHT.h"
#include <SdFat.h>
#include <Wire.h> 

/***************************************************************************
 * SDFAT BEGINN Define
 */
// Here are the pins for the soft-spi-bus defined
const uint8_t SOFT_MISO_PIN = 37;
const uint8_t SOFT_MOSI_PIN = 39;
const uint8_t SOFT_SCK_PIN  = 48;
const uint8_t SD_CHIP_SELECT_PIN = 46;
const int8_t DISABLE_CHIP_SELECT = -1;
// You can select every pin you want, just don't put them on an existing hardware SPI pin.
SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> sd;
SdFile file;

static void file_float(float val, float invalid, int len, int prec);
static void file_int(unsigned long val, unsigned long invalid, int len);
static void file_date(TinyGPS &gps);
 /*************************************************************************
  * SDFAT END Define
  */
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define DHTPIN 22
//#define DHTTYPE DHT22 //DHT11, DHT21, DHT22

//DHT dht(DHTPIN, DHTTYPE);

float humidity;
float temperature;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
TinyGPS gps;

#define SERIALOUT 1

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

static char date_out[32];
float flat, flon;
unsigned long age, date, time, chars = 0;
unsigned short sentences = 0, failed = 0;
static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static const PROGMEM u1_t NWKSKEY[16] = { 0x07, 0xC8, 0x20, 0x15, 0x44, 0x8B, 0x95, 0xE9, 0xEB, 0xD0, 0xDE, 0x5D, 0x11, 0xDB, 0x11, 0x7A };
static const u1_t PROGMEM APPSKEY[16] = { 0x15, 0xF7, 0xE0, 0x1E, 0x0D, 0xA9, 0x85, 0x3C, 0x3D, 0x97, 0xC3, 0xB2, 0xAA, 0x3D, 0x00, 0xFE };
int counter =0;

u4_t myPins[] = {0x26011FB1, 0x26011BE5};
u4_t DEVADDR = myPins[(counter %2)]; 

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


char buf [4] = "0000" ;
uint8_t mydata[4] = "0000";
static osjob_t sendjob;

const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};


void onEvent (ev_t ev) {
        switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
             if(counter >= 999){
              counter =0;
            }

            
            sprintf(buf,"%03i%", counter);
            mydata[0] = buf[0];
            mydata[1] = buf[1];
            mydata[2] = buf[2];
            mydata[3] = buf[3];
            for (int i = 0; i < 4; i++)
            {
              Serial.print(buf[i]);
             }
             Serial.print(";");
             
            print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
            Serial.print(";");
            gps.f_get_position(&flat, &flon, &age);
            print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
            Serial.print(";");
            print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
            Serial.print(";");
            print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
            Serial.print(";");
            print_date(gps);
            Serial.println(";");
      

/*************************************************************************************************
 * SD Card Init
 *************************************************************************************************/
 //FILE erstellen
sd.begin(SD_CHIP_SELECT_PIN);

 file.open("Dev.csv", O_WRITE | O_CREAT | O_AT_END);
      //File befüllen
      for (int i = 0; i < 4; i++)
            {
              file.print(buf[i]);
             }
             file.print(";");
/*********************************************
 * 
 */
file_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
file.print(";");
file_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
file.print(";");
file_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
file.print(";");
file_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
file.print(";");
file_date(gps);
file.println(";");


//Fileschließen
        file.close();

  
/*************************************************************************************************
 * SD Card init fnish
 *************************************************************************************************/

           
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            smartdelay(TX_INTERVAL*1000);
            counter++;
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}



void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial.println(F("Starting"));
    /*
    file.open("Temp.csv", O_WRITE | O_CREAT | O_AT_END);
     file.println(F("H"));
file.close();*/
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
   
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(1);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);

  // dht.begin();

}

void loop() {
    os_runloop_once();
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    
  }
  smartdelay(0);
}
/********************************************************************************************************************************************************************************
 * 
 *
 */
static void file_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      file.print('*');
  }
  else
  {
    file.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    
  }
  smartdelay(0);
}
 /********************************************************************************************************************************************************************************
 * 
 *
 */
static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
   Serial.print(sz);
  smartdelay(0);
}
/********************************************************************************************************************************************************************************
 * 
 *
 */

static void file_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  file.print(sz);
  smartdelay(0);
}
 /************************************************************************************************************************************************************************************************
  * 
  */

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    
    sprintf( date_out, "%02d/%02d/%02d %02d:%02d:%02d",
        month, day, year, hour, minute, second);
       
    Serial.print(date_out);
    
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}
 /************************************************************************************************************************************************************************************************
  * 
  */
static void file_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    file.print("********** ******** ");
  else
  {
    
    sprintf( date_out, "%02d/%02d/%02d %02d:%02d:%02d",
        month, day, year, hour, minute, second);
       
    file.print(date_out);
    
  }
  file_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}
 /************************************************************************************************************************************************************************************************
  * 
  */

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartdelay(0);
}

