

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
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include "DFRobot_EC.h"
#include <avr/wdt.h>
#define CFG_in866 1;
#define TEMP_PIN A0
#define PH_PIN A1
#define TDS_PIN A7
#define EC_PIN A6
#define PUMP_PIN D4
float voltage,ecValue,temperature = 0;
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;
DFRobot_EC ec;
OneWire ds(TEMP_PIN);  // on analog pin 0


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] ={ 0x57, 0x57, 0x34, 0x51, 0xAC, 0xBE, 0x5A, 0xD9, 0x1C, 0xC3, 0x99, 0x9B, 0xB7, 0x79, 0xD8, 0x92};

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] ={ 0xF2, 0x40, 0xB8, 0x65, 0x9E, 0x65, 0x3B, 0xF4, 0x2A, 0xA6, 0x2E, 0x15, 0x90, 0xCC, 0xE4, 0x01 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260BEEDA; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
uint8_t mydata[] = "hello world";
//String abc = "Temperature = " + String(DHT.temperature) + "\nHumidity = " + String(DHT.humi

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN ,
    .rst = 9,
    .dio = {2, 6, 7},
};


float ph() {  
 
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(PH_PIN);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      } 
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;  
  return phValue;
}

float TDS() {
  
  int sensorValue = analogRead(TDS_PIN);// read the input on analog pin 7:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  return(voltage); // print out the value you read:
}

float ecV() {
 
  static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      voltage = analogRead(EC_PIN)/1024.0*5000;  // read the voltage
      ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
    }
    ec.calibration(voltage,temperature);  // calibration process by Serail CMD
    return ecValue;
}
float BOD()
{
  return 0.707;
}
float COD()
{
  return 6.424;
}
void pump()
{
  digitalWrite(4,HIGH);
  Serial.println("high");
  delay(5000);
  digitalWrite(4,LOW);
  Serial.println("low");
}
float getTemp(){
  //wdt_reset();
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}
void onEvent (ev_t ev) {
  Serial.println(ev);
    Serial.print(os_getTime());
    Serial.print(": ");
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
        case EV_TXCOMPLETE:
        Serial.println("control is in ev_txcomplete");
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
           
            
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
        char buff[5];
        float temp = getTemp1();
        dtostrf(temp, 0, 5, buff);
        char finalData[51];
        finalData[0] = 'T';        
        finalData[1] = ':';
        finalData[2] = buff[0];
        finalData[3] = buff[1];
        finalData[4] = buff[2];
        finalData[5] = buff[3];
        finalData[6] = buff[4];
        finalData[7] = ' ';
        float eC = ecV();
        dtostrf(eC, 0, 5, buff);
        finalData[8] = 'E';
        finalData[9] = 'C';
        finalData[10] = ':';
        finalData[11] = buff[0];
        finalData[12] = buff[1];
        finalData[13] = buff[2];
        finalData[14] = buff[3];
        finalData[15] = buff[4];
        finalData[16] = ' ';
        float pH = ph();
        dtostrf(pH, 0, 5, buff);
        finalData[17] = 'p';
        finalData[18] = 'H';
        finalData[19] = ':';
        finalData[20] = buff[0];
        finalData[21] = buff[1];
        finalData[22] = buff[2];
        finalData[23] = buff[3];
        finalData[24] = buff[4];
        finalData[25] = ' ';
        float tds = TDS();
        dtostrf(tds, 0, 5, buff);
        finalData[26] = 'D';       
        finalData[27] = ':';
        finalData[28] = buff[0];
        finalData[29] = buff[1];
        finalData[30] = buff[2];
        finalData[31] = buff[3];
        finalData[32] = buff[4];
        finalData[33] =' ';
        float bod = BOD();
        dtostrf(bod, 0, 5, buff);
        finalData[34] = 'B';
        finalData[35] = 'O';
        finalData[36] = ':';
        finalData[37] = buff[0];
        finalData[38] = buff[1];
        finalData[39] = buff[2];
        finalData[40] = buff[3];
        finalData[41] = buff[4];
        finalData[42] = ' ';
        float cod = COD();
        dtostrf(cod, 0, 5, buff);
        finalData[43] = 'C';
        finalData[44] = 'O';
        finalData[45] = ':';
        finalData[46] = buff[0];
        finalData[47] = buff[1];
        finalData[48] = buff[2];
        finalData[49] = buff[3];
        finalData[50] = buff[4];
        finalData[51] = ' ';
       
        LMIC_setTxData2(1, finalData, sizeof(finalData), 0);
        Serial.println(F("Packet queued"));
        Serial.println(finalData);
        pump();
     
    }
    
   
    // Next TX is scheduled after TX_COMPLETE event.
    
    
}

void setup() {
    //wdt_disable();
    Serial.println("Started--------------------");
    pinMode(4,OUTPUT);
    delay (1000);   
    
   
    Serial.begin(115200);
    Serial.println(F("Starting"));
    pinMode(TEMP_PIN, INPUT_PULLUP);
    ec.begin();

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

    #if defined(CFG_in866)    // three channels are set for india as per ttn guidelines
    
    LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 865785000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 865602500, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 866300000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 866500000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 866550000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);
    #elif defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
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
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF7;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF10,14);

    // Start job
    Serial.println("channels are initialized");
    do_send(&sendjob);
    Serial.println("control is coming end of set up loop indicates successfull txion of one packet");
        
}

void loop() {
  

  os_runloop_once();
}
 
