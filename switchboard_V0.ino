
/* Voltage readers, Victron VE.direct, switch board
 *  Olli Rinne 2019, Urban Eco Island, Forum Virium Helsinki Oy
 *  
 *  LoRa Wan based on https://github.com/matthijskooijman/arduino-lmic 
 *  Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *  
 *  Victron connection code:
 *  https://www.romlea.nl/Arduino%20MPPT/Page.htm , https://www.youtube.com/watch?v=w-9kYkSuCwc
 *  Victron VE.Direct protocol: https://www.victronenergy.com/live/vedirect_protocol:faq
 *  
 *  */



 /* Connections for Victron.VE:
    MPPT pin     MPPT        Arduino     ESP pin
    1            GND         GND         GND
    2            RX          TX          -              do not use!
    3            TX          RX          25
    4            Power+      none        -              do not use!

    Disconnect all cables to Victron before flashing!
 */

#include <lmic.h>
#include "settings.h"

#undef LMIC_PRINTF_TO 
#include <hal/hal.h>


// Pin mapping
#define DEBUG_PRINT TRUE

#define SERIAL1_RXPIN 25 // Victron charge controller,
#define SERIAL1_TXPIN 2   // NOT IN USE, do not send anything 

#define PIN_ID_BATTERY_VOLTAGE 34   // connect to the battery via voltage diver
#define PIN_ID_PANEL_VOLTAGE 35     // connect to panel via voltage diver



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 120;



// Pins for LORA chip SPI interface, reset line and interrupt lines
#define LORA_SCK  (5) 
#define LORA_CS   (18)
#define LORA_MISO (19)
#define LORA_MOSI (27)
#define LORA_RST  (23)
#define LORA_IRQ  (26)
#define LORA_IO1  (33)
#define LORA_IO2  (32)

const lmic_pinmap lmic_pins = {
  .nss = LORA_CS, 
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LORA_RST == NOT_A_PIN ? LMIC_UNUSED_PIN : LORA_RST,
   .dio = {LORA_IRQ, LORA_IO1,
            LORA_IO2 == NOT_A_PIN ? LMIC_UNUSED_PIN : LORA_IO2}
};

typedef struct  {
  int16_t batteryVoltageRaw;
  int16_t panelVoltageRaw;
  float mainVoltage_V;      // mV
  float panelVoltage_VPV;   // mV
  float panelPower_PPV;     // W
  float batteryCurrent_I;   // mA
  float yieldTotal_H19;     // 0.01 kWh
  float yieldToday_H20;     // 0.01 kWh
  float maxPowerToday_H21;  // W
  float yieldYesterday_H22; // 0.01 kWh
  float maxPowerYesterday_H23; // W
  int errorCode_ERR;
  int stateOfOperation_CS;
} measurement ;

measurement status = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};



void onEvent (ev_t ev) {
  #ifdef DEBUG_PRINT
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
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
          //  os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
#endif

    if (ev==EV_TXCOMPLETE) {
         // Schedule next transmission
         os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
   
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        //readSensors();

    int16_t batteryVoltageRaw = analogRead(PIN_ID_BATTERY_VOLTAGE) ; //define constant
    int16_t panelVoltageRaw = analogRead(PIN_ID_PANEL_VOLTAGE) ; //define constant

    status.batteryVoltageRaw = batteryVoltageRaw;
    status.panelVoltageRaw = panelVoltageRaw;
   // float batteryVoltage = batteryVoltageRaw /72.75;
   // float panelVoltage = analogRead(35) /26.16;
    
      LMIC_setTxData2(1, (byte*)&status, sizeof(status), 0);
    Serial.print( sizeof(status));
    Serial.println(F("Packet queued 1")); 
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


char buf[80];

float floatFromBuffer(String val) {
 val.toCharArray(buf, sizeof(buf));                  
 return atof(buf);  
}
int intFromBuffer(String val) {
 val.toCharArray(buf, sizeof(buf));                  
 return atoi(buf);  
}

void readVEDirect() 
{
  //The device transmits blocks of data at 1 second intervals. Each field is sent using the following format:
  // Serial.setTimeout(); // ms default 1000
  String label, val;
  
  if (Serial1.available())
   {
        label = Serial1.readStringUntil('\t');                // this is the actual line that reads the label from the MPPT controller
        val = Serial1.readStringUntil('\r\r\n');              // this is the line that reads the value of the label

     if (label =="V") {
         status.mainVoltage_V = floatFromBuffer(val);
     }
     else if (label =="VPV") {
         status.panelVoltage_VPV = floatFromBuffer(val);
     }
     else if (label =="PPV") {
         status.panelPower_PPV = floatFromBuffer(val);
     }
     else if (label =="I") {
         status.batteryCurrent_I = floatFromBuffer(val);
     }
     else if (label =="H19") {
         status.yieldTotal_H19 = floatFromBuffer(val);
     }
     else if (label =="H20") {
         status.yieldToday_H20 = floatFromBuffer(val);
     }
     else if (label =="H21") {
         status.maxPowerToday_H21 = floatFromBuffer(val);
     }
     else if (label =="H22") {
         status.yieldYesterday_H22 = floatFromBuffer(val);
     }
     else if (label =="H23") {
         status.maxPowerYesterday_H23 = floatFromBuffer(val);
     }
   else if (label =="ERR") {
         status.errorCode_ERR = intFromBuffer(val);
     }
   else if (label =="CS") {
         status.stateOfOperation_CS = intFromBuffer(val);
     }

   }
}




// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Victron
  Serial1.begin(19200, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
 
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
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}


// the loop routine runs over and over again forever:
void loop() {
 readVEDirect();
  os_runloop_once();

  
}
