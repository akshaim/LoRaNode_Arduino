#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <LowPower.h>

#include <MechaQMC5883.h>
MechaQMC5883 qmc;

#include <readVcc.h>
int MIN_V = 2600;
int MAX_V = 3600;
int batteryPcnt;
float batteryV;


// Enable debug prints to serial monitor
#define DEBUG

static const PROGMEM u1_t NWKSKEY[16] = {xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx };

static const u1_t PROGMEM APPSKEY[16] = {xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx };

static const u4_t DEVADDR = 0xXXXXXXXXX;


void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

struct {
  unsigned short high_axy;
  unsigned short high_ayz;
  unsigned short high_axz;
  unsigned short battery_percentage;

} mydata;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL_MINUTES = 1;
const unsigned TX_INTERVAL = 30 * TX_INTERVAL_MINUTES;

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 6,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      Serial.println("Sleep for 32s");
      Serial.flush();
      for (int i = 0; i < 4; i++) {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

      }
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);

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
#ifdef DEBUG
  Serial.println(F("Leave onEvent"));
#endif
}

void do_send(osjob_t* j) {

  qmc.init();
  int x, y, z, a_xy, a_yz, a_xz;
  qmc.read(&x, &y, &z);
  a_xy = qmc.azimuth(&x, &y);
  a_xz = qmc.azimuth(&x, &z);
  a_yz = qmc.azimuth(&y, &z);
  qmc.setMode(Mode_Standby, ODR_200Hz, RNG_8G, OSR_256);

  mydata.high_axy = a_xy;
  mydata.high_ayz = a_yz;
  mydata.high_axz = a_xz;
  mydata.battery_percentage = getbattery();

  LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
#ifdef DEBUG
  Serial.print("a_xy");
  Serial.print(a_xy);
  Serial.print(" a_xz");
  Serial.print(a_xz);
  Serial.print(" a_yz");
  Serial.println(a_yz);
#endif
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

  Serial.begin(9600);
  Serial.println(F("Starting"));
  Wire.begin();


#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif
  os_init();
  LMIC_reset();

#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif


  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF10;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);

#ifdef DEBUG
  Serial.println(F("Leave setup"));
#endif
}


int getbattery()
{
  batteryV = readVcc();
  batteryPcnt = min(map(batteryV, MIN_V, MAX_V, 0, 100), 100);
  return batteryPcnt;

}

void loop() {
  os_runloop_once();
}
