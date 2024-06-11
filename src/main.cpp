/*
  RadioLib LoRaWAN ABP Example

  This example joins a LoRaWAN network and will send
  uplink packets. Before you start, you will have to
  register your device at https://www.thethingsnetwork.org/
  After your device is registered, you can run this example.
  The device will join the network and start uploading data.

  Running this examples REQUIRES you to check "Resets DevNonces"
  on your LoRaWAN dashboard. Refer to the network's
  documentation on how to do this.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/

  For LoRaWAN details, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/LoRaWAN

*/

#include <RadioLib.h>
#include "LoRaBoards.h"

// !!!!! APP EUI
#define RADIOLIB_LORAWAN_JOIN_EUI 0x0000000000000000
// !!!!! DEV EUI
#define RADIOLIB_LORAWAN_DEV_EUI 0x70B3D57ED8003016
// !!!!! APP EUI
#define RADIOLIB_LORAWAN_APP_KEY 0xCB, 0x63, 0x69, 0x7F, 0x53, 0xC7, 0x08, 0x26, 0x63, 0x5E, 0xF8, 0x45, 0xEE, 0x67, 0xEB, 0x7F
// !!! APP KEY
#define RADIOLIB_LORAWAN_NWK_KEY 0xBE, 0xE2, 0x2B, 0x42, 0x29, 0xDC, 0x23, 0x01, 0xEB, 0xC9, 0x58, 0x45, 0xC6, 0x80, 0xAE, 0xBE
// !!!!! DEV ADDR
#define RADIOLIB_LORAWAN_ADDR 0x2608C435
// !!!!! APP SKey
#define RADIOLIB_LORAWAN_APPSKEY 0x05, 0x6C, 0x1E, 0x4C, 0x14, 0x50, 0xC7, 0x54, 0xBA, 0xAD, 0xC6, 0xB3, 0xD0, 0xE9, 0xAD, 0x94
// !!!!! fNwkSIntKey
#define RADIOLIB_LORAWAN_FNSKEY 0xCF, 0xAD, 0xF1, 0xA5, 0x82, 0x34, 0x17, 0xDA, 0xD3, 0xEA, 0x3F, 0xE4, 0x86, 0x5C, 0xEA, 0x41
// !!!!! sNwkSIntKey
#define RADIOLIB_LORAWAN_SNSKEY 0xEE, 0x0B, 0x55, 0xCC, 0x60, 0x50, 0x1F, 0x8C, 0xCD, 0xC9, 0xC5, 0x14, 0x89, 0xA8, 0xFC, 0xCD
// !!!!! nwkSEncKey
#define RADIOLIB_LORAWAN_NSKEY 0xE1, 0x04, 0x41, 0x4D, 0x3E, 0xCC, 0x6D, 0x93, 0xD6, 0xFC, 0xAB, 0x39, 0xC1, 0x82, 0xB3, 0x95

#if defined(USING_SX1276)
SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
#elif defined(USING_SX1262)
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#elif defined(USING_SX1278)
SX1278 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
#elif defined(USING_LR1121)
LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

// how often to send an uplink - consider legal & FUP constraints - see notes
const uint32_t uplinkIntervalSeconds = 5UL * 60UL; // minutes x seconds

// for the curious, the #ifndef blocks allow for automated testing &/or you can
// put your EUI & keys in to your platformio.ini - see wiki for more tips

// regional choices: EU868, US915, AU915, AS923, IN865, KR920, CN780, CN500
const LoRaWANBand_t Region = EU868;
const uint8_t subBand = 0; // For US915, change this to 2, otherwise leave on 0

// ============================================================================

// copy over the EUI's & keys in to the something that will not compile if incorrectly formatted
uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = {RADIOLIB_LORAWAN_APP_KEY};
uint8_t nwkKey[] = {RADIOLIB_LORAWAN_NWK_KEY};
uint32_t addr = RADIOLIB_LORAWAN_ADDR;
uint8_t fNwkSIntKey[] = {RADIOLIB_LORAWAN_FNSKEY};
uint8_t sNwkSIntKey[] = {RADIOLIB_LORAWAN_SNSKEY};
uint8_t nwkSEncKey[] = {RADIOLIB_LORAWAN_NSKEY};
uint8_t appSKey[] = {RADIOLIB_LORAWAN_APPSKEY};

// create the LoRaWAN node
LoRaWANNode node(&radio, &Region, subBand);

// helper function to display any issues
void debug(bool isFail, const __FlashStringHelper *message, int state, bool Freeze)
{
  if (isFail)
  {
    Serial.print(message);
    Serial.print("(");
    Serial.print(state);
    Serial.println(")");
    while (Freeze)
      ;
  }
}

// helper function to display a byte array
void arrayDump(uint8_t *buffer, uint16_t len)
{
  for (uint16_t c = 0; c < len; c++)
  {
    char b = buffer[c];
    if (b < 0x10)
    {
      Serial.print('0');
    }
    Serial.print(b, HEX);
  }
  Serial.println();
}

static uint32_t txCounter = 0;

void setup()
{
  Serial.begin(115200);

  while (!Serial)
    ;

  setupBoards();

#ifdef RADIO_TCXO_ENABLE
  pinMode(RADIO_TCXO_ENABLE, OUTPUT);
  digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif

  delay(5000); // Give time to switch to the serial monitor

  Serial.println(F("\nSetup ... "));

  Serial.println(F("Initialise the radio"));

  int state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);
  printResult(state == RADIOLIB_ERR_NONE);
  delay(2000);

#ifdef USING_DIO2_AS_RF_SWITCH
#ifdef USING_SX1262
  // Some SX126x modules use DIO2 as RF switch. To enable
  // this feature, the following method can be used.
  // NOTE: As long as DIO2 is configured to control RF switch,
  //       it can't be used as interrupt pin!
  if (radio.setDio2AsRfSwitch() != RADIOLIB_ERR_NONE)
  {
    Serial.println(F("Failed to set DIO2 as RF switch!"));
    while (true)
      ;
  }
#endif // USING_SX1262
#endif // USING_DIO2_AS_RF_SWITCH

#ifdef RADIO_SWITCH_PIN
  // T-MOTION
  const uint32_t pins[] = {
      RADIO_SWITCH_PIN,
      RADIO_SWITCH_PIN,
      RADIOLIB_NC,
  };
  static const Module::RfSwitchMode_t table[] = {
      {Module::MODE_IDLE, {0, 0}},
      {Module::MODE_RX, {1, 0}},
      {Module::MODE_TX, {0, 1}},
      END_OF_MODE_TABLE,
  };
  radio.setRfSwitchTable(pins, table);
#endif

  int retry = 0;
  while (1)
  {
    if (u8g2)
    {
      u8g2->clearBuffer();
      u8g2->setFont(u8g2_font_NokiaLargeBold_tf);
      uint16_t str_w = u8g2->getStrWidth(BOARD_VARIANT_NAME);
      u8g2->drawStr((u8g2->getWidth() - str_w) / 2, 16, BOARD_VARIANT_NAME);
      u8g2->drawHLine(5, 21, u8g2->getWidth() - 5);

      u8g2->setCursor(0, 38);
      u8g2->print("Join LoRaWAN :");
      u8g2->println(String(retry++));
      u8g2->sendBuffer();
    }
    Serial.println(F("Join ('login') to the LoRaWAN Network"));
    state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey, true);
    // state = node.beginABP(addr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey, true);
    debug(state < RADIOLIB_ERR_NONE, F("Join failed"), state, false);
    if (state == RADIOLIB_ERR_NONE)
    {
      break;
    }
    delay(3000);
    Serial.print("Retry ");
  }

  Serial.println(F("Joined!\n"));
}

void loop()
{
  Serial.println(F("Sending uplink"));

  // This is the place to gather the sensor inputs
  // Instead of reading any real sensor, we just generate some random numbers as example
  uint8_t value1 = radio.random(100);
  uint16_t value2 = radio.random(2000);
  // uint8_t value1 = 100;
  // uint16_t value2 = 2000;

  // Build payload byte array
  uint8_t uplinkPayload[3];
  uplinkPayload[0] = value1;
  uplinkPayload[1] = highByte(value2); // See notes for high/lowByte functions
  uplinkPayload[2] = lowByte(value2);

  // Perform an uplink
  int state = node.sendReceive(uplinkPayload, sizeof(uplinkPayload));
  debug((state != RADIOLIB_LORAWAN_NO_DOWNLINK) && (state != RADIOLIB_ERR_NONE), F("Error in sendReceive"), state, false);

  if (u8g2)
  {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_NokiaLargeBold_tf);
    uint16_t str_w = u8g2->getStrWidth(BOARD_VARIANT_NAME);
    u8g2->drawStr((u8g2->getWidth() - str_w) / 2, 16, BOARD_VARIANT_NAME);
    u8g2->drawHLine(5, 21, u8g2->getWidth() - 5);
    u8g2->setCursor(0, 38);
    // u8g2->print(node.isJoined() ? "Joined." : "NoJoin");
    u8g2->print(node.isJoined() ? "Joined." : "NoJoin");
    u8g2->print("\tTx:");
    u8g2->println(++txCounter);
#ifdef ARDUINO_ARCH_ESP32
    u8g2->setCursor(0, 54);
    u8g2->println("MAC:");
    u8g2->println(ESP.getEfuseMac(), HEX);
#endif
    u8g2->sendBuffer();
  }

  Serial.print(F("Uplink complete, next in "));
  Serial.print(uplinkIntervalSeconds);
  Serial.println(F(" seconds"));

#ifdef BOARD_LED
  digitalWrite(BOARD_LED, LED_ON);
  delay(200);
  digitalWrite(BOARD_LED, !LED_ON);
#endif

  // Wait until next uplink - observing legal & TTN FUP constraints
  delay(uplinkIntervalSeconds * 1000UL); // delay needs milli-seconds
}
