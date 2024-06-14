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

// #define TTGO_SUPREME_01 ;
#define TTGO_SUPREME_02 ;

#if defined(TTGO_SUPREME_01)
// --------------- Supremem 01 -----------------

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
// --------------------------------------------
#elif defined(TTGO_SUPREME_02)
// --------------- Supremem 02 -----------------
// !!!!! APP EUI
#define RADIOLIB_LORAWAN_JOIN_EUI 0x0000000000000000
// !!!!! DEV EUI
#define RADIOLIB_LORAWAN_DEV_EUI 0x70B3D57ED800304F
// !!!!! APP EUI
#define RADIOLIB_LORAWAN_APP_KEY 0x4A, 0x54, 0x85, 0x2C, 0x48, 0x52, 0x53, 0xA4, 0xAA, 0x32, 0xA8, 0x01, 0x7D, 0xFC, 0xF1, 0x2D
// !!! APP KEY
#define RADIOLIB_LORAWAN_NWK_KEY 0x4A, 0x54, 0x85, 0x2C, 0x48, 0x52, 0x53, 0xA4, 0xAA, 0x32, 0xA8, 0x01, 0x7D, 0xFC, 0xF1, 0x2D

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
// ----------------------------------------------
#endif

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
const uint32_t uplinkIntervalSeconds = 5UL * 30UL; // minutes x seconds

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

// ------------------ Declare BME280 Sensor -------------------
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// -------------------- End of BME280 set up ------------------------

// ------------------ Declare QMI8658 Sensor -------------------
#include "SensorQMI8658.hpp"
#include <MadgwickAHRS.h>

unsigned long delayTime;
// SH1106Wire display(0x3c, I2C_SDA, I2C_SCL);
SensorQMI8658 qmi;

IMUdata acc;
IMUdata gyr;

Madgwick IMUfilter;
unsigned long microsPerReading, microsPrevious;

// -------------------- End of QMI8658 set up ------------------------

// ------------------ Declare PMU Sensor -------------------
// ---------- Already done in LoRaBoards.cpp
// -------------------- End of PMU set up ------------------------

// ------------------ Declare GPS Sensor -------------------
#include "SparkFun_Ublox_Arduino_Library.h"

SFE_UBLOX_GPS myGPS;

#include <MicroNMEA.h> //https://github.com/stevemarple/MicroNMEA

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// -------------------- End of GPS set up ------------------------

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
// --------------- GPS function added by Tim -------------------
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  // Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  // for sentence cracking
  nmea.process(incoming);
}
// ---------------------------------------------------------------

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

  // ----------------- Initialise BME280 sensor -------------------
  if (!bme.begin(0x77, &Wire))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
  // ----------------- End of BME280 initialise --------------------------

  // ----------------- Initialise QMI8658 sensor -------------------
  pinMode(SPI_CS, OUTPUT); // sdcard pin set high
  digitalWrite(SPI_CS, HIGH);

  if (!qmi.begin(IMU_CS, -1, -1, -1, SDCardSPI))
  {
    Serial.println("Failed to find QMI8658 - check your wiring!");
    while (1)
    {
      delay(1000);
    }
  }

  // Get chip id
  Serial.print("Device ID:");
  Serial.println(qmi.getChipID(), HEX);

  qmi.configAccelerometer(
      /*
       * ACC_RANGE_2G
       * ACC_RANGE_4G
       * ACC_RANGE_8G
       * ACC_RANGE_16G
       * */
      SensorQMI8658::ACC_RANGE_2G,
      /*
       * ACC_ODR_1000H
       * ACC_ODR_500Hz
       * ACC_ODR_250Hz
       * ACC_ODR_125Hz
       * ACC_ODR_62_5Hz
       * ACC_ODR_31_25Hz
       * ACC_ODR_LOWPOWER_128Hz
       * ACC_ODR_LOWPOWER_21Hz
       * ACC_ODR_LOWPOWER_11Hz
       * ACC_ODR_LOWPOWER_3H
       * */
      SensorQMI8658::ACC_ODR_1000Hz,
      /*
       *  LPF_MODE_0     //2.66% of ODR
       *  LPF_MODE_1     //3.63% of ODR
       *  LPF_MODE_2     //5.39% of ODR
       *  LPF_MODE_3     //13.37% of ODR
       * */
      SensorQMI8658::LPF_MODE_0,
      // selfTest enable
      true);

  qmi.configGyroscope(
      /*
       * GYR_RANGE_16DPS
       * GYR_RANGE_32DPS
       * GYR_RANGE_64DPS
       * GYR_RANGE_128DPS
       * GYR_RANGE_256DPS
       * GYR_RANGE_512DPS
       * GYR_RANGE_1024DPS
       * */
      SensorQMI8658::GYR_RANGE_256DPS,
      /*
       * GYR_ODR_7174_4Hz
       * GYR_ODR_3587_2Hz
       * GYR_ODR_1793_6Hz
       * GYR_ODR_896_8Hz
       * GYR_ODR_448_4Hz
       * GYR_ODR_224_2Hz
       * GYR_ODR_112_1Hz
       * GYR_ODR_56_05Hz
       * GYR_ODR_28_025H
       * */
      SensorQMI8658::GYR_ODR_896_8Hz,
      /*
       *  LPF_MODE_0     //2.66% of ODR
       *  LPF_MODE_1     //3.63% of ODR
       *  LPF_MODE_2     //5.39% of ODR
       *  LPF_MODE_3     //13.37% of ODR
       * */
      SensorQMI8658::LPF_MODE_3,
      // selfTest enable
      true);

  // In 6DOF mode (accelerometer and gyroscope are both enabled),
  // the output data rate is derived from the nature frequency of gyroscope
  qmi.enableGyroscope();
  qmi.enableAccelerometer();

  // Print register configuration information
  Serial.println("dumping register config");
  qmi.dumpCtrlRegister();
  qmi.enableSyncSampleMode();

  // start  filter
  IMUfilter.begin(25);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  Serial.println("Read data now...");

  // ----------------- End of QMI8658 initialise --------------------------
  // ------------------- Already done in LoRaBoards.cpp
  // ----------------- Initialise PMU sensor -------------------

  // ----------------- End of PMU initialise --------------------------

  // ----------------- Initialise GPS sensor -------------------

  if (myGPS.begin(SerialGPS) == false)
  {
    Serial.println(F("Ublox GPS not detected . Please check wiring. Freezing."));
    while (1)
      ;
  }

  // ----------------- End of GPS initialise --------------------------
}

void loop()
{
  Serial.println(F("Preparing uplink data"));

  // This is the place to gather the sensor inputs
  // Instead of reading any real sensor, we just generate some random numbers as example

  // ------------------- Get BME280 sensor data ---------------------
  int32_t vTemp = (bme.readTemperature() * 10000);
  uint32_t vPress = (bme.readPressure() * 100); // already mult by 100 so still div 100 to decode.
  uint32_t vHumid = (bme.readHumidity() * 10000);

  // Print values to serial
  Serial.print("Temp:");
  Serial.println(vTemp);
  Serial.print("Press:");
  Serial.println(vPress);
  Serial.print("Humid:");
  Serial.println(vHumid);

  // ------------------- End of BME 280 sensor ----------------------

  // ------------------- Get QMI8658 sensor data ---------------------
  // check if it's time to read data and update the filter
  if (micros() - microsPrevious >= microsPerReading)
  {

    // read raw data from IMU

    if (qmi.getDataReady())
    {

      qmi.getAccelerometer(acc.x, acc.y, acc.z);
      qmi.getGyroscope(gyr.x, gyr.y, gyr.z);

      // update the filter, which computes orientation
      IMUfilter.updateIMU(gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z);
    }
  }

  int32_t vRoll = (IMUfilter.getRoll() * 10000);
  int32_t vPitch = (IMUfilter.getPitch() * 10000);
  int32_t vHeading = (IMUfilter.getYaw() * 10000);

  // Print values to serial
  Serial.print("Roll:");
  Serial.println(vRoll);
  Serial.print("Pitch:");
  Serial.println(vPitch);
  Serial.print("Heading:");
  Serial.println(vHeading);

  //}
  microsPrevious = microsPrevious + microsPerReading;

  // ------------------- End of BME 280 sensor ----------------------

  // ------------------- Get PMU sensor data ---------------------

  uint8_t vPmu_charging = (PMU->isCharging() ? 1 : 0);
  uint8_t vPmu_discharge = (PMU->isDischarge() ? 1 : 0);
  uint8_t vPmu_vbusIn = (PMU->isVbusIn() ? 1 : 0);
  uint32_t vPmu_battV = (PMU->getBattVoltage());
  uint32_t vPmu_vbusV = (PMU->getVbusVoltage());
  uint32_t vPmu_SysV = (PMU->getSystemVoltage());
  uint32_t vPmu_battperc = (PMU->getBatteryPercent() * 10000);

  // Print values to serial
  Serial.print("Charging:");
  Serial.println(vPmu_charging);
  Serial.print("Discharging:");
  Serial.println(vPmu_discharge);
  Serial.print("vBus In:");
  Serial.println(vPmu_vbusIn);
  Serial.print("Battery V:");
  Serial.println(vPmu_battV);
  Serial.print("vBus V:");
  Serial.println(vPmu_vbusV);
  Serial.print("Sys V:");
  Serial.println(vPmu_SysV);
  Serial.print("Batt Perc:");
  Serial.println(vPmu_battperc);

  // ------------------- End of PMU sensor ----------------------

  // ------------------- Get GPS sensor data ---------------------
  // myGPS.checkUblox(); // See if new data is available. Process bytes as they come in.
  // set the readings to zero
  uint32_t vLatitude = 0;
  uint32_t vLongitude = 0;
  uint8_t vnumSatellites = 0;

  myGPS.checkUblox(); // See if new data is available. Process bytes as they come in.

  if (nmea.isValid() == true)
  {
    vLatitude = (nmea.getLatitude()); // already an interger mult by 1000000, still divide in decode
    vLongitude = (nmea.getLongitude());
  }
  vnumSatellites = (nmea.getNumSatellites());

  // Print values to serial
  Serial.print("Latitude:");
  Serial.println(vLatitude);
  Serial.print("Longitude:");
  Serial.println(vLongitude);
  Serial.print("No Satellites:");
  Serial.println(vnumSatellites);

  // ------------------- End of GPS sensor ----------------------
  Serial.println(F("Building bytye array"));

  // Build payload byte array
  uint8_t uplinkPayload[52];
  // Add environmental data
  uplinkPayload[0] = (byte)((vTemp & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[1] = (byte)((vTemp & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[2] = (byte)((vTemp & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[3] = (byte)((vTemp & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[4] = (byte)((vPress & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[5] = (byte)((vPress & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[6] = (byte)((vPress & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[7] = (byte)((vPress & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[8] = (byte)((vHumid & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[9] = (byte)((vHumid & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[10] = (byte)((vHumid & 0x0000FF00) >> 8); // See notes for high/lowByte functions
  uplinkPayload[11] = (byte)((vHumid & 0x000000FF));      // See notes for high/lowByte functions

  // Add orientation data
  uplinkPayload[12] = (byte)((vRoll & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[13] = (byte)((vRoll & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[14] = (byte)((vRoll & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[15] = (byte)((vRoll & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[16] = (byte)((vPitch & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[17] = (byte)((vPitch & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[18] = (byte)((vPitch & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[19] = (byte)((vPitch & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[20] = (byte)((vHeading & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[21] = (byte)((vHeading & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[22] = (byte)((vHeading & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[23] = (byte)((vHeading & 0x000000FF));       // See notes for high/lowByte functions

  // Add PMU data
  uplinkPayload[24] = vPmu_charging;
  uplinkPayload[25] = vPmu_discharge;
  uplinkPayload[26] = vPmu_vbusIn;

  uplinkPayload[27] = (byte)((vPmu_battV & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[28] = (byte)((vPmu_battV & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[29] = (byte)((vPmu_battV & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[30] = (byte)((vPmu_battV & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[31] = (byte)((vPmu_vbusV & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[32] = (byte)((vPmu_vbusV & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[33] = (byte)((vPmu_vbusV & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[34] = (byte)((vPmu_vbusV & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[35] = (byte)((vPmu_SysV & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[36] = (byte)((vPmu_SysV & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[37] = (byte)((vPmu_SysV & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[38] = (byte)((vPmu_SysV & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[39] = (byte)((vPmu_battperc & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[40] = (byte)((vPmu_battperc & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[41] = (byte)((vPmu_battperc & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[42] = (byte)((vPmu_battperc & 0x000000FF));       // See notes for high/lowByte functions

  // Add positioning data
  uplinkPayload[43] = (byte)((vLatitude & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[44] = (byte)((vLatitude & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[45] = (byte)((vLatitude & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[46] = (byte)((vLatitude & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[47] = (byte)((vLongitude & 0xFF000000) >> 24); // See notes for high/lowByte functions
  uplinkPayload[48] = (byte)((vLongitude & 0x00FF0000) >> 16); // See notes for high/lowByte functions
  uplinkPayload[49] = (byte)((vLongitude & 0x0000FF00) >> 8);  // See notes for high/lowByte functions
  uplinkPayload[50] = (byte)((vLongitude & 0x000000FF));       // See notes for high/lowByte functions

  uplinkPayload[51] = vnumSatellites;

  // Set f_port to 21 for later use
  uint8_t vf_Port = 21;

  Serial.println(F("Sending uplink"));

  // Perform an uplink
  // int state = node.sendReceive(uplinkPayload, sizeof(uplinkPayload));
  int state = node.sendReceive(uplinkPayload, sizeof(uplinkPayload), vf_Port);
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
  Serial.println();

#ifdef BOARD_LED
  digitalWrite(BOARD_LED, LED_ON);
  delay(200);
  digitalWrite(BOARD_LED, !LED_ON);
#endif

  // Wait until next uplink - observing legal & TTN FUP constraints
  delay(uplinkIntervalSeconds * 1000UL); // delay needs milli-seconds
}
