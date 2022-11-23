

//  Based on https://github.com/RAKWireless/WisBlock/blob/master/examples/RAK4630/communications/LoRa/LoRaWAN/LoRaWAN_OTAA_ABP/LoRaWAN_OTAA_ABP.ino
//  Note: This program needs SX126x-Arduino Library version 2.0.0 or later. In platformio.ini, set...
//    lib_deps = beegee-tokyo/SX126x-Arduino@^2.0.0
/**
 * @file LoRaWAN_OTAA_ABP.ino
 * @author rakwireless.com
 * @brief LoRaWan node example with OTAA/ABP registration
 * @version 0.1
 * @date 2020-08-21
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */
#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680
#include <U8g2lib.h>	// Click to install library: http://librarymanager/ALL#u8g2

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
Adafruit_BME680 bme;

#define PIN_VBAT WB_A0

uint32_t vbat_pin = PIN_VBAT;

#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)      // Compensation factor for the VBAT divider, depend on the board

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
#define SEALEVELPRESSURE_HPA (1010.0)


// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

//  TODO: Set this to your LoRaWAN Region
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;

//  Set to true to select Over-The-Air Activation (OTAA), false for Activation By Personalisation (ABP)
bool doOTAA = true;

//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ABP keys
uint32_t nodeDevAddr = 0x00000000;
uint8_t nodeNwsKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppsKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0									  /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5							      /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3										  /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;					          /* class definition*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							  /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {
    LORAWAN_ADR_ON, 
    LORAWAN_DATERATE, 
    LORAWAN_PUBLIC_NETWORK, 
    JOINREQ_NBTRIALS, 
    LORAWAN_TX_POWER, 
    LORAWAN_DUTYCYCLE_OFF
};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {
    BoardGetBatteryLevel, 
    BoardGetUniqueId, 
    BoardGetRandomSeed,
    lorawan_rx_handler, 
    lorawan_has_joined_handler, 
    lorawan_confirm_class_handler, 
    lorawan_join_failed_handler
};

// Private definition
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 15*60000                        /**< Defines for user timer, the application data transmission interval. (15min), value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

/**
 * @brief Get RAW Battery Voltage
 * */
float readVBAT(void)
{
  float raw;
  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(vbat_pin);
  return raw * REAL_VBAT_MV_PER_LSB;
} 

/**
 * @brief Convert from raw mv to percentage
 * @param mvolts
 *    RAW Battery Voltage
 */
uint8_t mvToPercent(float mvolts)
{
    if (mvolts < 3300)
      return 0;

    if (mvolts < 3600)
    {
      mvolts -= 3300;
      return mvolts / 30;
    }

  mvolts -= 3600;
  return (10 + (mvolts * 0.15F)); // thats mvolts /6.66666666
}

/**
 * @brief get LoRaWan Battery value
 * @param mvolts
 *    Raw Battery Voltage
 */
uint8_t mvToLoRaWanBattVal(float mvolts)
{
    if (mvolts < 3300)
        return 0;

    if (mvolts < 3600)
    {
        mvolts -= 3300;
        return mvolts / 30 * 2.55;
    }

    mvolts -= 3600;
    return (10 + (mvolts * 0.15F)) * 2.55;
}


void current_vbat(){
   // Get a raw ADC reading
    float vbat_mv = readVBAT();

    // Convert from raw mv to percentage (based on LIPO chemistry)
    uint8_t vbat_per = mvToPercent(vbat_mv);

    // Display the results
    Serial.print("LIPO = ");
    Serial.print(vbat_mv);
    Serial.print(" mV (");
    Serial.print(vbat_per);
    Serial.print("%) LoRaWan Batt ");
    Serial.println(mvToLoRaWanBattVal(vbat_mv));
}



// At startup, we join the LoRaWAN network, in either OTAA or ABP mode
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN2, OUTPUT);
  digitalWrite(LED_BUILTIN2, LOW);

  // Initialize LoRa chip.
  lora_rak4630_init();

  // bme680 init
  init_bme680();

  //DISPLAY init
  u8g2.begin();

  // Initialize Serial for debug output   
  // On nRF52840 the USB serial is not available immediately

  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    else
    {
      break;
    }
  }
  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }

  switch (g_CurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case  LORAMAC_REGION_AS923_2:
      Serial.println("Region: AS923_2");
      break;
    case LORAMAC_REGION_AS923_3:
      Serial.println("Region: AS923_3");
      break;
    case LORAMAC_REGION_AS923_4:
      Serial.println("Region: AS923_4");
      break;
    case LORAMAC_REGION_RU864: 
      Serial.println("Region: RU864");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_CN779:
      Serial.println("Region: CN779");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
      break;
  }
  Serial.println("=====================================");
  
  //creat a user timer to send data to server period
  uint32_t err_code;
  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
    return;
  }

  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWaN
  err_code = lmh_init(  //  lmh_init now takes 3 parameters instead of 5
    &g_lora_callbacks,  //  Callbacks 
    g_lora_param_init,  //  Functions
    doOTAA,             //  Set to true for OTAA
    g_CurrentClass,     //  Class 
    g_CurrentRegion     //  Region
  );
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  Serial.println("Joining LoRaWAN network...");
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font

  u8g2.drawStr(20, 39, "Joining ...");
  u8g2.sendBuffer(); // transfer internal memory to the display
  
  lmh_join();
  
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);
  // Get a single ADC sample and throw it away
  readVBAT();
}

// Nothing here for now
void loop()
{
  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions. 
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  // When we have joined the LoRaWAN network, start a 20-second timer
  Serial.println("OTAA Mode, Network Joined!");
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font

  u8g2.drawStr(20, 39, "Joined");
  u8g2.sendBuffer(); // transfer internal memory to the display

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}

/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  // If we can't join the LoRaWAN network, show the error
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  // When we receive a LoRaWAN Packet from the LoRaWAN Gateway, display it.
  // TODO: Ensure that app_data->buffer is null-terminated.
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}
// Callback Function that is called when the LoRaWAN Class has been changed
void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}


// This is called when the timer expires. We send a LoRaWAN Packet.
void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    // Not joined, try again later
    return;
  }

  //@brief read bme Sensor and signal uplink with blue led
  if (!bme.performReading()) {
    Serial.println("No BME Reading");
    return;
  }
  bme680_get();
  //current_vbat();
  
  // Transmit the LoRaWAN Packet
  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}

/**@brief Function for handling user timerout event.
 */
void tx_lora_periodic_handler(void)
{
  //Turn off green LED to preserve Battery, Signal uplink with blue LED
   digitalWrite(LED_BUILTIN2, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN2, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  // When the timer has expired, send a LoRaWAN Packet and restart the timer
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
  Serial.println("Sending frame now...");
  send_lora_frame();
  Serial.println("Packet transmitted.");
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}

void init_bme680(void)
{
  Wire.begin();

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

String data = "";
// read bme sensor and battery charge in % and send to LORA App data buffer
void bme680_get()
{
  char oled_data[32] = {0};
  Serial.print("result: ");
  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;


  double temp = bme.temperature;
  double pres = bme.pressure / 100.0;
  double hum = bme.humidity;
  uint32_t gas = bme.gas_resistance; //show in kOhms -> better quality, higher resistance
  data = "Tem:" + String(temp) + "C " + "Hum:" + String(hum) + "% " + "Pres:" + String(pres) + "KPa " + "Gas:" + String(gas) + "Ohms";
  Serial.println(data);
  // display bme680 sensor data on OLED
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font

  memset(oled_data, 0, sizeof(oled_data));
  sprintf(oled_data, "T=%.2fC", temp);
  u8g2.drawStr(3, 15, oled_data);

  memset(oled_data, 0, sizeof(oled_data));
  snprintf(oled_data, 64, "RH=%.2f%%", hum);
  u8g2.drawStr(3, 30, oled_data);

  memset(oled_data, 0, sizeof(oled_data));
  sprintf(oled_data, "P=%.2fhPa", pres);
  u8g2.drawStr(3, 45, oled_data);

  memset(oled_data, 0, sizeof(oled_data));
  sprintf(oled_data, "G=%d Ohms", gas);
  u8g2.drawStr(3, 60, oled_data);

  u8g2.sendBuffer(); // transfer internal memory to the display

  uint16_t t = temp * 100;
  uint16_t h = hum * 100;
  uint32_t pre = pres * 100;

  Serial.println("==========Battery ================");
  float vbat_mv = readVBAT();
  uint8_t vbat_per = mvToPercent(vbat_mv);
  if(vbat_per >= 100){
    vbat_per = 100;
  }
                
  Serial.print("LIPO  in %: ");
  Serial.print(vbat_per);
  Serial.println("");
  Serial.println("======================================");
  


  //result: T=28.25C, RH=50.00%, P=958.57hPa, G=100406 Ohms
  m_lora_app_data.buffer[i++] = 0x01;
  m_lora_app_data.buffer[i++] = (uint8_t)(t >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)t;
  m_lora_app_data.buffer[i++] = (uint8_t)(h >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)h;
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0xFF000000) >> 24);
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0x00FF0000) >> 16);
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0x0000FF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(pre & 0x000000FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0xFF000000) >> 24);
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0x00FF0000) >> 16);
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0x0000FF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(gas & 0x000000FF);
  m_lora_app_data.buffer[i++] = (uint8_t)vbat_per;
  m_lora_app_data.buffsize = i;
}
