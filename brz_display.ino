//BRZ display - pressure and CANBus receiver
// Mini D1 ESP32
//  pressure sensor 150PSI
//   0 psi outputs 0.5V, 75 psi outputs 2.5V, 150 psi outputs 4.5V
//     pressure = (voltage - 0.461) / 4 * 1000; //in kPa

#define VERSION "1.1"

#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN_cfg; // CAN Config
CAN_frame_t rx_frame; //define frame instance

const int rx_queue_size = 10;       // Receive Queue size
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
int oilTemperature = 255;
int coolantTemperature = 255;
int rpm = 0;
static unsigned long last_time = 0;

//sprite - pressure value display
int sx = 70; // center x
int sy = 30; // center y

//Sprite 2 data graph
int gw = 165; //graph width
int gh = 85; //graph heigth
int gx = 22; //graph x
int gy = 84; // graph y
int values[20] = {0};
int values2[20] = {0};
int calib = 0;
int curent = 0;

float rawPressure = 0;
int pressure = 0;
float voltage = 0;
const int interval = 500;          // interval at which do... (milliseconds)

//ADC
#define I2C_SDA 16
#define I2C_SCL 17

#include "ADS1X15.h"
ADS1115 ADS(0x48, &Wire);
// Resistors value in kOhm for the voltage divider.
#define OIL_PRESSURE_R1 47
#define OIL_PRESSURE_R2 22.16
//static float voltage_divider = (22.16/47)+1;  //1.470213; //voltage divider 22.16/47
// Characteristics of the oil pressure sensor.
// Voltage in mV.
#define OIL_PRESSURE_VL 0.5
#define OIL_PRESSURE_VH 4.5
#define OIL_PRESSURE_PMAX 1034.21359 //150psi in kilopascals

#include "MedianFilterLib2.h"
MedianFilter2<float> medianFilter2(5);

#include <SPI.h>

#include <TFT_eSPI.h>       // Hardware-specific library
#include "subaru_logo.h"

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library
TFT_eSprite sprite = TFT_eSprite(&tft); //pressure display
TFT_eSprite sprite2 = TFT_eSprite(&tft); //graph
TFT_eSprite sprite3 = TFT_eSprite(&tft); //rpm

//tft display
#define color1 TFT_WHITE
#define color2 0x8410
#define color3 0x3828
#define color4 0xF2DF
#define color5 0x00A3
#define color6 0x00A3 //olive

//onboard LED
#define LED_RX 2
#define LED_OFF LOW
#define LED_ON HIGH


// setup =======================================
void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.println("ESP32-BRZ display "VERSION);

  //tft display init
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.pushImage(26, 14, 268, 132, subaru_logo); //small logo

  // CAN Bus setup
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_15;
  CAN_cfg.rx_pin_id = GPIO_NUM_13;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

  // Set CAN Filter
  // for PeliCAN Mode https://www.nxp.com/docs/en/application-note/AN97076.pdf
  CAN_filter_t p_filter;
  p_filter.FM = Single_Mode;

  p_filter.ACR0 = 0x6C; //id 0x140+360 shifted left 5 bits
  p_filter.ACR1 = 0x00;
  p_filter.ACR2 = 0;
  p_filter.ACR3 = 0;

  p_filter.AMR0 = 0x44;
  p_filter.AMR1 = 0x1F;
  p_filter.AMR2 = 0xFF;
  p_filter.AMR3 = 0xFF;
  ESP32Can.CANConfigFilter(&p_filter);
  // Init CAN Module
  ESP32Can.CANInit();

  //ADC setup
  Wire.begin(I2C_SDA, I2C_SCL);
  ADS.begin();
  ADS.setGain(0);      // 6.144 volt
  ADS.setDataRate(6);  // fast
  ADS.setMode(0);      // continuous mode
  ADS.readADC(3);      // first read to trigger - A3 input
//  ADS.readADC(2);      // first read to trigger - A2 input, just now unused
  
  //tft display
  delay(2000); //wait for logo
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(4);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString(" OIL PRESSURE kPa", 70, 10, 2);
  tft.drawString("OIL C", 190, 10, 2);
  tft.drawString("COOLANT C", 273, 10, 2);
  tft.drawString("RPM", 70, 100, 2);
  tft.drawString("V ", 138, 82, 2);
  tft.setTextColor(TFT_SILVER, color6);

  sprite.createSprite(310, 50);  //sprite pressure display
  sprite.setTextDatum(4);
  sprite.setTextColor(TFT_SILVER, color6);

  sprite2.createSprite(165, 85);  //sprite2 graph
  sprite2.setTextDatum(4);

  sprite3.createSprite(140, 50);  //sprite rpm
  sprite3.setTextDatum(4);
  sprite3.setTextColor(TFT_SILVER, color6);

}

// main loop =======================================
void loop() {
  unsigned long currentMillis = millis();
  //ADC readout
  int16_t raw = ADS.readADC(3);
//  int16_t raw1 = ADS.readADC(2);
  // Serial.println("raw");
  // Serial.println(ADS.toVoltage(raw), 4);
  // Serial.println("raw1");
  // Serial.println(ADS.toVoltage(raw1), 4);
  // Serial.println(String((float)voltage)+"V");

  voltage = ( ADS.toVoltage(raw) * (OIL_PRESSURE_R1+OIL_PRESSURE_R2) / OIL_PRESSURE_R1 ) * 0.997;  //resistor divider 0.997 calibration constant
  if (voltage > OIL_PRESSURE_VL) {
    rawPressure = (voltage - OIL_PRESSURE_VL) * OIL_PRESSURE_PMAX / (OIL_PRESSURE_VH - OIL_PRESSURE_VL);   // pressure sensor 150PSI - 0 psi outputs 0.5V, 75 psi outputs 2.5V, 150 psi outputs 4.5V
    pressure = medianFilter2.AddValue(rawPressure); //median filtering
  } else {
    pressure = 0;
    rawPressure = 0;
  }
  Serial.println("Voltage:" + String((float)voltage) + ",Pressure:" + String((float)pressure) + ",RawPressure:" + String((float)rawPressure) + ",OilTemperature:" + String((int)oilTemperature) );
  //ADC readout END

  // CAN Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.MsgID == 0x360) {
      oilTemperature = (int) rx_frame.data.u8[2] - 40;
      coolantTemperature = (int) rx_frame.data.u8[3] - 40;
      Serial.print(" - oil temp is ");
      Serial.print(oilTemperature);
      Serial.print(" - coolant temp is ");
      Serial.println(coolantTemperature);
    }
    if (rx_frame.MsgID == 0x140) {
//      rpm = ((rx_frame.data.u8[5] & 0x0F) << 8) | rx_frame.data.u8[4];
      rpm = ((rx_frame.data.u8[3] & 0x3F) << 8) | rx_frame.data.u8[2];

      Serial.print(" - rpm is ");
      Serial.println(rpm);
    }

  } //CAN receive END

  //// display loop fast (debug)
  tft.drawString("     ", 105, 82, 2);
  tft.drawString(String((float)voltage, 3), 105, 82, 2);

  //// display loop slow
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    sprite.fillRoundRect(0, 0, 140, 50, 3, color6); //oil pressure
    sprite.fillRoundRect(145, 0, 80, 50, 3, color6); //oil temp
    sprite.fillRoundRect(230, 0, 80, 50, 3, color6); //coolant temp
    sprite.setTextColor(TFT_WHITE, color6);
    if (pressure > 0) {
      sprite.drawString(String((int)round((float)pressure)), sx, sy, 6);
    } else {
      sprite.drawString("0", sx, sy, 6);
    }
    sprite.setTextColor(TFT_SILVER, color6);
    sprite.drawString(String(oilTemperature), 184, sy, 6); //oil
    sprite.drawString(String(coolantTemperature), 269, sy, 6); //coolant
    sprite.pushSprite(5, 20);

    //RPM
    sprite3.fillRoundRect(0, 0, 140, 50, 3, color6); //rpm
    sprite3.drawString(String(rpm), sx, sy, 6); //rpm
    sprite3.pushSprite(5, 110); //rpm

    //graph
    curent = map(int(pressure), 0, 1000, 0, gh);
    for (int i = 0; i < 20; i++)
      values2[i] = values[i];
    for (int i = 19; i > 0; i--)
      values[i - 1] = values2[i];
    values[19] = curent;

    sprite2.fillSprite(TFT_BLACK);
    for (int i = 1; i < 10; i++)
      sprite2.drawLine(gx + (i * gw / 10), gy, gx + (i * gw / 10), gy - gh, color2);

    for (int i = 1; i < 6; i++) {
      sprite2.drawLine(gx, gy - (i * 14), gx + gw, gy - (i * 14), color2);
      if (i == 1 || i == 3 || i == 5)
        sprite2.drawString(String((i * 7) * 25 - 25), gx - 12, gy - (i * 14));
    }
    sprite2.drawLine(gx, gy, gx, gy - gh, TFT_WHITE); //left white line
    sprite2.drawLine(gx, gy, gx + gw, gy, TFT_WHITE); //bottom line

    for (int i = 0; i < 19; i++) {
      sprite2.drawLine(gx + (i * 8), gy - values[i] - calib, gx + ((i + 1) * 9), gy - values[i + 1] - calib, TFT_RED);
      sprite2.drawLine(gx + (i * 8), gy - values[i] - 1 - calib, gx + ((i + 1) * 8), gy - values[i + 1] - 1 - calib, TFT_RED);
    }

    sprite2.pushSprite(150, 75); //graph

  } //display loop

}
