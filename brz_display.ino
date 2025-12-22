//BRZ display - pressure and CANBus receiver
//  ESP32-D0WDQ6
//  pressure sensor 150PSI
//   0 psi outputs 0.5V, 75 psi outputs 2.5V, 150 psi outputs 4.5V
//     pressure = (voltage - 0.461) / 4 * 1000; //in kPa


#define VERSION "1.3"

#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include "oil_pressure_predict.h" //calculated pressure value

CAN_device_t CAN_cfg; // CAN Config
CAN_frame_t rx_frame; //define frame instance

const int rx_queue_size = 10;       // Receive Queue size
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
int oilTemperature = 255;
int coolantTemperature = 255;
int rpm = 0;
static unsigned long last_time = 0;
int predPressure = 0; //calculated pressure value
int pressurePercent = 0;
float filteredPercent = 0;

unsigned long lastAlarmTime = 0;
const unsigned long alarmTimeout = 2500; // 2 sekundy v ms
    
//sprite - pressure value display
const int sx = 70; // center x
const int sy = 30; // center y

//Sprite 2 data graph
// reduced size to avoid legend overlap and save RAM
const int gw = 152; //graph width (was 165)
const int gh = 76;  //graph heigth (was 85)
const int gx = 22; //graph x
const int gy = 84; // graph y
int values[20] = {0};
int values2[20] = {0};
int calib = 0;
int curent = 0;

float rawPressure = 0;
int pressure = 0;
float voltage = 0;
const int interval = 500;     // interval for slow loop (milliseconds)

// --- HEATMAP storage ---
// We'll keep one counter per pixel in the sprite (X=RPM, Y=pressure)
// smaller array to avoid overlap with legend and reduce RAM usage
static uint16_t heatmapBins[gw][gh]; // gw x gh density bins
static bool heatmapRenderToggle = false; // render only every 2nd slow loop

//ADC
#define I2C_SDA 16
#define I2C_SCL 17

#include "ADS1X15.h"
ADS1115 ADS(0x48, &Wire);
// Resistors value in kOhm for the voltage divider.
#define OIL_PRESSURE_R1 47.0
#define OIL_PRESSURE_R2 22.16
//static float voltage_divider = (22.16/47)+1;  //1.470213; //voltage divider 22.16/47
// Characteristics of the oil pressure sensor.
// Voltage in mV.
#define OIL_PRESSURE_VL 0.5
#define OIL_PRESSURE_VH 4.5
#define OIL_PRESSURE_PMAX 1034.21359 //150psi in kilopascals

#define MIN_PRESSURE_PERCENT 30  // 30 %
#define ALARM_BLINK_INTERVAL 300   // ms, blink interval for alarm icon
static bool alarmActive = false;
static unsigned long alarmLastBlink = 0;
static bool alarmIconVisible = false;

#include "MedianFilterLib2.h"
MedianFilter2<float> medianFilter2(5);
MedianFilter2<float> medianPercent(5);

#include <SPI.h>

#include <TFT_eSPI.h>       // Hardware-specific library
#include "subaru_logo.h"
#include "warning_icon2.h"

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library
TFT_eSprite sprite = TFT_eSprite(&tft); //pressure display  310x50
TFT_eSprite sprite2 = TFT_eSprite(&tft); //graph gw x gh
TFT_eSprite sprite3 = TFT_eSprite(&tft); //rpm 140x50

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


// helper: 'hot' colormap mapping (like MATLAB hot)
// shifted so lowest non-zero density isn't pure black
uint16_t heatmapColorFromNormalized(float v) {
  // shift the colormap upward so the lowest non-zero density isn't black
  const float vmin = 0.12f; // minimal visual intensity for v>0 (tuneable: 0.0..0.3)
  if (v <= 0.0f) return tft.color565(6, 6, 6); // very dark gray background instead of pure black
  if (v >= 1.0f) return tft.color565(255,255,255);
  // map v from [0..1] onto [vmin..1]
  float v2 = v * (1.0f - vmin) + vmin;
  // 'hot' style on shifted value: r = clamp(3*v2), g = clamp(3*v2-1), b = clamp(3*v2-2)
  float r_f = min(1.0f, 3.0f * v2);
  float g_f = min(1.0f, max(0.0f, 3.0f * v2 - 1.0f));
  float b_f = min(1.0f, max(0.0f, 3.0f * v2 - 2.0f));
  uint8_t r = (uint8_t)(r_f * 255.0f);
  uint8_t g = (uint8_t)(g_f * 255.0f);
  uint8_t b = (uint8_t)(b_f * 255.0f);
  return tft.color565(r, g, b);
}

// setup =======================================
void setup() {
  // backlight pin setup
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, !TFT_BACKLIGHT_ON); // backlight off

  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.println("ESP32-BRZ display "VERSION);
  
  //tft display init
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  //  tft.pushImage(26, 14, 268, 132, subaru_logo); //small logo



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
  ADS.setGain(1);      //  0 = 6.144V, 2 = 2.048V
  ADS.setDataRate(4);  // 128 SPS
  ADS.setMode(0);      // continuous mode
  ADS.readADC(3);      // first read to trigger - A3 input
//  ADS.readADC(2);      // first read to trigger - A2 input, just now unused
  
  //tft display
 // delay(2000); //wait for logo
 //  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(4);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString(" OIL PRESSURE kPa", 70, 10, 2);
  tft.drawString("OIL C", 190, 10, 2);
  tft.drawString("COOLANT C", 273, 10, 2);
  tft.drawString("RPM", 105, 100, 2);
  tft.drawString("V ", 138, 82, 2);
  tft.setTextColor(TFT_SILVER, color6);

  sprite.createSprite(310, 50);  //sprite pressure display
  sprite.setTextDatum(4);
  sprite.setTextColor(TFT_SILVER, color6);

  sprite2.createSprite(gw, gh);  //sprite2 graph (kept same size)
  sprite2.setTextDatum(4);

  sprite3.createSprite(140, 50);  //sprite rpm
  sprite3.setTextDatum(4);
  sprite3.setTextColor(TFT_SILVER, color6);

//  tft.pushImage(5, 74, 32, 32, warning_icon); //small logo
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // backlight on

  // initialize heatmap bins to zero
  memset(heatmapBins, 0, sizeof(heatmapBins));
}

// main loop =======================================
void loop() {
  unsigned long currentMillis = millis();

  //ADC readout
  int16_t raw = ADS.readADC(3);
  voltage = ( ADS.toVoltage(raw) * (OIL_PRESSURE_R1+OIL_PRESSURE_R2) / OIL_PRESSURE_R1 ) * 0.997;  //resistor divider 0.997 calibration constant

  if (voltage > OIL_PRESSURE_VL) {
    rawPressure = (voltage - OIL_PRESSURE_VL) * OIL_PRESSURE_PMAX / (OIL_PRESSURE_VH - OIL_PRESSURE_VL);  // pressure sensor 150PSI - 0 psi outputs 0.5V, 75 psi outputs 2.5V, 150 psi outputs 4.5V
    pressure = medianFilter2.AddValue(rawPressure);  //median filtering
  } else {
    pressure = 0;
    rawPressure = 0;
  }
  //ADC readout END

  // CAN Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.MsgID == 0x360) {
      oilTemperature = (int) rx_frame.data.u8[2] - 40;
      coolantTemperature = (int) rx_frame.data.u8[3] - 40;
    }
    if (rx_frame.MsgID == 0x140) {
      rpm = ((rx_frame.data.u8[3] & 0x3F) << 8) | rx_frame.data.u8[2];
    }

  } 
  //CAN receive END

  // oil pressure calculation
  predPressure = expectedOilPressure(rpm);
  if (predPressure > 0) {
    pressurePercent = (pressure * 100) / predPressure;
  } else {
    pressurePercent = 0;
  }
  filteredPercent = medianPercent.AddValue(pressurePercent);
  
  //serial output
  //  Serial.println("Voltage:" + String((float)voltage) + ",Pressure:" + String((float)pressure) + ",RawPressure:" + String((float)rawPressure) + ",OilTemperature:" + String((int)oilTemperature) );
  Serial.println("RPM:" + String((float)rpm) + ",Pressure:" + String((float)pressure) + ",RawPressure:" + String((float)rawPressure) + ",OilTemperature:" + String((int)oilTemperature) );
  Serial.println("predPressure:" + String(predPressure) );

  //// display loop fast (debug)
  tft.drawString("     ", 105, 82, 2);
  tft.drawString(String((float)voltage, 3), 105, 82, 2);
  
  // ALARM HANDLING LOGIC WITH BLINKING WARNING ICON (ONLY ICON)
  if (pressure < predPressure * (100 - MIN_PRESSURE_PERCENT) / 100 && rpm > 0) {
    lastAlarmTime = currentMillis;
  }

  // Show alarm icon as blinking for 2 seconds after last alarm trigger
  if ((currentMillis - lastAlarmTime) < alarmTimeout) {
    if (currentMillis - alarmLastBlink > ALARM_BLINK_INTERVAL) {
      alarmLastBlink = currentMillis;
      alarmIconVisible = !alarmIconVisible;
      tft.fillRect(15, 74, 32, 32, TFT_BLACK); // clear previous icon area
      if (alarmIconVisible) {
        tft.pushImage(15, 74, 32, 32, warning_icon);
      }
    }
  } else {
    if (alarmIconVisible) {
      alarmIconVisible = false;
      tft.fillRect(15, 74, 32, 32, TFT_BLACK);
    }
  }
  // ALARM end
  
  //// display loop slow
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    tft.drawString("     ", 57, 88, 4);
//    tft.drawString(String(pressurePercent), 57, 67, 4); //display pressure Percent
//    tft.drawString(String((int)round(filteredPercent)) + " %", 57, 87, 4);
    tft.drawString(String((int)round(filteredPercent)), 57, 88, 4);

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

    //graph -> REPLACED BY HEATMAP (X=RPM, Y=pressure)

    // accumulate histogram for current sample (RPM vs pressure)
    // X axis: rpm 600 .. 7600  -> 0 .. (gw-1)
    // Y axis: pressure 0 .. OIL_PRESSURE_PMAX -> bottom..top (low pressure at bottom)
    int x = map(rpm, 600, 7600, 0, gw - 1);
    x = constrain(x, 0, gw - 1);
    int y = map((int)pressure, 0, (int)OIL_PRESSURE_PMAX, gh - 1, 0);
    y = constrain(y, 0, gh - 1);
    // increment bin with cap
    if (heatmapBins[x][y] < 65530) heatmapBins[x][y]++;

    // toggle render every second slow loop
    heatmapRenderToggle = !heatmapRenderToggle;
    if (heatmapRenderToggle) {
      // Render heatmap
      // find maximum bin count for normalization
      uint16_t maxCount = 0;
      for (int xi = 0; xi < gw; xi++) {
        for (int yi = 0; yi < gh; yi++) {
          if (heatmapBins[xi][yi] > maxCount) maxCount = heatmapBins[xi][yi];
        }
      }

      // use a very dark gray background so low-values' colors don't blend into pure black
      uint16_t heat_bg = tft.color565(6,6,6);
      sprite2.fillSprite(TFT_BLACK);

      // draw heatmap using row buffer + tft.pushImage (faster than drawPixel loops)
      static uint16_t rowBuf[gw];
      for (int yi = 0; yi < gh; yi++) {
        for (int xi = 0; xi < gw; xi++) {
          uint16_t cnt = heatmapBins[xi][yi];
          if (cnt == 0) rowBuf[xi] = heat_bg;
          else {
            float norm = (float)cnt / (float)maxCount;
            rowBuf[xi] = heatmapColorFromNormalized(norm);
          }
        }
        // push single row to absolute screen position (150,75 is used in current code)
        tft.pushImage(163, 84 + yi, gw, 1, rowBuf);
      }

      // draw coarser grid/labels over the heatmap (absolute coordinates)
      // vertical: 4 internal divisions (5 columns including edges)
      for (int i = 1; i < 5; i++) {
     //   tft.drawLine(150 + (i * gw / 5), 75, 150 + (i * gw / 5), 75 + gh - 1, color2);
      }
      // horizontal: 3 internal divisions (4 rows including edges)
      for (int i = 1; i < 4; i++) {
        int y_local = 75 + (i * gh / 4);
      //  tft.drawLine(150, y_local, 150 + gw - 1, y_local, color2);
        // put sparse labels (left side), consistent with reduced grid density
        if (i == 1 || i == 2 || i == 3) {
          float labelP = (float)(gh - 1 - (i * gh / 4)) / (float)(gh - 1) * OIL_PRESSURE_PMAX;
          tft.setTextColor(TFT_SILVER, TFT_BLACK);
          tft.drawString(String((int)labelP), 152, y_local);
        }
      }

    }

    // --- End heatmap rendering ---

  } 
  //display slow loop

}
