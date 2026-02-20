#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Wire.h>

#define TFT_CS     2
#define TFT_DC     4
#define TFT_RST    15

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

struct can_frame canMsg;
MCP2515 mcp2515(5);

#define CAN_TX_ID 0x036   
#define CAN_MPU_ID 0x038   
#define CAN_ACK_ID 0x037   

unsigned long lastDisplayUpdate = 0;
#define DISPLAY_REFRESH_INTERVAL 200 

typedef struct {
  float temperature = 0.0;  
  bool doorState = false;   
  bool buttonState = false; 
  float accelX = 0.0, accelY = 0.0, accelZ = 0.0; 
  float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;    
  unsigned long lastUpdate = 0; 
} SensorData;

SensorData currentData;
bool newDataAvailable = false;

void setup() {
  Serial.begin(115200);
  
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3); 
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  drawStaticUI();

  Serial.println("CAN Receiver Ready");
}

void loop() {

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    processIncomingData();
    newDataAvailable = true;
  }

  if (newDataAvailable || (millis() - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL)) {
    updateDisplay();
    lastDisplayUpdate = millis();
    newDataAvailable = false;
  }
}

void processIncomingData() {
  switch (canMsg.can_id) {
    case CAN_TX_ID:
      if (canMsg.can_dlc >= 4) {
        int16_t tempInt = (canMsg.data[0] << 8) | canMsg.data[1];
        currentData.temperature = tempInt / 100.0f;
        
        currentData.doorState = canMsg.data[2] > 0;
        
        currentData.buttonState = canMsg.data[3] > 0;
        
        Serial.println("Received primary sensor data");
      }
      break;
      
    case CAN_MPU_ID:
  
      if (canMsg.can_dlc >= 6) {
        if (canMsg.data[0] & 0x80) {
  
          currentData.gyroX = ((int16_t)(canMsg.data[0] << 8) | canMsg.data[1]) / 131.0f;
          currentData.gyroY = ((int16_t)(canMsg.data[2] << 8) | canMsg.data[3]) / 131.0f;
          currentData.gyroZ = ((int16_t)(canMsg.data[4] << 8) | canMsg.data[5]) / 131.0f;
          Serial.println("Received gyroscope data");
        } else {
        
          currentData.accelX = ((int16_t)(canMsg.data[0] << 8) | canMsg.data[1]) / 16384.0f;
          currentData.accelY = ((int16_t)(canMsg.data[2] << 8) | canMsg.data[3]) / 16384.0f;
          currentData.accelZ = ((int16_t)(canMsg.data[4] << 8) | canMsg.data[5]) / 16384.0f;
          Serial.println("Received accelerometer data");
        }
      }
      break;
  }
  
  currentData.lastUpdate = millis();
  sendAcknowledgment();
}

void sendAcknowledgment() {
  canMsg.can_id = CAN_ACK_ID;
  canMsg.can_dlc = 0; 
  if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.println("Sent ACK");
  } else {
    Serial.println("Failed to send ACK");
  }
}

void drawStaticUI() {
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 5);
  tft.print("CAN Sensor Monitor");

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 25);
  tft.print("Temperature:");

  tft.setCursor(10, 45);
  tft.print("Door Status:");

  tft.setCursor(10, 65);
  tft.print("Button Status:");

  tft.setTextColor(ST77XX_MAGENTA);
  tft.setCursor(10, 85);
  tft.print("Accel (g):");
  tft.setCursor(20, 95);
  tft.print("X:");
  tft.setCursor(20, 105);
  tft.print("Y:");
  tft.setCursor(20, 115);
  tft.print("Z:");

  tft.setCursor(10, 125);
  tft.print("Gyro (/s):");
  tft.setCursor(20, 135);
  tft.print("X:");
  tft.setCursor(20, 145);
  tft.print("Y:");
  tft.setCursor(20, 155);
  tft.print("Z:");

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 175);
  tft.print("Last Update:");
}

void updateDisplay() {

  tft.fillRect(80, 25, tft.width() - 90, 8, ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(80, 25);
  tft.print(currentData.temperature, 1);
  tft.print(" C");

  tft.fillRect(80, 45, tft.width() - 90, 8, ST77XX_BLACK);
  tft.setTextColor(currentData.doorState ? ST77XX_RED : ST77XX_GREEN);
  tft.setCursor(80, 45);
  tft.print(currentData.doorState ? "OPEN" : "CLOSED");

  tft.fillRect(80, 65, tft.width() - 90, 8, ST77XX_BLACK);
  tft.setTextColor(currentData.buttonState ? ST77XX_RED : ST77XX_GREEN);
  tft.setCursor(80, 65);
  tft.print(currentData.buttonState ? "PRESSED" : "RELEASED");

  tft.fillRect(40, 95, 60, 8, ST77XX_BLACK);
  tft.fillRect(40, 105, 60, 8, ST77XX_BLACK);
  tft.fillRect(40, 115, 60, 8, ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  
  tft.setCursor(40, 95);
  tft.print(currentData.accelX, 2);
  tft.setCursor(40, 105);
  tft.print(currentData.accelY, 2);
  tft.setCursor(40, 115);
  tft.print(currentData.accelZ, 2);

  tft.fillRect(40, 135, 60, 8, ST77XX_BLACK);
  tft.fillRect(40, 145, 60, 8, ST77XX_BLACK);
  tft.fillRect(40, 155, 60, 8, ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  
  tft.setCursor(40, 135);
  tft.print(currentData.gyroX, 1);
  tft.setCursor(40, 145);
  tft.print(currentData.gyroY, 1);
  tft.setCursor(40, 155);
  tft.print(currentData.gyroZ, 1);

  tft.fillRect(80, 175, tft.width() - 90, 8, ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  
  unsigned long secondsAgo = (millis() - currentData.lastUpdate) / 2000;
  String timeStr;
  if (currentData.lastUpdate == 0) {
    timeStr = "Never";
  } else if (secondsAgo < 60) {
    timeStr = String(secondsAgo) + "s ago";
  } else {
    timeStr = String(secondsAgo / 60) + "m ago";
  }
  
  tft.setCursor(80, 175);
  tft.print(timeStr);


  bool dataIsFresh = (millis() - currentData.lastUpdate) < 5000; 
  tft.fillCircle(tft.width() - 10, 10, 5, dataIsFresh ? ST77XX_GREEN : ST77XX_RED);
}