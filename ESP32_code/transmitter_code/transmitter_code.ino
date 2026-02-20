#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ONE_WIRE_BUS 14
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


struct can_frame canMsg;
struct MCP2515 mcp2515(5); 

#define MAX_RETRIES 3
#define CAN_TX_ID 0x036
#define CAN_ACK_ID 0x037
#define CAN_MPU_ID 0x038 

#define DOOR_SENSOR_PIN 34
bool lastDoorState = false;
unsigned long lastDoorDebounceTime = 0;

#define BUTTON_PIN 12
bool lastButtonState = false;
unsigned long lastButtonDebounceTime = 0;

#define RED_PIN 25
#define GREEN_PIN 33
#define BLUE_PIN 32

MPU6050 mpu;
bool mpuInitialized = false;
const int mpuUpdateInterval = 100;
unsigned long lastMpuUpdate = 0;


typedef struct {
  int16_t temperature;
  uint8_t doorState;
  uint8_t buttonState;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
} SensorData;

void setup() {
  Serial.begin(115200);
  SPI.begin();

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  sensors.begin();

  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  setRGBColor(0, 0, 255);

  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected");
    mpuInitialized = true;
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    mpu.setXAccelOffset(-1100);
    mpu.setYAccelOffset(500);
    mpu.setZAccelOffset(1500);
    mpu.setXGyroOffset(50);
    mpu.setYGyroOffset(30);
    mpu.setZGyroOffset(20);
  } else {
    Serial.println("MPU6050 failed");
  }

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Setup complete");
  display.display();
  delay(1000);
  setRGBColor(0, 255, 0);
}

void loop() {
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  
  SensorData sensorData = readSensors();
  
  if ((currentTime - lastSendTime >= 1000) || 
      (sensorData.doorState != lastDoorState) || 
      (sensorData.buttonState != lastButtonState)) {
    
    lastDoorState = sensorData.doorState;
    lastButtonState = sensorData.buttonState;
    
    bool success = sendSensorData(sensorData);
    if (mpuInitialized) {
      success &= sendMpuData(sensorData);
    }
    
    if (success) {
      setRGBColor(0, 255, 0);
      lastSendTime = currentTime;
    } else {
      setRGBColor(255, 0, 0);
    }
    
    updateDisplay(sensorData, success);
  }
  
  delay(10);
}

SensorData readSensors() {
  SensorData data;
  
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC != DEVICE_DISCONNECTED_C) {
    data.temperature = (int16_t)(tempC * 100);
  } else {
    data.temperature = -1000; // Error value
  }
  
  bool doorReading = digitalRead(DOOR_SENSOR_PIN);
  if (doorReading != lastDoorState) {
    lastDoorDebounceTime = millis();
  }
  if ((millis() - lastDoorDebounceTime) > 50) {
    data.doorState = doorReading ? 0 : 1;
  } else {
    data.doorState = lastDoorState;
  }
  
  bool buttonReading = digitalRead(BUTTON_PIN);
  if (buttonReading != lastButtonState) {
    lastButtonDebounceTime = millis();
  }
  if ((millis() - lastButtonDebounceTime) > 50) {
    data.buttonState = buttonReading ? 0 : 1;
  } else {
    data.buttonState = lastButtonState;
  }
  
  if (mpuInitialized && (millis() - lastMpuUpdate >= mpuUpdateInterval)) {
    lastMpuUpdate = millis();
    
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    data.accelX = ax;
    data.accelY = ay;
    data.accelZ = az;
    data.gyroX = gx;
    data.gyroY = gy;
    data.gyroZ = gz;
  }
  
  return data;
}

bool sendSensorData(SensorData data) {
  canMsg.can_id = CAN_TX_ID;
  canMsg.can_dlc = 4;
  
  canMsg.data[0] = (data.temperature >> 8) & 0xFF;
  canMsg.data[1] = data.temperature & 0xFF;
  canMsg.data[2] = data.doorState;
  canMsg.data[3] = data.buttonState;
  
  return sendCanMessageWithRetry();
}

bool sendMpuData(SensorData data) {
  canMsg.can_id = CAN_MPU_ID;
  canMsg.can_dlc = 6;
  canMsg.data[0] = (data.accelX >> 8) & 0xFF;
  canMsg.data[1] = data.accelX & 0xFF;
  canMsg.data[2] = (data.accelY >> 8) & 0xFF;
  canMsg.data[3] = data.accelY & 0xFF;
  canMsg.data[4] = (data.accelZ >> 8) & 0xFF;
  canMsg.data[5] = data.accelZ & 0xFF;
  
  bool success = sendCanMessageWithRetry();
  
  canMsg.data[0] = (data.gyroX >> 8) & 0xFF;
  canMsg.data[1] = data.gyroX & 0xFF;
  canMsg.data[2] = (data.gyroY >> 8) & 0xFF;
  canMsg.data[3] = data.gyroY & 0xFF;
  canMsg.data[4] = (data.gyroZ >> 8) & 0xFF;
  canMsg.data[5] = data.gyroZ & 0xFF;
  
  return success && sendCanMessageWithRetry();
}

bool sendCanMessageWithRetry() {
  int retries = 0;
  while (retries < MAX_RETRIES) {
    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
      unsigned long startTime = millis();
      while (millis() - startTime < 500) {
        if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
          if (canMsg.can_id == CAN_ACK_ID) {
            return true;
          }
        }
      }
    }
    retries++;
  }
  return false;
}

void updateDisplay(SensorData data, bool commSuccess) {
  display.clearDisplay();
  display.setCursor(0,0);

  display.print("Temp: ");
  if (data.temperature != -1000) {
    display.print(data.temperature / 100.0, 2);
    display.println(" C");
  } else {
    display.println("Error");
  }
  
  display.print("Door: ");
  display.println(data.doorState ? "OPEN" : "CLOSED");
  display.print("Button: ");
  display.println(data.buttonState ? "PRESSED" : "RELEASED");
  

  if (mpuInitialized) {
    display.print("AccX: ");
    display.print(data.accelX / 16384.0, 2);
    display.print(" g");
    display.setCursor(0, 40);
    display.print("GyroZ: ");
    display.print(data.gyroZ / 131.0, 1);
    display.print(" /s");
  } else {
    display.println("MPU: Not connected");
  }
  
  display.setCursor(0, 56);
  display.print("CAN: ");
  display.println(commSuccess ? "OK" : "FAIL");
  
  display.display();
}

void setRGBColor(uint8_t red, uint8_t green, uint8_t blue) {
  analogWrite(RED_PIN, 255 - red);
  analogWrite(GREEN_PIN, 255 - green);
  analogWrite(BLUE_PIN, 255 - blue);
}
