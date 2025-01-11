#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <Wire.h>
#include <MPU6050.h>
#include <NewPing.h>
#include <VL53L0X.h>
#include <math.h>

// WiFi credentials
#define WIFI_SSID "PERRO"
#define WIFI_PASSWORD "Jr123456789"

// Firebase credentials
#define API_KEY "AIzaSyCbNMA6WmF0pwl20wVCIGtKtE-wkrJXk5A"
#define DATABASE_URL "https://algoritmos-1485d-default-rtdb.firebaseio.com"
#define USER_EMAIL "esp32_070216@gmail.com"
#define USER_PASSWORD "123456abc"

// Pin definitions
#define TRIG_PIN_FRONT 13
#define ECHO_PIN_FRONT 12
#define TRIG_PIN_RIGHT 14
#define ECHO_PIN_RIGHT 27
#define TRIG_PIN_LEFT 26
#define ECHO_PIN_LEFT 25
#define TRIG_PIN_BACK 33
#define ECHO_PIN_BACK 32
#define MOTOR_LEFT_1 4
#define MOTOR_LEFT_2 5
#define MOTOR_RIGHT_1 18
#define MOTOR_RIGHT_2 19
#define TRANS_PULSE 2
#define TRANS_CONSTANT 0
#define BATTERY_PIN 34 // Adjusted for ADC pin

// Battery configuration
#define MAX_BATTERY_VOLTAGE 12.6
#define MIN_BATTERY_VOLTAGE 10.6

// Voltage divider constants
const float R1 = 30000.0;
const float R2 = 10000.0;
const float factorDivisor = (R1 + R2) / R2;
const float Vref = 3.3;
const float calibracion = 1;//.3111111222;

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Sensor objects
MPU6050 mpu;
VL53L0X laser;
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, 400);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, 400);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, 400);
NewPing sonarBack(TRIG_PIN_BACK, ECHO_PIN_BACK, 400);

// Global variables
float batteryLevel = 100.0;
unsigned long operatingTime = 0;
bool isOperating = false;
bool isMapping = false;
bool isDefiningArea = false;
bool isReturningToBase = false;
float startX = 0, startY = 0;
float currentX = 0, currentY = 0;
float currentAngle = 0;
unsigned long lastCommandTime = 0;
unsigned long lastPulseTime = 0;
unsigned long lastUpdateTime = 0;
float workArea[4][2];
int workAreaPoints = 0;

const unsigned long UPDATE_INTERVAL = 5000; // Update every 5 seconds

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  fbdo.setBSSLBufferSize(4096, 1024);
  Firebase.setDoubleDigits(5);

  // Initialize sensors
  Wire.begin();
  mpu.initialize();
  laser.init();
  laser.setTimeout(500);

  // Configure pins
  pinMode(MOTOR_LEFT_1, OUTPUT);
  pinMode(MOTOR_LEFT_2, OUTPUT);
  pinMode(MOTOR_RIGHT_1, OUTPUT);
  pinMode(MOTOR_RIGHT_2, OUTPUT);
  pinMode(TRANS_PULSE, OUTPUT);
  pinMode(TRANS_CONSTANT, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);

  digitalWrite(TRANS_CONSTANT, HIGH);
  Serial.println("Setup complete");
}

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    WiFi.disconnect();
    delay(1000); // Espera 1 segundo antes de intentar reconectar
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void loop() {
  reconnectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    if (Firebase.ready()) {
      unsigned long currentMillis = millis();

      updateBatteryAndTime();
      checkBatteryLevel();
      checkCommands();

      if (isOperating) {
        if (isMapping) {
          mapEnvironment();
        } else if (isDefiningArea) {
          defineWorkArea();
        } else {
          navigateWorkArea();
        }
        sendPulseSignal();
      }

      if (isReturningToBase) {
        returnToBase();
      }

      // Update Firebase periodically, even when not re
      if (currentMillis - lastUpdateTime >= UPDATE_INTERVAL) {
        updateFirebase();
        lastUpdateTime = currentMillis;
      }
    } else {
      Serial.println("Firebase not ready");
    }
  } else {
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  delay(100);
}

float getBatteryPercentage() {
  int adcValue = analogRead(BATTERY_PIN);
  float Vout = (adcValue / 4095.0) * Vref;
  float Vin = Vout * factorDivisor * calibracion;

  float percentage = ((Vin - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0;
  return constrain(percentage, 0.0, 100.0);
}

float getBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  float Vout = (adcValue / 4095.0) * Vref;
  float Vin = Vout * factorDivisor * calibracion;
  return Vin;
}

void updateBatteryAndTime() {
  batteryLevel = getBatteryPercentage();
  float batteryVoltage = getBatteryVoltage();

  if (Firebase.ready()) {
    Firebase.setFloat(fbdo, F("/robot/battery/percentage"), batteryLevel);
    Firebase.setFloat(fbdo, F("/robot/battery/voltage"), batteryVoltage);
  }

  if (isOperating) {
    operatingTime += 100;
  }
}

void checkBatteryLevel() {
  if (batteryLevel <= 10.0) {
    isOperating = false;
    isMapping = false;
    isDefiningArea = false;
    isReturningToBase = false;
    stopMotors();
    Serial.println("Low battery. Robot stopped.");
    Firebase.setString(fbdo, F("/robot/estado"), F("Stopped due to low battery"));
    Firebase.setString(fbdo, F("/robot/comando"), F("null"));
  }
}


void checkCommands() {
  if (Firebase.getString(fbdo, F("/robot/comando"))) {
    String command = fbdo.stringData();
    lastCommandTime = millis();
    
    if (command == "start") startOperation();
    else if (command == "start_mapping") startMapping();
    else if (command == "define_area") startDefiningArea();
    else if (command == "return") startReturningToBase();
    
  }
}

void startOperation() {
  isOperating = true;
  isMapping = false;
  isDefiningArea = false;
  isReturningToBase = false;
  Serial.println("Starting operation in established area");
}

void startMapping() {
  isOperating = true;
  isMapping = true;
  isDefiningArea = false;
  isReturningToBase = false;
  startX = currentX;
  startY = currentY;
  Serial.println("Starting mapping");
}

void startDefiningArea() {
  isOperating = true;
  isMapping = false;
  isDefiningArea = true;
  isReturningToBase = false;
  workAreaPoints = 0;
  Serial.println("Starting area definition");
}

void startReturningToBase() {
  isOperating = false;
  isMapping = false;
  isDefiningArea = false;
  isReturningToBase = true;
  Serial.println("Returning to base");
}

void addObstacleToMap(float x, float y) {
  if (Firebase.ready()) {
    Firebase.setString(fbdo, F("/robot/mapa"), F("X"));
}
}
/*void addObstacleToMap(float x, float y) {
  String mapPath = "/robot/mapa/" + String(random(1000000)); // Generate unique key
  if (Firebase.ready()) {
    Firebase.setFloat(fbdo, mapPath + "/x", x);
    Firebase.setFloat(fbdo, mapPath + "/y", y);
    Firebase.setString(fbdo, mapPath + "/type", "X");
  }
}*/
void updatePosition() {
  float distance = 0.1; // Simulated encoder
  currentX += distance * cos(currentAngle * M_PI/180);
  currentY += distance * sin(currentAngle * M_PI/180);
}
void mapEnvironment() {
  int frontDistance = sonarFront.ping_cm();
  int rightDistance = sonarRight.ping_cm();
  int leftDistance = sonarLeft.ping_cm();
  int backDistance = sonarBack.ping_cm();
  int laserDistance = laser.readRangeSingleMillimeters() / 10;
  
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  currentAngle = atan2(ay, ax) * 180/M_PI;
  
  updatePosition();
  
  // Check for obstacles at 30cm and update map
  if (frontDistance <= 30 || rightDistance <= 30 || leftDistance <= 30 || backDistance <= 30 || laserDistance <= 30) {
    // Calculate obstacle position based on sensor readings and current position
    float obstacleX = currentX;
    float obstacleY = currentY;
    
    // Adjust obstacle position based on which sensor detected it
    if (frontDistance <= 30) {
      obstacleX += 30 * cos(currentAngle * M_PI/180);
      obstacleY += 30 * sin(currentAngle * M_PI/180);
    }
    else if (rightDistance <= 30) {
      obstacleX += 30 * cos((currentAngle + 90) * M_PI/180);
      obstacleY += 30 * sin((currentAngle + 90) * M_PI/180);
    }
    else if (leftDistance <= 30) {
      obstacleX += 30 * cos((currentAngle - 90) * M_PI/180);
      obstacleY += 30 * sin((currentAngle - 90) * M_PI/180);
    }
    else if (backDistance <= 30) {
      obstacleX += 30 * cos((currentAngle + 180) * M_PI/180);
      obstacleY += 30 * sin((currentAngle + 180) * M_PI/180);
    }
    
    addObstacleToMap(obstacleX, obstacleY);
  }
  
  navigate(frontDistance, rightDistance, leftDistance, backDistance, laserDistance);
}

void navigate(int front, int right, int left, int back, int laser) {
  if (front <= 30 || right <= 30 || left <= 30 || back <= 30 || laser <= 30) {
    avoidObstacle(front, right, left, back, laser);
  } else {
    moveForward();
  }
}

void avoidObstacle(int front, int right, int left, int back, int laser) {
  stopMotors();
  delay(500); // Brief pause to ensure complete stop
  
  if (left > right && left > front) {
    // Turn left if there's more space to the left
    turn(-90);
    moveForward();
    delay(1000);
  } 
  else if (right > left && right > front) {
    // Turn right if there's more space to the right
    turn(90);
    moveForward();
    delay(1000);
  } 
  else {
    // If no clear path, turn around
    turn(180);
    moveForward();
    delay(1000);
  }
}

void turn(float angle) {
  float targetAngle = currentAngle + angle;
  while (abs(currentAngle - targetAngle) > 1) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    currentAngle = atan2(ay, ax) * 180/M_PI;
    
    if (currentAngle < targetAngle) {
      turnRight();
    } else {
      turnLeft();
    }
  }
  stopMotors();
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_1, HIGH);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, HIGH);
  digitalWrite(MOTOR_RIGHT_2, LOW);
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_1, HIGH);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, LOW);
  digitalWrite(MOTOR_RIGHT_2, HIGH);
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_1, LOW);
  digitalWrite(MOTOR_LEFT_2, HIGH);
  digitalWrite(MOTOR_RIGHT_1, HIGH);
  digitalWrite(MOTOR_RIGHT_2, LOW);
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_1, LOW);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, LOW);
  digitalWrite(MOTOR_RIGHT_2, LOW);
}

void defineWorkArea() {
  if (workAreaPoints < 4) {
    workArea[workAreaPoints][0] = currentX;
    workArea[workAreaPoints][1] = currentY;
    workAreaPoints++;
    Serial.println("Area point defined: " + String(workAreaPoints));
  }
  if (workAreaPoints == 4) {
    isDefiningArea = false;
    isOperating = true;
    Serial.println("Area fully defined");
  }
}

void navigateWorkArea() {
  moveForward();
}

void returnToBase() {
  float distanceToBase = sqrt(pow(currentX - startX, 2) + pow(currentY - startY, 2));
  if (distanceToBase > 0.1) {
    float angleToBase = atan2(startY - currentY, startX - currentX) * 180 / M_PI;
    turn(angleToBase - currentAngle);
    moveForward();
  } else {
    stopMotors();
    isReturningToBase = false;
    Serial.println("Arrived at base");
  }
}

void sendPulseSignal() {
  if (millis() - lastPulseTime >= 30000) {
    digitalWrite(TRANS_PULSE, HIGH);
    delay(1000);
    digitalWrite(TRANS_PULSE, LOW);
    lastPulseTime = millis();
  }
}

void updateFirebase() {
  if (Firebase.ready()) {
    Firebase.setFloat(fbdo, F("/robot/bateria"), batteryLevel);
    Firebase.setInt(fbdo, F("/robot/tiempo"), operatingTime / 60000);
    Firebase.setString(fbdo, F("/robot/estado"), getStatusString());
    Firebase.setFloat(fbdo, F("/robot/posicionX"), currentX);
    Firebase.setFloat(fbdo, F("/robot/posicionY"), currentY);
    
    // Additional status information
    Firebase.setBool(fbdo, F("/robot/conectado"), true);
    Firebase.setInt(fbdo, F("/robot/rssi"), WiFi.RSSI());
    Firebase.setString(fbdo, F("/robot/ip"), WiFi.localIP().toString());
  }
}

String getStatusString() {
  if (isOperating) {
    if (isMapping) return "Mapping";
    if (isDefiningArea) return "Defining area";
    return "Operating";
  }
  if (isReturningToBase) return "Returning to base";
  return "Inactive";
}

