#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// =====================================================
// PYNQ / FPGA UART
// UART2: GPIO16 = RX2, GPIO17 = TX2
// =====================================================
HardwareSerial FPgaSerial(2);

// =====================================================
// LCD
// =====================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// SENSOR PINS
// =====================================================
const int DS18B20_PIN = 19;
const int ACS712_PIN  = 35;
const int VOLTAGE_PIN = 32;

// =====================================================
// FAULT POTENTIOMETERS
// =====================================================
const int POT_VOLT = 33;
const int POT_CURR = 36;  // VP, input-only
const int POT_TEMP = 39;  // VN, input-only

// =====================================================
// OUTPUTS
// =====================================================
const int RELAY_PIN  = 26;  // relay remains controlled by ESP32 for now
const int BUZZER_PIN = 25;
const int LED_GREEN  = 2;
const int LED_YELLOW = 4;
const int LED_RED    = 15;

// =====================================================
// BUTTONS
// =====================================================
const int BTN_START = 27;
const int BTN_RESET = 14;
const int BTN_TEST  = 13;
const int BTN_MENU  = 23;

// =====================================================
// DS18B20
// =====================================================
OneWire oneWire(DS18B20_PIN);
DallasTemperature temperatureSensor(&oneWire);

// =====================================================
// ACS712 CALIBRATION
// =====================================================
const float ACS_ZERO_MV = 1749.5;       // measured zero-current value
const float ACS_SENSITIVITY = 185.0;    // mV/A for ACS712-5A
const float ACS_DIVIDER_FACTOR = 1.5;   // your external divider correction

// =====================================================
// SAFETY THRESHOLDS
// =====================================================
const float MAX_VOLTAGE     = 5.5;
const float MAX_CURRENT     = 7.5;
const float TEMP_WARNING    = 32.0;
const float MAX_TEMPERATURE = 40.0;
const int FAULT_POT_LIMIT   = 90;

// =====================================================
// SYSTEM VARIABLES
// =====================================================
bool systemRunning = false;
bool faultLatched  = false;
bool manualFault   = false;
bool tempSensorFault = false;

unsigned long lastSample = 0;
unsigned long lastLCD    = 0;
unsigned long lastUART   = 0;

float voltage     = 0.0;
float current     = 0.0;
float temperature = 0.0;
float power       = 0.0;

int potV = 0;
int potI = 0;
int potT = 0;

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);

  // Only ONE UART2 object is used for the PYNQ link.
  FPgaSerial.begin(115200, SERIAL_8N1, 16, 17);

  analogReadResolution(12);

  analogSetPinAttenuation(ACS712_PIN, ADC_11db);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
  analogSetPinAttenuation(POT_VOLT, ADC_11db);
  analogSetPinAttenuation(POT_CURR, ADC_11db);
  analogSetPinAttenuation(POT_TEMP, ADC_11db);

  temperatureSensor.begin();
  temperatureSensor.setResolution(10);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_TEST, INPUT_PULLUP);
  pinMode(BTN_MENU, INPUT_PULLUP);

  // Active-LOW relay: HIGH = OFF at startup.
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SAFETY SYSTEM");
  lcd.setCursor(0, 1);
  lcd.print("INITIALIZING");
  delay(2000);

  Serial.println();
  Serial.println("==============================");
  Serial.println(" ADAPTER SAFETY CONTROLLER");
  Serial.println("==============================");
  Serial.println("DS18B20 INITIALIZED");
  Serial.println("FPGA UART2 INITIALIZED");
  Serial.println("TX2 = GPIO17");
  Serial.println("RX2 = GPIO16");
}

// =====================================================
// READ SENSORS
// =====================================================
void readSensors() {

  // DS18B20
  temperatureSensor.requestTemperatures();
  float dsTemperature = temperatureSensor.getTempCByIndex(0);

  if (dsTemperature == DEVICE_DISCONNECTED_C) {
    tempSensorFault = true;
    temperature = -127.0;
  } else {
    tempSensorFault = false;
    temperature = dsTemperature;
  }

  // 0-25V voltage sensor, approximately 5:1.
  float sensorVoltage = analogReadMilliVolts(VOLTAGE_PIN) / 1000.0;
  voltage = sensorVoltage * 5.0;

  // ACS712 current measurement.
  float acs_mV = analogReadMilliVolts(ACS712_PIN);
  float difference = acs_mV - ACS_ZERO_MV;
  float sensorDifference = difference * ACS_DIVIDER_FACTOR;
  current = sensorDifference / ACS_SENSITIVITY;
  current = fabs(current);

  power = voltage * current;

  potV = map(analogRead(POT_VOLT), 0, 4095, 0, 100);
  potI = map(analogRead(POT_CURR), 0, 4095, 0, 100);
  potT = map(analogRead(POT_TEMP), 0, 4095, 0, 100);
}

// =====================================================
// SAFETY CHECK
// =====================================================
void safetyCheck() {

  bool voltageFault     = voltage > MAX_VOLTAGE;
  bool currentFault     = current > MAX_CURRENT;
  bool temperatureFault = temperature > MAX_TEMPERATURE;
  bool sensorFault      = tempSensorFault;

  bool potVoltageFault     = potV >= FAULT_POT_LIMIT;
  bool potCurrentFault     = potI >= FAULT_POT_LIMIT;
  bool potTemperatureFault = potT >= FAULT_POT_LIMIT;

  bool anyFault = voltageFault ||
                  currentFault ||
                  temperatureFault ||
                  sensorFault ||
                  potVoltageFault ||
                  potCurrentFault ||
                  potTemperatureFault ||
                  manualFault;

  if (anyFault) {
    faultLatched = true;
  }
}

// =====================================================
// OUTPUT CONTROL
// =====================================================
void updateOutputs() {

  if (faultLatched) {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    return;
  }

  if (!systemRunning) {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  bool warning = temperature >= TEMP_WARNING ||
                 potV >= 70 ||
                 potI >= 70 ||
                 potT >= 70;

  if (warning) {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

// =====================================================
// LCD
// =====================================================
void updateLCD() {

  if (millis() - lastLCD < 2000)
    return;

  lastLCD = millis();
  lcd.clear();

  if (tempSensorFault) {
    lcd.setCursor(0, 0);
    lcd.print("TEMP SENSOR ERR");
    lcd.setCursor(0, 1);
    lcd.print("RELAY CUT OFF");
    return;
  }

  if (faultLatched) {
    lcd.setCursor(0, 0);
    lcd.print("!! FAULT !!");
    lcd.setCursor(0, 1);
    lcd.print("RELAY CUT OFF");
    return;
  }

  if (!systemRunning) {
    lcd.setCursor(0, 0);
    lcd.print("SYSTEM READY");
    lcd.setCursor(0, 1);
    lcd.print("Press START");
    return;
  }

  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(voltage, 2);
  lcd.print(" I:");
  lcd.print(current, 2);

  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print(" P:");
  lcd.print(power, 1);
}

// =====================================================
// BUTTONS
// =====================================================
void checkButtons() {

  if (digitalRead(BTN_START) == LOW) {
    systemRunning = true;
    Serial.println("SYSTEM STARTED");
    delay(300);
  }

  if (digitalRead(BTN_RESET) == LOW) {
    faultLatched = false;
    manualFault = false;
    Serial.println("FAULT RESET");
    delay(300);
  }

  if (digitalRead(BTN_TEST) == LOW) {
    manualFault = true;
    Serial.println("MANUAL FAULT TEST");
    delay(300);
  }

  if (digitalRead(BTN_MENU) == LOW) {
    Serial.println("MENU BUTTON");
    delay(300);
  }
}

// =====================================================
// ESP32 -> FPGA LIVE PACKET
// Format expected by FPGA:
// V=millivolts,I=mA,T=centi-degC,F=fault,S=sensor_ok
// =====================================================
void sendToPYNQ() {

  if (millis() - lastUART < 500)
    return;

  lastUART = millis();

  int voltage_mV = (int)(voltage * 1000.0);
  int current_mA = (int)(current * 1000.0);
  int temperature_centi = (int)(temperature * 100.0);

  // Live packet to FPGA UART2.
  FPgaSerial.print("V=");
  FPgaSerial.print(voltage_mV);
  FPgaSerial.print(",I=");
  FPgaSerial.print(current_mA);
  FPgaSerial.print(",T=");
  FPgaSerial.print(temperature_centi);
  FPgaSerial.print(",F=");
  FPgaSerial.print(faultLatched ? 1 : 0);
  FPgaSerial.print(",S=");
  FPgaSerial.println(tempSensorFault ? 0 : 1);

  // Human-readable USB log.
  Serial.print("DATA,");
  Serial.print("V=");
  Serial.print(voltage, 2);
  Serial.print(",I=");
  Serial.print(current, 3);
  Serial.print(",T=");
  Serial.print(temperature, 2);
  Serial.print(",P=");
  Serial.print(power, 2);
  Serial.print(",FV=");
  Serial.print(potV);
  Serial.print(",FI=");
  Serial.print(potI);
  Serial.print(",FT=");
  Serial.print(potT);
  Serial.print(",FAULT=");
  Serial.print(faultLatched ? 1 : 0);
  Serial.print(",TEMP_SENSOR=");
  Serial.print(tempSensorFault ? 0 : 1);
  Serial.println();
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {

  checkButtons();

  if (millis() - lastSample >= 500) {
    lastSample = millis();

    readSensors();
    safetyCheck();
    updateOutputs();
  }

  updateLCD();
  sendToPYNQ();
}