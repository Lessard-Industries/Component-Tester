/*=====================================================================
  COMPONENT TESTER v5.4  |  Arduino Nano Edition
  =====================================================================
  Features:
    • Auto-detect: Resistors, Capacitors, Diodes, LEDs, BJTs (NPN/PNP)
    • LED color detection: Red, Yellow, Green, Blue/White
    • Continuity tester with audible feedback
    • Battery analyzer (AA/AAA, Li-ion, 9V, 12V)
  
  Hardware:
    • Arduino Nano
    • 20x4 I2C LCD (address 0x23)
    • 3x 680Ω pullup resistors (to VCC)
    • 3x 470kΩ pulldown resistors (to GND)
    • Rotary encoder with button
    • Test button, buzzer
    • Battery voltage divider (100k/22k)
    • Battery pack: 2x18650 cells / power switch
    • Battery pack measurement at startup (100k/22k)

  (c) 2025 Lessard Industries
======================================================================*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define F_VERSION "5.5"

// =============================================================================
// PINS
// =============================================================================
#define TP1           A0
#define TP2           A1
#define TP3           A2
#define POWER_PIN     A3
#define CONT_DRIVE    7
#define CONT_SENSE    A6
#define BATTERY_PIN   A7
#define TEST_BUTTON   2
#define BUZZER        3
#define ENCODER_CLK   4
#define ENCODER_DT    5
#define ENCODER_SW    6

// =============================================================================
// CALIBRATION - Update these with your measured values!
// =============================================================================
#define R_LOW           670.0     // Measured 680Ω resistor value
#define R_HIGH          465000.0  // Measured 470kΩ resistor value
#define VCC             5.05      // Measured supply voltage
#define BATTERY_DIVIDER 5.55      // (100k + 22k) / 22k

// =============================================================================
// LCD & CUSTOM CHARACTERS
// =============================================================================
LiquidCrystal_I2C lcd(0x23, 20, 4);

const byte char_resistor[8] PROGMEM    = {0b00000, 0b10101, 0b01010, 0b10101, 0b01010, 0b10101, 0b00000, 0b00000};
const byte char_diode_arrow[8] PROGMEM = {0b00110, 0b00110, 0b10000, 0b11111, 0b11000, 0b10000, 0b00000, 0b00000};
const byte char_diode_line[8] PROGMEM  = {0b00000, 0b10000, 0b11000, 0b11111, 0b11000, 0b10000, 0b00000, 0b00000};
const byte char_capacitor[8] PROGMEM   = {0b00000, 0b11111, 0b00000, 0b11111, 0b00000, 0b11111, 0b00000, 0b00000};
const byte char_omega[8] PROGMEM       = {0b01110, 0b10001, 0b10001, 0b10001, 0b01010, 0b11011, 0b00000, 0b00000};
const byte char_transistor[8] PROGMEM  = {0b00100, 0b00110, 0b00101, 0b11101, 0b00101, 0b00110, 0b00100, 0b00000};
const byte char_copyright[8] PROGMEM   = {0b01110, 0b10001, 0b10101, 0b10111, 0b10101, 0b10001, 0b01110, 0b00000};
const byte char_logo[8] PROGMEM        = {0b00011, 0b10101, 0b01110, 0b00100, 0b00100, 0b01110, 0b10101, 0b11000};

enum { CHAR_RESISTOR, CHAR_DIODE_ARROW, CHAR_DIODE_LINE, CHAR_CAPACITOR, 
       CHAR_OMEGA, CHAR_TRANSISTOR, CHAR_COPYRIGHT, CHAR_LOGO };

// =============================================================================
// ENUMS & TYPES
// =============================================================================
enum ComponentType {
  COMP_NONE, COMP_RESISTOR, COMP_CAPACITOR, COMP_DIODE,
  COMP_LED_RED, COMP_LED_YELLOW, COMP_LED_GREEN, COMP_LED_BLUE,
  COMP_NPN, COMP_PNP, COMP_UNKNOWN
};

enum Mode { MODE_COMPONENT_TEST, MODE_CONTINUITY_TEST, MODE_BATTERY_VOLTAGE, MODE_VOLTAGE_METER, MODE_COUNT };
enum TestState { TEST_WAITING, TEST_RUNNING, TEST_RESULTS };

struct PinMeasurement {
  float resistance;
  float voltage;
  bool conducted;
};

// =============================================================================
// GLOBAL STATE
// =============================================================================
Mode currentMode = MODE_COMPONENT_TEST;
Mode selectedMode = MODE_COMPONENT_TEST;
Mode lastActiveMode = MODE_COUNT;
bool inMenu = true;
bool needsRedraw = true;

int encoderPos = 0;
int lastCLK = HIGH;
unsigned long lastEncoderChange = 0;
const unsigned long menuTimeout = 3000;

TestState testState = TEST_WAITING;
unsigned long lastTestPress = 0;

ComponentType detectedComponent = COMP_NONE;
float measuredValue = 0;
char pinout[20] = "";
float capMeasurements[3] = {0, 0, 0};
PinMeasurement measurements[6];

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void handleEncoder();
void handleMenu();
void componentTestMode();
void continuityTestMode();
void batteryVoltageMode();
void voltageMeterMode();
void displayComponentResults();
void performComponentTest();
void measureCapacitance(uint8_t pinA, uint8_t pinB, int index);
void testPinPair(uint8_t hiPin, uint8_t loPin, uint8_t unPin, uint8_t index);
void identifyComponent();
bool checkTransistor();
bool checkDiode();
bool checkCapacitor();
bool checkResistor();

// =============================================================================
// UTILITIES
// =============================================================================
inline void beep(unsigned int freq, unsigned int dur) {
  noTone(BUZZER);
  tone(BUZZER, freq, dur);
}

float readVoltage(uint8_t pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) { 
    sum += analogRead(pin); 
    delayMicroseconds(100); 
  }
  return (sum / 8.0 / 1023.0) * VCC;
}

float readVoltageFast(uint8_t pin) {
  return (analogRead(pin) / 1023.0) * VCC;
}

void loadCustomChar(uint8_t index, const byte* charData) {
  byte buf[8];
  memcpy_P(buf, charData, 8);
  lcd.createChar(index, buf);
}

void showPowerStatus() {
  long sum = 0;
  for (int i = 0; i < 10; i++) { 
    sum += analogRead(POWER_PIN); 
    delay(5); 
  }
  // Direct calculation: (100k + 22k) / 22k = 5.545 ratio
  float voltage = (sum / 10.0f / 1023.0f) * VCC * 5.595f;
  
  // 2S 18650: 6.0V (empty) to 8.4V (full)
  float pct = (voltage - 6.0f) / (8.4f - 6.0f) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  int pctInt = (int)(pct + 0.5f);
  
  lcd.clear();
  
  if (pctInt <= 15) {
    //lcd.setCursor(0, 0); lcd.print("====================");
    lcd.setCursor(0, 0); lcd.print("==== !! LOW !! =====");
    lcd.setCursor(5, 1); 
    lcd.print(voltage, 1); lcd.print("V  ["); lcd.print(pctInt); lcd.print("%]");
    lcd.setCursor(4, 2); lcd.print("Charge soon!");
    lcd.setCursor(0, 3); lcd.print("====================");
    beep(800, 300);
    delay(2000);
  } else {
    lcd.setCursor(0, 0); lcd.print("====================");
    lcd.setCursor(2, 1); lcd.print("Internal Battery");
    lcd.setCursor(3, 2);
    lcd.print(voltage, 2); lcd.print("V  ["); lcd.print(pctInt); lcd.print("%]");
    lcd.setCursor(0, 3); lcd.print("====================");
    delay(1500);
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  Wire.setClock(40000);

  loadCustomChar(CHAR_RESISTOR, char_resistor);
  loadCustomChar(CHAR_DIODE_ARROW, char_diode_arrow);
  loadCustomChar(CHAR_DIODE_LINE, char_diode_line);
  loadCustomChar(CHAR_CAPACITOR, char_capacitor);
  loadCustomChar(CHAR_OMEGA, char_omega);
  loadCustomChar(CHAR_TRANSISTOR, char_transistor);
  loadCustomChar(CHAR_COPYRIGHT, char_copyright);
  loadCustomChar(CHAR_LOGO, char_logo);

  pinMode(TEST_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(CONT_DRIVE, OUTPUT);
  digitalWrite(CONT_DRIVE, LOW);
  analogReference(DEFAULT);

  // Splash screen
  lcd.clear();
  lcd.setCursor(2, 0); lcd.print("COMPONENT TESTER");
  lcd.setCursor(7, 1); lcd.print("v"); lcd.print(F_VERSION);
  lcd.setCursor(0, 2); lcd.write(CHAR_COPYRIGHT); lcd.print("Lessard Industries");
  lcd.setCursor(9, 3); lcd.write(CHAR_LOGO);

  while (digitalRead(TEST_BUTTON) == LOW) {
    lcd.setCursor(0, 3);
    lcd.print(F("Release TEST button "));
    delay(100);
  }

  beep(1000, 100);
  delay(150);
  beep(1500, 100);
  delay(2000);
 
  // Check internal battery
  showPowerStatus();

  lcd.clear();
  lastCLK = digitalRead(ENCODER_CLK);
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  handleEncoder();

  if (currentMode != lastActiveMode) {
    lastActiveMode = currentMode;
    needsRedraw = true;
    lcd.clear();
    if (currentMode == MODE_COMPONENT_TEST) testState = TEST_WAITING;
  }

  if (inMenu) {
    handleMenu();
  } else {
    switch (currentMode) {
      case MODE_COMPONENT_TEST:  componentTestMode();  break;
      case MODE_CONTINUITY_TEST: continuityTestMode(); break;
      case MODE_BATTERY_VOLTAGE: batteryVoltageMode(); break;
      case MODE_VOLTAGE_METER:   voltageMeterMode();   break;
      default: break;
    }
  }
}

// =============================================================================
// ENCODER & MENU
// =============================================================================
void handleEncoder() {
  static unsigned long lastEncoderAction = 0;
  unsigned long now = millis();
  
  if (now - lastEncoderAction < 50) return;

  int currentCLK = digitalRead(ENCODER_CLK);
  if (currentCLK != lastCLK && currentCLK == LOW) {
    encoderPos += (digitalRead(ENCODER_DT) == LOW) ? -1 : 1;
    if (encoderPos < 0) encoderPos = MODE_COUNT - 1;
    if (encoderPos >= MODE_COUNT) encoderPos = 0;
    
    selectedMode = (Mode)encoderPos;
    lastEncoderChange = now;
    lastEncoderAction = now;

    if (!inMenu) {
      inMenu = true;
      needsRedraw = true;
      beep(2000, 20);
    }
  }
  lastCLK = currentCLK;
}

void handleMenu() {
  static Mode lastDisplayedMode = (Mode)-1;
  
  if (millis() - lastEncoderChange > menuTimeout) {
    inMenu = false;
    selectedMode = currentMode;
    encoderPos = currentMode;
    needsRedraw = true;
    lastDisplayedMode = (Mode)-1;
    beep(1500, 30);
    lcd.clear();
    return;
  }

  if (needsRedraw || lastDisplayedMode != selectedMode) {
    if (needsRedraw) {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("=== SELECT MODE ====");
      lcd.setCursor(2, 1); lcd.print("Component Test");
      lcd.setCursor(2, 2); lcd.print("Continuity Test");
      // Row 3: two modes share the line
      lcd.setCursor(2, 3); lcd.print("Battery");
      lcd.setCursor(13, 3); lcd.print("Voltage");
      needsRedraw = false;
    }

    // Clear old cursor
    if (lastDisplayedMode != (Mode)-1) {
      if (lastDisplayedMode <= MODE_CONTINUITY_TEST) {
        lcd.setCursor(0, lastDisplayedMode + 1);
        lcd.print(" ");
      } else if (lastDisplayedMode == MODE_BATTERY_VOLTAGE) {
        lcd.setCursor(0, 3);
        lcd.print(" ");
      } else {
        lcd.setCursor(11, 3);
        lcd.print(" ");
      }
    }
    // Draw new cursor
    if (selectedMode <= MODE_CONTINUITY_TEST) {
      lcd.setCursor(0, selectedMode + 1);
      lcd.print(">");
    } else if (selectedMode == MODE_BATTERY_VOLTAGE) {
      lcd.setCursor(0, 3);
      lcd.print(">");
    } else {
      lcd.setCursor(11, 3);
      lcd.print(">");
    }
    lastDisplayedMode = selectedMode;
  }

  if (digitalRead(ENCODER_SW) == LOW) {
    delay(50);
    if (digitalRead(ENCODER_SW) == LOW) {
      currentMode = selectedMode;
      inMenu = false;
      needsRedraw = true;
      lastDisplayedMode = (Mode)-1;
      beep(2000, 50);
      lcd.clear();
      while (digitalRead(ENCODER_SW) == LOW);
      delay(50);
    }
  }
}

// =============================================================================
// COMPONENT TEST MODE
// =============================================================================
void componentTestMode() {
  if (needsRedraw) {
    lcd.clear();
    switch (testState) {
      case TEST_WAITING:
        lcd.setCursor(0, 0); lcd.print("== COMPONENT TEST ==");
        lcd.setCursor(2, 1); lcd.print("Insert component");
        lcd.setCursor(3, 2); lcd.print("1  -  2  -  3");
        lcd.setCursor(5, 3); lcd.print("Press TEST");
        break;
      case TEST_RUNNING:
        lcd.setCursor(0, 0); lcd.print("== COMPONENT TEST ==");
        lcd.setCursor(3, 1); lcd.print("Testing...");
        lcd.setCursor(3, 2); lcd.print("Please wait");
        break;
      case TEST_RESULTS:
        displayComponentResults();
        break;
    }
    needsRedraw = false;
  }

  if (digitalRead(TEST_BUTTON) == LOW && millis() - lastTestPress > 300) {
    lastTestPress = millis();
    
    if (testState == TEST_WAITING) {
      testState = TEST_RUNNING;
      needsRedraw = true;
      while (digitalRead(TEST_BUTTON) == LOW);
      delay(50);
      
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("== COMPONENT TEST ==");
      lcd.setCursor(3, 1); lcd.print("Testing...");
      lcd.setCursor(3, 2); lcd.print("Please wait");
      needsRedraw = false;
      
      performComponentTest();
      testState = TEST_RESULTS;
      needsRedraw = true;
      
    } else if (testState == TEST_RESULTS) {
      testState = TEST_WAITING;
      needsRedraw = true;
      while (digitalRead(TEST_BUTTON) == LOW);
      delay(50);
    }
  }
}

// =============================================================================
// COMPONENT TEST ENGINE
// =============================================================================
void dischargeAll() {
  pinMode(TP1, OUTPUT); pinMode(TP2, OUTPUT); pinMode(TP3, OUTPUT);
  digitalWrite(TP1, LOW); digitalWrite(TP2, LOW); digitalWrite(TP3, LOW);
  delay(300);
  pinMode(TP1, INPUT); pinMode(TP2, INPUT); pinMode(TP3, INPUT);
}

void performComponentTest() {
  capMeasurements[0] = capMeasurements[1] = capMeasurements[2] = 0;

  dischargeAll();
  delay(100);

  measureCapacitance(TP1, TP2, 0);
  measureCapacitance(TP1, TP3, 1);
  measureCapacitance(TP2, TP3, 2);

  dischargeAll();
  delay(100);

  testPinPair(TP1, TP2, TP3, 0);
  testPinPair(TP2, TP1, TP3, 1);
  testPinPair(TP1, TP3, TP2, 2);
  testPinPair(TP3, TP1, TP2, 3);
  testPinPair(TP2, TP3, TP1, 4);
  testPinPair(TP3, TP2, TP1, 5);

  identifyComponent();
  beep(2000, 100);
  delay(150);
}

void testPinPair(uint8_t hiPin, uint8_t loPin, uint8_t unPin, uint8_t index) {
  pinMode(unPin, INPUT);
  pinMode(hiPin, INPUT);
  pinMode(loPin, OUTPUT);
  digitalWrite(loPin, LOW);
  delay(5);

  float Vnode = readVoltage(hiPin);

  const float R_OUT = 25.0f;  // Arduino OUTPUT impedance compensation
  float denom = VCC - Vnode;
  if (denom < 0.001f) denom = 0.001f;

  float Rpar_raw = R_LOW * (Vnode / denom);
  float Rpar = Rpar_raw - R_OUT;
  if (Rpar < 0.1f) Rpar = 0.1f;

  float R_est = 9.999e6f;
  if (Rpar > 0.0f) {
    float diff = R_HIGH - Rpar;
    if (diff > (0.005f * R_HIGH)) {
      R_est = (Rpar * R_HIGH) / diff;
      if (R_est > 50000.0f) R_est *= 0.85f;  // High-R ADC correction
    }
  }

  bool conducted = (Vnode > 0.005f) && (Vnode < (VCC - 0.010f));

  if (conducted) {
    pinMode(loPin, INPUT);
    pinMode(hiPin, OUTPUT);
    digitalWrite(hiPin, LOW);
    delay(3);
    readVoltage(loPin);  // Reverse bias check
    pinMode(hiPin, INPUT);
  }
  pinMode(loPin, INPUT);

  measurements[index].resistance = R_est;
  measurements[index].voltage = Vnode;
  measurements[index].conducted = conducted;

  Serial.print("Test "); Serial.print(index);
  Serial.print(": Vnode="); Serial.print(Vnode, 3);
  Serial.print(" R="); Serial.print(R_est, 1);
  Serial.println(conducted ? " YES" : " NO");
}

void measureCapacitance(uint8_t pinA, uint8_t pinB, int index) {
  dischargeAll();
  delay(20);

  pinMode(pinA, OUTPUT);
  digitalWrite(pinA, LOW);
  pinMode(pinB, INPUT);

  unsigned long t_start = micros();
  float v_target = VCC * 0.632f;

  while (readVoltageFast(pinB) < v_target && (micros() - t_start) < 1000000);

  unsigned long t_delta = micros() - t_start;
  capMeasurements[index] = (t_delta >= 10 && t_delta < 1000000) 
                           ? ((float)t_delta / 1000000.0f) / R_LOW : 0;
  dischargeAll();
}

// =============================================================================
// COMPONENT IDENTIFICATION
// =============================================================================
void identifyComponent() {
  Serial.println("\n=== ANALYZING ===");
  
  bool allOpen = true;
  for (int i = 0; i < 6; i++) {
    if (measurements[i].conducted) { allOpen = false; break; }
  }
  
  if (allOpen) {
    detectedComponent = COMP_NONE;
    Serial.println("NONE");
    return;
  }

  if (checkTransistor()) return;
  if (checkDiode()) return;
  if (checkCapacitor()) return;
  if (checkResistor()) return;

  detectedComponent = COMP_UNKNOWN;
  measuredValue = 0;
  strcpy(pinout, "Unknown");
  Serial.println("UNKNOWN");
}

bool checkTransistor() {
  int diodeJunctions = 0;
  int forwardDir[3] = {-1, -1, -1};

  for (int i = 0; i < 6; i += 2) {
    int pair = i / 2;
    float r_a = measurements[i].resistance;
    float r_b = measurements[i + 1].resistance;

    if (r_a < 500.0f && (r_b / r_a) > 10.0f) {
      forwardDir[pair] = 0;
      diodeJunctions++;
    } else if (r_b < 500.0f && (r_a / r_b) > 10.0f) {
      forwardDir[pair] = 1;
      diodeJunctions++;
    }
  }

  if (diodeJunctions < 2) return false;

  // Find base pin (appears in both junctions)
  int countPin[4] = {0, 0, 0, 0};
  for (int p = 0; p < 3; p++) {
    if (forwardDir[p] != -1) {
      switch (p) {
        case 0: countPin[1]++; countPin[2]++; break;
        case 1: countPin[1]++; countPin[3]++; break;
        case 2: countPin[2]++; countPin[3]++; break;
      }
    }
  }

  int basePin = 0;
  for (int p = 1; p <= 3; p++) {
    if (countPin[p] >= 2) { basePin = p; break; }
  }
  if (basePin == 0) return false;

  // Determine NPN vs PNP
  auto pairFromPins = [](int a, int b) -> int {
    if ((a == 1 && b == 2) || (a == 2 && b == 1)) return 0;
    if ((a == 1 && b == 3) || (a == 3 && b == 1)) return 1;
    if ((a == 2 && b == 3) || (a == 3 && b == 2)) return 2;
    return -1;
  };

  bool baseForward = false;
  for (int p = 1; p <= 3; p++) {
    if (p == basePin) continue;
    int pair = pairFromPins(basePin, p);
    if (pair < 0 || forwardDir[pair] == -1) continue;
    
    int pinA = (pair == 0) ? 1 : (pair == 1) ? 1 : 2;
    int pinB = (pair == 0) ? 2 : (pair == 1) ? 3 : 3;
    
    if ((forwardDir[pair] == 0 && pinA == basePin) ||
        (forwardDir[pair] == 1 && pinB == basePin)) {
      baseForward = true;
      break;
    }
  }

  detectedComponent = baseForward ? COMP_NPN : COMP_PNP;
  strcpy(pinout, baseForward ? "NPN Transistor" : "PNP Transistor");
  Serial.println(baseForward ? "NPN" : "PNP");
  return true;
}

bool checkDiode() {
  const float VF_MIN = 0.40f, VF_RED = 2.1f, VF_YEL = 2.4f, VF_GRN = 2.75f, VF_BLU = 3.8f;

  for (int i = 0; i < 6; i++) {
    int opp = (i % 2 == 0) ? i + 1 : i - 1;
    float vf = measurements[i].voltage;
    float vf_opp = measurements[opp].voltage;
    float r = measurements[i].resistance;

    bool conducts = (vf >= VF_MIN && vf <= VF_BLU);
    bool blocks = (vf_opp < VF_MIN || vf_opp > VF_BLU);
    bool notCap = (vf < 1.0f) ? (r < 1000.0f) : (vf > 3.5f) ? (r < 100.0f) : (r < 2000.0f);

    if (conducts && blocks && notCap) {
      if (vf < 1.0f)           detectedComponent = COMP_DIODE;
      else if (vf < VF_RED)    detectedComponent = COMP_LED_RED;
      else if (vf < VF_YEL)    detectedComponent = COMP_LED_YELLOW;
      else if (vf < VF_GRN)    detectedComponent = COMP_LED_GREEN;
      else                     detectedComponent = COMP_LED_BLUE;

      measuredValue = vf;
      
      const char* pins[] = {"1(+) 2(-)", "2(+) 1(-)", "1(+) 3(-)", "3(+) 1(-)", "2(+) 3(-)", "3(+) 2(-)"};
      strcpy(pinout, pins[i]);

      Serial.print("DIODE Vf="); Serial.println(vf, 3);
      return true;
    }
  }
  return false;
}

bool checkCapacitor() {
  for (int i = 0; i < 3; i++) {
    if (capMeasurements[i] > 5e-7f) {
      detectedComponent = COMP_CAPACITOR;
      measuredValue = capMeasurements[i];
      const char* pins[] = {"1-2", "1-3", "2-3"};
      strcpy(pinout, pins[i]);
      Serial.print("CAP "); Serial.print(measuredValue * 1e6, 2); Serial.println("uF");
      return true;
    }
  }
  return false;
}

bool checkResistor() {
  int bestIdx = -1;
  float bestR = 0;
  float bestV = VCC;

  for (int i = 0; i < 6; i += 2) {
    if (!measurements[i].conducted || !measurements[i + 1].conducted) continue;

    float r1 = measurements[i].resistance;
    float r2 = measurements[i + 1].resistance;
    float r_avg = (r1 + r2) / 2.0f;
    
    if (r_avg <= 0.5f || r_avg >= 500000.0f) continue;
    if (fabs(r1 - r2) / r_avg >= 0.30f) continue;

    float v_avg = (measurements[i].voltage + measurements[i + 1].voltage) / 2.0f;

    if (bestIdx < 0 || v_avg < bestV || (v_avg >= VCC - 0.2f && r_avg > bestR)) {
      bestIdx = i;
      bestR = r_avg;
      bestV = v_avg;
    }
  }

  if (bestIdx < 0) return false;

  detectedComponent = COMP_RESISTOR;
  measuredValue = bestR;
  const char* pins[] = {"1-2", "?", "1-3", "?", "2-3"};
  strcpy(pinout, pins[bestIdx]);

  Serial.print("RES "); Serial.print(bestR, 1); Serial.println("Ω");
  return true;
}

// =============================================================================
// DISPLAY RESULTS
// =============================================================================
void displayResistance(float r) {
  lcd.print("R = ");
  if (r < 1e3)      { lcd.print(r, 1); lcd.print(" "); }
  else if (r < 1e6) { lcd.print(r / 1e3, 2); lcd.print(" k"); }
  else              { lcd.print(r / 1e6, 2); lcd.print(" M"); }
  lcd.write(CHAR_OMEGA);
}

void displayCapacitance(float c) {
  lcd.print("C = ");
  if (c < 1e-9)      { lcd.print(c * 1e12, 0); lcd.print(" pF"); }
  else if (c < 1e-6) { lcd.print(c * 1e9, 1); lcd.print(" nF"); }
  else if (c < 1e-3) { lcd.print(c * 1e6, 2); lcd.print(" uF"); }
  else               { lcd.print(c * 1e3, 2); lcd.print(" mF"); }
}

void displayComponentResults() {
  switch (detectedComponent) {
    case COMP_NONE:
      lcd.setCursor(0, 0); lcd.print("=== NO COMPONENT ===");
      lcd.setCursor(0, 1); lcd.print("Check connections");
      break;

    case COMP_RESISTOR:
      lcd.setCursor(0, 0); lcd.print("=== "); lcd.write(CHAR_RESISTOR); lcd.print(" RESISTOR =====");
      lcd.setCursor(0, 1); displayResistance(measuredValue);
      lcd.setCursor(0, 2); lcd.print("Pins: "); lcd.print(pinout);
      break;

    case COMP_DIODE:
    case COMP_LED_RED:
    case COMP_LED_YELLOW:
    case COMP_LED_GREEN:
    case COMP_LED_BLUE: {
      lcd.setCursor(0, 0);
      lcd.print("== DIODE ");
      if (detectedComponent == COMP_DIODE) {
        lcd.print("(Std) "); lcd.write(CHAR_DIODE_LINE);
      } else {
        lcd.print("(LED) "); lcd.write(CHAR_DIODE_ARROW);
      }
      lcd.print(" ==");

      lcd.setCursor(0, 1);
      lcd.print("Vf = "); lcd.print(measuredValue, 3); lcd.print(" V ");
      
      const char* color = "";
      switch (detectedComponent) {
        case COMP_LED_RED:    color = "(Red)"; break;
        case COMP_LED_YELLOW: color = "(Yel)"; break;
        case COMP_LED_GREEN:  color = "(Grn)"; break;
        case COMP_LED_BLUE:   color = "(B/W)"; break;
        default: break;
      }
      lcd.print(color);
      lcd.setCursor(0, 2); lcd.print("Pins: "); lcd.print(pinout);
      break;
    }

    case COMP_CAPACITOR:
      lcd.setCursor(0, 0); lcd.print("=== "); lcd.write(CHAR_CAPACITOR); lcd.print(" CAPACITOR ====");
      lcd.setCursor(0, 1); displayCapacitance(measuredValue);
      lcd.setCursor(0, 2); lcd.print("Pins: "); lcd.print(pinout);
      break;

    case COMP_NPN:
    case COMP_PNP:
      lcd.setCursor(0, 0); lcd.print("=== "); lcd.write(CHAR_TRANSISTOR); lcd.print(" TRANSISTOR ===");
      lcd.setCursor(0, 1); lcd.print("Type: ");
      lcd.print(detectedComponent == COMP_NPN ? "NPN" : "PNP");
      break;

    default:
      lcd.setCursor(0, 0); lcd.print("===== UNKNOWN ======");
      lcd.setCursor(0, 1); lcd.print("Unrecognized signal");
      break;
  }
  lcd.setCursor(0, 3); lcd.print("TEST=retry Turn=menu");
}

// =============================================================================
// CONTINUITY TEST MODE
// =============================================================================
void continuityTestMode() {
  static unsigned long lastUpdate = 0;
  static float lastR = -999;
  static bool beeping = false;
  static bool connected = false;

  if (needsRedraw) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("==== CONTINUITY ====");
    lcd.setCursor(0, 1); lcd.print("Touch probes A & B");
    needsRedraw = false;
    lastR = -999;
    beeping = false;
    connected = false;
    noTone(BUZZER);
  }

  if (millis() - lastUpdate < 30) return;
  lastUpdate = millis();

  pinMode(CONT_DRIVE, OUTPUT);
  digitalWrite(CONT_DRIVE, HIGH);
  pinMode(CONT_SENSE, INPUT);
  delay(2);

  float voltage = readVoltage(CONT_SENSE);
  float R_unknown = 9999999.0f;

  if (voltage < 0.01f) {
    R_unknown = 9999999.0f;
  } else if (voltage > VCC - 0.01f) {
    R_unknown = 0.0f;
  } else {
    float R_total = 1000.0f * ((VCC / voltage) - 1.0f);
    R_unknown = R_total - 680.0f;
    if (R_unknown < 0) R_unknown = 0;
    if (R_unknown > 9e6f) R_unknown = 9999999.0f;
  }

  if (!connected && R_unknown < 30.0f) connected = true;
  if (connected && R_unknown > 80.0f) connected = false;

  if (fabs(R_unknown - lastR) > 2.0f) {
    char line[21];
    const char* label = (R_unknown < 30) ? "SHORT" : (R_unknown < 200) ? "GOOD" : 
                        (R_unknown < 100000) ? "HIGH" : "OPEN";

    if (R_unknown >= 100000) {
      snprintf(line, 21, "%-20s", "OPEN");
    } else if (R_unknown < 1.0f) {
      snprintf(line, 21, "%-6s< 1 Ohm        ", label);
    } else if (R_unknown < 1000.0f) {
      snprintf(line, 21, "%-6s%.0f Ohm        ", label, R_unknown);
    } else {
      snprintf(line, 21, "%-6s%.1fk Ohm       ", label, R_unknown / 1000.0f);
    }
    lcd.setCursor(0, 2); lcd.print(line);

    // Bar graph
    int barCount = (R_unknown < 1000) ? (int)(18.0f * (1.0f - R_unknown / 1000.0f)) : 0;
    if (barCount < 0) barCount = 0;
    if (barCount > 18) barCount = 18;
    
    char bar[21] = "[                  ]";
    for (int i = 0; i < barCount; i++) bar[i + 1] = (char)255;
    lcd.setCursor(0, 3); lcd.print(bar);

    lastR = R_unknown;
  }

  if (connected && !beeping) { tone(BUZZER, 2000); beeping = true; }
  if (!connected && beeping) { noTone(BUZZER); beeping = false; }
}

// =============================================================================
// BATTERY VOLTAGE MODE
// =============================================================================
void batteryVoltageMode() {
  static unsigned long lastUpdate = 0;
  static float filtered = -1.0f;
  static float lastV = -999.0f;
  static int lastPct = -1;
  static bool warned = false;

  enum { BT_NONE, BT_AA, BT_2CELL, BT_LIION, BT_9V, BT_12V, BT_UNK };

  if (needsRedraw) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("===== BATTERY ======");
    needsRedraw = false;
    filtered = -1.0f;
    lastV = -999.0f;
    lastPct = -1;
    warned = false;
  }

  if (millis() - lastUpdate < 500) return;
  lastUpdate = millis();

  long sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(BATTERY_PIN); delay(5); }
  float raw = (sum / 10.0f / 1023.0f) * VCC * BATTERY_DIVIDER;

  if (raw < 0.5f) {
    lcd.setCursor(0, 1); lcd.print("No battery detected ");
    lcd.setCursor(0, 2); lcd.print("Voltage: ----       ");
    lcd.setCursor(0, 3); lcd.print("[                  ]");
    return;
  }

  filtered = (filtered < 0) ? raw : filtered * 0.75f + raw * 0.25f;

  int type = (filtered < 2.2f) ? BT_AA : (filtered < 3.4f) ? BT_2CELL :
             (filtered < 4.5f) ? BT_LIION : (filtered < 10.5f) ? BT_9V : 
             (filtered < 15.5f) ? BT_12V : BT_UNK;

  float pct = 0;
  switch (type) {
    case BT_AA:    pct = (filtered - 1.0f) / 0.55f * 100; break;
    case BT_2CELL: pct = ((filtered / 2) - 1.0f) / 0.55f * 100; break;
    case BT_LIION: pct = (filtered - 3.0f) / 1.2f * 100; break;
    case BT_9V:    pct = (filtered - 6.0f) / 3.2f * 100; break;
    case BT_12V:   pct = (filtered - 11.8f) / 0.9f * 100; break;
    default:       pct = (filtered - 0.5f) / 19.5f * 100; break;
  }
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  int pctInt = (int)(pct + 0.5f);

  if (fabs(filtered - lastV) > 0.03f || pctInt != lastPct) {
    const char* typeStr[] = {"", "AA/AAA 1.5V", "2-cell 3V", "Li-Ion 1S", "9V battery", "12V SLA", "Unknown"};
    const char* status = (pctInt >= 90) ? "Full" : (pctInt >= 60) ? "Good" : 
                         (pctInt >= 30) ? "OK" : (pctInt >= 10) ? "Low" : "Very Low";

    char line[21];
    snprintf(line, 21, "%-12s%8s", typeStr[type], status);
    lcd.setCursor(0, 1); lcd.print(line);

    int whole = (int)filtered;
    int frac = (int)((filtered - whole) * 100);
    snprintf(line, 21, "Voltage: %d.%02d V     ", whole, frac);
    lcd.setCursor(0, 2); lcd.print(line);

    int bars = pctInt * 18 / 100;
    char bar[21] = "[                  ]";
    for (int i = 0; i < bars; i++) bar[i + 1] = (char)255;
    lcd.setCursor(0, 3); lcd.print(bar);

    lastV = filtered;
    lastPct = pctInt;
  }

  if (type != BT_UNK && type != BT_NONE && pctInt <= 10 && !warned) {
    beep(1200, 120);
    warned = true;
  }
  if (pctInt > 10) warned = false;
}

// =============================================================================
// VOLTAGE METER MODE
// =============================================================================
void voltageMeterMode() {
  static unsigned long lastUpdate = 0;
  static float lastV = -999.0f;

  if (needsRedraw) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("==== VOLTMETER =====");
    lcd.setCursor(0, 1); lcd.print("Probes on Red/Black ");
    lcd.setCursor(0, 2); lcd.print("Range: 0 - 28V");
    needsRedraw = false;
    lastV = -999.0f;
  }

  if (millis() - lastUpdate < 120) return;
  lastUpdate = millis();

  int raw = analogRead(BATTERY_PIN);
  float voltage = (raw / 1023.0f) * VCC * BATTERY_DIVIDER;

  if (fabs(voltage - lastV) < 0.005f) return;
  lastV = voltage;

  char line[21];
  if (voltage < 0.5f) {
    snprintf(line, 21, "  --- no input ---  ");
  } else if (voltage < 1.0f) {
    int mv = (int)(voltage * 1000.0f + 0.5f);
    snprintf(line, 21, "  %d mV              ", mv);
  } else {
    int whole = (int)voltage;
    int frac = (int)((voltage - whole) * 1000.0f + 0.5f);
    snprintf(line, 21, "  %d.%03d V            ", whole, frac);
  }
  line[20] = '\0';
  lcd.setCursor(0, 3); lcd.print(line);
}
