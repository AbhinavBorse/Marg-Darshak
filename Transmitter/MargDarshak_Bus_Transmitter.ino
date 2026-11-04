#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

// --- LCD Setup ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// --- Keypad Setup (30-Pin Safe Mapping) ---
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {19, 18, 25, 16}; 
byte colPins[COLS] = {4, 13, 14, 26};  
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// --- State Machine & Variables ---
// RENAMED to avoid conflict with Keypad library's internal "IDLE"
enum AppState { STATE_IDLE, STATE_ENTRY, STATE_BROADCASTING };
AppState currentState = STATE_IDLE;
String routeEntry = ""; // Stores the 3 digits

// --- BLE Setup ---
BLEAdvertising *pAdvertising;

// ============================================================
// BLE PAYLOAD PROTOCOL  (the Receiver must use the same values)
//
// Manufacturer data, 5 bytes in total:
//   [0] 0x4C   Company ID lo   (unchanged from the original code)
//   [1] 0x01   Company ID hi   (unchanged from the original code)
//   [2] 0xA5   MAGIC byte      (receiver ignores packets without it)
//   [3] 0x80 | (route >> 7)    route high 7 bits
//   [4] 0x80 | (route & 0x7F)  route low 7 bits
//
// Why 7+7 bits instead of a plain little-endian uint16_t:
// bytes 3 and 4 can never be 0x00, so a route like 256 or 512 can't
// be cut short by a C-string null terminator inside the BLE library.
// Receiver decode:  route = ((b3 & 0x7F) << 7) | (b4 & 0x7F)
// Supports routes 0..16383 (3-digit routes need only 0..999).
// ============================================================
#define MFG_ID_LO   0x4C
#define MFG_ID_HI   0x01
#define MD_MAGIC    0xA5

String buildPayload(uint16_t routeNo) {
  String p = "";
  p += (char)MFG_ID_LO;
  p += (char)MFG_ID_HI;
  p += (char)MD_MAGIC;
  p += (char)(0x80 | ((routeNo >> 7) & 0x7F));
  p += (char)(0x80 | (routeNo & 0x7F));
  return p;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Initialize BLE Hardware
  BLEDevice::init("MargDarshak-Bus");
  // Fixed advertising TX power so the receiver's RSSI thresholds stay valid
  BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);
  pAdvertising = BLEDevice::getAdvertising();
  
  updateDisplay(); 
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    if (key == '*') {
      stopBroadcasting();
      routeEntry = "";
      currentState = STATE_IDLE;
      updateDisplay();
    } 
    else if (currentState != STATE_BROADCASTING) {
      if (key >= '0' && key <= '9' && routeEntry.length() < 3) {
        routeEntry += key;
        currentState = STATE_ENTRY;
        updateDisplay();
      }
      else if (key == '#' && routeEntry.length() == 3) {
        startBroadcasting();
        currentState = STATE_BROADCASTING;
        updateDisplay();
      }
    }
  }
}

// --- Helper Functions ---

void updateDisplay() {
  lcd.clear();
  if (currentState == STATE_IDLE) {
    lcd.setCursor(0, 0);
    lcd.print("Enter Route:");
  } 
  else if (currentState == STATE_ENTRY) {
    lcd.setCursor(0, 0);
    lcd.print("Enter Route:");
    lcd.setCursor(0, 1);
    lcd.print(routeEntry);
  } 
  else if (currentState == STATE_BROADCASTING) {
    lcd.setCursor(0, 0);
    lcd.print("Broadcasting:");
    lcd.setCursor(0, 1);
    lcd.print(routeEntry);
    lcd.setCursor(15, 1);
    lcd.print("*"); 
  }
}

void startBroadcasting() {
  // Convert the typed 3-digit String into the numeric route number
  uint16_t routeNo = (uint16_t)routeEntry.toInt();

  BLEAdvertisementData advData;
  advData.setFlags(0x06); // BLE only
  advData.setName("MargDarshak-Bus");

  String mfgData = buildPayload(routeNo);
  advData.setManufacturerData(mfgData); 
  
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setMinInterval(160); 
  pAdvertising->setMaxInterval(160);
  
  pAdvertising->start();

  // Debug: print route and the exact payload bytes being sent
  Serial.print("BLE Broadcast Started for Route: ");
  Serial.print(routeNo);
  Serial.print("  payload:");
  for (size_t i = 0; i < mfgData.length(); i++) {
    Serial.printf(" %02X", (uint8_t)mfgData[i]);
  }
  Serial.println();
}

void stopBroadcasting() {
  if (currentState == STATE_BROADCASTING) {
    pAdvertising->stop();
    Serial.println("BLE Broadcast Stopped.");
  }
}
