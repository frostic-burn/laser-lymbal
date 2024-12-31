#include <Keypad.h>

// Keypad configuration
const byte ROWS = 4;
const byte COLS = 4;
const char KEYS[ROWS][COLS] = {
  {'D', 'C', 'B', 'A'},  // Corrected row mapping
  {'#', '9', '6', '3'},  // Corrected row mapping for 4 -> #
  {'0', '8', '5', '2'},  // Corrected row mapping for 7 -> 5, 0 -> 7
  {'*', '7', '4', '1'}   // Corrected row mapping for # -> 4, 0 -> 7
};

// Arduino pins for keypad rows and columns
byte rowPins[ROWS] = {9, 8, 7, 6};    // Connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2};    // Connect to the column pinouts of the keypad

// Create keypad object
Keypad keypad = Keypad(makeKeymap(KEYS), rowPins, colPins, ROWS, COLS);

// Solenoid lock pin
const int LOCK_PIN = 10;              // Connect solenoid to this pin
const String CORRECT_PIN = "3777";    // Set the correct PIN
const int UNLOCK_DURATION = 10000;     // Lock stays open for 10 seconds

// Variables for PIN handling
String enteredPin = "";
bool doorLocked = true;
unsigned long unlockTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW);        // Ensure lock is initially locked
  Serial.println("System Ready");
}

void loop() {
  char key = keypad.getKey();
  
  // Check if door should be relocked
  if (!doorLocked && (millis() - unlockTime >= UNLOCK_DURATION)) {
    lockDoor();
  }
  
  if (key) {
    Serial.print("Key pressed: ");
    Serial.println(key);
    
    if (key == '#') {
      checkPin();
    }
    else if (key == '*') {
      resetPin();
    }
    else if (enteredPin.length() < 4) {
      enteredPin += key;
      Serial.print("PIN entry: ");
      for (int i = 0; i < enteredPin.length(); i++) {
        Serial.print("*");
      }
      Serial.println();
    }
  }
}

void checkPin() {
  if (enteredPin == CORRECT_PIN) {
    Serial.println("PIN Correct - Unlocking door");
    unlockDoor();
  } else {
    Serial.println("PIN Incorrect - Please try again");
    resetPin();
  }
}

void unlockDoor() {
  digitalWrite(LOCK_PIN, HIGH);
  doorLocked = false;
  unlockTime = millis();
  Serial.println("Door unlocked");
  resetPin();
}

void lockDoor() {
  digitalWrite(LOCK_PIN, LOW);
  doorLocked = true;
  Serial.println("Door locked");
}

void resetPin() {
  enteredPin = "";
  Serial.println("PIN cleared");
}
