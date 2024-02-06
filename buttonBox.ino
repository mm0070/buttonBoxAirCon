#include <Joystick.h>
#include <Encoder.h>
#include <IRremote.h>
#include "AirconIRCodes.h"

// Encoder for air conditioning control (removing from joystick control)
Encoder encoderAC(8, 9);  // Assuming encoder3 is connected to pins 8 and 9

const int buttonPin = 10;  // Button for toggling air conditioner on/off
const int irLedPin = 16;   // IR LED pin for sending signals

bool airconPower = false;     // Air conditioner power state, false = off, true = on
int temperatureSetting = 21;  // Default temperature

IRsend irsend(irLedPin);  // Initialize IRsend object

long oldPositionAC = 0;  // Track the old position for encoder3
int stepsCounterAC = 0;  // Steps counter for encoder3 to confirm movement direction

// Joystick Encoder setup
Encoder encoder1(2, 3);
Encoder encoder2(5, 6);

// Initial joystick encoder positions for tracking movement
long oldPosition1 = 0;
long oldPosition2 = 0;

const int stepsThreshold = 4;  // Threshold of steps for considering movement intentional

// Joystick setup

// Define button pins, including A6 mapped as pin 4
const int buttonPins[] = {1, 0, 4, 7, 14, 15, 18, 19, 20, 21};
const int numberOfPhysicalButtons = sizeof(buttonPins) / sizeof(int);
const int numberOfVirtualButtons = 6; // 3 encoders * 2 directions each
const int totalButtons = numberOfPhysicalButtons + numberOfVirtualButtons;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, totalButtons, 0,
                   false, false, false, false, false, false,
                   false, false, false, false, false);

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  // Setup button with internal pull-up
  irsend.begin(irLedPin);            // Start the IRsend object
  
  // Joystick init
  // Set all button pins as inputs with internal pull-up resistors
  for (int i = 0; i < numberOfPhysicalButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  Joystick.begin();
}

void loop() {
  for (int i = 0; i < numberOfPhysicalButtons; i++) {
    bool currentButtonState = !digitalRead(buttonPins[i]); // Invert because of INPUT_PULLUP
    Joystick.setButton(i, currentButtonState);
  }

  handleAirconControl();  // Function to handle air conditioner control
  // Handle each encoder with its specific tracking and button logic
  handleEncoder(encoder1, oldPosition1, 11, 12);
  handleEncoder(encoder2, oldPosition2, 13, 14);

  delay(10);  // Short delay for loop
}

void handleEncoder(Encoder &encoder, long &oldPosition, int buttonIndexCW, int buttonIndexCCW) {
  static int steps1 = 0;
  static int steps2 = 0;
  int *steps = &steps1;

  if (&encoder == &encoder2) steps = &steps2;

  long newPosition = encoder.read();
  if (newPosition > oldPosition) {
    (*steps)++;
  } else if (newPosition < oldPosition) {
    (*steps)--;
  }

  if (*steps >= stepsThreshold) {
    Joystick.setButton(buttonIndexCW, 1);
    delay(50);
    Joystick.setButton(buttonIndexCW, 0);
    *steps = 0;
  } else if (*steps <= -stepsThreshold) {
    Joystick.setButton(buttonIndexCCW, 1);
    delay(50);
    Joystick.setButton(buttonIndexCCW, 0);
    *steps = 0;
  }

  oldPosition = newPosition;
}

void handleAirconControl() {
    static bool lastButtonState = HIGH;
    static unsigned long buttonPressTime = 0;
    bool buttonState = digitalRead(buttonPin);
    unsigned long currentMillis = millis();

    // Button Press Handling for toggling AC power
    if (buttonState != lastButtonState) {
        if (buttonState == LOW) {
            buttonPressTime = currentMillis;  // Mark the time when button goes LOW
        }
        lastButtonState = buttonState;
    }

    if (buttonState == HIGH && buttonPressTime > 0 && (currentMillis - buttonPressTime > 200)) {
        // Toggle AC power only after button release and if pressed longer than 200ms
        airconPower = !airconPower;
        if (airconPower) {
            sendAirconCommand(AC_ON);
            delay(1000);  // Wait 1s before setting the temperature
            temperatureSetting = 21;  // Default temperature setting
            sendTemperatureCommand(TEMP_21C);
        } else {
            sendAirconCommand(AC_OFF);
        }
        buttonPressTime = 0;  // Reset button press time
    }

    // Encoder Temperature Adjustment with Improved Sensitivity
    static long lastEncoderPosition = encoderAC.read();
    long currentEncoderPosition = encoderAC.read();
    const int stepsPerAdjustment = 4; // Adjust this value based on your encoder's sensitivity

    if (abs(currentEncoderPosition - lastEncoderPosition) >= stepsPerAdjustment) {
        int direction = (currentEncoderPosition > lastEncoderPosition) ? 1 : -1;
        adjustTemperature(direction);
        lastEncoderPosition = currentEncoderPosition - (currentEncoderPosition % stepsPerAdjustment); // Align to step grid
    }
}

void adjustTemperature(int direction) {
    if (!airconPower) return;  // Only adjust if AC is on

    temperatureSetting += direction;
    temperatureSetting = constrain(temperatureSetting, 19, 22); // Ensure the setting is within bounds

    AirconCommands tempCommand;
    switch (temperatureSetting) {
        case 19: tempCommand = TEMP_19C; break;
        case 20: tempCommand = TEMP_20C; break;
        case 21: tempCommand = TEMP_21C; break;
        case 22: tempCommand = TEMP_22C; break;
        default: return; // In case of an invalid setting, exit without sending a command
    }
    sendTemperatureCommand(tempCommand);
}

void sendAirconCommand(AirconCommands command) {
  irsend.sendRaw(commands[command].rawData, commands[command].length, 38);  // Send the IR command at 38 kHz
}

void sendTemperatureCommand(AirconCommands command) {
  if (command >= TEMP_19C && command <= TEMP_22C) {                           // Check if command is within the valid range
    irsend.sendRaw(commands[command].rawData, commands[command].length, 38);  // Send command
  }
}