// DMT Group 16C - Code for PID Controller Implementation to the Physical System

#include <Wire.h>
#include <ModbusMaster.h>

// Function Prototypes
void readPressureSensor();
float calculatePressure(uint16_t sensorValue);
void controlVFD(unsigned long currentMillis);
void checkBuzzer();

// Create a Modbus object for VFD control
ModbusMaster vfd;

// Adjustable parameters for system configuration and tuning
#define SWITCH_PIN 2 // Pin number for the system's start/stop switch
#define SENSOR_ADDRESS 0x28 // I2C address of the pressure sensor
#define SENSOR_STARTUP_TIME 3 // Startup time for the sensor in milliseconds
#define SENSOR_READ_INTERVAL 1000 // Time between sensor reads in milliseconds
#define BUZZER_PIN 8 // Pin number for the system's buzzer
#define ADJUSTMENT_INTERVAL 2000 // Reduced time between VFD adjustments in milliseconds for faster response
#define INITIAL_READINGS_COUNT 5 // Number of initial readings to average for EMA initialization
#define FREQUENCY_DEADBAND 0.05 // Decreased deadband for finer control
#define BUZZER_DEBOUNCE_DELAY 3000 // Debounce delay for the buzzer in milliseconds

// Variables for internal state management
unsigned long lastPrintTime = 0; // Time of the last debug print
unsigned long lastSensorReadTime = 0; // Time of the last sensor read
unsigned long lastAdjustmentTime = 0; // Time of the last VFD frequency adjustment
unsigned long startTime = 0; // Time when the VFD was started
bool vfdRunning = false;
bool controlStarted = false; // Flag to indicate if control has started
unsigned int initialReadings = 0;
float sumInitialPressures = 0;

// EMA filter settings
float emaPressure = 0;
const float alpha = 0.045; // Smoothing factor for EMA, adjustable for filter responsiveness

// Pressure-to-flow conversion and target settings
float latestPressure = 0;
float currentFlowRate = 0;
float frequencyHz = 30; // Ensure this is below 60Hz to start
float lastFlowRateError = 0; // Store the last flow rate error for derivative calculation
float integralError = 0; // Integral of the flow rate error
bool systemInitialized = false; // Add a flag to check system initialization
const float targetFlowRate = 20; // Target flow rate, adjustable
const float minFlowRate = targetFlowRate * 0.9425; // Lower flow rate boundary
const float maxFlowRate = targetFlowRate * 1.0575; // Upper flow rate boundary

// Control parameters
const float Kp = 0.054; // Proportional gain, adjustable
const float Ki = 0.0000001; // Integral gain, adjustable
const float Kd = 1.3; // Derivative gain, adjustable

// Buzzer Operation
unsigned long lastBuzzerActivationTime = 0; // Time when the frequency last reached 60Hz

void setup() {
  Serial.begin(9600); // Debug serial port
  Serial1.begin(9600); // Start Modbus communication on Serial1 for VFD control
  Wire.begin();
  Wire.setClock(400000); // I2C clock speed, adjustable for sensor compatibility

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH); // Ensure buzzer is off at start for active-low

  delay(SENSOR_STARTUP_TIME);

  vfd.begin(1, Serial1);
  Serial.println("Setup complete. Time (ms),Pressure (mbar),Flow Rate (m3/h),Frequency (Hz)");
}

void loop() {
  // Read the switch state; assuming LOW means 'ON'
  bool switchState = digitalRead(SWITCH_PIN) == LOW;
  unsigned long currentMillis = millis();

  // Manage VFD based on the switch state
  if (switchState && !vfdRunning) {
    // Start the VFD if the switch is 'ON' and the VFD isn't already running
    vfd.writeSingleRegister(0x2001, frequencyHz * 100); // Set the frequency of the VFD
    vfd.writeSingleRegister(0x2000, 0x0002); // VFD start command
    vfdRunning = true;
    startTime = currentMillis; // Record the start time
  } else if (!switchState && vfdRunning) {
    // Stop the VFD if the switch is 'OFF' and the VFD is running
    vfd.writeSingleRegister(0x2000, 0x0001); // VFD stop command
    vfdRunning = false;
    controlStarted = false; // Reset the control start flag
    digitalWrite(BUZZER_PIN, HIGH); // Turn off the buzzer for active-low buzzers
  }

  if (vfdRunning) {
    // Sensor reading should occur if the system is running.
    if (currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
      readPressureSensor();
      currentFlowRate = 15.079 * pow(emaPressure, 0.5921); // Placeholder calculation for flow rate

      Serial.print(millis());
      Serial.print(",");
      Serial.print(emaPressure);
      Serial.print(",");
      Serial.print(currentFlowRate);
      Serial.print(",");
      Serial.println(frequencyHz, 2);

      lastSensorReadTime = currentMillis;
    }

    // Start control after 240 seconds
    if (currentMillis - startTime >= 240000) {
      controlStarted = true;
    }

    if (controlStarted) {
      controlVFD(currentMillis); // Adjust VFD as needed based on current conditions
    }
  }
  checkBuzzer(); // Check whether conditions are met to turn the buzzer on or off
}

void readPressureSensor() {
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDRESS, 2);

  if (Wire.available() == 2) {
    uint16_t rawData = (Wire.read() << 8) | Wire.read();
    uint8_t status = (rawData >> 14) & 0x03;

    if (status == 0) { // Normal operation status
      latestPressure = calculatePressure(rawData & 0x3FFF);
      // EMA initialization with average of first few readings
      if (initialReadings < INITIAL_READINGS_COUNT) {
        sumInitialPressures += latestPressure;
        initialReadings++;
        if (initialReadings == INITIAL_READINGS_COUNT) {
          emaPressure = sumInitialPressures / INITIAL_READINGS_COUNT;
        }
      } else {
        emaPressure = alpha * latestPressure + (1 - alpha) * emaPressure; // Update EMA
      }
    } else {
      Serial.println("Sensor status error.");
    }
  } else {
    Serial.println("Sensor read error.");
  }
}

float calculatePressure(uint16_t sensorValue) {
  const float Output_max = 14745;
  const float Output_min = 1638;
  const float P_max = 40.0;
  const float P_min = -40.0;

  return (((sensorValue - Output_min) / (Output_max - Output_min)) * (P_max - P_min)) + P_min;
}

void controlVFD(unsigned long currentMillis) {
    // This function is now solely for VFD control logic, not for output
    if (currentMillis - lastAdjustmentTime >= ADJUSTMENT_INTERVAL) {
        float flowRateError = targetFlowRate - currentFlowRate;
        
        // Check if the flow rate is within the deadband range
        if (currentFlowRate < minFlowRate || currentFlowRate > maxFlowRate) {
            integralError += flowRateError * (currentMillis - lastAdjustmentTime); // Calculate integral of error
            float adjustmentFactor = (Kp * flowRateError) + (Ki * integralError) + (Kd * (flowRateError - lastFlowRateError) / ADJUSTMENT_INTERVAL);

            if (abs(adjustmentFactor) > FREQUENCY_DEADBAND) {
                frequencyHz += adjustmentFactor; // Adjust frequency
                frequencyHz = constrain(frequencyHz, 30, 60); // Keep frequency within bounds
                vfd.writeSingleRegister(0x2001, frequencyHz * 100); // Apply new frequency to VFD
                lastAdjustmentTime = currentMillis;
            }
            lastFlowRateError = flowRateError; // Update last flow rate error for next derivative calculation
        } else {
            // Reset the integral and derivative components when within the deadband
            integralError = 0;
            lastFlowRateError = 0;
        }
    }
}


void checkBuzzer() {
  // Check if the VFD is not running or the switch is 'OFF'; if so, ensure the buzzer is turned off
  if (!vfdRunning || digitalRead(SWITCH_PIN) == HIGH) { // Assuming LOW means 'ON'
    digitalWrite(BUZZER_PIN, HIGH); // Turn off buzzer for active-low buzzers
    return; // Exit the function as there's no need to check further conditions
  }
  
  // Check if the frequency has reached 60Hz
  if (frequencyHz >= 60) {
    // Start timing once frequency reaches 60Hz
    if (lastBuzzerActivationTime == 0) {
      lastBuzzerActivationTime = millis();
    } else if (millis() - lastBuzzerActivationTime >= BUZZER_DEBOUNCE_DELAY) {
      // Activate buzzer if the frequency has been at 60Hz for longer than the debounce delay
      digitalWrite(BUZZER_PIN, LOW); // Activate buzzer for active-low buzzers
    }
  } else {
    // Reset timing and turn off buzzer if frequency drops below 60Hz
    lastBuzzerActivationTime = 0;
    digitalWrite(BUZZER_PIN, HIGH); // Turn off buzzer for active-low buzzers
  }
}
