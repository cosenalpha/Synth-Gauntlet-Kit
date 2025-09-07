#include <mpu6050.h>
#include "MIDIUSB.h"
#define MPU_ADDRESS 0x68
#define MIDI_CHANNEL 0        // Default MIDI channel
#define MIDI_CONTROL 74       // Filter cutoff control number
#define UPDATE_INTERVAL 200    // Minimum interval between updates (ms)

// Sensor variables
float rawGX, rawGY, rawGZ;
float dpsGX, dpsGY, dpsGZ;
float gRoll, gPitch, gYaw;
float rawAX, rawAY, rawAZ;
float gForceAX, gForceAY, gForceAZ;
float aPitch, aRoll;
double gyroOffsetX, gyroOffsetY, gyroOffsetZ;
double accelOffsetX, accelOffsetY, accelOffsetZ;

float gX, gY, gZ; // initialise gyroscope variables
float aX, aY, aZ; // initialise accelerometer variables
float temp; // initialise temperature variables


// Pin definitions
const int buttonPin = 7;
const int FLEX_PIN = A2; // Pin connected to voltage divider output

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 11500.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg
float angle;

const int flexPin = A1;

float rotation;

// State tracking
int buttonState = 0;
int flexVal;
byte normalizedValue;
unsigned long lastUpdate = 0; // Track last update time

void setup() {
  Serial.begin(115200);
  
  // Initialize MPU6050
  wakeSensor(MPU_ADDRESS);
  delay(1000);
  calculateGyroOffset(MPU_ADDRESS, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  calculateAccelOffset(MPU_ADDRESS, accelOffsetX, accelOffsetY);
  
  // Initialize pins
  pinMode(buttonPin, INPUT);

  pinMode(FLEX_PIN, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensor values
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH && currentTime - lastUpdate >= UPDATE_INTERVAL) {
    readGesture();
    detectRotation();    
    // Handle flex sensor values
    flexVal = angle;
    midiComm(getMIDIEffect(flexVal), normalizedValue);
    lastUpdate = currentTime;
  }
}

void readGesture() {
  int flexADC = analogRead(FLEX_PIN);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
   angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);

  delay(500);
}

void detectRotation() {
  readGyroData(MPU_ADDRESS , gX, gY, gZ); // pass MPU6050 address and gyroscope values are written to 3 provided variables
  readAccelData(MPU_ADDRESS, aX, aY, aZ); // pass MPU6050 address and accelerometer values are written to 3 provided variables
  readTempData(MPU_ADDRESS, temp); // pass MPU6050 address and temperature values are written to 3 provided variables
  
  // readGyroData(MPU_ADDRESS, rawGX, rawGY, rawGZ);
  // rawGyroToDPS(rawGX, rawGY, rawGZ, dpsGX, dpsGY, dpsGZ);
  
  // Apply gyro offsets
  dpsGX -= gyroOffsetX;
  dpsGY -= gyroOffsetY;
  dpsGZ -= gyroOffsetZ;
  
  // Convert to angles
  dpsToAngles(dpsGX, dpsGY, dpsGZ, gPitch, gRoll, gYaw);
  
  // Handle accelerometer data
  readAccelData(MPU_ADDRESS, rawAX, rawAY, rawAZ);
  rawAccelToGForce(rawAX, rawAY, rawAZ, gForceAX, gForceAY, gForceAZ);
  calculateAnglesFromAccel(gForceAX, gForceAY, gForceAZ, aPitch, aRoll);
  
  // Map rotation to MIDI value (handles both positive and negative rotations)
   rotation = aRoll;
  normalizedValue = map((rotation), -20, 200, 0, 127);
}

byte getMIDIEffect(int flexVal) {
  if (flexVal < 10) {
    return 20;  // Reverb amount
  } else if (flexVal > 10 && flexVal < 50) {
    return 21;  // Delay feedback
  } else if (flexVal > 50) {
    return 22;  // Filter cutoff
  }
}

void midiComm(byte effect, byte value) {
  controlChange(MIDI_CHANNEL, effect, value);
  Serial.println("a");

  MidiUSB.flush();
  delay(UPDATE_INTERVAL);
  // Serial.println(flexVal);
}

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}