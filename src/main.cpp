#include <MPU6050_6Axis_MotionApps20.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <math.h>

// ================================================================
// ===                        SYSTEM                            ===
// ================================================================

#define SYSTEM_PRIORITY   0       // The system priority (primary - 0, secondary - 1, tertiary - 2)
#define VERSION_MAJOR     0
#define VERSION_MINOR     1
#define VERSION_PATCH     0

// ================================================================
// ===                     CONFIGURATION                        ===
// ================================================================

// Pin configuration
#define INTERRUPT_PIN   2           // MPU interrupt pin (INT pin)
#define ESC_A_PIN       5           // ESC A pin (~PWM pin)
#define ESC_B_PIN       6           // ESC B pin (~PWM pin)
#define ESC_C_PIN       9           // ESC C pin (~PWM pin)
#define ESC_D_PIN       10          // ESC D pin (~PWM pin)
#define LED_PIN         LED_BUILTIN // LED pin (indicator)

// ESC configuration
#define ESC_MIN         1000        // ESC min PWM microseconds value
#define ESC_MAX         1500        // ESC max PWM microseconds value
#define ESC_ARM_DELAY   5000        // ESC arm delay milliseconds

// ================================================================
// ===                        VARIABLES                         ===
// ================================================================

// MPU control/status vars
bool          dmpReady = false;     // set true if DMP init was successful
MPU6050       mpu;                  // the MPU
uint8_t       mpuIntStatus;         // holds actual interrupt status byte from MPU
uint8_t       devStatus;            // return status after each device operation (0 = success, !0 = error)
uint16_t      packetSize;           // expected DMP packet size (default is 42 bytes)
uint16_t      fifoCount;            // count of all bytes currently in FIFO
uint8_t       fifoBuffer[64];       // FIFO storage buffer
Quaternion    q;                    // [w, x, y, z]         quaternion container
VectorFloat   gravity;              // [x, y, z]            gravity vector
float         ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

// Motors vars
Servo escA, escB, escC, escD;       // ESC A, B, C and D
bool escReady = false;              // set true if ESC init was successful
int speedEscA, speedEscB, speedEscC, speedEscD;


// ================================================================
// ===                       DECLARATIONS                       ===
// ================================================================

// Hardware
void initI2c();
void initStatusIndicators();

// Serial
void initSerial();
void serialBufferFree();

// Gyro
void initGyro();
void dmpDataReady();
void getGyroData();

// ESC
void initEscs();
void updateEscSpeeds();

// Program
void programLoop();
bool anyInterrupt();
bool allReady();

// ================================================================
// ===                          SERIAL                          ===
// ================================================================

void initSerial() {
  // initialize serial communication
  Serial.begin(115200);                 // 115200 baud rate
  Serial.setTimeout(20);                // 20ms serial read timeout
}

void serialBufferFree() {
  while (Serial.available() && Serial.read());
}

// ================================================================
// ===                         HARDWARE                         ===
// ================================================================

void initI2c() {
  Fastwire::setup(400, true);
}

void initStatusIndicators() {
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      GYRO METHODS                        ===
// ================================================================
void initGyro() {
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // TODO: Maybe include calibration code and run it on request 
    mpu.setXAccelOffset(-1373);
    mpu.setYAccelOffset(-445);
    mpu.setZAccelOffset(657);
    mpu.setXGyroOffset(159);
    mpu.setYGyroOffset(-11);
    mpu.setZGyroOffset(-12);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void getGyroData() {
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // TODO: Send proper message packet
    Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display ypr angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // TODO: Remove (debugging only)
    // TODO: Send proper message packet when requested
    Serial.print("ypr\t");
    Serial.print(ypr[0]);
    Serial.print("\t");
    Serial.print(ypr[1]);
    Serial.print("\t");
    Serial.println(ypr[2]);
  }
}

// ================================================================
// ===                           ESC                            ===
// ================================================================

void initEscs() {
  // Attach ESC pins
  escA.attach(ESC_A_PIN);
  escB.attach(ESC_B_PIN);
  escC.attach(ESC_C_PIN);
  escD.attach(ESC_D_PIN);

  // Apply small delay before arming
  delay(100);

  // Set to minimum speed (arm)
  escA.writeMicroseconds(ESC_MIN);
  escB.writeMicroseconds(ESC_MIN);
  escC.writeMicroseconds(ESC_MIN);
  escD.writeMicroseconds(ESC_MIN);

  // Apply arm delay
  delay(ESC_ARM_DELAY);

  escReady = true;
}

void updateEscSpeeds() {
  // Apply new speeds
  escA.writeMicroseconds(speedEscA);
  escC.writeMicroseconds(speedEscB);
  escB.writeMicroseconds(speedEscC);
  escD.writeMicroseconds(speedEscD);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

bool anyInterrupt() {
  return mpuInterrupt;
}

bool allReady() {
  return dmpReady && escReady;
}

void programLoop() {
  // NOTE: "divide and conquer"
  // you can frequently test in between other stuff to
  // see if anyInterrupt() is true, and if so, "return;"
  // to immediately process the sensor data

  // Read serial data
  while(!anyInterrupt()) {
  }
}

void loop() {
  // if setup failed, don't try to do anything
  if (!allReady())
      return;

  // wait for MPU interrupt or extra packet(s) available
  // in the meantime - process peripheries (non essential)
  while (!mpuInterrupt && fifoCount < packetSize)
    programLoop();

  getGyroData();
  updateEscSpeeds();
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  initI2c();
  initSerial();
  initStatusIndicators();
  initGyro();
  initEscs();
}
