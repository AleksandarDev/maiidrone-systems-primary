#include <mpu.h>
#include <Arduino.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <math.h>

// ================================================================
// ===                        SYSTEM                            ===
// ================================================================

#define SYSTEM_PRIORITY       0       // The system priority (primary - 0, secondary - 1, tertiary - 2)
#define VERSION_MAJOR         0
#define VERSION_MINOR         1
#define VERSION_PATCH         0

// ================================================================
// ===                     CONFIGURATION                        ===
// ================================================================

#define DEBUG_YPR
//#define DEBUG_PID
//#define DEBUG_SPD

// Pin configuration
#define BUZZER_PIN            3           // Indicator buzzer
#define ESC_A_PIN             5           // ESC A pin (~PWM pin)
#define ESC_B_PIN             6           // ESC B pin (~PWM pin)
#define ESC_C_PIN             9           // ESC C pin (~PWM pin)
#define ESC_D_PIN             10          // ESC D pin (~PWM pin)
#define LED_PIN               LED_BUILTIN // LED pin (indicator)

// Controls configuration
#define CONTROL_PITCH_MIN     1000
#define CONTROL_PITCH_MAX     2000
#define CONTROL_ROLL_MIN      1000
#define CONTROL_ROLL_MAX      2000
#define CONTROL_YAW_MIN       1000
#define CONTROL_YAW_MAX       2000
#define CONTROL_VERTICAL_MIN  1000
#define CONTROL_VERTICAL_MAX  2000

#define CONTROL_PITCH_DEG_MIN    -20          // Control min. pitch (mapped from CONTROL_PITCH_MIN)
#define CONTROL_PITCH_DEG_MAX     20          // Control max. pitch (mapped from CONTROL_PITCH_MAX)
#define CONTROL_ROLL_DEG_MIN     -20          // Control min. roll (mapped from CONTROL_ROLL_MIN)
#define CONTROL_ROLL_DEG_MAX      20          // Control max. roll (mapped from CONTROL_ROLL_MAX)
#define CONTROL_YAW_DEG_MIN      -180         // Control min. yaw (mapped from CONTROL_YAW_MIN)
#define CONTROL_YAW_DEG_MAX       180         // Control max. yaw (mapped from CONTROL_YAW_MAX)

// ESC configuration
#define ESC_MIN               1000        // ESC min PWM microseconds value
#define ESC_MAX               2000        // ESC max PWM microseconds value
#define ESC_ARM_DELAY         5000        // ESC arm delay milliseconds
#define ESC_UPDATE_FREQ       1000        // ESC speed update frequency (1000us = 100Hz)

// PID configuration
#define PITCH_P_VAL           1
#define PITCH_I_VAL           0.1
#define PITCH_D_VAL           0.2

#define ROLL_P_VAL            1
#define ROLL_I_VAL            0.1
#define ROLL_D_VAL            0.2

#define YAW_P_VAL             1
#define YAW_I_VAL             0.1
#define YAW_D_VAL             0.2

#define PID_PITCH_INFLUENCE   10        // Control to PID degrees pitch influence
#define PID_ROLL_INFLUENCE    10        // Control to PID degrees roll influence
#define PID_YAW_INFLUENCE     10        // Control to PID degrees yaw influence

// ================================================================
// ===                        VARIABLES                         ===
// ================================================================

// Indicators vars
unsigned long buzzerEndMillis;

// Control vars
float pitchControl, rollControl, yawControl, verticalControl;

// MPU control/status vars
bool          dmpReady = false;     // set true if DMP init was successful
//MPU6050       mpu;                  // the MPU
//uint8_t       mpuIntStatus;         // holds actual interrupt status byte from MPU
//uint8_t       devStatus;            // return status after each device operation (0 = success, !0 = error)
//uint16_t      packetSize;           // expected DMP packet size (default is 42 bytes)
//uint16_t      fifoCount;            // count of all bytes currently in FIFO
//uint8_t       fifoBuffer[64];       // FIFO storage buffer
//Quaternion    q;                    // [w, x, y, z]         quaternion container
//VectorFloat   gravity;              // [x, y, z]            gravity vector
float         ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
int ret, retc;

// Motors vars
unsigned long lastUpdateMicros;
Servo escA, escB, escC, escD;       // ESC A, B, C and D
bool escReady = false;              // set true if ESC init was successful
int speedEscA, speedEscB, speedEscC, speedEscD;
int speedAC, speedBD;

// PID vars
float balanceAC, balanceBD, balanceAxis;
PID pitchPid(&ypr[1], &balanceBD, &pitchControl, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollPid(&ypr[2], &balanceAC, &rollControl, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawPid(&ypr[0], &balanceAxis, &yawControl, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);


// ================================================================
// ===                       DECLARATIONS                       ===
// ================================================================

// Hardware
void initI2c();
void initStatusIndicators();
void indicatorsLoop();
void activateBuzzer(int pitch, unsigned long duration);
void activateBuzzerSync(int pitch, unsigned long duration);

// Serial
void initSerial();
void serialBufferFree();

// PID
void initPID();
void computePID();

// Gyro
void initGyro();
void dmpDataReady();
void getGyroData();

// ESC
void initEscs();
void calculateEscSpeeds();
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
  Fastwire::setup(400,0);
}

void initStatusIndicators() {
  // Configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // Buzzer initial value
  analogWrite(BUZZER_PIN, 0);
}


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
//void dmpDataReady() {
//    mpuInterrupt = true;
//}


// ================================================================
// ===                           PID                            ===
// ================================================================
void initPID() {
  pitchPid.SetMode(AUTOMATIC);
  pitchPid.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);

  rollPid.SetMode(AUTOMATIC);
  rollPid.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);

  yawPid.SetMode(AUTOMATIC);
  yawPid.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

  activateBuzzerSync(2000, 100);
}

void computePID() {
  pitchPid.Compute();
  rollPid.Compute();
  yawPid.Compute();

#ifdef DEBUG_PID
  // TODO: Remove (debugging only)
  // TODO: Send proper message packet when requested
  Serial.print("PID YPR:\t");
  Serial.print(balanceAxis);
  Serial.print("\t");
  Serial.print(balanceBD);
  Serial.print("\t");
  Serial.println(balanceAC);
#endif // DEBUG_PID
}

// ================================================================
// ===                      GYRO METHODS                        ===
// ================================================================
void initGyro() {
  ret = mympu_open(200, (uint8_t)136);
  while(ret != 0) {Serial.println("MPU INIT FAILED"); delay(1000);}
  dmpReady = true;

  activateBuzzerSync(2000, 100);

/*
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
  */
}

void getGyroData() {
  /*
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
    */

    // TODO: Check result == 0 (success)
    ret =  mympu_update();
    retc = mympu_update_compass();

    switch (ret) {
	case 0: c++; break;
	case 1: np++; return;
	case 2: err_o++; return;
	case 3: err_c++; return;
	default:
		Serial.print("READ ERROR!  ");
		Serial.println(ret);
		return;
    }

    ypr[0] = mympu.ypr[0];
    ypr[1] = mympu.ypr[1];
    ypr[2] = mympu.ypr[2];

    // Convert to degrees
    // TODO: Remove and adjust PID values (this is only for debugging)
    //ypr[0] = ypr[0] * 180/M_PI;
    //ypr[1] = ypr[1] * 180/M_PI;
    //ypr[2] = ypr[2] * 180/M_PI;

    // TODO: Remove (debugging only)
    // TODO: Send proper message packet when requested
#ifdef DEBUG_YPR
    if (!(c%5)) {
      Serial.print(np);
      Serial.print(" "); Serial.print(err_c);
      Serial.print(" "); Serial.print(err_o);
      Serial.print(" Y:\t"); Serial.print(ypr[0]);
      Serial.print(" P:\t"); Serial.print(ypr[1]);
      Serial.print(" R:\t"); Serial.print(ypr[2]);
      Serial.println();
    }
#endif // DEBUG_YPR
  //}
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

  // Apply small delay before arming vis buzzer sync function delay((duration + 10)*2)
  activateBuzzerSync(2000, 100);

  // Set to minimum speed (arm)
  escA.writeMicroseconds(ESC_MIN);
  escB.writeMicroseconds(ESC_MIN);
  escC.writeMicroseconds(ESC_MIN);
  escD.writeMicroseconds(ESC_MIN);

  // Apply arm delay (4 ESCs * (duration + 10)*2)
  activateBuzzerSync(3000, ESC_ARM_DELAY / 8);
  activateBuzzerSync(3000, ESC_ARM_DELAY / 8);
  activateBuzzerSync(3000, ESC_ARM_DELAY / 8);
  activateBuzzerSync(3000, ESC_ARM_DELAY / 8);

  escReady = true;
}

void calculateEscSpeeds() {
  const int mult = 10;

  speedAC = verticalControl * (abs(-(PID_ROLL_INFLUENCE * mult) + balanceAxis) / (PID_ROLL_INFLUENCE * mult));
  speedBD = verticalControl * (abs((PID_PITCH_INFLUENCE * mult) + balanceAxis) / (PID_PITCH_INFLUENCE * mult));

  speedEscA = (((PID_ROLL_INFLUENCE * mult) + balanceAC) / (PID_ROLL_INFLUENCE * mult)) * speedAC;
  speedEscB = (((PID_PITCH_INFLUENCE * mult) + balanceBD) / (PID_PITCH_INFLUENCE * mult)) * speedBD;
  speedEscC = (abs((-(PID_ROLL_INFLUENCE * mult) + balanceAC) / (PID_ROLL_INFLUENCE * mult))) * speedAC;
  speedEscD = (abs((-(PID_PITCH_INFLUENCE * mult) + balanceBD) / (PID_PITCH_INFLUENCE * mult))) * speedBD;

  // TODO: Remove (debugging only)
  // TODO: Send proper message packet when requested
#ifdef DEBUG_SPD
  Serial.print("Speed ABCD:\t");
  Serial.print(speedEscA);
  Serial.print("\t");
  Serial.print(speedEscB);
  Serial.print("\t");
  Serial.print(speedEscC);
  Serial.print("\t");
  Serial.println(speedEscD);
#endif // DEBUG_SPD
}

void updateEscSpeeds() {
  // Don't update if it's not time yet (default 100Hz, check ESC_UPDATE_FREQ for configuration)
  // This will handle overflow by updating speed anyway
  unsigned long currentMicros = micros();
  if (lastUpdateMicros > currentMicros ||
      currentMicros - lastUpdateMicros > ESC_UPDATE_FREQ)
    lastUpdateMicros = currentMicros;
  else return;

  speedEscA = 1000 + min(speedEscA, 600);
  speedEscB = 1000 + min(speedEscB, 600);
  speedEscC = 1000 + min(speedEscC, 600);
  speedEscD = 1000 + min(speedEscD, 600);

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
  //return mpuInterrupt;
  return false;
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
  while(Serial.available() > 0 && !anyInterrupt()) {
    pitchControl = 0;
    rollControl = 0;
    yawControl = 0;

    if (verticalControl > 0)
      verticalControl = 0;
    else verticalControl = 350;
    Serial.read();
  }
}

void indicatorsLoop() {
  // Handle buzzer
  if (buzzerEndMillis >= millis()) {
    buzzerEndMillis = 0;
    analogWrite(BUZZER_PIN, 0);
  }
}

void activateBuzzerSync(int pitch, unsigned long duration) {
  activateBuzzer(pitch, duration);
  delay(duration + 10);
  analogWrite(BUZZER_PIN, 0);
  delay(duration + 10);
}

void activateBuzzer(int pitch, unsigned long duration) {
  buzzerEndMillis = millis() + duration;
  analogWrite(BUZZER_PIN, pitch);
}

void loop() {
  // if setup failed, don't try to do anything
  while (!allReady()) {
    indicatorsLoop();
  }

  programLoop();
  getGyroData();
  computePID();
  calculateEscSpeeds();
  updateEscSpeeds();
  indicatorsLoop();
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  initI2c();
  initSerial();
  initStatusIndicators();
  initGyro();
  initPID();
  initEscs();

  activateBuzzerSync(956, 500);
}
