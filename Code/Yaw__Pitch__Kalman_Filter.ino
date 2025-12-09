#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <BMI088.h>     // Library: "Bolder Flight Systems BMI088"
#include <MS5611.h>     // Library: "MS5611" by Rob Tillaart
#include <RadioLib.h>   // Library: "RadioLib" by Jan Gromes

// ================================================================
//                       USER CONFIGURATION
// ================================================================

// --- LORA SETTINGS ---
// Frequency: 915.0 (US), 868.0 (EU), 433.0 (Asia)
#define LORA_FREQUENCY 915.0 

// --- SERVO PINS (Teensy 4.1) ---
const int servoPitchPin = 2; 
const int servoYawPin   = 3; 

// --- LORA PINS ---
#define LORA_CS   10
#define LORA_DIO0 4
#define LORA_RST  9
#define LORA_DIO1 5

// --- LAUNCH DETECT ---
// Gravity is 9.8. We set this to 13.0 to detect the gentle F15 liftoff.
const float LAUNCH_THRESHOLD = 13.0; 

// ================================================================
//                       OBJECTS & VARIABLES
// ================================================================

// Sensors
// BMI088 Address: 0x18 (Accel) / 0x68 (Gyro). If fails, try 0x19 / 0x69
Bmi088 bmi(Wire, 0x18, 0x68); 
MS5611 baro(0x77); // Default I2C address for MS5611
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1); 

// Servos
Servo servoPitch; 
Servo servoYaw;   

// PID Tuning (Adjust these via Serial if needed)
float KpPitch = 2.5, KiPitch = 0.0, KdPitch = 0.8;
float KpYaw   = 1.5, KiYaw   = 0.0, KdYaw   = 0.0;

// PID Variables
float errorPitch=0, prevErrorPitch=0, integralPitch=0;
float errorYaw=0, prevErrorYaw=0, integralYaw=0;
float derivativePitch=0, derivativeYaw=0;

// Servo Limits
int servoCenterPitch = 90;
int servoCenterYaw   = 90;
int servoMin = 60;
int servoMax = 120;

// Deadband (Don't move servos for tiny errors)
const float deadbandPitch = 1.0;
const float deadbandYaw   = 2.0;

// Timers
unsigned long lastServoUpdate = 0;
const int servoUpdateInterval = 20; // 50Hz update for Servos

unsigned long lastTelemetry = 0;
const int telemetryInterval = 100; // 10Hz LoRa update

unsigned long lastTime = 0;
float elapsedTime;

// State Machine
enum RocketState { IDLE, ARMED, VECTORING, LOCKED };
RocketState state = IDLE; // Starts in Safe Mode

// Orientation Data
float pitchOffset = 0;
float yawOffset   = 0;
float pitchFiltered = 0;
float yawFiltered   = 0;

// ================================================================
//                       KALMAN FILTER
// ================================================================
struct Kalman {
  float Q_angle; float Q_bias; float R_measure; 
  float angle; float bias; float rate;  
  float P[2][2];
};

void kalmanInit(Kalman &k) {
  k.Q_angle = 0.001f; k.Q_bias = 0.003f; k.R_measure = 0.03f;  
  k.angle = 0.0f; k.bias = 0.0f; k.rate = 0.0f;
  k.P[0][0] = 0.0f; k.P[0][1] = 0.0f; k.P[1][0] = 0.0f; k.P[1][1] = 0.0f;
}

float kalmanGetAngle(Kalman &k, float newAngle, float newRate, float dt) {
  k.rate = newRate - k.bias;
  k.angle += dt * k.rate;
  k.P[0][0] += dt * (dt*k.P[1][1] - k.P[0][1] - k.P[1][0] + k.Q_angle);
  k.P[0][1] -= dt * k.P[1][1];
  k.P[1][0] -= dt * k.P[1][1];
  k.P[1][1] += k.Q_bias * dt;
  float S = k.P[0][0] + k.R_measure; 
  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;
  float y = newAngle - k.angle;
  k.angle += K0 * y;
  k.bias  += K1 * y;
  float P00_temp = k.P[0][0];
  float P01_temp = k.P[0][1];
  k.P[0][0] -= K0 * P00_temp;
  k.P[0][1] -= K0 * P01_temp;
  k.P[1][0] -= K1 * P00_temp;
  k.P[1][1] -= K1 * P01_temp;
  return k.angle;
}

Kalman kalmanPitch;
Kalman kalmanYaw;

// ================================================================
//                       SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();       
  Wire.setClock(400000); // Fast I2C

  // 1. Attach Servos & Center Them
  servoPitch.attach(servoPitchPin);
  servoYaw.attach(servoYawPin);
  servoPitch.write(servoCenterPitch);
  servoYaw.write(servoCenterYaw);

  delay(1000); 

  // 2. Initialize BMI088 (IMU)
  Serial.println("Initializing BMI088...");
  int status = bmi.begin();
  if (status < 0) {
    Serial.println("BMI088 Initialization Failed!");
    Serial.println("Check Wiring or try 0x19 address.");
    while (1);
  }
  // Note: We removed setAccelRange to prevent compilation errors. 
  // The sensor defaults are high performance and will work for F15.

  // 3. Initialize MS5611 (Barometer)
  Serial.println("Initializing Barometer...");
  if (!baro.begin()) {
    Serial.println("Barometer not found (Check wiring). Continuing anyway...");
  }

  // 4. Initialize LoRa
  Serial.println("Initializing LoRa...");
  if (radio.begin(LORA_FREQUENCY) == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa Ready.");
    radio.setOutputPower(20); 
  } else {
    Serial.println("LoRa Failed!");
  }

  // 5. Calibration
  Serial.println("Calibrating IMU (Keep Rocket Still)...");
  float sumPitch=0, sumYaw=0;
  const int samples = 500;
  for(int i=0; i<samples; i++){
    bmi.readSensor();
    float ax = bmi.getAccelX_mss();
    float ay = bmi.getAccelY_mss();
    float az = bmi.getAccelZ_mss();
    float pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180.0/PI;
    float yaw   = atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI;
    sumPitch += pitch;
    sumYaw   += yaw;
    delay(2);
  }
  pitchOffset = sumPitch / samples;
  yawOffset   = sumYaw / samples;
  
  // 6. Init Kalman Filter
  kalmanInit(kalmanPitch);
  kalmanInit(kalmanYaw);

  // Seed Filter with current angle
  bmi.readSensor();
  float ax = bmi.getAccelX_mss();
  float ay = bmi.getAccelY_mss();
  float az = bmi.getAccelZ_mss();
  kalmanPitch.angle = (atan2(ax, sqrt(ay*ay + az*az)) * 180.0/PI) - pitchOffset;
  kalmanYaw.angle   = (atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI) - yawOffset;

  lastTime = micros();
  
  // 7. Ready
  state = IDLE; 
  Serial.println("----------------------------------------");
  Serial.println("SYSTEM READY. MODE: IDLE (SAFE).");
  Serial.println("Type 'arm' in Serial Monitor to activate.");
  Serial.println("----------------------------------------");
}

// ================================================================
//                       MAIN LOOP
// ================================================================
void loop() {
  // Time keeping
  unsigned long now = micros();
  elapsedTime = (now - lastTime) / 1000000.0; 
  lastTime = now;
  // Prevent division by zero or huge jumps
  if(elapsedTime <= 0) elapsedTime = 0.001;
  if(elapsedTime > 0.05) elapsedTime = 0.05;

  // --- 1. READ SERIAL COMMANDS ---
  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Arming Command
    if(cmd.equalsIgnoreCase("arm")){
      if(state == IDLE){
        // Only arm if vertical (+/- 20 deg)
        if(abs(pitchFiltered) < 20.0 && abs(yawFiltered) < 20.0) {
           state = ARMED;
           Serial.println(">>> ROCKET ARMED <<< Waiting for Launch (13 m/s^2)...");
        } else {
           Serial.println("ERROR: Rocket tilted! Cannot Arm.");
        }
      }
    }
    
    // Abort Command
    if(cmd.equalsIgnoreCase("abort")){
      state = IDLE;
      Serial.println("ABORTED! Returning to IDLE.");
    }
  }

  // --- 2. READ SENSORS ---
  bmi.readSensor();
  float ax = bmi.getAccelX_mss();
  float ay = bmi.getAccelY_mss();
  float az = bmi.getAccelZ_mss(); // Z is UP

  // Calculate Accelerometer Angles
  float accPitch = (atan2(ax, sqrt(ay*ay + az*az)) * 180.0/PI) - pitchOffset;
  float accYaw   = (atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI) - yawOffset;

  // Get Gyro Rates (Rad/s -> Deg/s)
  float gyroPitchRate = bmi.getGyroX_rads() * 180.0/PI; 
  float gyroYawRate   = bmi.getGyroY_rads() * 180.0/PI;

  // --- 3. FILTERING (KALMAN) ---
  pitchFiltered = kalmanGetAngle(kalmanPitch, accPitch, gyroPitchRate, elapsedTime);
  yawFiltered   = kalmanGetAngle(kalmanYaw,   accYaw,   gyroYawRate,   elapsedTime);

  float pitch = pitchFiltered;
  float yaw   = yawFiltered;

  // Deadband
  if(abs(pitch) < deadbandPitch) pitch = 0;
  if(abs(yaw)   < deadbandYaw)   yaw   = 0;

  // --- 4. STATE MACHINE ---
  switch(state){
    case IDLE:
      servoPitch.write(servoCenterPitch);
      servoYaw.write(servoCenterYaw);
      integralPitch = 0;
      integralYaw = 0;
      break;

    case ARMED:
      servoPitch.write(servoCenterPitch);
      servoYaw.write(servoCenterYaw);
      
      // LAUNCH DETECT (Threshold 13 m/s^2 for F15)
      // Check absolute value of Z accel
      if (abs(az) > LAUNCH_THRESHOLD) {
         state = VECTORING;
         Serial.println("LAUNCH DETECTED! TVC ACTIVE!");
      }
      break;

    case VECTORING: 
    { // <--- START CURLY BRACE FOR VECTORING CASE
      // --- PITCH PID ---
      errorPitch = 0 - pitch;
      integralPitch += errorPitch * elapsedTime;
      integralPitch = constrain(integralPitch, -15, 15); // Anti-windup
      derivativePitch = (errorPitch - prevErrorPitch) / elapsedTime;
      
      float outputPitch = (KpPitch * errorPitch) + (KiPitch * integralPitch) + (KdPitch * derivativePitch);
      prevErrorPitch = errorPitch;

      // --- YAW PID ---
      errorYaw = 0 - yaw;
      integralYaw += errorYaw * elapsedTime;
      integralYaw = constrain(integralYaw, -15, 15);
      derivativeYaw = (errorYaw - prevErrorYaw) / elapsedTime;
      
      float outputYaw = (KpYaw * errorYaw) + (KiYaw * integralYaw) + (KdYaw * derivativeYaw);
      prevErrorYaw = errorYaw;

      // --- SERVO MIXING ---
      int servoAnglePitch = constrain(servoCenterPitch + outputPitch, servoMin, servoMax);
      int servoAngleYaw   = constrain(servoCenterYaw   + outputYaw,   servoMin, servoMax);

      // Update Servos (limit update rate to prevent jitter)
      if(millis() - lastServoUpdate >= servoUpdateInterval){
        servoPitch.write(servoAnglePitch);
        servoYaw.write(servoAngleYaw);
        lastServoUpdate = millis();
      }
      break;
    } // <--- END CURLY BRACE FOR VECTORING CASE

    case LOCKED:
      break;
  }

  // --- 5. TELEMETRY (LORA) ---
  if (millis() - lastTelemetry >= telemetryInterval) {
    // Read altitude only when needed (saves CPU time)
    // Formula: 44330 * (1 - (p/p0)^(1/5.255))
    float pressure = baro.getPressure();
    float altitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));
    
    String telem = "";
    telem += String(millis()); telem += ",";
    telem += String(pitchFiltered, 2); telem += ",";
    telem += String(yawFiltered, 2); telem += ",";
    telem += String(altitude, 1); telem += ",";
    telem += String(state); // 0=IDLE, 1=ARMED, 2=VECTORING
    
    // Send Packet
    radio.transmit(telem);
    
    lastTelemetry = millis();
  }
}