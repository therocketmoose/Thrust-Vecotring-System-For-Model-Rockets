#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <BMI088.h>     // Install "Bolder Flight Systems BMI088"
#include <MS5611.h>     // Install "MS5611" by Rob Tillaart
#include <RadioLib.h>   // Install "RadioLib"

// --- HARDWARE PIN DEFINITIONS (TEENSY 4.1) ---
// I2C: SDA=18, SCL=19
const int servoPitchPin = 2; // Moved from 18 to avoid I2C conflict
const int servoYawPin   = 3; // Moved from 19 to avoid I2C conflict

// LoRa Pins (Standard SX127x setup, adjust for your specific module)
#define LORA_CS   10
#define LORA_DIO0 4
#define LORA_RST  9
#define LORA_DIO1 5

// --- OBJECT INSTANTIATION ---
/* BMI088 address is usually 0x18 (Accel) and 0x68 (Gyro)
   If you have jumper issues, try 0x19 and 0x69 */
Bmi088 bmi(Wire, 0x18, 0x68); 
MS5611 baro(0x77); // Check if your MS5611 is 0x77 or 0x76
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1); 

Servo servoPitch; 
Servo servoYaw;   

// --- PID & CONTROL VARIABLES ---
float KpPitch = 2.5, KiPitch = 0.0, KdPitch = 0.8;
float KpYaw   = 1.5, KiYaw   = 0.0, KdYaw   = 0.0;

float errorPitch=0, prevErrorPitch=0, integralPitch=0;
float errorYaw=0, prevErrorYaw=0, integralYaw=0;
float derivativePitch=0, derivativeYaw=0;

int servoCenterPitch = 90;
int servoCenterYaw   = 90;
int servoMin = 60;
int servoMax = 120;

const float deadbandPitch = 2.0;
const float deadbandYaw   = 4.0;

unsigned long lastServoUpdate = 0;
const int servoUpdateInterval = 20; // 50Hz Servo update

unsigned long lastTime = 0;
float elapsedTime;

// LoRa Timer
unsigned long lastTelemetry = 0;
const int telemetryInterval = 100; // Send data every 100ms (10Hz)

enum RocketState { IDLE, ARMED, VECTORING, LOCKED };
RocketState state = IDLE;

float pitchOffset = 0;
float yawOffset   = 0;
float groundAltitude = 0;

// Filtered Data
float pitchFiltered = 0;
float yawFiltered   = 0;

// ---------- Kalman filter implementation ----------
struct Kalman {
  float Q_angle;   
  float Q_bias;    
  float R_measure; 
  float angle; 
  float bias;  
  float rate;  
  float P[2][2];
};

void kalmanInit(Kalman &k) {
  k.Q_angle   = 0.001f; 
  k.Q_bias    = 0.003f; 
  k.R_measure = 0.03f;  
  k.angle = 0.0f; k.bias  = 0.0f; k.rate  = 0.0f;
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
// ---------- End Kalman ----------

void setup() {
  Serial.begin(115200);
  Wire.begin();       // Teensy 4.1 uses Pins 18/19 for main Wire
  Wire.setClock(400000); // Fast I2C mode

  // --- 1. Initialize Servos ---
  servoPitch.attach(servoPitchPin);
  servoYaw.attach(servoYawPin);
  servoPitch.write(servoCenterPitch);
  servoYaw.write(servoCenterYaw);

  delay(1000); // Wait for sensors to power up

  // --- 2. Initialize BMI088 ---
  Serial.println("Initializing BMI088...");
  int status = bmi.begin();
  if (status < 0) {
    Serial.println("BMI088 Initialization Failed!");
    Serial.print("Error Code: "); Serial.println(status);
    while (1);
  }
  
  // Set ranges (Adjust based on your rocket's vibration/dynamics)
  bmi.setAccelRange(Bmi088::ACCEL_RANGE_6G);
  bmi.setGyroRange(Bmi088::GYRO_RANGE_1000DPS);

  // --- 3. Initialize MS5611 ---
  Serial.println("Initializing MS5611...");
  if (!baro.begin()) {
    Serial.println("MS5611 not found!");
    // We don't halt here, just warn, in case baro is optional for flight logic
  }

  // --- 4. Initialize LoRa ---
  Serial.println("Initializing LoRa...");
  // 915.0 for US, 433.0 for EU/Asia
  int stateRadio = radio.begin(915.0); 
  if (stateRadio == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa Init Success!");
    radio.setOutputPower(20); // Max power (check local laws)
  } else {
    Serial.print("LoRa Failed, code ");
    Serial.println(stateRadio);
    while (1);
  }

  // --- 5. Calibration ---
  Serial.println("Calibrating Gyro/Accel Offsets...");
  float sumPitch=0, sumYaw=0;
  const int samples = 500;
  
  for(int i=0; i<samples; i++){
    bmi.readSensor();
    
    // BMI088 returns m/s^2. We need to convert to orientation.
    float ax = bmi.getAccelX_mss();
    float ay = bmi.getAccelY_mss();
    float az = bmi.getAccelZ_mss();
    
    // Calculate Pitch/Yaw from Accel (Unit conversion not needed for atan2)
    float pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180.0/PI;
    float yaw   = atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI;
    
    sumPitch += pitch;
    sumYaw   += yaw;
    delay(2);
  }
  
  pitchOffset = sumPitch / samples;
  yawOffset   = sumYaw / samples;
  
  // Get Ground Altitude
  groundAltitude = baro.getPressure(); // Simplify: using pressure as baseline or readAltitude()
  
  Serial.print("Offsets -> Pitch: "); Serial.print(pitchOffset);
  Serial.print(" Yaw: "); Serial.println(yawOffset);

  kalmanInit(kalmanPitch);
  kalmanInit(kalmanYaw);

  // Seed Kalman
  bmi.readSensor();
  float ax = bmi.getAccelX_mss();
  float ay = bmi.getAccelY_mss();
  float az = bmi.getAccelZ_mss();
  kalmanPitch.angle = (atan2(ax, sqrt(ay*ay + az*az)) * 180.0/PI) - pitchOffset;
  kalmanYaw.angle   = (atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI) - yawOffset;

  lastTime = micros();
  state = ARMED; 
}

void loop() {
  unsigned long now = micros();
  elapsedTime = (now - lastTime) / 1000000.0; // Convert micros to seconds
  lastTime = now;

  // Prevent large dt spikes
  if(elapsedTime > 0.05) elapsedTime = 0.05;

  // --- READ SENSORS ---
  bmi.readSensor();
  
  // 1. Accelerometer Angles
  float ax = bmi.getAccelX_mss();
  float ay = bmi.getAccelY_mss();
  float az = bmi.getAccelZ_mss();

  float accPitch = (atan2(ax, sqrt(ay*ay + az*az)) * 180.0/PI) - pitchOffset;
  float accYaw   = (atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI) - yawOffset;

  // 2. Gyro Rates
  // BMI Library returns radians/sec, convert to deg/s for your PID/Kalman logic
  float gyroPitchRate = bmi.getGyroX_rads() * 180.0/PI; 
  float gyroYawRate   = bmi.getGyroY_rads() * 180.0/PI;
  float gyroRollRate  = bmi.getGyroZ_rads() * 180.0/PI;

  // 3. Barometer (Read occasionally to save time, or every loop if fast enough)
  // baro.read() can be slow depending on oversampling settings. 
  // For high-speed TVC, consider reading baro only every 20-50ms.
  
  // --- FILTERING ---
  pitchFiltered = kalmanGetAngle(kalmanPitch, accPitch, gyroPitchRate, elapsedTime);
  yawFiltered   = kalmanGetAngle(kalmanYaw,   accYaw,   gyroYawRate,   elapsedTime);

  float pitch = pitchFiltered;
  float yaw   = yawFiltered;

  // Deadband
  if(abs(pitch) < deadbandPitch) pitch = 0;
  if(abs(yaw)   < deadbandYaw)   yaw   = 0;

  // --- STATE MACHINE ---
  switch(state){
    case IDLE:
      servoPitch.write(servoCenterPitch);
      servoYaw.write(servoCenterYaw);
      break;

    case ARMED:
      state = VECTORING;
      break;

    case VECTORING:
      // PITCH PID
      errorPitch = 0 - pitch; // Target is 0
      integralPitch += errorPitch * elapsedTime;
      integralPitch = constrain(integralPitch, -15, 15); // Anti-windup
      derivativePitch = (errorPitch - prevErrorPitch) / elapsedTime;
      
      float outputPitch = (KpPitch * errorPitch) + (KiPitch * integralPitch) + (KdPitch * derivativePitch);
      prevErrorPitch = errorPitch;

      // YAW PID
      errorYaw = 0 - yaw;
      integralYaw += errorYaw * elapsedTime;
      integralYaw = constrain(integralYaw, -15, 15);
      derivativeYaw = (errorYaw - prevErrorYaw) / elapsedTime;
      
      float outputYaw = (KpYaw * errorYaw) + (KiYaw * integralYaw) + (KdYaw * derivativeYaw);
      prevErrorYaw = errorYaw;

      // Apply to Servos
      int servoAnglePitch = constrain(servoCenterPitch + outputPitch, servoMin, servoMax);
      int servoAngleYaw   = constrain(servoCenterYaw   + outputYaw,   servoMin, servoMax);

      // Servo Update Timer
      if(millis() - lastServoUpdate >= servoUpdateInterval){
        servoPitch.write(servoAnglePitch);
        servoYaw.write(servoAngleYaw);
        lastServoUpdate = millis();
      }
      break;

    case LOCKED:
      break;
  }

  // --- LORA TELEMETRY ---
  // Send data at 10Hz (every 100ms) to avoid blocking the PID loop
  if (millis() - lastTelemetry >= telemetryInterval) {
    baro.read(1); // Read baro now
    float altitude = baro.getAltitude(baro.getPressure(), baro.getTemperature());
    
    String telem = "";
    telem += String(millis());
    telem += ",";
    telem += String(pitchFiltered, 2);
    telem += ",";
    telem += String(yawFiltered, 2);
    telem += ",";
    telem += String(altitude, 1);
    telem += ",";
    telem += String(state);
    
    // Send packet
    radio.transmit(telem);
    
    // Check for incoming commands (PIDs)
    // Note: Receive is blocking in basic mode. Ideally use Interrupts for receive.
    // For now, we focus on transmitting.
    
    lastTelemetry = millis();
  }
}