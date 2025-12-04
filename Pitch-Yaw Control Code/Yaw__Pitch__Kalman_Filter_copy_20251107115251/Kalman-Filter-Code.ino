#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050.h>

MPU6050 mpu;
Servo servoPitch; // GPIO18 (top servo)
Servo servoYaw;   // GPIO19 (bottom servo)

const int servoPitchPin = 18;
const int servoYawPin   = 19;

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
const int servoUpdateInterval = 20; 
unsigned long lastTime = 0;
float elapsedTime;

enum RocketState { IDLE, ARMED, VECTORING, LOCKED };
RocketState state = IDLE;

float pitchOffset = 0;
float yawOffset   = 0;

// Remove complementary filter alphas (we'll use Kalman)
float pitchFiltered = 0;
float yawFiltered   = 0;

// ---------- Kalman filter implementation (2-state: angle, gyro bias) ----------
struct Kalman {
  float Q_angle;   // Process noise variance for the angle
  float Q_bias;    // Process noise variance for the gyro bias
  float R_measure; // Measurement noise variance

  float angle; // The angle calculated by the Kalman filter - part of the 2x1 state
  float bias;  // The gyro bias calculated by the filter - part of the 2x1 state
  float rate;  // Unbiased rate calculated from the rate and the calculated bias

  // Error covariance matrix
  float P[2][2];
};

void kalmanInit(Kalman &k) {
  k.Q_angle   = 0.001f; // tune these
  k.Q_bias    = 0.003f; // tune these
  k.R_measure = 0.03f;  // tune these

  k.angle = 0.0f;
  k.bias  = 0.0f;
  k.rate  = 0.0f;

  k.P[0][0] = 0.0f;
  k.P[0][1] = 0.0f;
  k.P[1][0] = 0.0f;
  k.P[1][1] = 0.0f;
}

// Call this every loop with new gyro rate (deg/s), accelerometer angle (deg), and dt (s).
float kalmanGetAngle(Kalman &k, float newAngle, float newRate, float dt) {
  // Predict
  k.rate = newRate - k.bias;
  k.angle += dt * k.rate;

  // Update covariance matrix: P = A*P*A^T + Q
  // A = [1 -dt; 0 1] applied implicitly for continuous-discrete; using discrete approximation:
  k.P[0][0] += dt * (dt*k.P[1][1] - k.P[0][1] - k.P[1][0] + k.Q_angle);
  k.P[0][1] -= dt * k.P[1][1];
  k.P[1][0] -= dt * k.P[1][1];
  k.P[1][1] += k.Q_bias * dt;

  // Measurement update
  float S = k.P[0][0] + k.R_measure; // Estimate error
  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;

  // Angle difference
  float y = newAngle - k.angle;

  // Apply correction
  k.angle += K0 * y;
  k.bias  += K1 * y;

  // Update P = (I - K * H) * P
  float P00_temp = k.P[0][0];
  float P01_temp = k.P[0][1];

  k.P[0][0] -= K0 * P00_temp;
  k.P[0][1] -= K0 * P01_temp;
  k.P[1][0] -= K1 * P00_temp;
  k.P[1][1] -= K1 * P01_temp;

  return k.angle;
}

// Instantiate Kalman filters for pitch and yaw
Kalman kalmanPitch;
Kalman kalmanYaw;

// ---------- End Kalman implementation ----------

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU connection failed!");
    while(1);
  }

  servoPitch.attach(servoPitchPin);
  servoYaw.attach(servoYawPin);
  servoPitch.write(servoCenterPitch);
  servoYaw.write(servoCenterYaw);

  delay(1000);

  Serial.println("Calibrating");
  float sumPitch=0, sumYaw=0;
  const int samples = 500;
  for(int i=0;i<samples;i++){
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    float pitch = atan2(ax,sqrt(ay*ay+az*az))*180/PI;
    float yaw   = atan2(ay,sqrt(ax*ax+az*az))*180/PI;
    sumPitch += pitch;
    sumYaw   += yaw;
    delay(5);
  }
  pitchOffset = sumPitch / samples;
  yawOffset   = sumYaw / samples;

  Serial.print("Calibration done: pitchOffset=");
  Serial.print(pitchOffset);
  Serial.print(" yawOffset=");
  Serial.println(yawOffset);

  // Initialize Kalman filters and seed with initial accel angles
  kalmanInit(kalmanPitch);
  kalmanInit(kalmanYaw);

  // Seed initial angles so filter starts near accel reading
  {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    float accPitch = atan2(ax,sqrt(ay*ay+az*az))*180/PI - pitchOffset;
    float accYaw   = atan2(ay,sqrt(ax*ax+az*az))*180/PI - yawOffset;
    kalmanPitch.angle = accPitch;
    kalmanYaw.angle   = accYaw;
  }

  lastTime = millis();
  state = ARMED; // start in ARMED for testing
}

void loop() {
  unsigned long now = millis();
  elapsedTime = (now - lastTime)/1000.0;
  // prevent dt=0
  if(elapsedTime <= 0) elapsedTime = 0.001;
  lastTime = now;

  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if(cmd.startsWith("P1 ")) KpPitch = cmd.substring(3).toFloat();
    if(cmd.startsWith("I1 ")) KiPitch = cmd.substring(3).toFloat();
    if(cmd.startsWith("D1 ")) KdPitch = cmd.substring(3).toFloat();
    if(cmd.startsWith("P2 ")) KpYaw   = cmd.substring(3).toFloat();
    if(cmd.startsWith("I2 ")) KiYaw   = cmd.substring(3).toFloat();
    if(cmd.startsWith("D2 ")) KdYaw   = cmd.substring(3).toFloat();
  }

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  float accPitch = atan2(ax,sqrt(ay*ay+az*az))*180/PI - pitchOffset;
  float accYaw   = atan2(ay,sqrt(ax*ax+az*az))*180/PI - yawOffset;

  // Convert raw gyro to deg/s (assuming MPU configured to 250 dps)
  float gyroPitchRate = gx * 250.0 / 32768.0;
  float gyroYawRate   = gy * 250.0 / 32768.0;

  // Use Kalman filters instead of complementary blend
  pitchFiltered = kalmanGetAngle(kalmanPitch, accPitch, gyroPitchRate, elapsedTime);
  yawFiltered   = kalmanGetAngle(kalmanYaw,   accYaw,   gyroYawRate,   elapsedTime);

  float pitch = pitchFiltered;
  float yaw   = yawFiltered;
  if(abs(pitch) < deadbandPitch) pitch = 0;
  if(abs(yaw)   < deadbandYaw)   yaw   = 0;

  switch(state){
    case IDLE: {
      servoPitch.write(servoCenterPitch);
      servoYaw.write(servoCenterYaw);
      break;
    }

    case ARMED: {
      state = VECTORING;
      break;
    }

    case VECTORING: {

      errorPitch = 0 - pitch;
      integralPitch += errorPitch * elapsedTime;
      derivativePitch = (errorPitch - prevErrorPitch)/elapsedTime;
      derivativePitch = constrain(derivativePitch, -3, 3);
      float outputPitch = KpPitch*errorPitch + KiPitch*integralPitch + KdPitch*derivativePitch;
      if(abs(outputPitch) < 1.0) outputPitch = 0; 
      prevErrorPitch = errorPitch;

      errorYaw = 0 - yaw;
      integralYaw += errorYaw * elapsedTime;
      derivativeYaw = 0; // derivative off for yaw
      float outputYaw = KpYaw*errorYaw + KiYaw*integralYaw + KdYaw*derivativeYaw;
      if(abs(outputYaw) < 1.0) outputYaw = 0;
      prevErrorYaw = errorYaw;

      int servoAnglePitch = constrain(servoCenterPitch + outputPitch, servoMin, servoMax);
      int servoAngleYaw   = constrain(servoCenterYaw   + outputYaw,   servoMin, servoMax);

      if(millis() - lastServoUpdate >= servoUpdateInterval){
        servoPitch.write(servoAnglePitch);
        servoYaw.write(servoAngleYaw);
        lastServoUpdate = millis();
      }

      Serial.print("Pitch: "); Serial.print(pitch);
      Serial.print(" | Yaw: "); Serial.print(yaw);
      Serial.print(" | ServoPitch: "); Serial.print(servoAnglePitch);
      Serial.print(" | ServoYaw: "); Serial.println(servoAngleYaw);

      break;
    }

    case LOCKED: {
      break;
    }
  }

  delay(5);
}
