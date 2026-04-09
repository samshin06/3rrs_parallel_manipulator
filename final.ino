#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =============================
// PCA9685 driver
// =============================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 102
#define SERVOMAX 512

// =============================
// 5-wire touch panel pins
// =============================
#define UL 2
#define UR 3
#define LL 4
#define LR 5
#define SENSE A0

// =============================
// Touch panel dimensions
// =============================
#define X_MIN 206
#define X_MAX 816
#define Y_MIN 215
#define Y_MAX 785

#define X_SIZE 214.0f
#define Y_SIZE 163.0f

// =============================
// Robot geometry
// =============================
const float H = 11.0f;
const float R = 10.0f;
const float R_BASE = 7.0f;
const float L1 = 6.53f;
const float L2 = 8.0f;

const float PHIS[3] = {PI, PI / 3.0f, 5.0f * PI / 3.0f};

// =============================
// PID Control
// =============================
float KP = 0.005f;
float KD = 0.002f;
float MAX_TILT = 15.0f;

// =============================
// State
// =============================
float x_prev = 0.0f;
float y_prev = 0.0f;
unsigned long lastMicros;

// =============================
// Vectors and vector operations
// =============================
struct Vec3 {
  float x, y, z;
};

static inline Vec3 makeVec3(float x, float y, float z) {
  return {x, y, z};
}

static inline Vec3 addVec3(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline Vec3 subVec3(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline Vec3 mulVec3(const Vec3& a, float s) {
  return {a.x * s, a.y * s, a.z * s};
}

static inline Vec3 crossVec3(const Vec3& a, const Vec3& b) {
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
}

static inline float normVec3(const Vec3& v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static inline Vec3 normalizeVec3(Vec3 v) {
  float n = normVec3(v);
  if (n < 1e-8f) return {0.0f, 0.0f, 1.0f};
  return {v.x / n, v.y / n, v.z / n};
}

// =============================
// Touch panel read
// =============================
int readX() {
  digitalWrite(UL, LOW);
  digitalWrite(LL, LOW);
  digitalWrite(UR, HIGH);
  digitalWrite(LR, HIGH);
  delayMicroseconds(50);
  return analogRead(SENSE);
}

int readY() {
  digitalWrite(UL, HIGH);
  digitalWrite(UR, HIGH);
  digitalWrite(LL, LOW);
  digitalWrite(LR, LOW);
  delayMicroseconds(50);
  return analogRead(SENSE);
}

// =============================
// IK
// =============================
bool planar2R(float x, float z, float& q1) {
  float D = sqrtf(x * x + z * z);

  float cos_q2 = (-D * D + L1 * L1 + L2 * L2) / (2.0f * L1 * L2);
  if (cos_q2 < -1.0f || cos_q2 > 1.0f) return false;

  float sin_q2 = sqrtf(fmaxf(0.0f, 1.0f - cos_q2 * cos_q2));

  float A = L1 - L2 * cos_q2;
  float B = L2 * sin_q2;

  q1 = atan2f(z * A + x * B, x * A - z * B);
  return true;
}

bool computeQ1Ext(const Vec3& n_in, float q_out[3]) {
  Vec3 n = normalizeVec3(n_in);

  for (int i = 0; i < 3; i++) {
    float phi = PHIS[i];

    Vec3 nk = makeVec3(-sinf(phi), cosf(phi), 0.0f);
    Vec3 d = crossVec3(nk, n);
    d = normalizeVec3(d);

    Vec3 C = makeVec3(0.0f, 0.0f, H);
    Vec3 P = addVec3(C, mulVec3(d, R));

    Vec3 M = makeVec3(R_BASE * cosf(phi), R_BASE * sinf(phi), 0.0f);
    Vec3 v = subVec3(P, M);

    float x_local = -sqrtf(v.x * v.x + v.y * v.y);
    float z_local = v.z;

    float q1;
    if (!planar2R(x_local, z_local, q1)) return false;

    q_out[i] = PI - q1;
  }

  return true;
}

// =============================
// Control law - PID
// =============================
Vec3 computeNormal(float x_mm, float y_mm, float vx, float vy) {
  float ux = KP * x_mm + KD * vx;
  float uy = KP * y_mm + KD * vy;

  float maxTilt = tanf(MAX_TILT * PI / 180.0f);
  float mag = sqrtf(ux * ux + uy * uy);

  if (mag > maxTilt && mag > 1e-8f) {
    float s = maxTilt / mag;
    ux *= s;
    uy *= s;
  }

  return normalizeVec3(makeVec3(-ux, -uy, 1.0f));
}

// =============================
// PWM and Offset
// =============================
uint16_t angleToPWM(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (uint16_t)(SERVOMIN + (deg / 180.0f) * (SERVOMAX - SERVOMIN));
}

float OFFSET[3] = {0, 0, 0};
int SIGN[3] = {+1, +1, +1};

void writeMotors(const float q_ext[3]) {
  for (int i = 0; i < 3; i++) {
    float deg = q_ext[i] * 180.0f / PI;
    deg = OFFSET[i] + SIGN[i]*deg;
    uint16_t pwmVal = angleToPWM(deg);
    pwm.setPWM(i, 0, pwmVal);
  }
}

// =============================
// Setup
// =============================
void setup() {
  Serial.begin(115200);

  pinMode(UL, OUTPUT);
  pinMode(UR, OUTPUT);
  pinMode(LL, OUTPUT);
  pinMode(LR, OUTPUT);

  pwm.begin();
  pwm.setPWMFreq(50);

  lastMicros = micros();
  Serial.println("Ready");
}

// =============================
// Loop
// =============================
void loop() {
  int x_raw = readX();
  int y_raw = readY();

  float x_mm = (x_raw - 511.0f) * (X_SIZE / (X_MAX - X_MIN));
  float y_mm = (y_raw - 500.0f) * (Y_SIZE / (Y_MAX - Y_MIN));

  float alpha = 0.8;
  x_mm = alpha * x_mm + (1 - alpha) * x_prev;
  y_mm = alpha * y_mm + (1 - alpha) * y_prev;

  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
  lastMicros = now;

  float vx_raw = (x_mm - x_prev)/dt;
  float vy_raw = (y_mm - y_prev)/dt;

  float vx = 0.5 * vx_raw + 0.5 * vx;
  float vy = 0.5 * vy_raw + 0.5 * vy;

  x_prev = x_mm;
  y_prev = y_mm;

  Vec3 n = computeNormal(x_mm, y_mm, vx, vy);

  float q_ext[3];
  if (computeQ1Ext(n, q_ext)) {
    writeMotors(q_ext);
  }

  Serial.print("x_mm: ");
  Serial.print(x_mm, 2);
  Serial.print("  y_mm: ");
  Serial.println(y_mm, 2);

  delay(10);
}