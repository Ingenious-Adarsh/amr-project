// === teleop_with_odometry.ino ===
//
// This code allow us to teleoperate our amr using keyboard and also gives encoders feedback
//
// Combines:
//  1) Teleop command parsing (“CMD <left> <right>\n”) → sets motor PWM+DIR.
//  2) Encoder tick counting (quadrature) → periodically sends “ODOM <L> <R>\n”.
//
// Wiring (Arduino Mega):
//  • Left encoder A = pin 20 (interrupt 5), B = pin 21
//  • Right encoder A = pin 19 (interrupt 4), B = pin 18
//  • Left motor DIR = pin 2,  PWM = pin 3 (must be PWM‐capable)
//  • Right motor DIR = pin 7, PWM = pin 6 (must be PWM‐capable)
//
// Serial: 115200 baud
//
// You can upload this sketch in the Arduino IDE. In the Serial Monitor (115200 baud, “Newline”),
// you’ll see lines like “⟨CMD 0.50 0.50⟩” and “Parsed → L=0.50 R=0.50” when commands arrive,
// and you’ll also see “ODOM 123 456” about every 100 ms.

#include <Arduino.h>

// ──────────────────────────
// === PIN ASSIGNMENTS ====
// ──────────────────────────

// Encoder inputs (left)
#define ENC_LEFT_A   20   // INT5 (rising‐edge)
#define ENC_LEFT_B   21   // read level

// Encoder inputs (right)
#define ENC_RIGHT_A  19   // INT4 (rising‐edge)
#define ENC_RIGHT_B  18   // read level

// Left motor
#define DIR1  2    // digital direction pin
#define PWM1  3    // PWM pin (must support analogWrite)

// Right motor
#define DIR2  7    // digital direction pin
#define PWM2  6    // PWM pin (must support analogWrite)

// ───────────────────────────────────────────
// === GLOBAL VARIABLES for Encoders ========
// ───────────────────────────────────────────

volatile long left_ticks  = 0;   // Left encoder tick count (increment/decrement)
volatile long right_ticks = 0;   // Right encoder tick count

// For sending odometry at regular intervals:
unsigned long lastOdomSend = 0;
const unsigned long ODOM_INTERVAL = 100;  // send odometry every 100 ms

// For parsing incoming serial lines:
String inputString = "";
bool   stringComplete = false;

// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
// ================================================= MOTOR SCALING CONSTANTS (TUNE HERE) ==================================================
// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
// Tune these values if one motor is weaker than the other
// Increase the LEFT_*_SCALE if your robot veers left (means right motor is stronger)
// Decrease the LEFT_*_SCALE if your robot veers right (means left motor is stronger)
// 1.0 = no scaling, increase it only if there is difference in speeds and ticks of both motors

const float LEFT_FWD_SCALE = 1.09;  // Forward direction 
const float LEFT_REV_SCALE = 1.0005;  // Reverse direction 

// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
// === SETUP() ===============================
// ───────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  // Configure motor pins
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Configure encoder pins
  pinMode(ENC_LEFT_A,   INPUT_PULLUP);
  pinMode(ENC_LEFT_B,   INPUT_PULLUP);
  pinMode(ENC_RIGHT_A,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_B,  INPUT_PULLUP);

  // Attach interrupts on encoder channel A (rising edge)
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, RISING);

  // Reserve buffer for incoming serial data
  inputString.reserve(64);

  Serial.println("Arduino ready. Waiting for CMD <left> <right>...");
}

// ───────────────────────────────────────────
// === LOOP() ================================
// ───────────────────────────────────────────

void loop() {
  // 1) Handle incoming serial (if a full line “\n” is available)
  if (stringComplete) {
    // Remove any leading/trailing whitespace including '\r'
    inputString.trim();

    // Debug: show exactly what line arrived
    Serial.print("⟨");
    Serial.print(inputString);
    Serial.println("⟩");

    // Copy to a C‐string buffer and split by spaces
    char buf[64];
    inputString.toCharArray(buf, sizeof(buf));

    char *tok = strtok(buf, " ");
    if (tok && strcmp(tok, "CMD") == 0) {
      // Next token = left_norm (as string)
      tok = strtok(nullptr, " ");
      if (tok) {
        float left_norm = atof(tok);  // e.g. "0.50" → 0.50f

        // Next token = right_norm (as string)
        tok = strtok(nullptr, " ");
        if (tok) {
          float right_norm = atof(tok);

          // Scale and clamp both wheels
          //left_norm  = left_norm; // * LEFT_SCALE;
          //right_norm = right_norm;

           // ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
          // Apply Direction-Specific Left Motor Scaling
         // ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
          if (left_norm > 0.0 && right_norm > 0.0) {
            // Forward motion
            left_norm *= LEFT_FWD_SCALE;
          } else if (left_norm < 0.0 && right_norm < 0.0) {
            // Reverse motion
            left_norm *= LEFT_REV_SCALE;
          }
          // Otherwise (turning), do not scale


          left_norm  = constrain(left_norm,  -1.0, 1.0);
          right_norm = constrain(right_norm, -1.0, 1.0);

          // Debug: show parsed values
          Serial.print("Parsed → L=");
          Serial.print(left_norm, 2);
          Serial.print("  R=");
          Serial.println(right_norm, 2);

          // Drive the motors
          driveWheel(left_norm,  DIR1, PWM1);
          driveWheel(right_norm, DIR2, PWM2);

// ============================================ Dynamic Encoder-Based Correction ===========================================================
          // If the robot drifts due to mechanical imbalance, we compare tick counts
          // after a short duration and slightly boost the slower side.
          static long last_left_ticks = left_ticks;
          static long last_right_ticks = right_ticks;
          static unsigned long last_correction_time = millis();

          unsigned long now_time = millis();
          if (now_time - last_correction_time > 100) {  // every 100 ms
            long dL = left_ticks - last_left_ticks;
            long dR = right_ticks - last_right_ticks;

            // Save for next time
            last_left_ticks = left_ticks;
            last_right_ticks = right_ticks;
            last_correction_time = now_time;

            // Avoid division by zero
            if (abs(dL) > 0 && abs(dR) > 0) {
              float ratio = (float)abs(dL) / (float)abs(dR);

              // Only apply correction if the difference is noticeable
              if (ratio < 0.95 || ratio > 1.05) {
                if (ratio < 1.0) {
                  // Left is slower
                  left_norm *= 1.05;
                  Serial.println("Adjusting: LEFT slightly boosted");
                } else {
                  // Right is slower
                  right_norm *= 1.05;
                  Serial.println("Adjusting: RIGHT slightly boosted");
                }

                // Clamp to [-1.0, 1.0]
                left_norm = constrain(left_norm, -1.0, 1.0);
                right_norm = constrain(right_norm, -1.0, 1.0);

                // Apply corrected values
                driveWheel(left_norm,  DIR1, PWM1);
                driveWheel(right_norm, DIR2, PWM2);
              }
            }
          }
// =========================================== Dynamic Encoder-Based Correction ==================================================          
        }
        else {
          Serial.println("ERR: missing right token");
        }
      }
      else {
        Serial.println("ERR: missing left token");
      }
    }
    else {
      Serial.println("ERR: does not start with CMD");
    }

    // Clear for next serial line
    inputString = "";
    stringComplete = false;
  }

  // 2) Periodically send odometry (every ODOM_INTERVAL ms)
  unsigned long now = millis();
  if (now - lastOdomSend >= ODOM_INTERVAL) {
    lastOdomSend = now;
    Serial.print("ODOM ");
    Serial.print(left_ticks);
    Serial.print(" ");
    Serial.println(right_ticks);
  }
}

// ───────────────────────────────────────────
// === serialEvent() =========================
// Called by Arduino automatically when new serial data arrives.
// This accumulates characters until '\n', then sets stringComplete=true.
// ───────────────────────────────────────────

void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += c;
    }
  }
}

// ───────────────────────────────────────────
// === driveWheel() ==========================
// speed ∈ [–1.0 .. +1.0] → PWM [0 .. 255], plus a small dead-band
// dirPin = digital pin for direction (HIGH=forward, LOW=reverse)
// pwmPin = PWM‐capable pin for speed
// ───────────────────────────────────────────

void driveWheel(float speed, int dirPin, int pwmPin) {
  // Convert magnitude to [0..255]
  int rawPWM = (int)(fabs(speed) * 255.0);

  // Ensure minimal PWM so wheel overcomes static friction
  if (rawPWM > 0 && rawPWM < 50) rawPWM = 50;
  if (speed == 0.0) rawPWM = 0;  // exact stop if speed=0

  // Set direction pin
  if (speed > 0.0) {
    digitalWrite(dirPin, HIGH);
  } else if (speed < 0.0) {
    digitalWrite(dirPin, LOW);
  }

  analogWrite(pwmPin, rawPWM);

  // Debug: show PWM value
  Serial.print(" → PWM pin ");
  Serial.print(pwmPin);
  Serial.print(": ");
  Serial.println(rawPWM);
}

// ───────────────────────────────────────────
// === Encoder ISRs ==========================
// Increment or decrement tick counts based on channel B
// ───────────────────────────────────────────

void leftEncoderISR() {
  if (digitalRead(ENC_LEFT_B) == HIGH) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

void rightEncoderISR() {
  if (digitalRead(ENC_RIGHT_B) == HIGH) {
    right_ticks--;
  } else {
    right_ticks++;
  }
}
