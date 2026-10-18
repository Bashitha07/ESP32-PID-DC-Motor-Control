#include <WiFi.h>

// --- New Pin Definitions ---
#define IN1_PIN         19
#define IN2_PIN         21
#define ENB_PIN         18   // PWM pin
#define ENC_A_PIN       5
#define ENC_B_PIN       17

// --- Control Timing ---
#define SAMPLE_TIME_MS  20   // Faster control loop
#define LOG_INTERVAL_MS 100
#define PWM_FREQ        5000
#define PWM_BITS        8
#define MAX_PWM         255

// --- WiFi ---
const char* ssid     = "BAS";
const char* password = "bas12345r";
WiFiServer server(8080);
WiFiClient client;

// --- Variables ---
float PPR = 350.0;
double Kp = 0.3, Ki = 0.1, Kd = 0.02;
double targetRPM = 150.0;
bool motorEnabled = false;

volatile long encoderCount = 0;
long lastEncoderCount = 0;
double measuredRPM = 0.0, pidOutput = 0.0, errorSum = 0.0, lastError = 0.0;
unsigned long lastTime = 0, lastLogTime = 0;

void IRAM_ATTR encoderISR_A() {
  (digitalRead(ENC_B_PIN) == HIGH) ? encoderCount++ : encoderCount--;
}

void setup() {
  Serial.begin(115200);
  
  // Set Direction Pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  
  // Initial Direction: Forward
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  server.begin();
  
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_BITS);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR_A, RISING);
}

void loop() {
  unsigned long now = millis();
  
  if (server.hasClient()) {
    if (client) client.stop();
    client = server.available();
  }

  // Control Loop (20ms)
  if (now - lastTime >= SAMPLE_TIME_MS) {
    lastTime = now;
    
    // Calculate RPM
    long delta = encoderCount - lastEncoderCount;
    lastEncoderCount = encoderCount;
    measuredRPM = abs(((double)delta / PPR) / (SAMPLE_TIME_MS / 60000.0));

    // Compute PID
    if (motorEnabled) {
      double dt = SAMPLE_TIME_MS / 1000.0;
      double error = targetRPM - measuredRPM;
      errorSum = constrain(errorSum + (error * dt), -255, 255); 
      double dError = (error - lastError) / dt;
      lastError = error;
      pidOutput = constrain((Kp * error) + (Ki * errorSum) + (Kd * dError), 0, MAX_PWM);
    } else {
      pidOutput = 0;
      errorSum = 0;
    }
    
    ledcWrite(ENB_PIN, (int)pidOutput);
  }

  // Telemetry (100ms)
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;
    String msg = "Setpoint:" + String(targetRPM,1) + ",Measured:" + String(measuredRPM,1) + 
                 ",PWM:" + String((int)pidOutput) + ",Mode:" + (motorEnabled ? "Drive" : "Off") + "\n";
    if (client && client.connected()) client.print(msg);
  }

  // Command Handling
  if (client && client.available()) {
    String input = client.readStringUntil('\n');
    if (input.startsWith("START")) motorEnabled = true;
    else if (input.startsWith("STOP")) motorEnabled = false;
    else {
      int c1 = input.indexOf(','), c2 = input.indexOf(',', c1+1), c3 = input.indexOf(',', c2+1);
      if (c3 > 0) {
        targetRPM = input.substring(0, c1).toDouble();
        Kp = input.substring(c1+1, c2).toDouble();
        Ki = input.substring(c2+1, c3).toDouble();
        Kd = input.substring(c3+1).toDouble();
        errorSum = 0;
      }
    }
  }
}