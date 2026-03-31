// ================= MOTOR PINS =================
#define FL_PWM 5
#define FL_DIR 4
#define FR_PWM 6
#define FR_DIR 12
#define RL_PWM 9
#define RL_DIR 8
#define RR_PWM 10
#define RR_DIR 11
// ==============================================

String input = "";
float fl = 0, fr = 0, rl = 0, rr = 0;

unsigned long lastCmdTime = 0;
bool receivedFirstCmd = false;

void parseCommand(String cmd) {
  if (cmd.length() < 4) return;

  String prefix = cmd.substring(0, 2);
  char sign = cmd.charAt(2);
  float value = cmd.substring(3).toFloat();
  if (sign == 'n') value = -value;

  if      (prefix == "fl") fl = value;
  else if (prefix == "fr") fr = value;
  else if (prefix == "rl") rl = value;
  else if (prefix == "rr") rr = value;

  lastCmdTime = millis();
  receivedFirstCmd = true;
}

void setMotor(int pwm_pin, int dir_pin, float speed) {
  // Tune this scale factor for your motors.
  // If rad/s from ROS tops out at ~10, and you want full PWM at that speed:
  // 10 * 25.5 = 255
  int pwm_val = constrain((int)(abs(speed) * 25.5f), 0, 255);

  digitalWrite(dir_pin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwm_pin, pwm_val);
}

void stopAllMotors() {
  analogWrite(FL_PWM, 0);
  analogWrite(FR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(RR_PWM, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(RL_PWM, OUTPUT); pinMode(RL_DIR, OUTPUT);
  pinMode(RR_PWM, OUTPUT); pinMode(RR_DIR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Read and parse serial
  while (Serial.available()) {
  char c = Serial.read();
  digitalWrite(LED_BUILTIN, HIGH);
  if (c == ',') {
    parseCommand(input);
    input = "";
  } else if (c == '\n') {
    input = "";  // frame reset — discard any partial garbage
  } else {
    input += c;
  }
}

  // Safety stop — only after we've received at least one command
  if (receivedFirstCmd && (millis() - lastCmdTime > 500)) {
    stopAllMotors();
    return; // skip setMotor calls below
  }

  setMotor(FL_PWM, FL_DIR, fl);
  setMotor(FR_PWM, FR_DIR, fr);
  setMotor(RL_PWM, RL_DIR, rl);
  setMotor(RR_PWM, RR_DIR, rr);
}