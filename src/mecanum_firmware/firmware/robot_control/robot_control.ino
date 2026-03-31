// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    Serial.println(chr);
  }
}