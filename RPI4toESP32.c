// ESP32 4-Wheel Rover Motor Control
// Receives commands from ROS2 via Serial
// Format: "left_motor,right_motor\n" where values are -100 to 100

// Motor Pin Definitions (based on your MicroPython setup)
// Wheel 1 (Back Left)
#define W1_FWD 5
#define W1_BWD 18

// Wheel 2 (Front Left)
#define W2_FWD 19
#define W2_BWD 21

// Wheel 3 (Back Right)
#define W3_FWD 26
#define W3_BWD 25

// Wheel 4 (Front Right)
#define W4_FWD 33
#define W4_BWD 32

// PWM Settings
#define PWM_FREQ 2000       // 2kHz - matches your MicroPython setup
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)

// PWM Channels (ESP32 has 16 channels, we need 8)
#define W1_FWD_CH 0
#define W1_BWD_CH 1
#define W2_FWD_CH 2
#define W2_BWD_CH 3
#define W3_FWD_CH 4
#define W3_BWD_CH 5
#define W4_FWD_CH 6
#define W4_BWD_CH 7

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  
  // Setup PWM for all motor pins
  // Left Side (Wheels 1 & 2)
  ledcSetup(W1_FWD_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(W1_BWD_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(W2_FWD_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(W2_BWD_CH, PWM_FREQ, PWM_RESOLUTION);
  
  // Right Side (Wheels 3 & 4)
  ledcSetup(W3_FWD_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(W3_BWD_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(W4_FWD_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(W4_BWD_CH, PWM_FREQ, PWM_RESOLUTION);
  
  // Attach pins to PWM channels
  ledcAttachPin(W1_FWD, W1_FWD_CH);
  ledcAttachPin(W1_BWD, W1_BWD_CH);
  ledcAttachPin(W2_FWD, W2_FWD_CH);
  ledcAttachPin(W2_BWD, W2_BWD_CH);
  ledcAttachPin(W3_FWD, W3_FWD_CH);
  ledcAttachPin(W3_BWD, W3_BWD_CH);
  ledcAttachPin(W4_FWD, W4_FWD_CH);
  ledcAttachPin(W4_BWD, W4_BWD_CH);
  
  Serial.println("ESP32 4-Wheel Rover Control Ready");
}

void setWheel(int speed, int fwd_channel, int bwd_channel) {
  // speed: -100 to 100
  // Uses sign-magnitude control (one PWM active at a time)
  
  int pwm_value = map(abs(speed), 0, 100, 0, 255);
  
  if (speed > 0) {
    // Forward
    ledcWrite(fwd_channel, pwm_value);
    ledcWrite(bwd_channel, 0);
  } else if (speed < 0) {
    // Backward
    ledcWrite(fwd_channel, 0);
    ledcWrite(bwd_channel, pwm_value);
  } else {
    // Stop
    ledcWrite(fwd_channel, 0);
    ledcWrite(bwd_channel, 0);
  }
}

void loop() {
  // Check if data is available
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    
    // Parse "left,right" format
    int comma_index = data.indexOf(',');
    if (comma_index > 0) {
      int left_motor = data.substring(0, comma_index).toInt();
      int right_motor = data.substring(comma_index + 1).toInt();
      
      // Constrain values
      left_motor = constrain(left_motor, -100, 100);
      right_motor = constrain(right_motor, -100, 100);
      
      // Set left side wheels (1 & 2)
      setWheel(left_motor, W1_FWD_CH, W1_BWD_CH);
      setWheel(left_motor, W2_FWD_CH, W2_BWD_CH);
      
      // Set right side wheels (3 & 4)
      setWheel(right_motor, W3_FWD_CH, W3_BWD_CH);
      setWheel(right_motor, W4_FWD_CH, W4_BWD_CH);
      
      // Debug
      Serial.print("L: ");
      Serial.print(left_motor);
      Serial.print(" | R: ");
      Serial.println(right_motor);
    }
  }
}
