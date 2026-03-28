// Joint1_test.ino
//
// Simple hardware test for joint 1 only.
// Requires an Arduino Mega because it uses pins 22 and 23.
//
// Behavior:
// - rotate one direction for TEST_STEPS
// - wait
// - rotate back
// - wait
// - repeat

namespace
{
constexpr uint8_t JOINT1_DIR_PIN = 22;
constexpr uint8_t JOINT1_STEP_PIN = 23;

// Change to false if the motor turns the wrong direction.
constexpr bool FORWARD_DIR_LEVEL = HIGH;

// Increase slowly if needed.
constexpr unsigned long STEP_PULSE_WIDTH_US = 5;
constexpr unsigned long STEP_INTERVAL_US = 800;
constexpr long TEST_STEPS = 2000;
constexpr unsigned long PAUSE_MS = 2000;

void doSteps(bool forward, long steps)
{
  digitalWrite(JOINT1_DIR_PIN, forward ? FORWARD_DIR_LEVEL : !FORWARD_DIR_LEVEL);

  for (long i = 0; i < steps; ++i)
  {
    digitalWrite(JOINT1_STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_WIDTH_US);
    digitalWrite(JOINT1_STEP_PIN, LOW);
    delayMicroseconds(STEP_INTERVAL_US);
  }
}
}  // namespace

void setup()
{
  pinMode(JOINT1_DIR_PIN, OUTPUT);
  pinMode(JOINT1_STEP_PIN, OUTPUT);
  digitalWrite(JOINT1_DIR_PIN, LOW);
  digitalWrite(JOINT1_STEP_PIN, LOW);
}

void loop()
{
  doSteps(true, TEST_STEPS);
  delay(PAUSE_MS);
  doSteps(false, TEST_STEPS);
  delay(PAUSE_MS);
}
