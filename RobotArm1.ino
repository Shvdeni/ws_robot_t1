// RobotArm1.ino
//
// Arduino firmware for a 5-axis step/dir robot arm controlled from ROS 2 via USB serial.
// The ROS side sends absolute joint targets in motor steps:
//   PING
//   ENABLE
//   DISABLE
//   STATE
//   MOVE s1 s2 s3 s4 s5
//
// Important:
// 1. This sketch assumes the arm starts at its ROS zero pose when the board boots.
// 2. There is no encoder or homing feedback here, so the tracked position is open-loop.
// 3. Tune MAX_STEP_RATE_HZ and DIRECTION_SIGN before moving the real robot at speed.

namespace
{
constexpr int JOINT_COUNT = 5;

const uint8_t DIR_PINS[JOINT_COUNT] = {22, 24, 26, 28, 30};
const uint8_t STEP_PINS[JOINT_COUNT] = {23, 25, 27, 29, 31};

// Flip any entry from 1 to -1 if a joint moves the wrong way.
const int8_t DIRECTION_SIGN[JOINT_COUNT] = {-1, 1, 1, 1, 1};

// Tuned for very high throughput while keeping acceleration moderate enough to limit shaking.
// If one joint starts to buzz or miss steps, lower only that joint's rate.
const float MAX_STEP_RATE_HZ[JOINT_COUNT] = {12800.0f, 7200.0f, 7200.0f, 8800.0f, 8800.0f};
const float MAX_STEP_ACCEL_HZ_PER_SEC[JOINT_COUNT] = {
  13000.0f, 6400.0f, 6400.0f, 8400.0f, 8400.0f};

// Short pulse width suitable for many stepper drivers.
constexpr unsigned long STEP_PULSE_WIDTH_US = 4;

long current_steps[JOINT_COUNT] = {0, 0, 0, 0, 0};
long target_steps[JOINT_COUNT] = {0, 0, 0, 0, 0};
float current_velocity_steps_per_sec[JOINT_COUNT] = {0, 0, 0, 0, 0};
float step_phase[JOINT_COUNT] = {0, 0, 0, 0, 0};
unsigned long last_update_us = 0;

bool outputs_enabled = false;
char serial_buffer[96];
uint8_t serial_buffer_len = 0;

void printState()
{
  Serial.print("STATE");
  for (int i = 0; i < JOINT_COUNT; ++i)
  {
    Serial.print(' ');
    Serial.print(current_steps[i]);
  }
  Serial.println();
}

void stepJoint(int joint_index, int logical_direction)
{
  const int physical_direction = logical_direction * DIRECTION_SIGN[joint_index];
  digitalWrite(DIR_PINS[joint_index], physical_direction > 0 ? HIGH : LOW);
  digitalWrite(STEP_PINS[joint_index], HIGH);
  delayMicroseconds(STEP_PULSE_WIDTH_US);
  digitalWrite(STEP_PINS[joint_index], LOW);
  current_steps[joint_index] += logical_direction;
}

float clampVelocityChange(float current_velocity, float target_velocity, float max_delta)
{
  if (target_velocity > current_velocity + max_delta)
  {
    return current_velocity + max_delta;
  }
  if (target_velocity < current_velocity - max_delta)
  {
    return current_velocity - max_delta;
  }
  return target_velocity;
}

float desiredVelocityForDelta(int joint_index, long delta_steps)
{
  if (delta_steps == 0)
  {
    return 0.0f;
  }

  const float stopping_speed = sqrtf(
    2.0f * MAX_STEP_ACCEL_HZ_PER_SEC[joint_index] * static_cast<float>(labs(delta_steps)));
  const float limited_speed = min(MAX_STEP_RATE_HZ[joint_index], stopping_speed);
  return (delta_steps > 0) ? limited_speed : -limited_speed;
}

void updateMotion()
{
  const unsigned long now_us = micros();
  if (last_update_us == 0)
  {
    last_update_us = now_us;
    return;
  }

  float dt = static_cast<float>(now_us - last_update_us) * 1.0e-6f;
  last_update_us = now_us;
  if (dt <= 0.0f)
  {
    return;
  }

  if (dt > 0.02f)
  {
    dt = 0.02f;
  }

  for (int i = 0; i < JOINT_COUNT; ++i)
  {
    const long delta_steps = target_steps[i] - current_steps[i];
    const float desired_velocity = outputs_enabled ? desiredVelocityForDelta(i, delta_steps) : 0.0f;
    const float max_velocity_change = MAX_STEP_ACCEL_HZ_PER_SEC[i] * dt;
    current_velocity_steps_per_sec[i] = clampVelocityChange(
      current_velocity_steps_per_sec[i], desired_velocity, max_velocity_change);

    if (labs(delta_steps) <= 1 && fabs(current_velocity_steps_per_sec[i]) < 1.0f)
    {
      current_velocity_steps_per_sec[i] = 0.0f;
      step_phase[i] = 0.0f;
      continue;
    }

    step_phase[i] += fabs(current_velocity_steps_per_sec[i]) * dt;
    long steps_to_issue = static_cast<long>(step_phase[i]);
    if (steps_to_issue <= 0)
    {
      continue;
    }

    step_phase[i] -= static_cast<float>(steps_to_issue);

    const int direction = (current_velocity_steps_per_sec[i] >= 0.0f) ? 1 : -1;
    while (steps_to_issue-- > 0)
    {
      const long remaining_delta = target_steps[i] - current_steps[i];
      if (remaining_delta == 0 || ((remaining_delta > 0) ? 1 : -1) != direction)
      {
        current_velocity_steps_per_sec[i] = 0.0f;
        step_phase[i] = 0.0f;
        break;
      }

      stepJoint(i, direction);
    }
  }
}

bool parseMoveCommand(char *line)
{
  long parsed_targets[JOINT_COUNT];
  char *token = strtok(line, " ");
  if (token == nullptr || strcmp(token, "MOVE") != 0)
  {
    return false;
  }

  for (int i = 0; i < JOINT_COUNT; ++i)
  {
    token = strtok(nullptr, " ");
    if (token == nullptr)
    {
      return false;
    }
    parsed_targets[i] = atol(token);
  }

  for (int i = 0; i < JOINT_COUNT; ++i)
  {
    target_steps[i] = parsed_targets[i];
  }

  Serial.println("OK");
  return true;
}

void processLine(char *line)
{
  if (strcmp(line, "PING") == 0)
  {
    Serial.println("PONG");
    return;
  }

  if (strcmp(line, "ENABLE") == 0)
  {
    outputs_enabled = true;
    Serial.println("OK");
    return;
  }

  if (strcmp(line, "DISABLE") == 0)
  {
    outputs_enabled = false;
    Serial.println("OK");
    return;
  }

  if (strcmp(line, "STATE") == 0)
  {
    printState();
    return;
  }

  if (strncmp(line, "MOVE ", 5) == 0 && parseMoveCommand(line))
  {
    return;
  }

  Serial.println("ERR");
}

void serviceSerial()
{
  while (Serial.available() > 0)
  {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      serial_buffer[serial_buffer_len] = '\0';
      if (serial_buffer_len > 0)
      {
        processLine(serial_buffer);
      }
      serial_buffer_len = 0;
      continue;
    }

    if (serial_buffer_len < sizeof(serial_buffer) - 1)
    {
      serial_buffer[serial_buffer_len++] = c;
    }
    else
    {
      serial_buffer_len = 0;
      Serial.println("ERR");
    }
  }
}
}  // namespace

void setup()
{
  for (int i = 0; i < JOINT_COUNT; ++i)
  {
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(STEP_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
    digitalWrite(STEP_PINS[i], LOW);
  }

  Serial.begin(115200);
  last_update_us = micros();
}

void loop()
{
  serviceSerial();
  updateMotion();
}
