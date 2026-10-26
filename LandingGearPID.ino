#include <Servo.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*
  Aircraft Landing Gear Actuator and Position Control Using PID

  The Arduino runs a simple second-order simulation. The SG90 moves to show
  the simulated angle, but this setup does not measure the servo shaft angle.

  The webpage sends these commands over USB serial at 115200 baud:
    HELLO, SETPOINT:x, CENTER, MANUAL
    MODE:P, MODE:PI, MODE:PD, MODE:PID, COMPARE, STOP

  The model starts at 20 degrees and uses 90 degrees as the usual target.
  The model values are for a classroom demo; they are not aircraft test data.
*/

// Pins used by the servo and potentiometer.
const byte SERVO_PIN = 9;
const byte POT_PIN = A0;
Servo indicatorServo;

// These numbers are also used in the data sent to the webpage.
const byte MODE_P = 1;
const byte MODE_PI = 2;
const byte MODE_PD = 3;
const byte MODE_PID = 4;

// Timing for the model and the serial updates.
const unsigned long SAMPLE_INTERVAL_MS = 20;
const unsigned long TRIAL_DURATION_MS = 12000;
const unsigned long POT_REPORT_INTERVAL_MS = 100;
const unsigned long MANUAL_SERVO_REFRESH_MS = 30;

// Parameters for the simulated landing-gear response.
const float NATURAL_FREQUENCY = 2.5f;  // rad/s
const float DAMPING_RATIO = 0.35f;

// Gains are kept the same between runs for this comparison.
const float KP = 2.0f;
const float KI = 1.2f;
const float KD = 0.65f;

// The target range and the servo command limits.
const float INITIAL_ANGLE_DEG = 20.0f;
const float MIN_REFERENCE_DEG = 20.0f;
const float MAX_REFERENCE_DEG = 160.0f;
const float MIN_SERVO_COMMAND_DEG = 0.0f;
const float MAX_SERVO_COMMAND_DEG = 180.0f;

// State for the current run.
byte activeMode = MODE_P;
byte nextComparisonMode = MODE_P;
bool trialRunning = false;
bool comparisonRunning = false;
bool manualControlEnabled = true;

float referenceAngleDeg = 90.0f;
float modelAngleDeg = INITIAL_ANGLE_DEG;
float modelAngularRate = 0.0f;
float integralContribution = 0.0f;
float lastControlCommandDeg = INITIAL_ANGLE_DEG;

unsigned long trialStartMs = 0;
unsigned long lastSampleMs = 0;
unsigned long lastSampleMicros = 0;
unsigned long lastPotReportMs = 0;
unsigned long lastManualRefreshMs = 0;

char serialBuffer[48];
byte serialBufferLength = 0;

const char* controllerName(byte mode) {
  switch (mode) {
    case MODE_P:   return "P";
    case MODE_PI:  return "PI";
    case MODE_PD:  return "PD";
    case MODE_PID: return "PID";
    default:       return "UNKNOWN";
  }
}

// PI and PID use integral action.
bool integralActionEnabled(byte mode) {
  return mode == MODE_PI || mode == MODE_PID;
}

// PD and PID use derivative action.
bool derivativeActionEnabled(byte mode) {
  return mode == MODE_PD || mode == MODE_PID;
}

int readPotentiometer() {
  return analogRead(POT_PIN);
}

// Turn the potentiometer reading into a manual angle.
float manualAngleFromKnob() {
  const int reading = readPotentiometer();
  const float fraction = reading / 1023.0f;
  return MIN_REFERENCE_DEG
       + fraction * (MAX_REFERENCE_DEG - MIN_REFERENCE_DEG);
}

void sendPotentiometerReading() {
  const int reading = readPotentiometer();
  const float manualAngle = MIN_REFERENCE_DEG
      + (MAX_REFERENCE_DEG - MIN_REFERENCE_DEG) * reading / 1023.0f;

  Serial.print(F("KNOB,"));
  Serial.print(reading);
  Serial.print(',');
  Serial.println(manualAngle, 1);
}

void sendReferenceAngle() {
  Serial.print(F("SETPOINT,"));
  Serial.println(referenceAngleDeg, 1);
}

// Start each controller from the same angle and clear its stored values.
void startTrial(byte mode) {
  activeMode = mode;
  manualControlEnabled = false;

  modelAngleDeg = INITIAL_ANGLE_DEG;
  modelAngularRate = 0.0f;
  integralContribution = 0.0f;
  lastControlCommandDeg = INITIAL_ANGLE_DEG;

  // Show the retracted position before starting the run.
  indicatorServo.write((int)INITIAL_ANGLE_DEG);
  delay(250);

  trialStartMs = millis();
  lastSampleMs = trialStartMs;
  lastSampleMicros = micros();
  trialRunning = true;

  Serial.print(F("START,"));
  Serial.print(activeMode);
  Serial.print(',');
  Serial.print(controllerName(activeMode));
  Serial.print(',');
  Serial.println(referenceAngleDeg, 1);
}

void startComparison() {
  comparisonRunning = true;
  nextComparisonMode = MODE_P;
  Serial.println(F("COMPARE_START"));
  startTrial(nextComparisonMode);
}

void finishCurrentTrial() {
  const bool wasComparison = comparisonRunning;

  Serial.print(F("END,"));
  Serial.print(activeMode);
  Serial.print(',');
  Serial.println(controllerName(activeMode));

  if (wasComparison && nextComparisonMode < MODE_PID) {
    ++nextComparisonMode;
    startTrial(nextComparisonMode);
    return;
  }

  trialRunning = false;
  comparisonRunning = false;
  manualControlEnabled = false;  // Hold the last position until another command.

  if (wasComparison) {
    Serial.println(F("COMPARE_END"));
  }
}

/*
  Calculate the actuator command from the selected controller.

  P:   u = Kp * error
  PI:  u = Kp * error + integralContribution
  PD:  u = Kp * error - Kd * modelAngularRate
  PID: u = Kp * error + integralContribution - Kd * modelAngularRate

  The derivative is taken from the model output (derivative-on-measurement).
  This avoids a sharp derivative kick when the target angle changes.
*/
float calculateControlCommand(float errorDeg, float sampleSeconds) {
  const bool useIntegral = integralActionEnabled(activeMode);
  const bool useDerivative = derivativeActionEnabled(activeMode);

  const float derivativeContribution = useDerivative
      ? -KD * modelAngularRate
      : 0.0f;

  // Avoid building up the integral term while the output is pinned at a limit.
  const float outputBeforeIntegralUpdate =
      KP * errorDeg + integralContribution + derivativeContribution;

  if (useIntegral) {
    const bool drivingFurtherHigh =
        outputBeforeIntegralUpdate >= MAX_SERVO_COMMAND_DEG && errorDeg > 0.0f;
    const bool drivingFurtherLow =
        outputBeforeIntegralUpdate <= MIN_SERVO_COMMAND_DEG && errorDeg < 0.0f;

    if (!drivingFurtherHigh && !drivingFurtherLow) {
      integralContribution += KI * errorDeg * sampleSeconds;
    }

    integralContribution = constrain(integralContribution, -180.0f, 180.0f);
  }

  const float output = KP * errorDeg + integralContribution
                     + derivativeContribution;
  return constrain(output, MIN_SERVO_COMMAND_DEG, MAX_SERVO_COMMAND_DEG);
}

// Update the second-order model for this sample.
void updatePlantModel(float controlCommandDeg, float sampleSeconds) {
  const float acceleration =
      NATURAL_FREQUENCY * NATURAL_FREQUENCY * (controlCommandDeg - modelAngleDeg)
      - 2.0f * DAMPING_RATIO * NATURAL_FREQUENCY * modelAngularRate;

  // Update speed first, then angle.
  modelAngularRate += acceleration * sampleSeconds;
  modelAngleDeg += modelAngularRate * sampleSeconds;

  // Keep the model inside the allowed angle range.
  if (modelAngleDeg < 0.0f) {
    modelAngleDeg = 0.0f;
    if (modelAngularRate < 0.0f) modelAngularRate = 0.0f;
  }

  if (modelAngleDeg > 180.0f) {
    modelAngleDeg = 180.0f;
    if (modelAngularRate > 0.0f) modelAngularRate = 0.0f;
  }
}

void sendTelemetry(unsigned long elapsedMs, float errorDeg) {
  Serial.print(F("TEL,"));
  Serial.print(elapsedMs);
  Serial.print(',');
  Serial.print(activeMode);
  Serial.print(',');
  Serial.print(referenceAngleDeg, 2);
  Serial.print(',');
  Serial.print(modelAngleDeg, 3);
  Serial.print(',');
  Serial.print(errorDeg, 3);
  Serial.print(',');
  Serial.print(lastControlCommandDeg, 3);
  Serial.print(',');
  Serial.println(readPotentiometer());
}

void runOneSample() {
  const unsigned long nowMs = millis();
  if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) return;

  const unsigned long nowMicros = micros();
  float sampleSeconds = (nowMicros - lastSampleMicros) / 1000000.0f;
  lastSampleMicros = nowMicros;
  lastSampleMs = nowMs;

  // Fall back to 20 ms if a sample took unexpectedly long.
  if (sampleSeconds <= 0.0f || sampleSeconds > 0.1f) {
    sampleSeconds = SAMPLE_INTERVAL_MS / 1000.0f;
  }

  const float errorDeg = referenceAngleDeg - modelAngleDeg;
  lastControlCommandDeg = calculateControlCommand(errorDeg, sampleSeconds);
  updatePlantModel(lastControlCommandDeg, sampleSeconds);

  // The servo follows the model output; it is not feedback from the real shaft.
  const int indicatorAngle = constrain((int)(modelAngleDeg + 0.5f), 0, 180);
  indicatorServo.write(indicatorAngle);

  const unsigned long elapsedMs = nowMs - trialStartMs;
  sendTelemetry(elapsedMs, errorDeg);

  if (elapsedMs >= TRIAL_DURATION_MS) {
    finishCurrentTrial();
  }
}

void handleSerialCommand(char* command) {
  if (strcmp(command, "HELLO") == 0) {
    if (!trialRunning) {
      manualControlEnabled = true;
      indicatorServo.write((int)(manualAngleFromKnob() + 0.5f));
    }

    // The webpage waits for this reply before enabling the controls.
    Serial.println(F("READY,Flight Systems Lab v6"));
    sendPotentiometerReading();
    sendReferenceAngle();
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    trialRunning = false;
    comparisonRunning = false;
    manualControlEnabled = false;
    Serial.println(F("STOPPED"));
    return;
  }

  if (strncmp(command, "SETPOINT:", 9) == 0) {
    if (trialRunning) {
      Serial.println(F("BUSY,Trial already running; setpoint unchanged"));
      return;
    }

    float requestedAngle = atof(command + 9);
    if (!isfinite(requestedAngle)) requestedAngle = 90.0f;
    referenceAngleDeg = constrain(requestedAngle,
                                  MIN_REFERENCE_DEG,
                                  MAX_REFERENCE_DEG);
    sendReferenceAngle();
    return;
  }

  if (strcmp(command, "CENTER") == 0) {
    if (trialRunning) {
      Serial.println(F("BUSY,Trial already running"));
      return;
    }

    manualControlEnabled = false;
    indicatorServo.write(90);
    Serial.println(F("CENTERED,90"));
    return;
  }

  if (trialRunning) {
    Serial.println(F("BUSY,Trial already running"));
    return;
  }

  if (strcmp(command, "MANUAL") == 0) {
    manualControlEnabled = true;
    indicatorServo.write((int)(manualAngleFromKnob() + 0.5f));
    Serial.println(F("MANUAL_READY"));
    sendPotentiometerReading();
    return;
  }

  // Start the controller selected in the webpage.
  if (strcmp(command, "MODE:P") == 0) {
    comparisonRunning = false;
    startTrial(MODE_P);
  } else if (strcmp(command, "MODE:PI") == 0) {
    comparisonRunning = false;
    startTrial(MODE_PI);
  } else if (strcmp(command, "MODE:PD") == 0) {
    comparisonRunning = false;
    startTrial(MODE_PD);
  } else if (strcmp(command, "MODE:PID") == 0) {
    comparisonRunning = false;
    startTrial(MODE_PID);
  } else if (strcmp(command, "COMPARE") == 0) {
    startComparison();
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char received = (char)Serial.read();
    if (received == '\r') continue;

    if (received == '\n') {
      serialBuffer[serialBufferLength] = '\0';
      if (serialBufferLength > 0) handleSerialCommand(serialBuffer);
      serialBufferLength = 0;
    } else if (serialBufferLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialBufferLength++] = received;
    } else {
      // Ignore a command that is too long for the buffer.
      serialBufferLength = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  indicatorServo.attach(SERVO_PIN);
  indicatorServo.write((int)INITIAL_ANGLE_DEG);

  Serial.println(F("READY,Flight Systems Lab v6"));
  sendPotentiometerReading();
  sendReferenceAngle();
}

void loop() {
  readSerialCommands();

  if (trialRunning) {
    runOneSample();

    // Fallback in case a run misses its normal end time.
    if (trialRunning && millis() - trialStartMs > TRIAL_DURATION_MS + 1000UL) {
      finishCurrentTrial();
    }
    return;
  }

  if (manualControlEnabled &&
      millis() - lastManualRefreshMs >= MANUAL_SERVO_REFRESH_MS) {
    lastManualRefreshMs = millis();
    indicatorServo.write((int)(manualAngleFromKnob() + 0.5f));
  }

  if (millis() - lastPotReportMs >= POT_REPORT_INTERVAL_MS) {
    lastPotReportMs = millis();
    sendPotentiometerReading();
  }
}
