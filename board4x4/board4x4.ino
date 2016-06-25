/*
 * 4x4 board
 * by JJurM
 */

//===== CONSTANTS =====

//----- LIGHT SYSTEM -----

// interval in ms between light check
#define LIGHT_SYSTEM_CHECK_INTERVAL 50

// how long to look into past
#define LIGHT_SYSTEM_TRESHOLD_TIME 1000

// ratio (positive checks / all checks) required to fire tilt alert
#define LIGHT_SYSTEM_TRESHOLD_RATIO 0.9

// minimal duration of tilt alert
#define LIGHTS_ON_DURATION 2000

//----- TILT ALERT -----

// interval in ms between tilt check
#define TILT_ALERT_CHECK_INTERVAL 50

// how long to look into past
#define TILT_ALERT_TRESHOLD_TIME 1000

// ratio (positive checks / all checks) required to fire tilt alert
#define TILT_ALERT_TRESHOLD_RATIO 0.9

// minimal duration of tilt alert
#define TILT_ALERT_DURATION 2000

//----- COMPUTED CONSTANTS -----
#define LIGHT_SYSTEM_CHECK_COUNT (LIGHT_SYSTEM_TRESHOLD_TIME / LIGHT_SYSTEM_CHECK_INTERVAL)
#define LIGHT_SYSTEM_TRESHOLD_COUNT (LIGHT_SYSTEM_CHECK_COUNT * LIGHT_SYSTEM_TRESHOLD_RATIO)

#define TILT_ALERT_CHECK_COUNT (TILT_ALERT_TRESHOLD_TIME / TILT_ALERT_CHECK_INTERVAL)
#define TILT_ALERT_TRESHOLD_COUNT (TILT_ALERT_CHECK_COUNT * TILT_ALERT_TRESHOLD_RATIO)

//===== PINOUT =====

//===== VARIABLES =====
boolean tiltAlertChecks[TILT_ALERT_CHECK_COUNT];
boolean tiltAlertChecksIndex = 0;
long tiltAlertLastCheck = 0;
long tiltAlertLastFired = 0;
boolean tiltAlertState = false;

//----- temporary -----
boolean result;
int count;

//===== UTILS =====
void setLED(int index, boolean value) {
  //+ todo
}

boolean checkLight() {
  //+ todo
}

boolean checkTilt() {
  //+ todo
}

//===== PROGRAM =====

void pLightSystem() {
  //+ todo
  
}

void pTiltAlert() {
  if (millis() >= tiltAlertLastCheck + TILT_ALERT_CHECK_INTERVAL) {
    result = checkTilt();
    tiltAlertChecksIndex = (tiltAlertChecksIndex + 1) % TILT_ALERT_CHECK_COUNT;
    tiltAlertChecks[tiltAlertChecksIndex] = result;
  }

  for (int i = 0, count = 0; i < TILT_ALERT_CHECK_COUNT; i++) {
    count += tiltAlertChecks[i];
  }
  if (count >= TILT_ALERT_TRESHOLD_COUNT) {
    tiltAlertLastFired = millis();
  }

  if ((millis() <= tiltAlertLastFired + TILT_ALERT_DURATION) != tiltAlertState) {
    tiltAlertState != tiltAlertState;
    //+ turn tilt alert on/off
  }
}

void setup() {
  //+ todo
}

void loop() {
  pLightSystem();
  pTiltAlert();
}
