#include "globals.h"
#include "Helper.h"
#include "CruiseControl.h"

CruiseControl::CruiseControl() {}

void CruiseControl::enable() { isCruiseEnabled = true; }

void CruiseControl::disable() { 
    isCruiseEnabled = false;
    cruiseValue = -1;
}

void CruiseControl::initialize(int initialPwm) {
    cruiseValue = mapd(initialPwm, ESC_MIN_PWM, ESC_MAX_PWM, 0, 100);
    cruiseAltitude = closestDivisibleBy(currentAltitude, 5);

    if (pid) delete pid;
    pid = new QuickPID(&currentAltitude, &cruiseValue, &cruiseAltitude);
    pid->SetOutputLimits(cruiseValueMin, cruiseValueMax);
    pid->SetSampleTimeUs(throttleAdjustmentFrequencyMs * 1000);
    pid->SetTunings(Kp, Ki, Kd);
    pid->SetMode(QuickPID::Control::automatic);
}

bool CruiseControl::isEnabled() { return isCruiseEnabled; }

bool CruiseControl::isInitialized() { return cruiseValue != -1; }

bool CruiseControl::hasRequiredAltitude() { return currentAltitude > requiredAltitudeInMetersAGL; }

void CruiseControl::setCurrentAltitude(double newAltitude) { currentAltitude = newAltitude; }

int CruiseControl::calculateCruisePwm() {
    if (!isInitialized()) return 0;

    pid->Compute();
    return mapd(cruiseValue, 0, 100, ESC_MIN_PWM, ESC_MAX_PWM);
}