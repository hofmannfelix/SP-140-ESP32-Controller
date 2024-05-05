#include "globals.h"
#include "Helper.h"
#include "CruiseControl.h"
#include <QuickPID.h>

void CruiseControl::enable() { isCruiseEnabled = true; }

void CruiseControl::disable() { 
    isCruiseEnabled = false;
    initialCruisePwm = 0;
}

void CruiseControl::initialize(int initialPwm) {
    initialCruisePwm = initialPwm;
    cruiseAltitude = currentAltitude;
}

bool CruiseControl::isEnabled() { return isCruiseEnabled; }

bool CruiseControl::isInitialized() { return initialCruisePwm != 0; }

void CruiseControl::setCurrentAltitude(double newAltitude) {
    if (lastAltitudeUpdate >= millis() - throttleAdjustmentFrequencyMs) return;
    lastAltitudeUpdate = millis();
    currentAltitude = newAltitude;

    Serial.print("Altitude: ");
    Serial.print(newAltitude);
    Serial.print(" Target Alt: ");
    Serial.println(cruiseAltitude);
}

int CruiseControl::calculateCruisePwm() {
    if (initialCruisePwm == 0) return 0;

    double altitudeDelta = cruiseAltitude - currentAltitude;
    int pwmRange = ESC_MAX_PWM - ESC_MIN_PWM;
    int tenPercentPwmStep = pwmRange / maxAdjustmentPercentage;
    double cruisePwmDelta = mapd(altitudeDelta, -2, 2, -tenPercentPwmStep, tenPercentPwmStep);
    double smoothedDelta = cruisePwmDelta * (previousAltitudeDelta > 0 ? (altitudeDelta / previousAltitudeDelta) : 1);
    previousAltitudeDelta = altitudeDelta;
    currentAltitude = cruiseAltitude;
    initialCruisePwm = constrain(initialCruisePwm + cruisePwmDelta, ESC_MIN_PWM + pwmRange * 0.25, ESC_MAX_PWM - pwmRange * 0.25);
    return initialCruisePwm;
}