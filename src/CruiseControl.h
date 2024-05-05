#ifndef CruiseControl_h
#define CruiseControl_h

class CruiseControl {
    // maximum pwm percentage adjustment that can be made between two altitude updates
    const int maxAdjustmentPercentage = 10;
    // minimum interval between a throttle adjustment
    const int throttleAdjustmentFrequencyMs = 1000;

    unsigned long lastAltitudeUpdate = 0;
    bool isCruiseEnabled = false;
    int initialCruisePwm = 0;
    double cruiseAltitude = 0;
    double currentAltitude = 0;

    double previousAltitudeDelta = 0;
    double previousPwmDelta = 0;

public:
    void enable();
    void disable();
    void initialize(int initialPwm);
    bool isEnabled();
    bool isInitialized();
    void setCurrentAltitude(double newAltitude);
    int calculateCruisePwm();
};

#endif