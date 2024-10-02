#ifndef CruiseControl_h
#define CruiseControl_h

#include <QuickPID.h>

class CruiseControl {
    const int throttleAdjustmentFrequencyMs = 200;
    const float requiredAltitudeInMetersAGL = 100;
    const float cruiseValueMin = 0, cruiseValueMax = 90;
    const float Kp = 1.0, Ki = 0.1, Kd = 0.1;
    
    QuickPID *pid;
    bool isCruiseEnabled = false;
    float cruiseValue = 0;
    float cruiseAltitude = 0;
    float currentAltitude = 0;

public:
    CruiseControl();
    void enable();
    void disable();
    void initialize(int initialPwm);
    bool isEnabled();
    bool isInitialized();
    bool hasRequiredAltitude(); 
    void setCurrentAltitude(double newAltitude);
    int calculateCruisePwm();
};

#endif