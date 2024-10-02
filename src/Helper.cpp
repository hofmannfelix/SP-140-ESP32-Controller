#include <Arduino.h>

double mapd(double x, double inMin, double inMax, double outMin, double outMax) {
  return (constrain(x, inMin, inMax) - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

double closestDivisibleBy(double value, int num) {
  return (((int)(value / num + 0.5)) * num);
}
