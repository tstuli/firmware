#pragma once

#ifndef _MT_PULSEWIND_H
#define _MT_PULSEWIND_H
#include "configuration.h"
#include "TelemetrySensor.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

class PulseWindSensor : public TelemetrySensor {
  private:
    // PulseWindSensor specific variables can be added here
    // For example, you might want to store the last wind speed and direction readings
    float lastWindSpeed = 0.0f;
    int lastWindDirection = 0;

    float getWindSpeed();
    int getWindDirection();

  protected:
    virtual void setup() override;

  public:
    PulseWindSensor();
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
};

#endif // HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
#endif