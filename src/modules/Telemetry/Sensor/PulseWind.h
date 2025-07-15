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

    unsigned long lastPulseCount = 0;
    unsigned long lastCheckTime = 0;

    typedef struct {
        float average;
        float gust;
        float lull;
    } WindSpeeds;
    WindSpeeds calculate_wind_speeds_filtered(const unsigned long *intervals_ms, size_t count);


    uint8_t getWindDirection();

    float pulseLengthToSpeed(unsigned long pulseLength) {
        // Convert pulse length to speed in m/s
        // Assuming 1 pulse = 2.4 km/h, convert to m/s
        return (pulseLength / 1000.0f) * (2.4f / 3.6f);
    }

  protected:
    virtual void setup() override;

  public:
    PulseWindSensor();
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
};

#endif // HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
#endif