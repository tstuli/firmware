#include "configuration.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "PulseWind.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include <string>

PulseWindSensor::PulseWindSensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "PULSEWIND") {}

void PulseWindSensor::setup()
{
    // Set up
}

int32_t PulseWindSensor::runOnce()
{
    LOG_INFO("Init sensor: %s", sensorName);

    status = true;
    if (!hasSensor()) {
        return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }
    return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
}

float PulseWindSensor::getWindSpeed()
{
    lastWindSpeed=lastWindSpeed + 1; // Simulate wind speed in m/s, cycling through 0 to 20 m/s
    if (lastWindSpeed > 20.0f) {
        lastWindSpeed = 0.0f; // Reset after reaching 20 m/s
    }
    
    return lastWindSpeed;
}

int PulseWindSensor::getWindDirection()
{
    return lastWindDirection++ % 360; // Simulate wind direction in degrees
}

bool PulseWindSensor::getMetrics(meshtastic_Telemetry *measurement)
{
    LOG_INFO("Read pulse wind sensor metrics");
    measurement->variant.environment_metrics.has_temperature = false;
    measurement->variant.environment_metrics.has_relative_humidity = false;
    measurement->variant.environment_metrics.has_wind_speed = true;
    measurement->variant.environment_metrics.has_wind_direction = true;
    measurement->variant.environment_metrics.has_barometric_pressure = false;

    measurement->variant.environment_metrics.wind_speed = getWindSpeed();
    measurement->variant.environment_metrics.wind_direction = getWindDirection();

    LOG_INFO("Wind Speed: %f", measurement->variant.environment_metrics.wind_speed);
    LOG_INFO("Wind Direction: %d", measurement->variant.environment_metrics.wind_direction);

    return true;
}

#endif