#include "configuration.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "PulseWind.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include <string>

PulseWindSensor::PulseWindSensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "PULSEWIND") {}

#define MAX_UINT32 4294967295
volatile unsigned long pulseCount = 0;
volatile unsigned long last_interrupt_time = 0;
volatile unsigned long maxPulseLength = 0;
volatile unsigned long minPulseLength = MAX_UINT32;


void IRAM_ATTR windSpeedInt() {
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 50ms, assume it's a bounce and ignore
    // 20ms is all we can really afford (2.4km/h * 20 = 300km/h)
    if ((interrupt_time - last_interrupt_time) > 20) 
    {
        pulseCount++;
    }

    unsigned long pulseLength = interrupt_time - last_interrupt_time;
    
    if (pulseLength > maxPulseLength) // min lull
        maxPulseLength = pulseLength;   

    if (pulseLength < minPulseLength) // max gust
        minPulseLength = pulseLength;
        
    last_interrupt_time = interrupt_time;
}

void PulseWindSensor::setup()
{
    // Set up

}

int32_t PulseWindSensor::runOnce()
{
    LOG_INFO("Init sensor: %s", sensorName);

    LOG_INFO("Setting up PulseWind sensor on GPIO %d", GPIO_SEL_48);
    pinMode(48, INPUT_PULLUP); // Set GPIO 48 as input with pull-up resistor
    attachInterrupt(digitalPinToInterrupt(48), windSpeedInt, FALLING);

    status = true;

    return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
}

float PulseWindSensor::getWindSpeed()
{
    //conversion is 1 pulse/s = 2.4 km/h
    unsigned long deltaPulseCount = pulseCount - lastPulseCount;
    unsigned long deltaTime = millis() - lastCheckTime;
    lastPulseCount = pulseCount;
    lastCheckTime = millis();

    LOG_INFO("PulseWind sensor calculated based on %lu pulses within the last %lu ms", deltaPulseCount, deltaTime);
    

    if ((deltaTime > 0) && (deltaPulseCount > 0))
    {
        lastWindSpeed = ((float)deltaPulseCount / ((float)deltaTime / 1000.0f)) * 2.4f;
        lastWindSpeed = lastWindSpeed / 60 / 60 * 1000; // Convert to m/s
    }
    else
    {
        lastWindSpeed = 0.0f; // No pulse detected, set speed to 0
    }   

    return lastWindSpeed;
}

float PulseWindSensor::getMinWindSpeed()
{
    if (minPulseLength == MAX_UINT32) {
        return 0.0f; // No pulse detected yet
    }
    float minLullSpeed = pulseLengthToSpeed(minPulseLength);
    minPulseLength = MAX_UINT32; // Reset minPulseLength for next calculation
    return minLullSpeed; // Convert pulse length to speed in m/s
}

float PulseWindSensor::getMaxWindSpeed()
{
    if (maxPulseLength == 0) {
        return 0.0f; // No pulse detected yet
    }
    float maxGustSpeed = pulseLengthToSpeed(maxPulseLength);
    maxPulseLength = 0; // Reset maxPulseLength for next calculation
    return maxGustSpeed; // Convert pulse length to speed in m/s
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
    measurement->variant.environment_metrics.has_wind_gust = true;
    measurement->variant.environment_metrics.has_wind_lull = true;
    measurement->variant.environment_metrics.has_wind_direction = true;
    measurement->variant.environment_metrics.has_barometric_pressure = false;

    measurement->variant.environment_metrics.wind_speed = getWindSpeed();
    measurement->variant.environment_metrics.wind_gust = getMaxWindSpeed();
    measurement->variant.environment_metrics.wind_lull = getMinWindSpeed();
    
    unsigned long maxPulseLength = 0;
    unsigned long minPulseLength = MAX_UINT32;
    measurement->variant.environment_metrics.wind_direction = getWindDirection();

    LOG_INFO("Wind Speed: %f", measurement->variant.environment_metrics.wind_speed);
    LOG_INFO("Wind Direction: %d", measurement->variant.environment_metrics.wind_direction);

    return true;
}

#endif