#include "configuration.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "PulseWind.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include <string>
#include <float.h>

PulseWindSensor::PulseWindSensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "PULSEWIND") {}

volatile unsigned long pulseCount = 0;
volatile unsigned long last_interrupt_time = 0;

#define MAX_SAMPLES 1000
#define SPEED_FACTOR 2.4f
#define OUTLIER_THRESHOLD_PERCENT 50  // Accept values within ±50% of the mean

unsigned long int pulseSamples[MAX_SAMPLES] = {0};
unsigned int sampleIndex = 0;
unsigned int numSamples = 0;
bool isFirstSample = true;

void IRAM_ATTR windSpeedInt() {
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 50ms, assume it's a bounce and ignore
    // 20ms is all we can really afford (2.4km/h * 20 = 300km/h)
    if ((interrupt_time - last_interrupt_time) > 20) 
    {
        pulseCount++;
    }

    unsigned long pulseLength = interrupt_time - last_interrupt_time;
    
    pulseSamples[sampleIndex] = pulseLength;
    sampleIndex = (sampleIndex + 1) % MAX_SAMPLES;
    numSamples ++;
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

PulseWindSensor::WindSpeeds PulseWindSensor::calculate_wind_speeds_filtered(const unsigned long *intervals_ms, size_t count) {
    WindSpeeds result = {0.0f, 0.0f, 0.0f};
    int valid_intervals[MAX_SAMPLES];
    size_t valid_count = 0;
    unsigned long sum = 0;

    //disable interrupts
    detachInterrupt(digitalPinToInterrupt(48));
    
    if (!intervals_ms || count == 0 || count > MAX_SAMPLES) goto finished;

    // Step 1: Filter out invalid (<=0) and store
    for (size_t i = 0; i < count; i++) {
        if (intervals_ms[i] <= 0) continue;
        valid_intervals[valid_count++] = intervals_ms[i];
        sum += intervals_ms[i];
    }

    if (valid_count == 0) goto finished;

    if (true)
    {
        // Step 2: Compute average interval
        unsigned int mean = sum / valid_count;

        // Step 3: Filter out outliers (±threshold%)
        unsigned int lower_bound = mean - (mean * OUTLIER_THRESHOLD_PERCENT / 100);
        unsigned int upper_bound = mean + (mean * OUTLIER_THRESHOLD_PERCENT / 100);

        float speed_sum = 0.0f;
        float gust = 0.0f;
        float lull = FLT_MAX;
        size_t filtered_count = 0;

        for (size_t i = 0; i < valid_count; i++) {
            int interval = valid_intervals[i];
            if (interval < lower_bound || interval > upper_bound) continue;

            float speed = (1000.0f / interval) * SPEED_FACTOR;

            speed_sum += speed;
            if (speed > gust) gust = speed;
            if (speed < lull) lull = speed;
            filtered_count++;
        }

        if (filtered_count == 0) goto finished;

        result.average = speed_sum / filtered_count;
        result.gust = gust;
        result.lull = lull;
    }

finished:
    // reset the sample index and count
    sampleIndex = 0;
    numSamples = 0;
    for (size_t i = 0; i < MAX_SAMPLES; i++) {
        pulseSamples[i] = 0;
    }
    //reenable interrupts
    attachInterrupt(digitalPinToInterrupt(48), windSpeedInt, FALLING);
    return result;
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


    PulseWindSensor::WindSpeeds windSpeeds = calculate_wind_speeds_filtered(pulseSamples, MAX_SAMPLES);


    measurement->variant.environment_metrics.wind_speed = windSpeeds.average;
    measurement->variant.environment_metrics.wind_gust = windSpeeds.gust;
    measurement->variant.environment_metrics.wind_lull = windSpeeds.lull;
    
    measurement->variant.environment_metrics.wind_direction = getWindDirection();

    LOG_INFO("Wind Speed: %f", measurement->variant.environment_metrics.wind_speed);
    LOG_INFO("Wind Gust: %f", measurement->variant.environment_metrics.wind_gust);
    LOG_INFO("Wind Lull;: %f", measurement->variant.environment_metrics.wind_lull);
    
    LOG_INFO("Wind Direction: %d", measurement->variant.environment_metrics.wind_direction);

    return true;
}

#endif