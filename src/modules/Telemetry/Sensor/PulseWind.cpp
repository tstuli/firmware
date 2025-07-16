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
    pulseCount++;
  
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

    pinMode(7, OUTPUT); // This is our voltage divider power
    digitalWrite(7, LOW); // Set GPIO 7 low to power off the sensor

    pinMode(6, ANALOG);
    
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


enum WindDirection {
    dir_0_degrees = 0,  // North
    dir_22_5_degrees = 1,  // North-Northeast
    dir_45_degrees = 2,  // Northeast
    dir_67_5_degrees = 3,  // East-Northeast
    dir_90_degrees = 4,  // East
    dir_112_5_degrees = 5,  // East-Southeast
    dir_135_degrees = 6,  // Southeast
    dir_157_5_degrees = 7,  // South-Southeast
    dir_180_degrees = 8,  // South
    dir_202_5_degrees = 9,  // South-Southwest
    dir_225_degrees = 10, // Southwest
    dir_247_5_degrees = 11, // West-Southwest
    dir_270_degrees = 12, // West
    dir_292_5_degrees = 13, // West-Northwest
    dir_315_degrees = 14, // Northwest
    dir_337_5_degrees = 15  // North-Northwest
};

uint16_t ClosestDirection[] =
{
    2533,
    1308,
    1487,
    270,
    300,
    212,
    595,
    408,
    926,
    789,
    2031,
    1932,
    3046,
    2667,
    2859,
    2265
};


uint8_t PulseWindSensor::getWindDirection()
{
    // Wind direction is represented in several discrete values
    digitalWrite(7, HIGH); // Set GPIO 7 high to power on the sensor
    delay(10);
    uint32_t reading = analogReadMilliVolts(GPIO_NUM_6); // Read the analog value from GPIO 6
    LOG_INFO("Raw direction sensor: %d", reading);

    WindDirection direction = dir_0_degrees; // Default to North

    //find the closest direction
    uint8_t numDirections = sizeof(ClosestDirection) / sizeof(ClosestDirection[0]);
    uint16_t closestDiff = UINT16_MAX;
    for (uint8_t i = 0; i < numDirections; i++) {
        uint16_t diff = static_cast<uint16_t>(reading > ClosestDirection[i] ? reading - ClosestDirection[i] : ClosestDirection[i] - reading);
        if (diff < closestDiff) {
            closestDiff = diff;
            direction = static_cast<WindDirection>(i);
        }
    }
    return direction; // Convert to degrees (each step is 22.5 degrees)
}

bool PulseWindSensor::getMetrics(meshtastic_Telemetry *measurement)
{
    LOG_INFO("Read pulse wind sensor metrics");
    
    measurement->variant.environment_metrics.has_wind_speed = true; // only including the encoded value
    measurement->variant.environment_metrics.has_wind_gust = false;
    measurement->variant.environment_metrics.has_wind_lull = false;
    measurement->variant.environment_metrics.has_barometric_pressure = false;
    measurement->variant.environment_metrics.has_wind_direction = false;

    measurement->variant.environment_metrics.has_temperature = false;
    measurement->variant.environment_metrics.has_relative_humidity = false;



    PulseWindSensor::WindSpeeds windSpeeds = calculate_wind_speeds_filtered(pulseSamples, MAX_SAMPLES);


    measurement->variant.environment_metrics.wind_speed = windSpeeds.average;
    measurement->variant.environment_metrics.wind_gust = windSpeeds.gust;
    measurement->variant.environment_metrics.wind_lull = windSpeeds.lull;
    

    //To reduce data volume, we are going to coerce some of the values into a single one.  This requires logic on the receiver to understand, but its better than modifying the protobuf.
    // To do this, wind_speed is represented as a x10 value in uint16_t and only use the first 12 bits
    // Then, wind_gust and wind_lull are respresnted as offset from wind_speed in x1 value as uint8_t
    // Finally, wind direction is repesented as the top most 4 bits of the uint32_t value
    uint32_t windSpeedValueEncoded = ((uint16_t)(windSpeeds.average * 10.0f) & 0x0FFF) << 16;
    windSpeedValueEncoded |= ((uint8_t)((windSpeeds.gust - windSpeeds.average) * 10.0f) & 0xFF) << 8;
    windSpeedValueEncoded |= ((uint8_t)((windSpeeds.average - windSpeeds.lull) * 10.0f) & 0xFF);

    measurement->variant.environment_metrics.wind_direction = getWindDirection();

    windSpeedValueEncoded |= ((measurement->variant.environment_metrics.wind_direction & 0x0F) << 28);

    LOG_INFO("Wind Speed Float: %f", measurement->variant.environment_metrics.wind_speed);
    LOG_INFO("Wind Gust Float: %f", measurement->variant.environment_metrics.wind_gust);
    LOG_INFO("Wind Lull Float: %f", measurement->variant.environment_metrics.wind_lull);
    LOG_INFO("Wind Direction Float: %d", measurement->variant.environment_metrics.wind_direction);

    // type punning
    typedef union {
        uint32_t i;
        float f;
    }  FloatIntUnion;

    FloatIntUnion u;
    u.i = windSpeedValueEncoded;
    measurement->variant.environment_metrics.wind_speed = u.f;
    LOG_INFO("Wind Speed Encoded: 0x%x", u.i);

    // decoded wind values
    float decodedWindSpeed = ((float)((windSpeedValueEncoded >> 16) & 0x0FFF)) / 10.0f;
    float decodedWindGust  = decodedWindSpeed + (((windSpeedValueEncoded >> 8) & 0xFF) / 10.0f);
    float decodedWindLull  = decodedWindSpeed - ((windSpeedValueEncoded & 0xFF) / 10.0f);

    float decodedWindDirection = ((float)((windSpeedValueEncoded >> 28) & 0x0F)) * 22.5f;

    LOG_INFO("Wind Speed Decode: %f", decodedWindSpeed);
    LOG_INFO("Wind Gust Decoded: %f", decodedWindGust);
    LOG_INFO("Wind Lull Decoded: %f", decodedWindLull);
    LOG_INFO("Wind Direction Decoded: %f", decodedWindDirection);
    
    return true;
}

#endif