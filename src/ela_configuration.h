#ifndef ELA_CONFIGURATION_H
#define ELA_CONFIGURATION_H

//#define DEBUG
#define NUM_OF_DIGITAL_PINS 8

#define SAMPLE_BUFFER_SIZE 52000U

#define BAUD_RATE 115200
//#define BAUD_RATE 921600
#define MAX_SAMPLES 50000U
#define MIN_POSTRIG_COUNT 0U
#define MAX_PRETRIG_COUNT SAMPLE_BUFFER_SIZE
#define MIN_SAMPLES 100U
#define DEFAULT_SAMPLE_COUNT 100U
#define DEFAULT_PRETRIG_COUNT (100U)
#define MAX_SAMPLERATE 45000000
#define DEVICE_ID 0x02U
#define MIN_SAMPLERATE 2000U             // Hz
#define DEFAULT_SAMPLERATE 50000U         // Hz

#define DEVICE_NAME "ELA_PICO_V0.1"

#endif
