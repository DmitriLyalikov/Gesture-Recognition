/**
 * @file main.c
 * @author Dmitri Lyalikov (dvl2013@nyu.edu)
 * @brief Main entry point for the application for Embedded Challenge 
 * project called 'Embedded Sentry'. Using STM32F4 Discovery board,
 * A mechanism to lock/unlock using gesture control via on-board IMU.
 * 
 * Use data collected from gyro and accelerometer to detect a specific
 * gesture to unlock a resource.
 * Recorded sequence will be saved on MCU (EEPROM) and used as a reference via a "Record Key"
 * feature. 
 * 
 * User then must replicate the key sequence to unlock the resource.
 * 
 * A successful unlock will trigger a green LED to turn on, and a message 
 * on the LCD screen.
 * 
 * @version 0.1
 * @date 2024-11-14
 * 
 * 
 */

#include "mbed.h"
#include "rtos.h"
#include "mbed_events.h"
#include "mbed-trace/mbed_trace.h"
#include "drivers/SPI.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>                      


// Pin definitions for SPI5 on Disco-F429ZI
#define SPI5_MOSI PF_9
#define SPI5_MISO PF_8
#define SPI5_SCLK PF_7
#define SPI5_CS   PF_6

// Recording button pin
#define BUTTON_PIN PC_13
// Comparison button pin
#define BUTTON_PIN2 PC_14

// Gyroscope register addresses (example, replace with actual addresses from the datasheet)
#define WHO_AM_I_REG 0x0F  // Device ID register
#define OUT_X_L      0x28  // X-axis low byte
#define OUT_X_H      0x29  // X-axis high byte
#define OUT_Y_L      0x30  // Y-axis low byte
#define OUT_Y_H      0x31  // Y-axis high byte
#define OUT_Z_L      0x32  // Z-axis low byte
#define OUT_Z_H      0x33  // Z-axis high byte

// Event queue and thread objects
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread eventThread(osPriorityNormal, 8 * 1024);

// SPI and GPIO objects
SPI spi5(SPI5_MOSI, SPI5_MISO, SPI5_SCLK);
DigitalOut cs(SPI5_CS);
InterruptIn learning_button(BUTTON_PIN);
InterruptIn unlock_button(BUTTON_PIN2);
Timer countdown_timer;

// Buffer of raw gyroscope data (1 second, 20 samples per second), as sensor sample rate is 20Hz
#define GYRO_DATA_LEN 20
int16_t learned_gyro_data[GYRO_DATA_LEN][3];
int16_t current_gyro_data[3];

// TextLCD lcd(D8, D9, D4, D5, D6, D7); // RS, E, DB4, DB5, DB6, DB7

// State machine states
enum State { 
    IDLE, 
    RECORDING,
    UNLOCK_ATTEMPT,
    LOCKED,
    UNLOCKED
};

volatile State currentState = IDLE;
volatile bool learningbuttonPressed = false;
volatile bool unlockbuttonPressed = false;

void learningbutton_handler() {
    learningbuttonPressed = true;
}

void unlockbutton_handler() {
    unlockbuttonPressed = true;
}

void handle_idle() {
    //lcd.cls();
    //lcd.printf("Press button to record");

    while (currentState == IDLE) {
        ThisThread::sleep_for(1s);
    }
}

void handle_recording() {
    // lcd.cls();
    // lcd.printf("Recording...");

    // Display countdown on LCD
    // Measure gesture data on IMU for 1 second and store in learned_gyro_data
    // Return to IDLE state

    while (currentState == RECORDING) {
        ThisThread::sleep_for(1s);
    }
}

void handle_unlock_attempt() {
    // lcd.cls();
    // lcd.printf("Enter key sequence");
    // Display countdown on LCD
    // Measure gesture data on IMU for 1 second and store in current_gyro_data
    // Compare current_gyro_data with learned_gyro_data
    // If match, transition to UNLOCKED state, else return to LOCKED state

    while (currentState == UNLOCK_ATTEMPT) {
        ThisThread::sleep_for(1s);
    }
}

void handle_locked() {
    // lcd.cls();
    // lcd.printf("Locked");

    while (currentState == LOCKED) {
        ThisThread::sleep_for(1s);
    }
}

void handle_unlocked() {
    // lcd.cls();
    // lcd.printf("Unlocked");

    while (currentState == UNLOCKED) {
        ThisThread::sleep_for(1s);
    }
}

void display_countdown() {
    //lcd.cls();
    //lcd.printf("Recording in 3...");

    countdown_timer.start();
    for (int i = 3; i > 0; i--) {
        //lcd.locate(0, 1);
        //lcd.printf("Counting: %d", i);
        ThisThread::sleep_for(1s);
    }

    countdown_timer.stop();
    countdown_timer.reset();
    //lcd.cls();
    //lcd.printf("Recording...");
}


void spi_setup() {
    // Configure SPI interface
    spi5.format(8, 3);  // 8-bit data, SPI mode 3 (replace mode if needed per datasheet)
    spi5.frequency(1000000);  // 1 MHz (adjust based on the gyroscope's specs)
    cs = 1;  // Set chip select high (inactive)
}

// Helper function to write/read data over SPI
uint8_t spi_transfer(uint8_t reg, uint8_t value = 0) {
    cs = 0;  // Pull CS low to select the device
    spi5.write(reg | 0x80);  // Send register address with read/write bit
    uint8_t result = spi5.write(value);  // Send data (or dummy byte for read) and receive response
    cs = 1;  // Release CS
    return result;
}

// Function to read 16-bit axis data (low and high bytes)
int16_t read_axis(uint8_t low_reg, uint8_t high_reg) {
    uint8_t low_byte = spi_transfer(low_reg);  // Read low byte
    uint8_t high_byte = spi_transfer(high_reg);  // Read high byte
    return (int16_t)((high_byte << 8) | low_byte);  // Combine bytes into a 16-bit signed integer
}

// Function to get gyroscope readings (X, Y, Z)
void get_gyro_readings(int16_t &x, int16_t &y, int16_t &z) {
    /** 
     * Read gyroscope data from the IMU and store in x, y, z.
     */
    x = read_axis(OUT_X_L, OUT_X_H);  // Read X-axis
    y = read_axis(OUT_Y_L, OUT_Y_H);  // Read Y-axis
    z = read_axis(OUT_Z_L, OUT_Z_H);  // Read Z-axis
}

void record_gesture() {
    /** 
     * Record gesture by measuring gyroscope data for 1 second
     * and storing it in learned_gyro_data array.
     * 
     */
    for (int i = 0; i < GYRO_DATA_LEN; i++) {
        get_gyro_readings(learned_gyro_data[i][0], learned_gyro_data[i][1], learned_gyro_data[i][2]);
        ThisThread::sleep_for(50ms);  // 20 samples per second
    }
}

// Moving average filter
void moving_average_filter(const float *data, float *filtered_data, int data_len, int window_size) {
    /**
     * Moving average filter formula:
     * y_i = (x_i + x_(i+1) + ... + x_(i+n-1)) / n
     *  
     */
    for (int i = 0; i < data_len - window_size + 1; i++) {
        float sum = 0;
        for (int j = 0; j < window_size; j++) {
            sum += data[i + j];
        }
        filtered_data[i] = sum / window_size;
    }
}

// Windowing function
int windowed_data(const float *data, float *windowed_data, int data_len, int window_size) {
    /** 
     * Windowed data formula:
     * windowed_data = [x_0, x_1, ..., x_(n-1), x_1, x_2, ..., x_n, ..., x_(n-1), x_n]
     * 
     * where:
     * - data: input data array
     * - windowed_data: output windowed data array
     * - data_len: length of the input data array
     * - window_size: size of the window
     * - x_i: individual data points
     * - n: number of data points
     */
    int num_windows = 0;
    for (int i = 0; i < data_len - window_size + 1; i++) {
        for (int j = 0; j < window_size; j++) {
            windowed_data[num_windows * window_size + j] = data[i + j];
        }
        num_windows++;
    }
    return num_windows;
}

// Statistical computation of mean, variance, and standard deviation
void compute_statistics(const float *data, int data_len, float &mean, float &variance, float &std_dev) {
    /** 
     * Mean formula:
     * mean = Σ(x_i) / n
     * 
     * Variance formula:
     * variance = Σ((x_i - mean)^2) / n
     * 
     * Standard deviation formula:
     * std_dev = sqrt(variance)
     * 
     * where:
     * - data: input data array
     * - data_len: length of the input data array
     * - mean: mean of the data
     * - variance: variance of the data
     * - std_dev: standard deviation of the data
     * - x_i: individual data points
     * - n: number of data pointS
     */
    float sum = 0;
    for (int i = 0; i < data_len; i++) {
        sum += data[i];
    }
    mean = sum / data_len;

    sum = 0;
    for (int i = 0; i < data_len; i++) {
        sum += pow(data[i] - mean, 2);
    }
    variance = sum / data_len;
    std_dev = sqrt(variance);
}

float normalized_covariance(const float *x, const float *y, int len) {
    /** 
     * Covariance formula:
     * cov(X, Y) = Σ((x_i - mean_x) * (y_i - mean_y)) / n
     * 
     * Normalized covariance formula:
     * R_xy = cov(X, Y) / (std_dev_x * std_dev_y)
     * 
     * where:
     * - x, y: input data arrays
     * - len: length of the input data arrays
     * - mean_x, mean_y: mean of x and y
     * - cov: covariance of x and y
     * - std_x, std_y: standard deviation of x and y
     * - R_xy: normalized covariance
     */
    float mean_x = 0.0, mean_y = 0.0, cov = 0.0;
    float std_x = 0.0, std_y = 0.0;

    for (int i = 0; i < len; i++) {
        mean_x += x[i];
        mean_y += y[i];
    }
    mean_x /= len;
    mean_y /= len;

    for (int i = 0; i < len; i++) {
        cov += (x[i] - mean_x) * (y[i] - mean_y);
        std_x += (x[i] - mean_x) * (x[i] - mean_x);
        std_y += (y[i] - mean_y) * (y[i] - mean_y);
    }
    std_x = sqrt(std_x / len);
    std_y = sqrt(std_y / len);

    return cov / (std_x * std_y);
}

// Profile structure
typedef struct {
    float mean;
    float variance;
    float std_dev;
} Profile;

Profile *learned_profiles = nullptr;
int profile_count = 0;

void learning_phase(const float *data, int data_len, int window_size) {
    int max_windows = data_len - window_size + 1;
    float *windows = (float *)malloc(max_windows * window_size * sizeof(float));
    int num_windows = windowed_data(data, windows, data_len, window_size);

    learned_profiles = (Profile *)malloc(num_windows * sizeof(Profile));
    profile_count = num_windows;

    for (int i = 0; i < num_windows; i++) {
        float mean, variance, std_dev;
        compute_statistics(&windows[i * window_size], window_size, mean, variance, std_dev);

        learned_profiles[i].mean = mean;
        learned_profiles[i].variance = variance;
        learned_profiles[i].std_dev = std_dev;
    }
    free(windows);
}

bool recognition_phase(const float *data, int data_len, int window_size, float threshold) {
    int max_windows = data_len - window_size + 1;
    float *windows = (float *)malloc(max_windows * window_size * sizeof(float));
    int num_windows = windowed_data(data, windows, data_len, window_size);

    for (int i = 0; i < num_windows; i++) {
        float mean, variance, std_dev;
        compute_statistics(&windows[i * window_size], window_size, mean, variance, std_dev);

        for (int j = 0; j < profile_count; j++) {
            float R_xy = normalized_covariance(&mean, &learned_profiles[j].mean, 1);
            if (R_xy >= threshold) {
                free(windows);
                return true;
            }
        }
    }
    free(windows);
    return false;
}




int main(void)
{
    // Initialize SPI interface
    spi_setup();

    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));

    // Set up button interrupt handler
    learning_button.fall(&learningbutton_handler);
    unlock_button.fall(&unlockbutton_handler);

    queue.call(handle_idle);

    // Read device ID
    uint8_t device_id = spi_transfer(WHO_AM_I_REG);
    printf("Device ID: 0x%02X\n", device_id);

    // Main loop
    while (1) {
        // Handle state transitions for learning and unlock button press
        if (learningbuttonPressed) {
            learningbuttonPressed = false;
            if (currentState == IDLE) {
                currentState = RECORDING;
                display_countdown();
            }
        }

        else if  (unlockbuttonPressed) {
            unlockbuttonPressed = false;
            if (currentState == IDLE) {
                currentState = UNLOCK_ATTEMPT;
                display_countdown();
            }
        }

        // Delay before next reading
        ThisThread::sleep_for(1s);
    }
    return 0;
}