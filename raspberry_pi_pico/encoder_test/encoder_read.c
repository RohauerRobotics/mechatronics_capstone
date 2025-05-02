#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define I2C_PORT            i2c0
#define AS5600_ADDR         0x36         // AS5600 I2C address
#define AS5600_RAW_ANGLE_REG 0x0C         // Register for raw angle reading

#define SDA_PIN 4
#define SCL_PIN 5

// useful global constants
#define BTI 0.100 // interval time

// define global angle variables
static float total_movement = 0.0;
static float last_angle = 0.0;
static float angle_delta = 0.0;

// angular velocity global variables
static float angular_velocity = 0.0;

// Function to read the 12-bit angle from the AS5600
uint16_t read_as5600_angle() {
    uint8_t reg = AS5600_RAW_ANGLE_REG;
    uint8_t buffer[2];

    // Write the register address to the sensor.
    // The 'true' parameter sends a repeated start instead of a stop.
    int ret = i2c_write_blocking(I2C_PORT, AS5600_ADDR, &reg, 1, true);
    if (ret < 0) {
        printf("I2C write error\n");
        return 0;
    }

    // Read 2 bytes from the sensor.
    ret = i2c_read_blocking(I2C_PORT, AS5600_ADDR, buffer, 2, false);
    if (ret < 0) {
        printf("I2C read error\n");
        return 0;
    }

    // Combine the two bytes into a 16-bit value.
    uint16_t angle = ((uint16_t)buffer[0] << 8) | buffer[1];
    // The AS5600 provides a 12-bit value (0-4095), so mask the upper bits.
    angle &= 0x0FFF;
    return angle;
}

void calculate_angular_velocity(void){
    angular_velocity = angle_delta/BTI;
    printf("Speed in RPM: %f\n",angular_velocity*(60.0/360.0));
}

// Function to calculate angle movement, accounting for rollover
void calculate_angle_movement(float current_angle) {
    angle_delta = current_angle - last_angle;
    calculate_angular_velocity();
    // Handle rollover cases
    if (angle_delta > 180.0) {
        angle_delta -= 360.0;  // Passed 0° clockwise
    } else if (angle_delta < -180.0) {
        angle_delta += 360.0;  // Passed 360° counterclockwise
    }

    total_movement += angle_delta;
    last_angle = current_angle;
}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();
    // init blinking
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN,GPIO_OUT);
    printf("Starting Scan");
    // wait 10 seconds to start serial output
    gpio_put(PICO_DEFAULT_LED_PIN,1);
    sleep_ms(1500);
    gpio_put(PICO_DEFAULT_LED_PIN,0);
    printf("Starting Program");

    // Initialize I2C at 100 kHz.
    i2c_init(I2C_PORT, 100 * 1000);
    // Set up GPIO pins for I2C functionality.
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    // Enable pull-ups on I2C lines.
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("AS5600 Magnetic Encoder Demo\n");
    static uint16_t raw_angle = 0;
    static float current_angle = 0;
    last_angle = read_as5600_angle();
    last_angle = (last_angle*360.0f)/4096.0f; 
    while (true) {
        raw_angle = read_as5600_angle();
        current_angle = (raw_angle*360.0f)/4096.0f; 
        calculate_angle_movement(current_angle);
        // The angle is a raw 12-bit value (0-4095). Optionally convert to degrees:
        printf("Current Angle: %6.2f° | Total Movement: %6.2f Increments\n", current_angle, total_movement*(4096.0/360.0));
        printf("\n");
        sleep_ms(100);
    }

    return 0;
}
