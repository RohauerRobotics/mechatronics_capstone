#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
// #include "hardware/pwm.h"

#define I2C_PORT            i2c0
#define AS5600_ADDR         0x36         // AS5600 I2C address
#define AS5600_RAW_ANGLE_REG 0x0C         // Register for raw angle reading
#define TCA9548A_ADDR        0x70

#define SDA_PIN 4 //Yellow
#define SCL_PIN 5 //Green

// useful global constants
#define BTI 0.010 // interval time seconds
#define BTI_DIVS 1000
#define ENC_DELAY 0.0002 // encoder read delay seconds
#define DIV_CAP 2
#define MICRO_MULT 1000000

#define ARM_LEN 0.400 // arm length meters
#define PERSON_H 0.350 // meters above ground
#define M_TO_MM 1000

// Define stepper motor pins
#define T1_PUL 0
#define T1_DIR 1

#define T2_PUL 2
#define T2_DIR 3

#define R_PUL 16
#define R_DIR 17

// Define stepper pulse states
static bool t1_pul_state = true;
static bool t2_pul_state = true;
static bool r_pul_state = true;

// define global angle variables
static float tot_move_radial = 0.0;
static float last_angle_radial = 0.0;
static float angle_delta_radial = 0.0;

static float tot_move_tan = 0.0;
static float last_angle_tan = 0.0;
static float angle_delta_tan = 0.0;

static float tot_move_rotary = 0.0;
static float last_angle_rotary = 0.0;
static float angle_delta_rotary = 0.0;

const static float rotary_ratio = 28.5f/150.0f;

static float hub_angle = 0.0;

static float tot_move_transverse = 0.0;
static float last_angle_transverse = 0.0;
static float angle_delta_transverse = 0.0;

const static float dist_per_deg =  (0.048f*M_PI)/360.0f; // meters per rotation distance traveled per degree
const static float DEG_TO_RAD = (M_PI/180.0f);
const static float STEP_PER_DEG = (200.0f/360.0f);

// global positions = (x,y)
static float current_cart_position[2] = {0.0f,0.0f};
static float last_person_position[2] = {0.0f,0.0f};
static float current_person_position[2] = {0.0f, 0.0f};
// static float next_person_position[2] = {0.0f,0.0f};

// relative displacements
static float dt = 0.0f;
static float dr = 0.0f;

static float y_error;
static float y_error_last;

static float static_side_steps;
static float slide_side_steps;

static float arm_ratio;
static float dy_ratio;
static float next_hub_angle;
static float hub_error;
static float hub_error_last;

// define model parameters
const float KP_T = 12000.0f;
const float KI_T = 800.0f;

const float KP_R = 5000.0f;
const float KI_R = 300.01f;

//define signal variables
static float t1_signal;
static float t2_signal;
static float r_signal;

// define gear ratios
const float t1_gear_ratio = 21.0f/43.5f;
const float t2_gear_ratio = 30.67f/61.33f;
const float r_gear_ratio = 150.0f/28.5f;

// define pause divisions
static double t1_delay_us;
static double t2_delay_us;
static double rot_delay_us;

static uint32_t t1_divs_i;
static uint32_t t2_divs_i;
static uint32_t rot_divs_i;

void mux_bus_select(uint8_t bus_num){
    uint8_t control = 1 << bus_num;
    int bus_ret = i2c_write_blocking(I2C_PORT,TCA9548A_ADDR,&control,1,false);
    if (bus_ret<0){
        printf("Bus write error.\n");
    }
    // sleep_us((int)(ENC_DELAY*MICRO_MULT));
}

// Function to read the 12-bit angle from the AS5600
uint16_t read_as5600_angle(uint8_t bus_val) {
    mux_bus_select(bus_val);
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

// Function to calculate angle movement, accounting for rollover
void calculate_angle_movement_radial(float current_angle_radial) {
    angle_delta_radial = last_angle_radial - current_angle_radial;
    // Handle rollover cases
    if (angle_delta_radial > 180.0) {
        angle_delta_radial -= 360.0;  // Passed 0° clockwise
    } else if (angle_delta_radial < -180.0) {
        angle_delta_radial += 360.0;  // Passed 360° counterclockwise
    }
    tot_move_radial += angle_delta_radial;
    last_angle_radial = current_angle_radial;
}

// Function to calculate angle movement, accounting for rollover
void calculate_angle_movement_tan(float current_angle_tan) {
    angle_delta_tan = last_angle_tan - current_angle_tan;
    // Handle rollover cases
    if (angle_delta_tan > 180.0) {
        angle_delta_tan -= 360.0;  // Passed 0° clockwise
    } else if (angle_delta_tan < -180.0) {
        angle_delta_tan += 360.0;  // Passed 360° counterclockwise
    }
    tot_move_tan += angle_delta_tan;
    last_angle_tan = current_angle_tan;
}

void calculate_angle_movement_rotary(float current_angle_rotary) {
    angle_delta_rotary = last_angle_rotary - current_angle_rotary;
    // Handle rollover cases
    if (angle_delta_rotary > 180.0) {
        angle_delta_rotary -= 360.0;  // Passed 0° clockwise
    } else if (angle_delta_rotary < -180.0) {
        angle_delta_rotary += 360.0;  // Passed 360° counterclockwise
    }
    tot_move_rotary += angle_delta_rotary;
    last_angle_rotary = current_angle_rotary;
}

void calculate_angle_movement_transverse(float current_angle_transverse) {
    angle_delta_transverse = last_angle_transverse - current_angle_transverse;
    // Handle rollover cases
    if (angle_delta_transverse > 180.0) {
        angle_delta_transverse -= 360.0;  // Passed 0° clockwise
    } else if (angle_delta_transverse < -180.0) {
        angle_delta_transverse += 360.0;  // Passed 360° counterclockwise
    }
    tot_move_transverse += angle_delta_transverse;
    last_angle_transverse = current_angle_transverse;
}

void cart_position(void){
    current_cart_position[1] = tot_move_transverse*dist_per_deg;
}

void person_position(void){
    last_person_position[0] = ARM_LEN*cosf(hub_angle*DEG_TO_RAD);
    last_person_position[1] = current_cart_position[1] + ARM_LEN*sinf(hub_angle*DEG_TO_RAD);
    // printf("LPP: %6.2f\n",last_person_position[1]*1000);  
}

void find_next_person(void){
    dr = sinf(tot_move_radial*DEG_TO_RAD)*PERSON_H;
    dt = sinf(tot_move_tan*DEG_TO_RAD)*PERSON_H;
    // printf("dr: %6.2f | dt: %6.2f\n", dr*M_TO_MM, dt*M_TO_MM);
    current_person_position[0] = last_person_position[0] + (cosf(hub_angle*DEG_TO_RAD)*dr) - (sinf(hub_angle*DEG_TO_RAD)*dt);
    current_person_position[1] = last_person_position[1] + (sinf(hub_angle*DEG_TO_RAD)*dr) +   (cosf(hub_angle*DEG_TO_RAD)*dt);
}

void init_motor(int PUL_PIN, int DIR_PIN){
    //Initialize Translational Motor
    gpio_init(PUL_PIN);
    gpio_set_dir(PUL_PIN, GPIO_OUT);
    gpio_put(PUL_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 1); // 1 for forward, 0 for reverse
}

void actuate_motors(int i){
    if (t1_divs_i==0){
        gpio_put(T1_PUL, false);
    }
    else if(i % t1_divs_i == 0){
        gpio_put(T1_PUL, !t1_pul_state);
        t1_pul_state = !t1_pul_state; 
    }

    if (t2_divs_i==0){
        gpio_put(T2_PUL, false);
    }
    else if(i % t2_divs_i == 0){
        gpio_put(T2_PUL, !t2_pul_state);
        t2_pul_state = !t2_pul_state;
    }

    if (rot_divs_i==0){
        gpio_put(R_PUL, false);
    }
    else if(i % rot_divs_i == 0){
        gpio_put(R_PUL, !r_pul_state);
        r_pul_state = !r_pul_state;
    }
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

    init_motor(T1_PUL,T1_DIR);
    init_motor(T2_PUL,T2_DIR);
    init_motor(R_PUL, R_DIR);

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
    static uint16_t raw_angle_radial = 0;
    static uint16_t raw_angle_tan = 0;
    static uint16_t raw_angle_rotary = 0;
    static uint16_t raw_angle_transverse = 0;

    static float current_angle_radial = 0;
    static float current_angle_tan = 0;
    static float current_angle_rotary = 0;
    static float current_angle_transverse = 0;

    last_angle_radial = read_as5600_angle(0);
    last_angle_radial = (last_angle_radial*360.0f)/4096.0f;
    
    last_angle_tan = read_as5600_angle(1);
    last_angle_tan = (last_angle_tan*360.0f)/4096.0f;
    
    last_angle_rotary = read_as5600_angle(2);
    last_angle_rotary = (last_angle_rotary*360.0f)/4096.0f;

    last_angle_transverse = read_as5600_angle(3);
    last_angle_transverse = (last_angle_transverse*360.0f)/4096.0f;

    y_error_last = 0;
    hub_error_last = 0;
    
    while (true) {
        raw_angle_radial = read_as5600_angle(0);
        current_angle_radial = (raw_angle_radial*360.0f)/4096.0f; 
        calculate_angle_movement_radial(current_angle_radial);

        raw_angle_tan = read_as5600_angle(1);
        current_angle_tan = (raw_angle_tan*360.0f)/4096.0f;
        calculate_angle_movement_tan(current_angle_tan);

        raw_angle_rotary = read_as5600_angle(2);
        current_angle_rotary = (raw_angle_rotary*360.0f)/4096.0f;
        calculate_angle_movement_rotary(current_angle_rotary);
        hub_angle = tot_move_rotary*rotary_ratio;

        raw_angle_transverse = read_as5600_angle(3);
        current_angle_transverse = (raw_angle_transverse*360.0f)/4096.0f;
        calculate_angle_movement_transverse(current_angle_transverse);

        cart_position();
        person_position();
        find_next_person();
        printf("Cart Disp: %6.2f | P_x: %6.2f | P_y: %6.2f\n",current_cart_position[1]*M_TO_MM, current_person_position[0]*M_TO_MM, current_person_position[1]*M_TO_MM);
        
        y_error = current_person_position[1] - current_cart_position[1]; // both units in mm
        // printf("y_error: %6.2f\n", y_error*M_TO_MM);

        arm_ratio =  current_person_position[0]/ARM_LEN;
        dy_ratio = (current_person_position[1] - current_cart_position[1])/ARM_LEN;

        // cap arm ratio
        if (arm_ratio > 1.0f){
            arm_ratio = 1.0f;
        }
        else if (arm_ratio<-1.0f) {
            arm_ratio = - 1.0f;
        }

        // cap dy ratio
        if (dy_ratio>1.0f){
            dy_ratio = 1.0f;
        }
        else if (dy_ratio<-1.0f){
            dy_ratio = -1.0f;
        }

        next_hub_angle = acosf(arm_ratio)*(1/DEG_TO_RAD);

        if (current_person_position[0]>=0 ){
            if (current_person_position[1] <= 0){
                next_hub_angle = -next_hub_angle;
            }
        }
        else if (current_person_position[0]<=0){
            if (current_person_position[1]<=0){
                next_hub_angle = 360.0f - next_hub_angle;
            }
        }

        // printf("Dy Ratio: %6.2f | Hub Angle: %6.2f | Next Hub Angle: %6.2f\n", dy_ratio, hub_angle, next_hub_angle);
        printf("Hub Angle: %6.2f | Next Hub Angle: %6.2f\n", hub_angle, next_hub_angle);

        hub_error = next_hub_angle - hub_angle; // error in degrees
        y_error = y_error*(1/dist_per_deg); // error in degrees
        
        // printf("Hub E: %6.2f | Track E: %6.2f\n", hub_error, y_error);

        hub_error = hub_error*STEP_PER_DEG; // error in steps
        y_error = y_error*STEP_PER_DEG;

        printf("Hub E: %6.2f | Track E: %6.2f\n", hub_error, y_error);

        // calculate PI signal value
        t1_signal = t1_gear_ratio*((KP_T*y_error) + KI_T*BTI*(y_error_last+(0.5*(y_error-y_error_last))));
        // t2_signal = t2_gear_ratio*((KP_T*y_error) + KI_T*BTI*(y_error_last+(0.5*(y_error-y_error_last))));
        r_signal = r_gear_ratio*((KP_R*hub_error) + KI_R*BTI*(hub_error_last+(0.5*(hub_error-hub_error_last))));
        
        // printf("t1 signal: %6.3f | t2 signal: %6.3f | r signals: %6.3f\n", t1_signal, t2_signal, r_signal);

        // set direction before actuating
        if(t1_signal>0){
            gpio_put(T1_DIR, false);
            gpio_put(T2_DIR, true);
        }
        else{
            gpio_put(T1_DIR, true);
            gpio_put(T2_DIR, false);
        }

        if (r_signal>0){
            gpio_put(R_DIR, true);
        }
        else{
            gpio_put(R_DIR, false);
        }

        if (fabs(t1_signal) > 10){
            t1_delay_us = fabs(((BTI*MICRO_MULT*0.5f*BTI_DIVS)/t1_signal));
            // t2_delay_us = fabs(((BTI*MICRO_MULT*0.5f*BTI_DIVS)/t2_signal));
        }
        else {
            t1_delay_us = BTI*MICRO_MULT;
            t2_delay_us = BTI*MICRO_MULT;
        }

        if (fabs(r_signal) > 10){
            rot_delay_us = fabs(((BTI*MICRO_MULT*0.5f*BTI_DIVS)/r_signal));
        }
        else{
            rot_delay_us = BTI*MICRO_MULT;
        }

        // printf("FLOAT t1 delay: %6.3f | t2 delay: %6.3f | r delay: %6.3f\n", t1_delay_us, t2_delay_us, rot_delay_us);

        t1_divs_i = (int)(t1_delay_us);
        t2_divs_i = (int)(t1_delay_us*1.0352);
        rot_divs_i = (int)(rot_delay_us);

        if (t1_divs_i < DIV_CAP){
            t1_divs_i = DIV_CAP;
        }
        if (t2_divs_i < DIV_CAP){
            t2_divs_i = DIV_CAP;
        }
        if (rot_divs_i < DIV_CAP){
            rot_divs_i = DIV_CAP;
        }

        printf("INT t1 divs: %d | t2 divs: %d | r divs: %d\n", t1_divs_i, t2_divs_i, rot_divs_i);
        t1_pul_state = true;
        t2_pul_state = true;
        r_pul_state = true;
            
        for (int i = 0; i < BTI_DIVS; i++){
            actuate_motors(i);
            sleep_us((int)((BTI*MICRO_MULT)/BTI_DIVS));
        }

        t1_pul_state = false;
        t2_pul_state = false;
        r_pul_state = false;

        y_error_last = y_error;
        hub_error_last = hub_error;
            
        // printf("Current Hub Angle: %6.2f | Next Hub Angle: %6.2f \n",hub_angle, next_hub_angle);
        // The angle is a raw 12-bit value (0-4095). Optionally convert to degrees:
        // printf("Radial: %6.2f | Tan: %6.2f | Hub: %6.2f | Transverse: %6.2f\n", tot_move_radial, tot_move_tan, hub_angle, tot_move_transverse);
        // printf("\n");
        // sleep_us((int)(BTI*MICRO_MULT)-(4*ENC_DELAY*MICRO_MULT));
    }

    return 0;
}
