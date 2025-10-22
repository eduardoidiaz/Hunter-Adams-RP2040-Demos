/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 4  ---> PWM Output
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
// Include custom libraries
#include "vga16_graphics_v2.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// Variable to hold PWM slice number
uint slice_num ;
// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  30.0f
#define PWM_OUT 4

// PWM duty cycle
volatile int control ;
volatile int old_control ;

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3] ;

// Complementary Filter vars
volatile float comp_angle_roll = 0.0 ; // filtered angle degrees
const float dt = 0.001 ; // time step in seconds (1.0/1000 Hz)
const float percent_gyro = 0.90 ; // filter coefficient (how much of gyro to mix in)

// P-controller vars
volatile float targ_ang = 0.0 ;
volatile float Kp = 42.0 ;
const int pwm_max = WRAPVAL ;
const int pwm_min = 0 ;
const int pwm_neut = 2500;

// other controller vars
volatile float Ki = 0.0 ;
volatile float integral_term = 0.0 ;
volatile float Kd = 2.0 ;
volatile float d_term = 0.0 ;
static int d_term_count = 0 ;
volatile float prev_angle = 0 ;
const float integral_deadband = 25.0 ;

// lowpass control signal vars
volatile float filt_control = pwm_neut ;
const float control_filt_coeff = 0.3 ;

// character array
char screentext[40];

// draw speed
int threshold = 10 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// PWM duty cycle vars
volatile int control ;
volatile int old_control ;

// PWM Interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(slice_num);
    
    mpu6050_read_raw(acceleration, gyro);

    /////////////// COMP FILTER BEGIN /////////////////////
    // convert raw fix15 values 
    float gyro_x_rate = fix2float15(gyro[0]) ; // DPS
    float acc_y = fix2float15(acceleration[1]) ;
    float acc_z = fix2float15(acceleration[2]) ;

    // Calculate accel angle
    // atan2f in rad, * (180/pi) to degrees
    float accel_angle_roll = atan2f(acc_z, acc_y) * (180.0 / M_PI) - 90.0;

    // calculate gyro angle from gyroscope
    // prev_angle + (rate * dt)
    float gyro_angle_roll = comp_angle_roll + (gyro_x_rate * dt) ;

    // do complementary filtering
    comp_angle_roll = percent_gyro * gyro_angle_roll 
    + (1.0 - percent_gyro) * accel_angle_roll ;
    /////////////// COMP FILTER END ////////////////////////

    ///////////////   PID CONTROLLER  ////////////////////////
    float error = targ_ang - comp_angle_roll ;
    
    // prop term calc
    float p_term = Kp * error ;

    // int term calc
    float i_term = Ki * integral_term ;

    // deriv term calc
    // float derivative = (comp_angle_roll - prev_angle) / dt ;
    // float d_term = Kd * derivative ;
    // prev_angle = comp_angle_roll ;
    d_term_count++;
    if (d_term_count > 2) {
        float derivative = (comp_angle_roll - prev_angle) / (dt * 3.0) ;
        d_term = Kd * derivative ;
        prev_angle = comp_angle_roll ;
        d_term_count = 0 ;
    }

    // Calc control signal 
    int new_control =  pwm_neut + (int)(p_term + i_term - d_term);

    // Constrain to pwm range
    int constrained_control = new_control ;
    if (new_control > pwm_max) new_control = pwm_max ;
    else if (new_control < pwm_min) new_control = pwm_min ;

    // Anti windup/deadzone overshoot fix
    if (fabs(error) > integral_deadband) {
        integral_term = 0.0 ;
    }
    else if (constrained_control == new_control) {
        integral_term += (error * dt) ;
    }
    // allow integral to change on edges of pwm range
    else if ((constrained_control > pwm_max) && (error < 0)) {
        integral_term += (error * dt);
    }
    else if ((constrained_control < pwm_min) && (error > 0)) {
        integral_term += (error * dt) ;
    }

    // LOW PASS FILTER
    filt_control = (control_filt_coeff * (float) new_control) + ((1.0 - control_filt_coeff) * filt_control) ;

    // Update volatile control var to enact pwm changes calculated
    control = (int)filt_control ;
    ///////////////  END CONTROLLER ////////////////////////

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

     // Update duty cycle
    if (control != old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control) ;
    }
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;

    // Angle Plot params
    static float min_angle = -90.0 ;
    static float angle_range = 180.0 ;
    static int angle_y_bot = 230 ;
    static int angle_y_top = 80 ;
    static int angle_px_h = 150 ;

    // Motor Plot params
    static float motor_min = 0.0 ;
    static float motor_rng = WRAPVAL ;
    static int motor_y_bot = 430 ;
    static int motor_y_top = 280 ;
    static int motor_px_h = 150 ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "2500") ;
    setCursor(45, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "5000") ;
    setCursor(45, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+90") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-90") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);

        // debug angle calc reading
        setCursor(150, 50) ;
        fillRect(240, 50, 50, 20, BLACK) ;
        char angle_buffer[6];
        char angle_out[22] = "measured angle: " ;
        sprintf(angle_buffer, "%2.2f", comp_angle_roll) ;
        strcat(angle_out, angle_buffer);
        writeString(angle_out) ;
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot (motor sig)
            int motor_y = motor_y_bot - (int)(motor_px_h * ((float)filt_control - motor_min) / motor_rng) ;
            // plot bounding
            if (motor_y < motor_y_top) motor_y = motor_y_top ;
            if (motor_y > motor_y_bot) motor_y = motor_y_bot ;
            drawPixel(xcoord, motor_y, RED) ;

            // draw top plot (angle)
            int angle_y = angle_y_bot - (int)(angle_px_h * ((comp_angle_roll - min_angle) / angle_range)) ;
            if (angle_y < angle_y_top) angle_y = angle_y_top ;
            if (angle_y > angle_y_bot) angle_y = angle_y_bot ;
            drawPixel(xcoord, angle_y, WHITE) ;

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static int test_in ;
    static float float_in ;
    while(1) {
        sprintf(pt_serial_out_buffer, "Commands: [t]hresh, [a]ngle, [p]gain, [i]gain, [d]gain");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%c", &classifier) ;

        if (classifier =='t') {
            sprintf(pt_serial_out_buffer, "Timestep: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            if (test_in > 0) {
                threshold = test_in ;
            }
        }
        else if (classifier == 'a') {
            sprintf(pt_serial_out_buffer, "Enter target angle (float): ") ;
            serial_write ;
            serial_read ;
            sscanf(pt_serial_in_buffer, "%f", &float_in) ;
            targ_ang = float_in ;
            integral_term = 0.0 ; // reset integral term for new target
            sprintf(pt_serial_out_buffer, "Target angle set to %.2f\n", targ_ang) ;
            serial_write ;
        }
        else if (classifier =='p') {
            sprintf(pt_serial_out_buffer, "Proportional Gain (float): ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%f", &float_in) ;
            Kp = float_in ;
            sprintf(pt_serial_out_buffer, "Kp set to: %.2f\n", Kp) ;
            serial_write ;
        }
        else if (classifier == 'i') {
            sprintf(pt_serial_out_buffer, "input a integral gain (Ki): ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%f", &float_in) ;
            Ki = float_in ;
            sprintf(pt_serial_out_buffer, "Ki set to %.2f\n", Ki) ;
            serial_write ;
        }
        else if (classifier == 'd') {
            sprintf(pt_serial_out_buffer, "input a derivative gain (Kd): ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%f", &float_in) ;
            Kd = float_in ;
            sprintf(pt_serial_out_buffer, "Kd set to %.2f\n", Kd) ;
            serial_write ;
        }
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // MPU6050 initialization
    mpu6050_reset();
    // let sensor stabilize and then seed with data for smooth start
    sleep_ms(100);
    for (int i = 0; i < 10 ; i++) {
        mpu6050_read_raw(acceleration, gyro);
        sleep_ms(10);
    }
    mpu6050_read_raw(acceleration, gyro) ;

    float acc_y = fix2float15(acceleration[1]) ;
    float acc_z = fix2float15(acceleration[2]) ;
    float initial_angle = atan2f(acc_z, acc_y) * (180.0 / M_PI) - 90.0 ;

    comp_angle_roll = initial_angle ;
    prev_angle = initial_angle ;
    integral_term = 0.0 ;
    filt_control = pwm_neut ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    // gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to PWM_OUT (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // Invert the PWM output
    pwm_set_output_polarity(slice_num, 0, 1) ;

    // This sets duty cycle
    //pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;
}
