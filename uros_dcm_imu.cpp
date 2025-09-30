#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <math.h>
#include <map>

#include "pico_uart_transports.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include "imu/mpu9250.hpp"
#include "imu/madgwick_filter.hpp"
#include <sensor_msgs/msg/imu.h>





// Motor class with two encoder pins (A and B) for each motor
class Motor {
private:
    const uint LED_PIN;
    const uint ENA_PIN;  // PWM pin for speed control
    const uint IN1_PIN;  // Motor direction pin 1
    const uint IN2_PIN;  // Motor direction pin 2
    const uint ENCODER_A_PIN;  // Encoder A pin
    const uint ENCODER_B_PIN;  // Encoder B pin
    const int TICKS_PER_REV;  // Encoder resolution (ticks per revolution)
    const float GR;  // Gear ratio

    uint pwmSlice;  // PWM slice for motor control

    // PID controller variables (placeholders)
    float Kp, Ki, Kd;
    float previous_error = 0.0f;
    float integral = 0.0f;
    float dt = 0.1f;  // Time step for PID calculations

    // Encoder tracking variables
    volatile int32_t encoder_ticks = 0;
    int32_t last_ticks = 0;
    float filtered_rpm = 0.0f;

    static constexpr float alpha = 0.15f;  // Low-pass filter coefficient

    // Map to store motor instances by encoder pins
    static std::map<uint, Motor*> motor_map;

    // Static interrupt handler (for both A and B pins)
    static void encoder_irq_handler(uint gpio, uint32_t events) {
        if (motor_map.find(gpio) != motor_map.end()) {
            motor_map[gpio]->handle_encoder_interrupt(gpio, events);
        }
    }

    // Non-static method to process encoder interrupts
    void handle_encoder_interrupt(uint gpio, uint32_t events) {
        bool encoder_a = gpio_get(ENCODER_A_PIN);
        bool encoder_b = gpio_get(ENCODER_B_PIN);

        if (gpio == ENCODER_A_PIN) {
            // Process A pin interrupt
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoder_ticks++;  // Forward direction
            } else {
                encoder_ticks--;  // Reverse direction
            }
        } else if (gpio == ENCODER_B_PIN) {
            // Process B pin interrupt
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoder_ticks--;  // Reverse direction
            } else {
                encoder_ticks++;  // Forward direction
            }
        }
    }

public:
    // Constructor to initialize the motor with pins and settings
    Motor(uint led_pin, uint ena_pin, uint in1_pin, uint in2_pin,
          uint enc_a_pin, uint enc_b_pin, int ticks_per_rev = 64,
          float gear_ratio = 50.0f, float kp = 0.1158f, float ki = 0.4634f, float kd = 0.0f)
        : LED_PIN(led_pin), ENA_PIN(ena_pin), IN1_PIN(in1_pin), IN2_PIN(in2_pin),
          ENCODER_A_PIN(enc_a_pin), ENCODER_B_PIN(enc_b_pin),
          TICKS_PER_REV(ticks_per_rev), GR(gear_ratio), Kp(kp), Ki(ki), Kd(kd) {
        
        // Register this motor's encoder pins in the motor map
        motor_map[ENCODER_A_PIN] = this;
        motor_map[ENCODER_B_PIN] = this;

        // Initialize motor driver pins
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, 0);

        gpio_init(IN1_PIN);
        gpio_init(IN2_PIN);
        gpio_set_dir(IN1_PIN, GPIO_OUT);
        gpio_set_dir(IN2_PIN, GPIO_OUT);

        // Set up PWM for speed control
        gpio_set_function(ENA_PIN, GPIO_FUNC_PWM);
        pwmSlice = pwm_gpio_to_slice_num(ENA_PIN);
        pwm_set_wrap(pwmSlice, 65535);
        pwm_set_chan_level(pwmSlice, PWM_CHAN_A, 0);
        pwm_set_enabled(pwmSlice, true);

        // Initialize encoder pins
        gpio_init(ENCODER_A_PIN);
        gpio_init(ENCODER_B_PIN);
        gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
        gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
        gpio_pull_up(ENCODER_A_PIN);
        gpio_pull_up(ENCODER_B_PIN);

        // Register interrupts for encoder pins
        gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
        gpio_set_irq_enabled_with_callback(ENCODER_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
    }

    // Method to control motor speed and direction
    void set_motor(float speed) {
        uint16_t pwm_value = (uint16_t)fabs(speed);

        if (speed > 0) {
            gpio_put(IN1_PIN, 1);
            gpio_put(IN2_PIN, 0);
        } else if (speed < 0) {
            gpio_put(IN1_PIN, 0);
            gpio_put(IN2_PIN, 1);
        } else {
            gpio_put(IN1_PIN, 0);
            gpio_put(IN2_PIN, 0);
        }

        pwm_set_gpio_level(ENA_PIN, (uint16_t)(pwm_value * 65535 / 100));
    }

    // Method to calculate and return RPM
    void calculate_rpm(float* revs, float* rpm) {
        int32_t ticks_since_last = encoder_ticks - last_ticks;
        last_ticks = encoder_ticks;

        *revs = (float)encoder_ticks * (1.0f / 64.0f) * (2*M_PI / 1.0f) * (1.0f / 50.0f);
 
        float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GR);
        *rpm = apply_low_pass_filter(raw_rpm);
    }

    void pos_control(float sp, float pos, float* error, float* cmd)
    {
        float e = (sp - pos);
        integral += e * dt;
        float u = Kp * e + Ki * integral + Kd * (e - previous_error) / dt;

        u = map(u, -1.0f, 1.0f, -100.0f, 100.0f);

        if (u > 100.0f) u = 100.0f;
        if (u < -100.0f) u = -100.0f;

        *cmd = u;
        *error = e;
    }

    void set_pid(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    // Low-pass filter for RPM
    float apply_low_pass_filter(float raw_rpm) {
        filtered_rpm = alpha * raw_rpm + (1.0f - alpha) * filtered_rpm;
        return filtered_rpm;
    }

        // Toggle the onboard LED for indication
    void toggleLED() {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
    }

    float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

// Initialize the static motor map
std::map<uint, Motor*> Motor::motor_map;

// Create a motor object with specific motor and encoder pins, and PID parameters
// Motor(uint led_pin = 25, uint ena_pin = 11, uint in1_pin = 13, uint in2_pin = 12, uint enc_a_pin = 26, uint enc_b_pin = 27, int ticks_per_rev = 64, float gear_ratio = 50.0f, float kp = 0.1158f, float ki = 0.4634f, float kd = 0.0f)

Motor motor1(25, 8, 9, 11, 18, 19, 64, 50.0f, 0.1f, 0.1f, 0.01f);     // (25, 11, 13, 12, 26, 27, 64, 50.0f, 0.1f, 0.1f, 0.01f);

MPU9250 imu(i2c0,12,13);
MadgwickFilter filter;

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

float sp = 0.0f; // position set-point Motor
float kp = 0.0f;
float ki = 0.0f;
float kd = 0.0f;

// Initialize global variables for ROS2
rcl_subscription_t cmd_subs; // Subscriber for speed setpoint
geometry_msgs__msg__Twist cmd_msg; // Message for speed setpoint

rcl_publisher_t debug_pub;    // Publisher for Debugging messages
geometry_msgs__msg__Twist debug_msg;     // Message type for the Debugging messages

// Function to map a value from one range to another
float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ROS2 subscriber callback for receiving Twist messages
void cmd_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * twist_msg_const = (const geometry_msgs__msg__Twist *)msgin;
    printf("Received speed setpoint: %f\n", twist_msg_const->angular.z);

    // Setpoint for the desired RPM
    sp = (float)twist_msg_const->angular.z;
    // PID parameters
    kp = (float)twist_msg_const->linear.x;
    ki = (float)twist_msg_const->linear.y;
    kd = (float)twist_msg_const->linear.z;

}

void imu_callback() {
    static absolute_time_t last_time = get_absolute_time();
    absolute_time_t now = get_absolute_time();
    float dt = to_ms_since_boot(now) - to_ms_since_boot(last_time);
    dt /= 1000.0f;
    last_time = now;

    if (!imu.read_accel_gyro()) return;

    // Update filter
    filter.update(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, dt);

    // Fill IMU message
    imu_msg.linear_acceleration.x = imu.ax;
    imu_msg.linear_acceleration.y = imu.ay;
    imu_msg.linear_acceleration.z = imu.az;

    imu_msg.angular_velocity.x = imu.gx;
    imu_msg.angular_velocity.y = imu.gy;
    imu_msg.angular_velocity.z = imu.gz;

    float x, y, z, w;
    filter.getQuaternion(x, y, z, w);

    imu_msg.orientation.x = x;
    imu_msg.orientation.y = y;
    imu_msg.orientation.z = z;
    imu_msg.orientation.w = w;

    // Optional: set covariance or frame_id
    imu_msg.orientation_covariance[0] = 0.02;
    imu_msg.orientation_covariance[4] = 0.02;
    imu_msg.orientation_covariance[8] = 0.02;

    rcl_publish(&imu_pub, &imu_msg, NULL);
}


// Timer callback to calculate and publish RPM
void debug_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    // Example usage: Calculate the current RPM
	float position1 = 0.0f;
	float speed1 = 0.0f;
    float error = 0.0f;
    float cmd = 0.0f;
    
    motor1.calculate_rpm(&position1, &speed1);

    motor1.set_pid(kp, 0.0f, 0.0f);

    motor1.pos_control(sp, position1, &error, &cmd);

    // Apply the control signal to the motor 1
    motor1.set_motor(cmd);

    // Publish the debug message
    debug_msg.linear.x = position1;
    debug_msg.linear.y = cmd;
	debug_msg.linear.z = error;

    //debug_msg.angular.x = Angles.fRoll;
    //debug_msg.angular.y = Angles.fPitch;
    //debug_msg.angular.z = Angles.fYaw;

    rcl_ret_t ret2 = rcl_publish(&debug_pub, &debug_msg, NULL);

    imu_callback();
    
}



int main() {
    //stdio_init_all();  // Initialize standard I/O (for debugging)
    
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize ROS2 components
    rcl_timer_t debug_timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    rcl_ret_t ret = rmw_uros_ping_agent(1000, 120);  // Wait for the Micro-ROS agent
    if (ret != RCL_RET_OK) {
        printf("Failed to connect to Micro-ROS agent.\n");
        return ret;
    }

    // Initialize support and node
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);


    // Initialize the publisher to publish setpoint RPM
    rclc_publisher_init_default(
        &debug_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug_pub"
    );

    rclc_publisher_init_default( 
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"
    );

    // Initialize the timer for RPM (100ms interval)
    rclc_timer_init_default2(
        &debug_timer,
        &support,
        RCL_MS_TO_NS(100),
        debug_timer_callback,
        true
    );

    // Initialize the subscriber to receive Twist messages
    rclc_subscription_init_default(
        &cmd_subs,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd"
    );


    // Initialize executor and add the subscription
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    
    rclc_executor_add_subscription(&executor, &cmd_subs, &cmd_msg, &cmd_callback, ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &debug_timer);

    motor1.toggleLED();

    // Initialize I2C for MPU9250


    imu.init();

    while (true) {

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    return 0;
}
