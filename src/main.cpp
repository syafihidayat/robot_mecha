#include <Arduino.h>

#include <stdio.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "odometry.h"
#include "config.h"
#include "kinematic.h"
#include "pid.h"
#include "imu.h"
#include "Servo.h"

#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

rcl_subscription_t twist_subscriber;
rcl_subscription_t button_sub;
rcl_subscription_t proxy_data_sub;
rcl_subscription_t allbutton;

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t checking_input;
rcl_publisher_t proxy1_publisher;

std_msgs__msg__Int8 button_msg;
std_msgs__msg__Int8 allbutton_msg;
std_msgs__msg__Bool proxy_data_msg;
std_msgs__msg__Float32MultiArray checking_input_msg;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Bool bool_msg;
// std_msgs__msg__Int8 int_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

volatile bool proxy_state = false;
volatile bool proxy_changed = false;

void setMotor(int cwPin, int ccwPin, float pwmVal);
void moveBase();
void publishData();
void proxyPublish();
void twistCallback(const void *msgin);
void allbuttonCallback(const void *msgin);
void proxy_data_callback(const void *msgin);
// void gripp_servo(bool state);
void init_servo_position();
void update_grip_sequence();
void start_grip_sequence();
void syncTime();
void error_loop();
struct timespec getTime();
bool createEntities();
bool destroyEntities();
void flashLED(int n_times);
template <int j>
void readEncoder();


#define RCCHECK(fn)                  \
{                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
    #define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)
    
    unsigned long long time_offset = 0;
    unsigned long prev_cmd_time = 0;
    unsigned long prev_odom_update = 0;
    unsigned long prevT = 0;
    
    enum states
    {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } state;

    enum GripStep{
        GRIP_IDLE,
        LIFTER_DOWN,
        GRIPPER_CLOSE,
        GRIPPER_OPEN,
        LIFTER_UP
    };

    GripStep grip_step = GRIP_IDLE;

    unsigned long grip_timer = 0;
    bool grip_target = false;
    
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
    
    const int enca[6] = {MOTOR1_ENCODER_A, MOTOR3_ENCODER_A, MOTOR4_ENCODER_A, launcher_up_A, launcher_down_A, encA_X};
    const int encb[6] = {MOTOR1_ENCODER_B, MOTOR3_ENCODER_B, MOTOR4_ENCODER_B, launcher_up_B, launcher_down_B, encB_X};
    
    volatile long pos[6];
    
    PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
    PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
    PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
    PID wheel4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
    PID external_encoder1(0, 0, 0, 0, 0);
// PID external_encoder2(0, 0, 0, 0, 0);
PID external_encoder3(0, 0, 0, 0, 0);

Kinematic Kinematics(
    Kinematic::LINO_BASE,
    MOTOR_MAX_RPS,
    MAX_RPS_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    ROBOT_DIAMETER);
    
    Odometry odometry;
    Servo srv_lifter;
    Servo srv_gripper;
    IMU imu_sensor;
    
    void setup()
    {
        Serial.begin(115200);
        set_microros_serial_transports(Serial);
        
    while (!imu_sensor.init())
    {
        flashLED(3);
    }

    for (int i = 0; i < 6; i++)
    {

        pinMode(cw[i], OUTPUT);
        pinMode(ccw[i], OUTPUT);

        pinMode(enca[i], INPUT);
        pinMode(encb[i], INPUT);

        analogWriteFrequency(cw[i], PWM_FREQUENCY);
        analogWriteFrequency(ccw[i], PWM_FREQUENCY);

        analogWriteResolution(PWM_BITS);
        analogWrite(cw[i], 0);
        analogWrite(ccw[i], 0);
    }

    // srv_lifter.attach(srv_gripper_pin);
    // srv_gripper.attach(srv_gripper_pin);

    // srv_lifter.write(0);
    // srv_gripper.write(120);


    pinMode(proxy1, INPUT);
    external_encoder1.ppr_total(1024);
    external_encoder3.ppr_total(1024);

    wheel1.ppr_total(COUNTS_PER_REV1);
    wheel2.ppr_total(COUNTS_PER_REV2);
    wheel3.ppr_total(COUNTS_PER_REV3);
    wheel4.ppr_total(COUNTS_PER_REV4);

    attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[4]), readEncoder<4>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[5]), readEncoder<5>, RISING);

    init_servo_position();

    pinMode(LED_PIN, OUTPUT);
}

bool servo_initialized = false;

void init_servo_position(){

    srv_lifter.attach(srv_lifter_pin);
    srv_gripper.attach(srv_gripper_pin);

    srv_gripper.write(120);
    srv_lifter.write(90);

    // delay(200);

    // srv_lifter.detach();
    // srv_gripper.detach();

    servo_initialized = true;
}

void loop()
{

    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
            for (int i = 0; i < 6; i++)
            {
                pos[i] = 0;
            }
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
            if(proxy_changed)
            {
                proxy_changed = false;

                if(!proxy_state && servo_initialized)
                {
                    start_grip_sequence();
                }
            }
            update_grip_sequence();
            publishData();
            moveBase();
            proxyPublish();


        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();

        setMotor(cw[0], ccw[0], 0);
        setMotor(cw[1], ccw[1], 0);
        setMotor(cw[2], ccw[2], 0);
        setMotor(cw[3], ccw[3], 0);
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

float toLinear(double pos, float radius)
{
    return pos * radius;
}

void moveBase()
{

    sensors_event_t event, angVelocityData;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    unsigned long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;

    if (((millis() - prev_cmd_time) >= 200))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }


    Kinematic::rps req_rps;
    req_rps = Kinematics.getRPS(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z,
        event.orientation.x);

    float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
    float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
    float controlled_motor3 = wheel3.control_speed(req_rps.motor3, pos[2], deltaT);

    float current_rps1 = wheel1.get_filt_vel();
    float current_rps2 = wheel2.get_filt_vel();
    float current_rps3 = wheel3.get_filt_vel();

    if (fabs(req_rps.motor1) < 0.02)
    {
        controlled_motor1 = 0.0;
    }
    if (fabs(req_rps.motor2) < 0.02)
    {
        controlled_motor2 = 0.0;
    }
    if (fabs(req_rps.motor3) < 0.02)
    {
        controlled_motor3 = 0.0;
    }

    setMotor(cw[0], ccw[0], controlled_motor1);
    setMotor(cw[1], ccw[1], controlled_motor2);
    setMotor(cw[2], ccw[2], controlled_motor3);

    Kinematic::velocities vel = Kinematics.getVelocities(
        current_rps1,
        current_rps2,
        current_rps3);

    float vel_enc1 = external_encoder1.convert_speed(pos[3], deltaT);
    float vel_enc3 = external_encoder3.convert_speed(pos[5], deltaT);

    float vx = toLinear(vel_enc1, 0.02375) * -1;
    float vy = toLinear(vel_enc3, 0.02375);

    float yaw = event.orientation.z * (M_PI / 180.0);

    if(fabs(angVelocityData.gyro.z) > 0.25)
    {
        vx = 0;
        vy = 0;
    }

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        vx,
        vy,
        angVelocityData.gyro.z,
        yaw        
    );

    prevT = currT;

    checking_input_msg.data.data[0] = current_rps1;
    checking_input_msg.data.data[1] = current_rps2;
    checking_input_msg.data.data[2] = current_rps3;
    checking_input_msg.data.data[3] = angVelocityData.gyro.z;
    checking_input_msg.data.data[4] = pos[3];
    checking_input_msg.data.data[5] = pos[5];

    RCSOFTCHECK(rcl_publish(&checking_input, &checking_input_msg, NULL));

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
}

void proxyPublish()
{

    int state = digitalRead(proxy1);

    bool_msg.data = (state == HIGH);
    
    RCSOFTCHECK(rcl_publish(&proxy1_publisher, &bool_msg, NULL));

}

void publishData()
{

    odom_msg = odometry.getData();
    imu_msg = imu_sensor.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCCHECK(rclc_node_init_default(&node, "hardware_node", "", &support));

    executor = rclc_executor_get_zero_initialized_executor();

    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "omni_cont/cmd_vel"));

    RCCHECK(rclc_subscription_init_default(
        &allbutton,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "allbutton"));

    RCCHECK(rclc_subscription_init_default(
        &proxy_data_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/true_sensor"
    ))

    RCCHECK(rclc_executor_init(&executor, &support.context, 9, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &allbutton,
        &allbutton_msg,
        &allbuttonCallback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &proxy_data_sub,
        &proxy_data_msg,
        &proxy_data_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_publisher_init_default(
        &checking_input,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "checking_input"));
    checking_input_msg.data.data = (float *)malloc(4 * sizeof(float)); // Sesuaikan jumlah elemen
    checking_input_msg.data.size = 6;

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_extern/data"));

    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));

    RCCHECK(rclc_publisher_init_default(
        &proxy1_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "proxydata"));

    syncTime();
    digitalWrite(LED_PIN, HIGH);
    return true;
}

bool destroyEntities()
{

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCCHECK(rcl_publisher_fini(&proxy1_publisher, &node));
    RCCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_timer_fini(&control_timer));
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void syncTime()
{
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();

    time_offset = ros_time_ms - now;
}

void error_loop()
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
}

void twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    twist_msg = *msg;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev_cmd_time = millis();
}

void proxy_data_callback(const void *msgin)
{
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;

    if(msg->data != proxy_state)
    {
        proxy_state = msg->data;
        proxy_changed = true;
    }
}

bool grip_running = false;

void start_grip_sequence()
{
    if(grip_step != GRIP_IDLE)return;


    grip_running = true;
    grip_step = LIFTER_DOWN;
    grip_timer = millis();

    srv_lifter.attach(srv_lifter_pin);
    srv_gripper.attach(srv_gripper_pin);

    
    srv_lifter.write(135);
    // srv_gripper.write(120);

}


void update_grip_sequence()
{
    if(grip_step == GRIP_IDLE) return;
    if(millis() - grip_timer < 1000) return;

    grip_timer = millis();

    switch(grip_step)
    {
        case LIFTER_DOWN:
            srv_gripper.write(30);
            grip_step = GRIPPER_CLOSE;
            break;

        case GRIPPER_CLOSE:
            srv_lifter.write(0);
            grip_step = LIFTER_UP;
            break;

        case LIFTER_UP:
            srv_gripper.write(120);
            grip_step = GRIPPER_OPEN;
            break;

        case GRIPPER_OPEN:
            srv_lifter.write(90);
            grip_step = GRIP_IDLE;
            grip_running = false;
            break;

        default:
            grip_step = GRIP_IDLE;
            grip_running = false;
            break;

    }
}

// void gripp_servo(bool state)
// {

//     srv_lifter.attach(srv_lifter_pin);
//     srv_gripper.attach(srv_gripper_pin);

//     if(state){

//         srv_lifter.write(180);
//         srv_gripper.write(35);
//     }
    
//     else
//     {
//         srv_lifter.write(0);
//         srv_gripper.write(120);
//     }

//     // delay(300);

//     // srv_lifter.detach();
//     // srv_gripper.detach();

// }

struct timespec getTime()
{
    struct timespec tp = {0};

    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }

    delay(1000);
}

void allbuttonCallback(const void *msgin)
{
    const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
    allbutton_msg = *msg;
    switch (allbutton_msg.data)
    {
    case 0:
        button.A = 1;
        break;
    case 1:
        button.B = 1;
        break;
    case 2:
        button.X = 1;
        break;
    case 3:
        button.Y = 1;
        break;
    case 4:
        button.LB = 1;
        break;
    case 5:
        button.RB = 1;
        break;
    case 6:
        button.LT = 1;
        break;
    case 7:
        button.RT = 1;
        break;
    case 8:
        button.select = 1;
        break;
    case 9:
        button.start = 1;
        break;
    case 10:
        button.home = 1;
        break;

    default:
        button.A = 0;
        button.B = 0;
        button.X = 0;
        button.Y = 0;
        button.LB = 0;
        button.RB = 0;
        button.LT = 0;
        button.RT = 0;
        button.select = 0;
        button.start = 0;
        button.home = 0;
        break;
    }
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
    if (pwmVal > 0)
    {
        analogWrite(cwPin, fabs(pwmVal));
        analogWrite(ccwPin, 0);
    }
    else if (pwmVal < 0)
    {
        analogWrite(cwPin, 0);
        analogWrite(ccwPin, fabs(pwmVal));
    }
    else
    {
        analogWrite(cwPin, 0);
        analogWrite(ccwPin, 0);
    }
}

template <int i>
void readEncoder()
{
    int b = digitalRead(encb[i]);
    if (b > 0)
    {
        pos[i]++;
    }
    else
    {
        pos[i]--;
    }
}
