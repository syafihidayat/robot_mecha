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

#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int8.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

rcl_subscription_t twist_subscriber;
rcl_subscription_t button_sub;
rcl_subscription_t allbutton;

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t checking_input;

std_msgs__msg__Int8 button_msg;
std_msgs__msg__Int8 allbutton_msg;
std_msgs__msg__Float32MultiArray checking_input_msg;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

void setMotor(int cwPin, int ccwPin, float pwmVal);
void moveBase();
void publishData();
void twistCallback(const void *msgin);
void allbuttonCallback(const void *msgin);
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
    external_encoder1.ppr_total(1024);
    // external_encoder2.ppr_total(1024);
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

    pinMode(LED_PIN, OUTPUT);
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
            publishData();
            moveBase();
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

// float toLinear(float omega)
// {
//     return omega * 0.02375;
// }

// float toLinear(double pos, double PPR)
// {
//     return pos * (2 * M_PI * 0.02375);
// }

float toLinear(double pos, float radius)
{
    return pos * radius;
}

// float toLinear(float ticks_per_sec, float PPR, float wheel_radius)
// {
//     float rev_per_sec = ticks_per_sec / PPR;
//     float meters_per_sec = rev_per_sec * (2 * M_PI * wheel_radius);
//     return meters_per_sec;
// }

// float toRad(float deg)
// {
//     return deg * M_PI / 180;
// }


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
    // float vel_enc2 = external_encoder2.convert_speed(pos[4], deltaT);
    float vel_enc3 = external_encoder3.convert_speed(pos[5], deltaT);

    // float yawH = 0.5 * ((toLinear(vel_enc1, 1024) + toLinear(vel_enc2, 1024))) + 0.5 * (angVelocityData.gyro.z);

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
        // toLinear(vel_enc1, 1024) * -1,
        // toLinear(vel_enc3, 1024),
        angVelocityData.gyro.z,
        yaw
        // event.orientation.x        
    );

    prevT = currT;

    checking_input_msg.data.data[0] = current_rps1;
    checking_input_msg.data.data[1] = current_rps2;
    checking_input_msg.data.data[2] = pos[3];
    checking_input_msg.data.data[3] = pos[5];
    checking_input_msg.data.data[4] = odometry.get_heading_();

    RCSOFTCHECK(rcl_publish(&checking_input, &checking_input_msg, NULL));

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
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

    RCCHECK(rclc_publisher_init_default(
        &checking_input,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "checking_input"));
    checking_input_msg.data.data = (float *)malloc(4 * sizeof(float)); // Sesuaikan jumlah elemen
    checking_input_msg.data.size = 4;

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
