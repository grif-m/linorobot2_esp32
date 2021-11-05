//THIS is a port of https://github.com/linorobot/linorobot2_hardware to ESP32

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
bool micro_ros_init_successful = false;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#define STRING_BUFFER_LEN 50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t sub_cmd;
rcl_publisher_t pub_joint;
const unsigned int timer_period = RCL_MS_TO_NS(500); //publish joint state every 500 ms
rcl_timer_t timer_joints;

std_msgs__msg__String incoming_cmd;

sensor_msgs__msg__JointState joint_state_msg;

int device_id;
int seq_no;
char inp_string[STRING_BUFFER_LEN];

void appMain(void *argument)
{
	/*
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "hexapod", "", &support));

	// Create a best effort publisher
	RCCHECK(rclc_publisher_init_default(&pub_joint, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
		"/hexapod/joint_states"));

	// Create and initialize timer object for publishing joints
	RCCHECK(rclc_timer_init_default(&timer_joints, &support, timer_period, pub_joint_callback));

	// Create a best effort subscriber
	RCCHECK(rclc_subscription_init_default(&sub_cmd, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/hexapod/cmd"));

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); //!!!Provide total number of timers + subscriptions
	RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &incoming_cmd,
		&sub_cmd_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer_joints));

	// Create and allocate the messages
	char incoming_cmd_buffer[STRING_BUFFER_LEN];
	incoming_cmd.data.data = incoming_cmd_buffer;
	incoming_cmd.data.capacity = STRING_BUFFER_LEN;

	joint_state_msg.name.capacity = 12;
	joint_state_msg.name.size = 12;
	joint_state_msg.name.data = (std_msgs__msg__String*) malloc(joint_state_msg.name.capacity*sizeof(std_msgs__msg__String));

	for(int i=0;i<12;i++) {
		joint_state_msg.name.data[i].data = malloc(5);
		joint_state_msg.name.data[i].capacity = 5;
		sprintf(joint_state_msg.name.data[i].data,"j%d",i);
		joint_state_msg.name.data[i].size = strlen(joint_state_msg.name.data[i].data);
	}

	joint_state_msg.position.size=12;
	joint_state_msg.position.capacity=12;
	joint_state_msg.position.data = malloc(joint_state_msg.position.capacity*sizeof(double));
	joint_state_msg.velocity.size=12;
	joint_state_msg.velocity.capacity=12;
	joint_state_msg.velocity.data = malloc(joint_state_msg.velocity.capacity*sizeof(double));
	joint_state_msg.effort.size=12;
	joint_state_msg.effort.capacity=12;
	joint_state_msg.effort.data = malloc(joint_state_msg.effort.capacity*sizeof(double));

	for(int i=0;i<12;i++) {
		joint_state_msg.position.data[i]=0.0;
		joint_state_msg.velocity.data[i]=0.0;
		joint_state_msg.effort.data[i]=0.0;
	}

	RCSOFTCHECK(rcl_publish(&pub_joint, &joint_state_msg, NULL));

	device_id = rand();

	mc_init();
	mc.state = MC_STATE_IDLE;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10000);
	}

	// Free resources
	RCCHECK(rcl_timer_fini(&timer_joints));
	RCCHECK(rcl_subscription_fini(&sub_cmd, &node));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rcl_publisher_fini(&pub_joint, &node));
	*/
}


void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(150 / portTICK_RATE_MS);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(150 / portTICK_RATE_MS);
    }
    vTaskDelay(1000 / portTICK_RATE_MS);
}