#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "MotionControl.h"
#include "Movements.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"


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

int parseInput();

void sub_cmd_callback(const void * msgin)
{
	const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
	ESP_LOGI("Callback", "Received message: %s", msg->data.data);
	strcpy(inp_string,msg->data.data);
	parseInput();
}

void pub_joint_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if( timer != NULL ) {
		for(int i=0;i<12;i++) joint_state_msg.position.data[i]=3.14*(mc.servo[i]-90)/180;
		RCSOFTCHECK(rcl_publish(&pub_joint, &joint_state_msg, NULL));
	}
}

void appMain(void *argument)
{
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
}


///////////////////////////////////////////////////////////////////////////
//PARSING                                                                //
///////////////////////////////////////////////////////////////////////////
void do_Move(mv_def* mv);

int parseInput() { //returns number of characters processed
	ESP_LOGI("Parse", "Parsing message: %s", inp_string);
	switch( inp_string[0] ) {
		case 'C':
		switch(inp_string[1]) {
			case 'F': do_Move(&mvForward); break;
			case 'B': do_Move(&mvBackward); break;
			case 'R': do_Move(&mvTurnRight); break;
			case 'L': do_Move(&mvTurnLeft); break;
			case 'S': do_Move(&mvStand); break;
			case 'M':
				switch(inp_string[2]) {
					case '0': do_Move(&mv0); break;
					case '1': do_Move(&mv1); break;
					case '2': do_Move(&mv2); break;
					case '3': do_Move(&mv3); break;
					case '4': do_Move(&mv4); break;
					case '5': do_Move(&mv5); break;
				}
				break;
		}
		return 2;
		break;
		case 'R':
			//for(int i=0;i<6;i++) mc_setServo(i,90);
			//for(int i=6;i<12;i++) mc_setServo(i,90);
		return 1;
		break;
	}
	return 0;
	
}


///////////////////////////////////////////////////////////////////////////
//TASKS                                                                  //
///////////////////////////////////////////////////////////////////////////

void task_Move(mv_def* mv) {
	ESP_LOGI("Task", "Executing movement %s on core %d. Current state: %d", mv->name, xPortGetCoreID(), mc.state);
	if( mc.state==MC_STATE_MOVING ) {
		mc.state = MC_STATE_FINISHING;
		ESP_LOGI("Task", "Set state to Finishing and waiting to be idle...");
	}
	while( mc.state!=MC_STATE_IDLE ) vTaskDelay( 500 / portTICK_PERIOD_MS );
	mc_doMotion(mv,-1);
	ESP_LOGI("Task", "Completed executing movement %s on core %d", mv->name, xPortGetCoreID());
	vTaskDelete(NULL);
}

void do_Move(mv_def* mv) {
	xTaskCreate(task_Move, "task_Move", 1024 * 2, mv, 0, NULL);
}

