#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <my_tfg_interfaces/msg/float_data_node.h>
#include <my_tfg_interfaces/msg/status_node.h>

#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// It need a motor library, it's just for simulate now

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t temperature_subscriber;
rcl_publisher_t status_publisher;

my_tfg_interfaces__msg__FloatDataNode msg_temperature;
my_tfg_interfaces__msg__StatusNode msg_status;


const float temperature_to_act = 20.0;

// 1-9 Controllers, 10-19 Sensors, 20-29 Heaters, 30-39 Windows, 40-49 Irrigation_Nodes
const float pos_x = 12.0;
const float pos_y = 12.0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
    {
		RCSOFTCHECK(rcl_publish(&status_publisher, &msg_status, NULL));
    }
}

void subscription_callback(const void * msgin)
{
	const my_tfg_interfaces__msg__FloatDataNode * msg = (const my_tfg_interfaces__msg__FloatDataNode *)msgin;
	printf("Temperature: %f\n From x: %f y: %f\n Device id: %d\n",
			msg->data,
			msg->position.x,
			msg->position.y,
			msg->device_id);
	if (msg->data >= temperature_to_act && msg_status.work_status == 0) {
		msg_status.work_status = 1;
		rcl_publish(&status_publisher, &msg_status, NULL);
	} else if ( msg->data < temperature_to_act && msg_status.work_status != 0) {
		msg_status.work_status = 0;
		rcl_publish(&status_publisher, &msg_status, NULL);
	}
}

void appMain(void * arg)
{
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "window_30", "", &support));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&temperature_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(my_tfg_interfaces, msg, FloatDataNode),
		"temperature"));

	RCCHECK(rclc_publisher_init_default(
		&status_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(my_tfg_interfaces, msg, StatusNode),
		"status"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 30000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &temperature_subscriber, &msg_temperature, &subscription_callback, ON_NEW_DATA));

	my_tfg_interfaces__msg__FloatDataNode__init(&msg_temperature);
	my_tfg_interfaces__msg__StatusNode__init(&msg_status);

	msg_status.work_status = 0;
	msg_status.position.x = pos_x;
	msg_status.position.y = pos_y;

	while(1){
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			usleep(100000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&temperature_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&status_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}
