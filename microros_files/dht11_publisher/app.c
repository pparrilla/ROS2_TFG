#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <custom_node_message/msg/float_data_node.h>
#include <custom_node_message/msg/status_node.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static const dht_sensor_type_t sensor_type = DHT_TYPE_DHT11;
static const gpio_num_t dht_gpio = 17;

rcl_publisher_t temperature_publisher;
rcl_publisher_t humidity_publisher;
rcl_publisher_t status_publisher;

custom_node_message__msg__FloatDataNode msg_temperature;
custom_node_message__msg__FloatDataNode msg_humidity;
custom_node_message__msg__StatusNode msg_status;

float temperature = 0;
float humidity = 0;

const int device_id = 11;
const float pos_x = 5.0;
const float pos_y = 5.0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
    {
		RCSOFTCHECK(rcl_publish(&temperature_publisher, &msg_temperature, NULL));
		RCSOFTCHECK(rcl_publish(&humidity_publisher, &msg_humidity, NULL));
        if (dht_read_float_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
        {
            msg_temperature.data = temperature;
            msg_humidity.data = humidity;
			msg_status.is_working = true;
        }
        else
        {
            printf("Could not read data from sensor\n");
			msg_status.is_working = false;
        }
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
	RCCHECK(rclc_node_init_default(&node, "sensor_11", "", &support));

	// create publisher for temperature
	RCCHECK(rclc_publisher_init_default(
		&temperature_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(custom_node_message, msg, FloatDataNode),
		"temperature"));

    // create publisher for humidity
	RCCHECK(rclc_publisher_init_default(
		&humidity_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(custom_node_message, msg, FloatDataNode),
		"humidity"));

	// create publisher for status
	RCCHECK(rclc_publisher_init_default(
		&status_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(custom_node_message, msg, StatusNode),
		"status"));

	custom_node_message__msg__FloatDataNode__init(&msg_temperature);
	custom_node_message__msg__FloatDataNode__init(&msg_humidity);
	custom_node_message__msg__StatusNode__init(&msg_status);

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

	// Assign values
	msg_temperature.device_id = device_id;
	msg_humidity.device_id = device_id;
	msg_status.device_id = device_id;

	msg_temperature.position.x = pos_x;
	msg_humidity.position.x = pos_x;
	msg_status.position.x = pos_x;
	msg_temperature.position.y = pos_y;
	msg_humidity.position.y = pos_y;
	msg_status.position.y = pos_y;

	msg_temperature.data = 0;
	msg_humidity.data = 0;
	msg_status.is_working = false;


	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&temperature_publisher, &node))
	RCCHECK(rcl_publisher_fini(&humidity_publisher, &node))
	RCCHECK(rcl_publisher_fini(&status_publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
