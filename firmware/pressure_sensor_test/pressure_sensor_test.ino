/* Blue Robotics MS5837 Library Example
-----------------------------------------------------
 
Title: Blue Robotics MS5837 Library Example

Description: This example demonstrates the MS5837 Library with a connected
sensor. The example reads the sensor and prints the resulting values
to the serial terminal.

The code is designed for the Arduino Uno board and can be compiled and 
uploaded via the Arduino 1.0+ software.

-------------------------------
The MIT License (MIT)

Copyright (c) 2015 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

#include <Wire.h>
#include "MS5837.h"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

MS5837 sensor;

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  // Serial.println("start");
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data = sensor.depth();
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  // Serial.println("Timer callback");
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  
  // Serial.begin(9600);
  
  // Serial.println("Starting");
  
  Wire2.begin();

  // Serial.println("Wire ports set");

  allocator = rcl_get_default_allocator();
  // Serial.println("1");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // Serial.println("2");
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  // Serial.println("3");
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pressure_sensor_test"));
  // Serial.println("Message type set");
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Serial.println("Allocator set");
  
  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init(Wire2)) {
    // Serial.println("Init failed!");
    // Serial.println("Are SDA/SCL connected correctly?");
    // Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    // Serial.println("\n\n\n");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(5000);
  }
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  msg.data = 0.0;
}

void loop() {
  // Update pressure and temperature readings
  // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  sensor.read();
  // digitalWrite(LED_PIN, 1);

  // Serial.print("Pressure: "); 
  // Serial.print(sensor.pressure()); 
  // Serial.println(" mbar");
  
  // Serial.print("Temperature: "); 
  // Serial.print(sensor.temperature()); 
  // Serial.println(" deg C");
  
  // Serial.print("Depth: "); 
  // Serial.print(sensor.depth()); 
  // Serial.println(" m");
  
  // Serial.print("Altitude: "); 
  // Serial.print(sensor.altitude()); 
  // Serial.println(" m above mean sea level");

  delay(20);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
}