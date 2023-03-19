#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include "light_tower.h"
#include "motors.h"
#include <vector>

#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

class MicroRos {

    public:
        enum State {
            waiting_agent, agent_available, agent_connected, agent_disconnected
        };
    
    private:

        State m_state {waiting_agent};

        std::vector<int> m_cmds{};

        void zero_cmds();

        rcl_subscription_t m_motor_sub;
        rclc_executor_t m_executor;
        rcl_allocator_t m_allocator;
        rclc_support_t m_support;
        rcl_node_t m_node;
        rcl_timer_t m_timer;

        std_msgs__msg__Float32MultiArray m_msg;
    
    public:

        MicroRos(int num_motors);

        void execute(LightTower& lt, Motors& motors);

        bool create_entities();

        void destroy_entities();

        void spin();
};

#endif