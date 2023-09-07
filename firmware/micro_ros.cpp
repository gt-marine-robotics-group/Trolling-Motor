#include "micro_ros.h"
#include "light_tower.h"
#include "motors.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32_multi_array.h>

// std::vector<float> ros_motor_cmds;

MicroRos::MicroRos(int num_motors) : m_cmds(num_motors) {
    // ros_motor_cmds.resize(6);
};

void MicroRos::zero_cmds() {
    for (int i {0}; i < m_cmds.size(); ++i) {
        m_cmds[i] = 0;
    }
}

void MicroRos::execute(LightTower& lt, Motors& motors) {
  bool created = false;
  switch (m_state) {
    case (State::waiting_agent):
      lt.configure(LightTower::LightStates::off, LightTower::LightStates::off,
        LightTower::LightStates::fast_flashing, LightTower::LightStates::off);
      zero_cmds();
      motors.zero_all_motors();
      EXECUTE_EVERY_N_MS(2000, m_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) 
        ? State::agent_available : State::waiting_agent;);
      break;
    case State::agent_available:
      lt.configure(LightTower::LightStates::off, LightTower::LightStates::off,
        LightTower::LightStates::flashing, LightTower::LightStates::off);
      zero_cmds();
      motors.zero_all_motors();
      created = create_entities();
      m_state = created ? State::agent_connected : State::waiting_agent;
      delay(100);
      if (m_state == State::waiting_agent) {
        destroy_entities();
      };
      break;
    case State::agent_connected:
      lt.configure(LightTower::LightStates::off, LightTower::LightStates::off,
        LightTower::LightStates::on, LightTower::LightStates::off);
      EXECUTE_EVERY_N_MS(1000, m_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) 
        ? State::agent_connected : State::agent_disconnected;);
      break;
    case State::agent_disconnected:
      lt.configure(LightTower::LightStates::fast_flashing, LightTower::LightStates::off,
        LightTower::LightStates::fast_flashing, LightTower::LightStates::off);
      zero_cmds();
      motors.zero_all_motors();
      destroy_entities();
      delay(100);
      m_state = State::waiting_agent;
      break;
    default:
      break;
  }
}

void my_msg_callback(const void* msgin) {
    const std_msgs__msg__Float32MultiArray* msg = static_cast<const std_msgs__msg__Float32MultiArray*>(msgin);
    // for (int i {0}; i < ros_motor_cmds.size(); ++i) {
    //     ros_motor_cmds[i] = msg->data.data[i];
    // }
}

bool MicroRos::create_entities() {
  delay(1000);
  m_allocator = rcl_get_default_allocator();

  //create init_options
  // RCCHECK(rclc_support_init(&m_support, 0, NULL, &m_allocator));
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  // create node
  // rcl_node_options_t node_ops = rcl_node_get_default_options();
  // node_ops.domain_id = (size_t)(12);
  size_t domain_id = (size_t) 12;

	rcl_init_options_set_domain_id(&init_options, domain_id);

  const char * node_name = "micro_ros_arduino_node";
  RCCHECK(rclc_node_init_default(&m_node, node_name, "", &m_support));

  RCCHECK(rclc_subscription_init_default(
    &m_motor_sub,
    &m_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/thrusters/all_commands"));

  RCCHECK(rclc_executor_init(&m_executor, &m_support.context, 1, &m_allocator)); // Increment this for more subs

  RCCHECK(rclc_executor_add_subscription(&m_executor, &m_motor_sub, &m_msg, &my_msg_callback, ON_NEW_DATA));

  m_msg.data.size = 0;
  return true;
}

void MicroRos::destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&m_support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&m_motor_sub, &m_node);
  rclc_executor_fini(&m_executor);
  rcl_node_fini(&m_node);
  rclc_support_fini(&m_support);
}

void MicroRos::spin() {
  if (m_state == State::agent_connected) {
    rclc_executor_spin_some(&m_executor, RCL_MS_TO_NS(100));
  }
}