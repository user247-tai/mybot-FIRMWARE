#include <micro_ros_arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <BH1750.h>
#include <MD_MAX72xx.h>
#include <geometry_msgs/msg/vector3.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <math.h>

// BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Constants
#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

BH1750 lightMeter;
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW  // For FC-16 32x8 modules
#define MAX_DEVICES 4                      // Four 8x8 matrices
#define CS_PIN 5                           // Chip Select on GPIO5
// Initialize the MD_MAX72XX object using hardware SPI
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// Micro-ROS variables
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t battery_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t rpy_publisher;
rcl_subscription_t mode_subscriber;
rcl_subscription_t light_control_subscriber;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Vector3 rpy_msg;
std_msgs__msg__Int32 battery_msg;
std_msgs__msg__String mode_msg;
std_msgs__msg__Bool light_control_msg;
bool micro_ros_init_successful;

String mode = "";
bool light_control = false;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void turnOnLED(){
  for (uint8_t row = 0; row < 8; row++) {
    mx.setColumn(0, row, 0x77);  // All LEDs ON for first matrix (leftmost)
    mx.setColumn(1, row, 0x08);  // All LEDs ON for first matrix (leftmost)
    mx.setColumn(2, row, 0x08);  // All LEDs ON for first matrix (leftmost)
    mx.setColumn(3, row, 0x77);  // All LEDs ON for last matrix (rightmost)
  }
}

void turnOffLED(){
  for (uint8_t row = 0; row < 8; row++) {
    mx.setColumn(0, row, 0x00);  // All LEDs ON for first matrix (leftmost)
    mx.setColumn(1, row, 0x00);  // All LEDs ON for first matrix (leftmost)
    mx.setColumn(2, row, 0x00);  // All LEDs ON for first matrix (leftmost)
    mx.setColumn(3, row, 0x00);  // All LEDs ON for last matrix (rightmost)
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    // Get quaternion for orientation
    imu::Quaternion quat = bno.getQuat();
    imu_msg.orientation.w = quat.w();
    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();

    // Get angular velocity (convert from deg/s to rad/s)
    sensors_event_t angVelocityData;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msg.angular_velocity.x = angVelocityData.gyro.x * M_PI / 180.0;
    imu_msg.angular_velocity.y = angVelocityData.gyro.y * M_PI / 180.0;
    imu_msg.angular_velocity.z = angVelocityData.gyro.z * M_PI / 180.0;

    // Get linear acceleration (m/s²)
    sensors_event_t linearAccelData;
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
    imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
    imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;

    // Get Euler angles for roll, pitch, yaw (convert to radians)
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    rpy_msg.x = orientationData.orientation.z * M_PI / 180.0; // roll
    rpy_msg.y = orientationData.orientation.y * M_PI / 180.0; // pitch
    rpy_msg.z = orientationData.orientation.x * M_PI / 180.0; // yaw

    // Set IMU message header
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");

    // Publish both messages
    rcl_publish(&imu_publisher, &imu_msg, NULL);
    rcl_publish(&rpy_publisher, &rpy_msg, NULL);

    //Read battery data from pin 34
    int sensorValue = analogRead(34);
    float voltage = sensorValue * (3.3 / 4095.0);
    float actualVoltage = voltage * (16.5 / 3.3);
    int battery = ((actualVoltage - 14.5) / (16.5 - 14.5)) * 100;
    battery_msg.data = battery;
    //Read lux data from light sensor
    float lux = lightMeter.readLightLevel();
    rcl_publish(&battery_publisher, &battery_msg, NULL);

    if (mode == "PATROL"){
      if (lux <= 300){
        turnOnLED();
      }
    }
    else if (mode == "NORMAL"){
      if (light_control == false){
        turnOffLED();
      }
      else if (light_control == true){
        turnOnLED();
      }
    }
  }
}

void mode_callback(const void * msgin)
{
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  if (mode != String(micro_ros_string_utilities_get_c_str(msg->data))){
    mode = String(micro_ros_string_utilities_get_c_str(msg->data));
  }
}

void light_control_callback(const void * msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  light_control = msg->data;
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 27));


  //create init_options
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  rclc_node_init_default(&node, "microROS", "", &support);

  // Create publishers
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"));
  RCCHECK(rclc_publisher_init_best_effort(
    &rpy_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "handsfree/imu_rpy"));
  RCCHECK(rclc_publisher_init_best_effort(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "battery"));

  // Create timer (10 Hz)
  const unsigned int timer_timeout = 50; // 100 ms = 10 Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create subscribers
  RCCHECK(rclc_subscription_init_default(
  &mode_subscriber,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
  "mode"));

  // create subscribers
  RCCHECK(rclc_subscription_init_default(
  &light_control_subscriber,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  "light_control"));

  // Create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_executor_add_subscription(&executor, &mode_subscriber, &mode_msg, &mode_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &light_control_subscriber, &light_control_msg, &light_control_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&battery_publisher, &node);
  rcl_publisher_fini(&rpy_publisher, &node);
  rcl_subscription_fini(&mode_subscriber, &node);
  rcl_subscription_fini(&light_control_subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();

  // Initialize BNO055
  unsigned long startAttemptTime = millis();

  while (millis() - startAttemptTime < 60000) {
    if (bno.begin()) {
      break;
    }
    delay(500);  // Small delay to avoid spamming I2C bus
  }

  // Apply calibration data
  adafruit_bno055_offsets_t calibrationData;
  calibrationData.accel_offset_x = -24;
  calibrationData.accel_offset_y = -61;
  calibrationData.accel_offset_z = -36;
  calibrationData.gyro_offset_x = -1;
  calibrationData.gyro_offset_y = 0;
  calibrationData.gyro_offset_z = 1;
  calibrationData.mag_offset_x = -1;  
  calibrationData.mag_offset_y = 0;  
  calibrationData.mag_offset_z = -36;  
  calibrationData.accel_radius = 1000;
  calibrationData.mag_radius = 546;
  bno.setSensorOffsets(calibrationData);

  delay(1000);

  state = WAITING_AGENT;

  mode_msg.data.capacity = 20;
  mode_msg.data.size = 0;
  mode_msg.data.data = (char*) malloc(mode_msg.data.capacity * sizeof(char));

  battery_msg.data = 0;
  mx.begin();           // Initialize the display
  // mx.setIntensity(8);   // Set brightness (0-15, 8 is medium)
  mx.clear(); 
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  digitalWrite(LED_PIN, state == AGENT_CONNECTED ? HIGH : LOW);
}