#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

// ASIGNACIÓN DE PINES SEGÚN TU HARDWARE ACTUAL
#define variador  34   // Potenciómetro
#define PWM_PIN   26   // CONECTADO A ENB (Control de velocidad)
#define In3       14   // CONECTADO A INT 3 (Dirección)
#define In4       27   // CONECTADO A INT 4 (Dirección)
#define LED_PIN    2   

#define PWM_CH     0
#define PWM_FREQ   500 // Frecuencia baja para mayor torque y evitar zumbido
#define PWM_RES    8   // 8 bits (0-255)

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;
rcl_publisher_t pub_duty, pub_voltaje, pub_state;
rcl_subscription_t sub_command;
std_msgs__msg__Float32 msg_duty, msg_voltaje;
std_msgs__msg__String msg_state, msg_command;

bool motorEnMovimiento = false;

// Macros de control de errores
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  char cmd = msg->data.data[0];

  // Limpieza preventiva de pines para evitar cortos o bloqueos del L298N
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);

  if (cmd == 'D') { 
    motorEnMovimiento = true; 
    digitalWrite(In3, HIGH); 
    digitalWrite(In4, LOW); 
  }
  else if (cmd == 'I') { 
    motorEnMovimiento = true; 
    digitalWrite(In3, LOW); 
    digitalWrite(In4, HIGH); 
  }
  else if (cmd == 'S') { 
    motorEnMovimiento = false; 
    ledcWrite(PWM_CH, 0); 
    digitalWrite(In3, LOW); 
    digitalWrite(In4, LOW);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    int pot = analogRead(variador);
    
    // Cálculo de voltaje y duty cycle (ADC de 12 bits: 0-4095)
    msg_voltaje.data = pot * (3.3 / 4095.0);
    msg_duty.data = (pot / 4095.0) * 100.0;
    
    // Si el motor debe moverse, aplicamos el mapeo a 8 bits (0-255) al ENB
    if (motorEnMovimiento) {
      ledcWrite(PWM_CH, map(pot, 0, 4095, 0, 255));
    }
    
    // Preparar mensaje de estado
    snprintf(msg_state.data.data, msg_state.data.capacity, motorEnMovimiento ? "ACTIVO" : "STOP");
    msg_state.data.size = strlen(msg_state.data.data);

    // Publicación de tópicos
    RCSOFTCHECK(rcl_publish(&pub_duty, &msg_duty, NULL));
    RCSOFTCHECK(rcl_publish(&pub_voltaje, &msg_voltaje, NULL));
    RCSOFTCHECK(rcl_publish(&pub_state, &msg_state, NULL));
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(In3, OUTPUT); 
  pinMode(In4, OUTPUT); 
  pinMode(LED_PIN, OUTPUT);
  
  // Configuración de PWM en el pin 26 (tu ENB)
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES); 
  ledcAttachPin(PWM_PIN, PWM_CH);

  delay(2000); // Espera para que el agente esté listo

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));

  // Reserva de memoria para Strings (Crucial en micro-ROS)
  msg_state.data.capacity = 50;
  msg_state.data.data = (char*) malloc(msg_state.data.capacity);
  msg_command.data.capacity = 20;
  msg_command.data.data = (char*) malloc(msg_command.data.capacity);

  // Inicialización de Publishers
  RCCHECK(rclc_publisher_init_default(&pub_duty, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "duty_cycle"));
  RCCHECK(rclc_publisher_init_default(&pub_voltaje, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "voltaje"));
  RCCHECK(rclc_publisher_init_default(&pub_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_state"));
  
  // Inicialización de Subscriber
  RCCHECK(rclc_subscription_init_default(&sub_command, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_command"));

  // Configuración de Timer y Executor
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_command, &msg_command, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
