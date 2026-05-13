#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <WiFi.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const char* WIFI_SSID = "RViz";
const char* WIFI_PASS = "1003712329A";
const char* AGENT_IP  = "10.73.141.141";
const size_t AGENT_PORT = 8888;

rcl_publisher_t pub_imu_raw;
rcl_publisher_t pub_imu_filtered;
rcl_publisher_t pub_motor;
rcl_subscription_t subscriber;

sensor_msgs__msg__Imu msg_imu_raw;
sensor_msgs__msg__Imu msg_imu_filtered;
geometry_msgs__msg__Twist msg_motor;
geometry_msgs__msg__Twist msg_cmd_vel;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

const float L = 0.124;    
const float R_RUEDA = 0.063;   
const float PPR = 340.0;    
const float K_OPEN_LOOP = 340.0;

// --- PINES DE CONTROL DRV8833 ---
const int IN1 = 33; // AIN1 Motor Izquierdo
const int IN2 = 25; // AIN2 Motor Izquierdo
const int IN3 = 26; // BIN1 Motor Derecho
const int IN4 = 27; // BIN2 Motor Derecho

const int freq = 5000;
const int resolution = 8;

// --- CANALES PWM ESP32 ---
const int ch_L1 = 0;
const int ch_L2 = 1;
const int ch_R1 = 2;
const int ch_R2 = 3;

// --- PINES DE ENCODERS ---
const int ENC_L_PIN = 19; // Motor Izquierdo
const int ENC_R_PIN = 18; // Motor Derecho

MPU6050 mpu;
const unsigned long PERIODO_IMU_US = 5000;
int16_t ax, ay, az, gx, gy, gz;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

volatile float ax_filtrada = 0, ay_filtrada = 0, az_filtrada = 0; 
volatile float gx_filtrada = 0, gy_filtrada = 0, gz_filtrada = 0; 

volatile float target_Vl = 0.0; 
volatile float target_Vr = 0.0;

// Variables Motor Izquierdo
volatile float real_Vl_cruda = 0.0; 
volatile float real_Vl_filtrada = 0.0; 
volatile long ticks_L = 0;
float error_L = 0, suma_error_L = 0, last_error_L = 0;

// Variables Motor Derecho
volatile float real_Vr_cruda = 0.0; 
volatile float real_Vr_filtrada = 0.0; 
volatile long ticks_R = 0;
float error_R = 0, suma_error_R = 0, last_error_R = 0;

const float alpha_AR = 0.8; 
float Kp = 2.0, Ki = 0.5, Kd = 0.0; 

TaskHandle_t TareaSensores;

double b[] = {0.00167819323226339, 0.00503457969679016, 0.00503457969679016, 0.00167819323226339, 0, 0};
double a_coef[] = {1, -2.48613629739364, 2.09606516429766, -0.596503321045908, 0, 0};

struct MemoriasFiltro { double x_p[5] = {0}; double y_p[5] = {0}; };
#define ORDEN_NLMS 15
#define RETRASO_NLMS 5
#define HISTORIAL_NLMS (ORDEN_NLMS + RETRASO_NLMS)

struct MemoriasNLMS { float w[ORDEN_NLMS] = {0}; float historial[HISTORIAL_NLMS] = {0}; float mu = 0.5; };
MemoriasFiltro mem_Ax, mem_Ay, mem_Az, mem_Gx, mem_Gy, mem_Gz;
MemoriasNLMS nlms_Ax, nlms_Ay, nlms_Az, nlms_Gx, nlms_Gy, nlms_Gz;

void IRAM_ATTR isr_enc_L() {
    if (target_Vl >= 0) ticks_L++; else ticks_L--;
}

void IRAM_ATTR isr_enc_R() {
    if (target_Vr >= 0) ticks_R++; else ticks_R--;
}

double aplicarFiltro(double x_n, MemoriasFiltro &mem) {
    double y_n = b[0]*x_n + b[1]*mem.x_p[0] + b[2]*mem.x_p[1] + b[3]*mem.x_p[2] + b[4]*mem.x_p[3] + b[5]*mem.x_p[4]
                 - a_coef[1]*mem.y_p[0] - a_coef[2]*mem.y_p[1] - a_coef[3]*mem.y_p[2] - a_coef[4]*mem.y_p[3] - a_coef[5]*mem.y_p[4];
    for(int i=4; i>0; i--) { mem.x_p[i] = mem.x_p[i-1]; mem.y_p[i] = mem.y_p[i-1]; }
    mem.x_p[0] = x_n; mem.y_p[0] = y_n;
    return y_n;
}

float aplicarNLMS(float x_n, MemoriasNLMS &f) {
    for(int i = HISTORIAL_NLMS - 1; i > 0; i--) { f.historial[i] = f.historial[i-1]; }
    f.historial[0] = x_n;
    float y_pred = 0, energia_u = 0;
    for(int i = 0; i < ORDEN_NLMS; i++) {
        float u_val = f.historial[i + RETRASO_NLMS];
        y_pred += f.w[i] * u_val;
        energia_u += u_val * u_val;
    }
    float error = x_n - y_pred;
    for(int i = 0; i < ORDEN_NLMS; i++) {
        float u_val = f.historial[i + RETRASO_NLMS];
        f.w[i] += 2.0 * (f.mu / (energia_u + 1e-6)) * error * u_val;
    }
    return y_pred;
}

void procesarIMU() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    double ax_m = (ax / 4096.0) * 9.81;
    double ay_m = (ay / 4096.0) * 9.81;
    double az_m = (az / 4096.0) * 9.81;
    
    double gx_d = (gx - gx_offset) / 131.0;
    double gy_d = (gy - gy_offset) / 131.0;
    double gz_d = (gz - gz_offset) / 131.0;
    
    ax_filtrada = aplicarNLMS(aplicarFiltro(ax_m, mem_Ax), nlms_Ax);
    ay_filtrada = aplicarNLMS(aplicarFiltro(ay_m, mem_Ay), nlms_Ay);
    az_filtrada = aplicarNLMS(aplicarFiltro(az_m, mem_Az), nlms_Az);
    
    gx_filtrada = aplicarNLMS(aplicarFiltro(gx_d, mem_Gx), nlms_Gx);
    gy_filtrada = aplicarNLMS(aplicarFiltro(gy_d, mem_Gy), nlms_Gy);
    gz_filtrada = aplicarNLMS(aplicarFiltro(gz_d, mem_Gz), nlms_Gz);
}

// --- FUNCIÓN MOVER MOTOR PARA DRV8833 ---
void moverMotor(int chA, int chB, float pwm_value) {
    int pwm_final = abs((int)pwm_value);
    
    if (pwm_final > 0 && pwm_final < 80) {
        pwm_final = 80; // Deadband compensation
    }
    
    // El DRV8833 usa control directo sobre los pines IN. 
    // Mantenemos uno en 0 y al otro le aplicamos el PWM (Fast decay / Coast-to-drive)
    if (pwm_value > 0) {
        ledcWrite(chA, constrain(pwm_final, 0, 255));
        ledcWrite(chB, 0);
    } else if (pwm_value < 0) {
        ledcWrite(chA, 0);
        ledcWrite(chB, constrain(pwm_final, 0, 255));
    } else {
        ledcWrite(chA, 0);
        ledcWrite(chB, 0); // Coast (giro libre). Si quieres freno total, pon ambos en 255.
    }
}

void actualizarPID(float dt) {
    // Control PID Motor Izquierdo
    real_Vl_cruda = (ticks_L / PPR) * (2.0 * PI * R_RUEDA) / dt;
    ticks_L = 0; 
    real_Vl_filtrada = (alpha_AR * real_Vl_filtrada) + ((1.0 - alpha_AR) * real_Vl_cruda);
    error_L = target_Vl - real_Vl_filtrada;
    suma_error_L = constrain(suma_error_L + (error_L * dt), -100, 100); 
    float output_L = (target_Vl * K_OPEN_LOOP) + (Kp * error_L) + (Ki * suma_error_L) + (Kd * ((error_L - last_error_L) / dt));
    last_error_L = error_L;

    // Control PID Motor Derecho
    real_Vr_cruda = (ticks_R / PPR) * (2.0 * PI * R_RUEDA) / dt;
    ticks_R = 0; 
    real_Vr_filtrada = (alpha_AR * real_Vr_filtrada) + ((1.0 - alpha_AR) * real_Vr_cruda);
    error_R = target_Vr - real_Vr_filtrada;
    suma_error_R = constrain(suma_error_R + (error_R * dt), -100, 100); 
    float output_R = (target_Vr * K_OPEN_LOOP) + (Kp * error_R) + (Ki * suma_error_R) + (Kd * ((error_R - last_error_R) / dt));
    last_error_R = error_R;

    // Pasamos los canales PWM directamente
    moverMotor(ch_L1, ch_L2, output_L);
    moverMotor(ch_R1, ch_R2, output_R);
}

void TareaFiltrosCode( void * pvParameters ){
  unsigned long t_anterior_imu = micros();
  unsigned long t_anterior_pid = micros();
  
  for(;;){
    unsigned long t_actual = micros();
    
    if (t_actual - t_anterior_imu >= PERIODO_IMU_US) {
      t_anterior_imu = t_actual;
      procesarIMU(); 
    }

    if (t_actual - t_anterior_pid >= 10000) {
      float dt = (t_actual - t_anterior_pid) / 1000000.0;
      t_anterior_pid = t_actual;
      actualizarPID(dt);
    }

    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float v = msg->linear.x;
  float w = msg->angular.z;
  
  target_Vr = v + (w*4 * L / 2.0);
  target_Vl = v - (w*4 * L / 2.0);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // Configuramos los pines de los encoders y sus interrupciones
  pinMode(ENC_L_PIN, INPUT_PULLUP);
  pinMode(ENC_R_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), isr_enc_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), isr_enc_R, RISING);
  
  // --- CONFIGURACIÓN PWM PARA DRV8833 ---
  ledcSetup(ch_L1, freq, resolution);
  ledcSetup(ch_L2, freq, resolution);
  ledcSetup(ch_R1, freq, resolution);
  ledcSetup(ch_R2, freq, resolution);

  ledcAttachPin(IN1, ch_L1);
  ledcAttachPin(IN2, ch_L2);
  ledcAttachPin(IN3, ch_R1);
  ledcAttachPin(IN4, ch_R2);

  // Inicializar motores apagados
  moverMotor(ch_L1, ch_L2, 0); 
  moverMotor(ch_R1, ch_R2, 0);

  Wire.begin(); Wire.setClock(400000);
  mpu.initialize(); mpu.setFullScaleAccelRange(2); mpu.setFullScaleGyroRange(0);
  long sgx = 0, sgy = 0, sgz = 0;
  for(int i=0; i<200; i++) {
    mpu.getRotation(&gx, &gy, &gz); 
    sgx += gx; sgy += gy; sgz += gz; 
    delay(2);
  }
  gx_offset = sgx / 200.0;
  gy_offset = sgy / 200.0;
  gz_offset = sgz / 200.0;

  WiFi.begin(WIFI_SSID,WIFI_PASS );
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
  allocator = rcl_get_default_allocator();
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) { delay(100); }

  rclc_node_init_default(&node, "minisumo_node", "", &support);
  rclc_publisher_init_best_effort(&pub_imu_raw, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw");
  rclc_publisher_init_best_effort(&pub_imu_filtered, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_filtered");
  rclc_publisher_init_best_effort(&pub_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/motor/telemetry");

  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_cmd_vel, &subscription_callback, ON_NEW_DATA);

  xTaskCreatePinnedToCore(TareaFiltrosCode, "SensoresPID", 15000, NULL, 1, &TareaSensores, 0);
}

void loop() {
  static unsigned long t_next = 0;
  unsigned long t_now = millis();

  if (t_now >= t_next) {
    t_next = t_now + 16;
    msg_imu_raw.linear_acceleration.x = (ax / 4096.0) * 9.81;
    msg_imu_raw.linear_acceleration.y = (ay / 4096.0) * 9.81;
    msg_imu_raw.linear_acceleration.z = (az / 4096.0) * 9.81;
    msg_imu_raw.angular_velocity.x = ((gx - gx_offset) / 131.0) * (PI / 180.0);
    msg_imu_raw.angular_velocity.y = ((gy - gy_offset) / 131.0) * (PI / 180.0);
    msg_imu_raw.angular_velocity.z = ((gz - gz_offset) / 131.0) * (PI / 180.0);
    
    msg_imu_filtered.linear_acceleration.x = ax_filtrada;
    msg_imu_filtered.linear_acceleration.y = ay_filtrada;
    msg_imu_filtered.linear_acceleration.z = az_filtrada;
    msg_imu_filtered.angular_velocity.x = gx_filtrada * (PI / 180.0);
    msg_imu_filtered.angular_velocity.y = gy_filtrada * (PI / 180.0);
    msg_imu_filtered.angular_velocity.z = gz_filtrada * (PI / 180.0);

    msg_motor.linear.x = real_Vl_cruda;    
    msg_motor.linear.y = real_Vl_filtrada; 
    msg_motor.linear.z = target_Vl;       
    
    msg_motor.angular.x = real_Vr_cruda;
    msg_motor.angular.y = real_Vr_filtrada;
    msg_motor.angular.z = target_Vr;
 
    rcl_publish(&pub_imu_raw, &msg_imu_raw, NULL);
    rcl_publish(&pub_imu_filtered, &msg_imu_filtered, NULL);
    rcl_publish(&pub_motor, &msg_motor, NULL);
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  delay(1); 
}