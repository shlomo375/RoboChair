#include <Arduino.h>
#include <WiFi.h>
#include <engine.h>
#include <pid.h>
#include <communication.h>


//setting gpio nubers
const uint8_t motor_left_p = 22;
const uint8_t motor_left_m = 23;
const uint8_t motor_right_p = 19;
const uint8_t motor_right_m = 21;
const uint8_t encoder_right = 35;
const uint8_t encoder_left = 34;

// setting PWM properties
const int freq = 10000;
const int pwm_motor_left_p = 0;
const int pwm_motor_left_m = 1;
const int pwm_motor_right_p = 2;
const int pwm_motor_right_m = 3;
const int resolution = 8;

//const param
const int8_t wheel_diameter = 65; //in mm
const int64_t sensor_read_time = 100000; //in us
const int8_t pulse_in_turn = 40;
const double speed_coefficient =1000 * 2 * 3.141 * wheel_diameter / pulse_in_turn  / sensor_read_time;
const int data_save_dt = 1000; // in us ->  1ms
const int max_itar = 1000; // total 1sec

//global veriable
double speed_motor_left = 0;
double speed_motor_right = 0;
int direction_m_l = 0;
int direction_m_r = 0;
int iter = 0;

//transmision veriable
uint8_t order[5];
float data[1000];
size_t order_buf_size = 5; // [order type, order, num_of_packege]

//engine
engine motor_right(1, (char*)"right", motor_right_p, motor_right_m, pwm_motor_right_p, pwm_motor_right_m, speed_coefficient);
engine motor_left( 2  ,(char*)"left", motor_left_p,  motor_left_m,  pwm_motor_left_p, pwm_motor_left_m, speed_coefficient);

//pid controler
float k[3]= {1,0,0};
pid speed_right(k);
pid speed_left(k);

//encoders
volatile int encoder_right_count = 0;
volatile int encoder_left_count = 0;

//wifi network
const char* ssid = "AndroidAP";
const char* password = "205540248";

// const char* ssid = "OnePlus 5T";
// const char* password = "086378110";

// Set your Static IP address
IPAddress local_IP(192, 168, 43, 217);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8); // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional

//TCP server and transmission:
const uint16_t server_port = 23;
WiFiServer server(server_port);
WiFiClient remote_client;
WiFiClient* c;
//

//timers interrupt
uint total_interrupts_counter = 0;
  //tcp connecion
  volatile uint8_t tmr_TCP_counter = 0;
  hw_timer_t *tmr_TCP = NULL;

  //recive transmission:
  volatile uint8_t tmr_transmission_counter = 0;
  hw_timer_t *tmr_transmission = NULL;

  //data save:
  volatile uint8_t tmr_data_save_counter = 0;
  hw_timer_t *tmr_data_save = NULL;

  //sensors interrupt
  volatile uint8_t tmr_sensor_counter = 0;
  hw_timer_t *tmr_sensor = NULL;



// open critical 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//function declaration
// void check_for_TCP_connection();
void IRAM_ATTR TCP_connect_interrupt();
// String translateEncryptionType(wifi_auth_mode_t encryptionType);
// void scanNetworks();
// void connectToNetwork();
// int receive_data(uint8_t* data);
// void send_float_data(float* data, uint8_t data_size);
// uint8_t get_float_data(float* float_data, uint8_t data_length);
void transmit_to_engine(engine *motor_right, engine *motor_left, uint8_t engine, uint8_t pwm, uint8_t direction);
bool get_wifi_instruction(WiFiClient *client, uint8_t *order ,float *data, pid *speed_right, pid *speed_left, engine *motor_right, engine *motor_left, hw_timer_t *tmr_data_save);
void IRAM_ATTR receive_transmission_interrupt();
void IRAM_ATTR update_encoder_right();
void IRAM_ATTR update_encoder_left();
void IRAM_ATTR sensor_ISR();
void read_sensor();
void update_motor_speed();
// bool get_order(uint8_t *order, float *data);
bool save_step_response_data(float *data, engine *motor, int iter, int max_itar);
void IRAM_ATTR data_save_interrupt();

void engine::update_motor_speed(volatile int encoder_count, portMUX_TYPE mux){
  Serial.print("dir: ");
  Serial.print(direction);
  Serial.print("  encoder: ");
  Serial.print(encoder_count);
  Serial.print("  coeff: ");
Serial.print(speed_coefficient);
  

  portENTER_CRITICAL(&mux);
  speed = (float)direction * (float)encoder_count * speed_coefficient; // mm/ms = m/s
  encoder_count = 0;
  portEXIT_CRITICAL(&mux);

  Serial.print("  sssspeed: ");
  Serial.println(speed);
  
  
}
float speed =0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  // conpig gpio:
    //encoders pin:
    pinMode(encoder_right,INPUT);
    pinMode(encoder_left,INPUT);

  motor_right.set_configuration(freq, resolution);
  motor_left.set_configuration(freq, resolution);

  //config static IP for the controller
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
  Serial.println("STA Failed to configure");
  }
  //connect to wifi network
  scanNetworks();
  connectToNetwork(ssid, password);
 
  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());
 
  //open TCP server
  server.begin();
  //config timers:
    //config timer for TCP connection:
    tmr_TCP = timerBegin(0,80,true);//increasement every 1us.
    timerAttachInterrupt(tmr_TCP,&TCP_connect_interrupt,true);
    timerAlarmWrite(tmr_TCP,500000,true); //interupt every 0.5 sec
    timerAlarmEnable(tmr_TCP);
  
    //config timer for transmission:
    tmr_transmission = timerBegin(1,80,true); //increasement every 1us.
    timerAttachInterrupt(tmr_transmission,&receive_transmission_interrupt,true);
    timerAlarmWrite(tmr_transmission,500000,true); // interrupt every 0.5 sec
    timerAlarmEnable(tmr_transmission);

    //config timer for sensor reading
    tmr_sensor = timerBegin(2,80,true); //increasement every 1us.
    timerAttachInterrupt(tmr_sensor,&sensor_ISR,true);
    timerAlarmWrite(tmr_sensor,sensor_read_time,true); //interrupt every 0.1 sec
    // timerAlarmEnable(tmr_sensor);

    //config timer for data seving
    tmr_data_save = timerBegin(3,80,true); //increasement every 1us.
    timerAttachInterrupt(tmr_data_save,&data_save_interrupt,true);
    timerAlarmWrite(tmr_data_save,data_save_dt,true); //interrupt every 0.1 sec
    // timerAlarmEnable(tmr_data_save);

    //encoder interupt
    attachInterrupt(encoder_right,&update_encoder_right,CHANGE);
    attachInterrupt(encoder_left,&update_encoder_left,CHANGE);

    

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
  // interrupt condition:
  if(tmr_TCP_counter)
  {
    portENTER_CRITICAL(&mux);
    tmr_TCP_counter--;
    portEXIT_CRITICAL(&mux);

    if(check_for_TCP_connection(&server, &remote_client)){
      timerAlarmWrite(tmr_TCP,5000000,true); // every 5 sec
    } 
  }
  
  if(tmr_transmission){
    portENTER_CRITICAL(&mux);
    tmr_transmission_counter--;
    portEXIT_CRITICAL(&mux);
    
    get_wifi_instruction(&remote_client, order, data, &speed_right, &speed_left, &motor_right, &motor_left, tmr_data_save);
  }

  if(tmr_sensor_counter){
    portENTER_CRITICAL(&mux);
    tmr_sensor_counter--;
    portEXIT_CRITICAL(&mux);

    speed_right.compute_pid(&motor_right, encoder_right_count, &mux);
    speed_left.compute_pid(&motor_left, encoder_left_count, &mux);
  }

  if(tmr_data_save_counter){
    portENTER_CRITICAL(&mux);
    tmr_data_save_counter--;
    portEXIT_CRITICAL(&mux);

    if(!save_step_response_data(data, &motor_right, iter, max_itar)){
      timerAlarmDisable(tmr_data_save);
    }
  }
  
  
  // Serial.print("encoder: ");
  // Serial.println(encoder_right_count);

  
  // Serial.print("speed: ");
  
  motor_right.update_motor_speed(encoder_right_count, mux);
  // portENTER_CRITICAL(&mux);
  // encoder_right_count = 0;
  // portEXIT_CRITICAL(&mux);
  // Serial.println(motor_right.get_speed());
  
  if(motor_right.get_speed() != speed){
    speed = motor_right.get_speed();
    // Serial.print("speed: ");
    // Serial.println(speed);
  }
  // c = &remote_client;
  // if(c->available() && c->connected()){
  // c->read(d,5);
  // Serial.println(d[0]);
  // Serial.println(d[1]);
  // Serial.println(d[2]);
  // Serial.println(d[3]);
  // Serial.println(d[4]);
  // }
}

//increasement counter at TCP_timer overflow
void IRAM_ATTR TCP_connect_interrupt()
{
  portENTER_CRITICAL_ISR(&mux);
  tmr_TCP_counter++;
  portEXIT_CRITICAL_ISR(&mux);
}

//increasement counter at transmission_timer overflow
void IRAM_ATTR receive_transmission_interrupt(){
  portENTER_CRITICAL_ISR(&mux);
  tmr_transmission_counter ++;
  portEXIT_CRITICAL_ISR(&mux);
}

//increasement counter at data_save_timer overflow
void IRAM_ATTR data_save_interrupt(){
  portENTER_CRITICAL_ISR(&mux);
  tmr_data_save_counter ++;
  portEXIT_CRITICAL_ISR(&mux);
}

void transmit_to_engine(engine *motor_right, engine *motor_left, uint8_t engine, uint8_t pwm, uint8_t direction){
  int16_t PWM;
    if(direction == 2){//engine forward
      PWM = int16_t(pwm);
    }
    else if(direction == 1){//engine backward
      PWM = -int16_t(pwm);
      
    }
    else // engine stop
    {
      PWM = 0;
    }
    Serial.print("PWM transmit_to_engine: ");
    Serial.println(PWM);
    if(engine == 1){
      motor_right->set_engine_pwm(PWM);
    }
    if(engine == 2){
      motor_left->set_engine_pwm(PWM);
    }
}

bool get_wifi_instruction(WiFiClient *client, uint8_t *order ,float *data, pid *speed_right, pid *speed_left, engine *motor_right, engine *motor_left, hw_timer_t *tmr_data_save){

  if(!get_order(client,order,data,order_buf_size)){
    return false;
  }
  switch (order[1])
  {
  case 100:
    transmit_to_engine(motor_right, motor_left, order[2], order[3], order[4]);
    break;
  
  case 101:

    speed_right-> set_input(data[0]);
    break;

  case 102:
    speed_left-> set_input(data[0]);
    break;

  case 103:
    timerAlarmEnable(tmr_data_save);
    speed_right-> set_input(data[0]);
    speed_left-> set_input(data[0]);

    break;
  case 111:
    speed_right-> set_all_k(data);
    break;
  case 112:
    speed_left-> set_all_k(data);
    break;
  
  case 200:
    break;


  default:
    Serial.println("Unknown command");
    remote_client.println("Unknown command");
    break;
  }
  return true;
}

void IRAM_ATTR update_encoder_right(){
  portENTER_CRITICAL_ISR(&mux);
  encoder_right_count++;
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR update_encoder_left(){
  portENTER_CRITICAL_ISR(&mux);
  encoder_left_count++;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR sensor_ISR(){
  portENTER_CRITICAL_ISR(&mux);
  tmr_sensor_counter++;
  portEXIT_CRITICAL_ISR(&mux);
}


void read_sensor(){
  portENTER_CRITICAL(&mux);
  tmr_sensor_counter--;
  portEXIT_CRITICAL(&mux);

  update_motor_speed();
}

bool save_step_response_data(float *data, engine *motor, int iter, int max_itar)
{
  if(iter >= max_itar){
    iter = 0;
    return false;
  }
  else{
    data[iter] = motor->get_speed();
    iter++;
    return true;
  }
}
