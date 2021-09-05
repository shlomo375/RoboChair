#include <Arduino.h>
#include <WiFi.h>
#include <engine.h>
#include <pid.h>
#include <communication.h>
#include <atomic>
#include <memory>
using namespace std;

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
const int data_save_dt = 100000; // in us ->  1ms
const int max_itar = 100; // total 1sec

//global veriable
double speed_motor_left = 0;
double speed_motor_right = 0;
int direction_m_l = 0;
int direction_m_r = 0;
int iter = 0;

//transmision veriable
uint8_t order[5];
float data_size = 1000;
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
std::atomic<int> encoder_right_count{0};
std::atomic<int> encoder_left_count{0};

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
struct timer_interrupt
{ 
  //tcp connecion
  hw_timer_t *TCP = NULL; 
  std::atomic<uint8_t> TCP_counter;
  
  //recive transmission:
  hw_timer_t *transmission = NULL;
  std::atomic<uint8_t> transmission_counter{0};
  
  //data save:
  hw_timer_t *data_save = NULL;
  std::atomic<uint8_t> data_save_counter{0};
  
  //sensors interrupt
  hw_timer_t *sensor = NULL;
  std::atomic<uint8_t> sensor_counter{0};
};


//function declaration
void IRAM_ATTR TCP_connect_interrupt();
void transmit_to_engine(engine *motor_right, engine *motor_left, uint8_t engine, uint8_t pwm, uint8_t direction);
bool get_wifi_instruction(WiFiClient *client, uint8_t *order ,float *data, pid *speed_right, pid *speed_left, engine *motor_right, engine *motor_left, timer_interrupt* timer);
void IRAM_ATTR receive_transmission_interrupt();
void IRAM_ATTR update_encoder_right();
void IRAM_ATTR update_encoder_left();
void IRAM_ATTR sensor_ISR();
void read_sensor();
void update_motor_speed();
bool save_step_response_data(float *data, engine *motor, int* iter, int max_itar);
void IRAM_ATTR data_save_interrupt();
void send_data(float* data, int data_size, WiFiClient* client);


timer_interrupt timer;
int speed = 0;
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
    timer.TCP = timerBegin(0,80,true);//increasement every 1us.
    timerAttachInterrupt(timer.TCP,&TCP_connect_interrupt,true);
    timerAlarmWrite(timer.TCP,500000,true); //interupt every 0.5 sec
    timerAlarmEnable(timer.TCP);
  
    //config timer for transmission:
    timer.transmission = timerBegin(1,80,true); //increasement every 1us.
    timerAttachInterrupt(timer.transmission,&receive_transmission_interrupt,true);
    timerAlarmWrite(timer.transmission,500000,true); // interrupt every 0.5 sec
    timerAlarmEnable(timer.transmission);

    //config timer for sensor reading
    timer.sensor = timerBegin(2,80,true); //increasement every 1us.
    timerAttachInterrupt(timer.sensor,&sensor_ISR,true);
    timerAlarmWrite(timer.sensor,sensor_read_time,true); //interrupt every 0.1 sec
    timerAlarmEnable(timer.sensor);

    //config timer for data seving
    timer.data_save = timerBegin(3,80,true); //increasement every 1us.
    timerAttachInterrupt(timer.data_save,&data_save_interrupt,true);
    timerAlarmWrite(timer.data_save,data_save_dt,true); //interrupt every 0.001 sec
    // timerAlarmEnable(timer.data_save);

    //encoder interupt
    attachInterrupt(encoder_right,&update_encoder_right,CHANGE);
    attachInterrupt(encoder_left,&update_encoder_left,CHANGE);

    

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
  // interrupt condition:
  if(timer.TCP_counter)
  {
    // portENTER_CRITICAL(&mux);
    timer.TCP_counter--;
    // portEXIT_CRITICAL(&mux);

    if(check_for_TCP_connection(&server, &remote_client)){
      timerAlarmWrite(timer.TCP,5000000,true); // every 5 sec
    } 
  }
  
  if(timer.transmission){
    // portENTER_CRITICAL(&mux);
    timer.transmission_counter--;
    // portEXIT_CRITICAL(&mux);
    
    get_wifi_instruction(&remote_client, order, data, &speed_right, &speed_left, &motor_right, &motor_left, &timer);
  }

  if(timer.sensor_counter){
    // portENTER_CRITICAL(&mux);
    timer.sensor_counter--;
    // portEXIT_CRITICAL(&mux);

    // speed_right.compute_pid(&motor_right, &encoder_right_count);
    // speed_left.compute_pid(&motor_left, &encoder_left_count);
    motor_right.update_motor_speed(&encoder_right_count);
  }

  if(timer.data_save_counter){

    timer.data_save_counter--;

    if(!save_step_response_data(data, &motor_right, &iter, max_itar)){
      timerAlarmDisable(timer.data_save);
    }
  }
  
  
  // Serial.print("encoder: ");
  // Serial.println(encoder_right_count);

  
  // Serial.print("speed: ");
  // delay(1000);
  // motor_right.update_motor_speed(&encoder_right_count);

  // Serial.println(motor_right.get_speed());
  
  // if(motor_right.get_speed() != speed){
    // speed = motor_right.get_speed();
    // Serial.print("speed: ");
    // Serial.println(speed);
}

//increasement counter at TCP_timer overflow
void IRAM_ATTR TCP_connect_interrupt()
{
  timer.TCP_counter++;
}

//increasement counter at transmission_timer overflow
void IRAM_ATTR receive_transmission_interrupt(){
  timer.transmission_counter ++;
}

//increasement counter at data_save_timer overflow
void IRAM_ATTR data_save_interrupt(){
  timer.data_save_counter ++;
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

bool get_wifi_instruction(WiFiClient *client, uint8_t *order ,float *data, pid *speed_right, pid *speed_left, engine *motor_right, engine *motor_left, timer_interrupt* timer){

  if(!get_order(client,order,data,order_buf_size)){
    return false;
  }
  Serial.print("order: ");
  Serial.println(order[1]);
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
    timerAlarmEnable(timer->data_save);
    // speed_right-> set_input(data[0]);
    // speed_left-> set_input(data[0]);

    break;
  case 111:
    speed_right-> set_all_k(data);
    break;
  case 112:
    speed_left-> set_all_k(data);
    break;
  
  case 200:
    send_data(data, data_size, client);
    break;


  default:
    Serial.println("Unknown command");
    remote_client.println("Unknown command");
    break;
  }
  return true;
}

void IRAM_ATTR update_encoder_right(){
  encoder_right_count++;
}
void IRAM_ATTR update_encoder_left(){
  encoder_left_count++;
}
void IRAM_ATTR sensor_ISR(){
  timer.sensor_counter++;
}


void read_sensor(){
  // portENTER_CRITICAL(&mux);
  timer.sensor_counter--;
  // portEXIT_CRITICAL(&mux);

  update_motor_speed();
}

bool save_step_response_data(float *data, engine *motor, int* iter, int max_itar)
{
  Serial.print("d2");
  if(*iter >= max_itar){
    *iter = 0;
    return false;
  }
  else{
    data[*iter] = motor->get_speed();
    Serial.println(data[*iter]);
    *iter++;
    return true;
  }
}

void send_data(float* data, int data_size, WiFiClient* client){
  float f_data_size[1] = {(float)data_size};
  send_float_data(client,f_data_size,1);
  send_float_data(client, data, data_size);
}