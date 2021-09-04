#include <Arduino.h>
#include <engine.h>

engine::engine(){
  num = 0;
  speed = 0;
  direction = 0;
  plus_pin = 0;
  minus_pin = 0;
  plus_channel = 0;
  minus_channel = 0;
  speed_coefficient = 0;
}

engine::engine(int number, char* motor_name, uint8_t plus, uint8_t minus, uint8_t plus_ch, uint8_t minus_ch, const float coeff)
{
  num = number;
  name = motor_name;
  plus_pin = plus;
  minus_pin = minus;
  plus_channel = plus_ch;
  minus_channel = minus_ch;
  speed_coefficient = coeff;
    
}

engine::~engine()
{
}

void engine::set_engine_pwm(int16_t pwm){
  
  
    if(pwm > 0){
    
      ledcWrite(plus_channel, pwm);
      ledcWrite(minus_channel, 0);

      direction = 1;
    }
    else if (pwm < 0)
    {
      ledcWrite(plus_channel, 0);
      ledcWrite(minus_channel, -pwm);

      direction = -1;
    }
    else
    {
      ledcWrite(plus_channel, 0);
      ledcWrite(minus_channel, 0);
    }
    Serial.print("dir: ");
  Serial.println(direction);
}

float engine::get_speed(){
  return speed;
}

// void engine::update_motor_speed(volatile int encoder_count, portMUX_TYPE* mux){
  
//   portENTER_CRITICAL(mux);
//   speed = (float)direction * (float)encoder_count * speed_coefficient; // mm/ms = m/s
//   encoder_count = 0;
//   portEXIT_CRITICAL(mux);
  
  
// }

void engine::set_configuration(const int freq, const int res){
  ledcSetup(plus_channel, freq, res);
  ledcSetup(minus_channel, freq, res);

  ledcAttachPin(plus_pin, plus_channel);
  ledcAttachPin(minus_pin, minus_channel);
}
