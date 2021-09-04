#include <Arduino.h>
#include <pid.h>
#include <engine.h>

pid::pid()
{
    kp = 0;
    ki = 0;
    kd = 0;
    error = 0;
    last_error = 0;
    out = 0;
    input = 0;
    current = 0;
    error_integral = 0;
}

pid::pid(float *k){
    kp = k[0];
    ki = k[1];
    kd = k[2];
    error = 0;
    last_error = 0;
    out = 0;
    input = 0;
    current = 0;
    error_integral = 0;
}

pid::~pid()
{
}

void pid::set_all_k(float *k)
{
    kp = k[0];
    ki = k[1];
    kd = k[2];
}
void pid::set_kp(float p){
    kp = p;
}
void pid::set_ki(float i){
    ki = i;
}
void pid::set_kd(float d){
    kd = d;
}

void pid::get_all_k(float *arr){
    arr[0] = kp;
    arr[1] = ki;
    arr[2] = kd;
}
void pid::set_input(float in){
    input = in;
    error_integral = 0;
}
int pid::get_output(){
    return out;
}
void pid::set_current(float c){
    current = c;
}
void pid::compute_pid(engine *motor, int counter, portMUX_TYPE* mux){
    // motor->update_motor_speed(counter, mux);
    
    error = input - motor->get_speed();
    float d_error = error - last_error;
    error_integral = error_integral + error;

    last_error = error;
    out = (int16_t)kp * error + ki * error_integral + kd * d_error;
    motor->set_engine_pwm(out);
}