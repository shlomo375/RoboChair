#ifndef engine_h
#define engine_h

#include <Arduino.h>
#include <pid.h>
#include <atomic>
#include <memory>

class engine
{
private:
    int num;
    char *name;
    float speed;
    int direction;
    uint8_t plus_pin;
    uint8_t minus_pin;
    uint8_t plus_channel;
    uint8_t minus_channel;
    float speed_coefficient;

public:
    engine();
    engine(int number, char* motor_name, uint8_t plus, uint8_t minus, uint8_t plus_ch, uint8_t minus_ch, const float coeff);
    ~engine();

    // void set_engine_param(int number, char* motor_name, uint8_t plus, uint8_t minus, double coeff);
    void set_configuration(const int freq, const int res);
    void set_engine_pwm(int16_t pwm);
    void get_info(char* info);
    float get_speed();
    void update_motor_speed(std::atomic<int>* encoder_count);

};

#endif