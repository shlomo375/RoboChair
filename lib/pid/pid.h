#ifndef pid_h
#define pid_h
#include <engine.h>
#include <atomic>
#include <memory>



class engine;
class pid
{
private:
    float kp;
    float ki;
    float kd;

    float error;
    float last_error;
    float error_integral;
    int out;
    float input;
    float current;

    
public:
    pid();
    pid(float *k);
    ~pid();

    void set_all_k(float *k);
    void set_kp(float p);
    void set_ki(float i);
    void set_kd(float d);

    void get_all_k(float *arr);
    void set_input(float in);
    void set_current(float c);
    int get_output();
    void compute_pid(engine *motor, std::atomic<int>* counter);

};

#endif