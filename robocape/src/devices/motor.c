/*******************************************************************************
* motor.c
*
* Peter Gaskell 2016
*
*******************************************************************************/
#include "motor.h"

motor_t init_motor(int pwm_num, char pwm_chan, int dir_pin, int nEn_pin){
    // Initialize your motor by setting up GPIO and PWM
    motor_t motor;
    motor.pwm_num = pwm_num;
    motor.pwm_chan = pwm_chan;
    motor.dir_pin = dir_pin;
    motor.nEn_pin = nEn_pin;
    motor.speed = 0;//default 0?

    // n_enable pin (nEn_pin), disables with high value.
    set_motor_off(motor);

    // export the direction pin and n_enable pin for GPIO
    gpio_export(dir_pin);
    gpio_export(nEn_pin);

    // Set the GPIO pins to either input or output
    // INPUT: read, write
    // OUTPUT: read
    gpio_set_dir(dir_pin,OUTPUT_PIN); //set direction and enable pin as output
    gpio_set_dir(nEn_pin,OUTPUT_PIN);
    // PWM
    // use simple_init_pwm
    simple_init_pwm(pwm_num,1000); // set pwm frequency to 1000 on channel pwm_num
    return motor;
}

int set_motor_speed(motor_t* motor, float speed){
    // Adjust motor speed via PWM and direction via the dir pins
    // SV1, SV3: are the actual pin locations on the board correspond to
    //           A0, A1. 
    int check = 0;
    // printf("%d %d %c %d ",motor->dir_pin, motor->nEn_pin, motor->pwm_chan, motor->pwm_num);
    if (speed >= 0)
    {
        if(speed > 1) speed = 1;
        mmap_gpio_write(motor->dir_pin, 0);
        mmap_gpio_write(motor->nEn_pin, 0);
        check = set_pwm_duty(motor->pwm_num, motor->pwm_chan, speed);
        // printf("%f \n", speed);
    }
    else
    {
        if(speed < -1) speed = -1;
        mmap_gpio_write(motor->dir_pin, 1);
        mmap_gpio_write(motor->nEn_pin, 0);
        check = set_pwm_duty(motor->pwm_num, motor->pwm_chan, -(speed));
        // printf("%f \n", -speed);
    }
    return check;
}

/* CODE BELOW IS ALREADY COMPLETE */
int set_motor_on(motor_t motor){
    // Enable motor via GPIO
    int check = mmap_gpio_write(motor.nEn_pin, 0);
    return check;
}

int set_motor_off(motor_t motor){
    // Disable motor via GPIO
    int check = mmap_gpio_write(motor.nEn_pin, 1);
    return check;
}

int uninit_motor(motor_t motor){
    // Unitialize motor by disableing the motor, and uninitializing PWMs

    // GPIO
    set_motor_off(motor);
    gpio_unexport(motor.dir_pin);
    gpio_unexport(motor.nEn_pin);

    // PWM
    simple_uninit_pwm(motor.pwm_num);

    return 0;
}
