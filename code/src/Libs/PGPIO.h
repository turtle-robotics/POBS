#ifndef PGPIO_H
#define PGPIO_H

void pigpio_stop(int pi);
int pigpio_start(const char *addrStr, const char *portStr);
int set_mode(int pi, unsigned gpio, unsigned mode);
int gpio_read(int pi, unsigned gpio);
int set_PWM_dutycycle(int pi, unsigned user_gpio, unsigned dutycycle);
int set_PWM_frequency(int pi, unsigned user_gpio, unsigned frequency);
int set_servo_pulsewidth(int pi, unsigned user_gpio, unsigned pulsewidth);
int hardware_clock(int pi, unsigned gpio, unsigned clkfreq);
int hardware_PWM(int pi, unsigned gpio, unsigned PWMfreq, uint32_t PWMduty);


#endif // MY_HEADER_H