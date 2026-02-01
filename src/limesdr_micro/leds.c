#include "la9310_gpio.h"

static uint32_t ledsOn = 1;

#define LED1_PIN 17
#define LED2_PIN 18

void BlinkLEDs()
{
    const int led1 = !(ledsOn & 0b0110);
    const int led2 = !(ledsOn & 0b1100);
    iGpioSetData(LED1_PIN, led1);
    iGpioSetData(LED2_PIN, led2);
    ledsOn <<= 1;
    if(ledsOn >= (1<<4))
        ledsOn = 1;
}

int InitBlinkLEDs()
{
    iGpioInit( LED1_PIN, output, false );
    iGpioInit( LED2_PIN, output, false );
    iGpioSetData(LED1_PIN, 1);
    iGpioSetData(LED2_PIN, 1);
    return 0;
}
