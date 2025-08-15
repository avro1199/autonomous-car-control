#include "HardwareTimer.h"

#define CHANNELS 8


volatile uint16_t ppmValues[CHANNELS];
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastCapture = 0;

HardwareTimer *MyTim;

void IC_Handler()
{                                               // No parameters
    uint32_t now = MyTim->getCaptureCompare(1); // CCR1 value
    uint32_t diff = (now >= lastCapture) ? (now - lastCapture) : ((0xFFFF - lastCapture) + now);
    lastCapture = now;

    if (diff > 3000)
    {
        ppmIndex = 0; // new frame
    }
    else if (ppmIndex < CHANNELS)
    {
        ppmValues[ppmIndex] = diff;
        ppmIndex++;
    }
}

void setup()
{
    Serial2.begin(115200);

    // TIM2 on PA0 (D2 on Nucleo F446RE)
    MyTim = new HardwareTimer(TIM2);

    MyTim->setMode(1, TIMER_INPUT_CAPTURE_RISING, PA0); // channel 1, rising edge, pin
    MyTim->setOverflow(0xFFFF, MICROSEC_FORMAT);        // microsecond resolution
    MyTim->attachInterrupt(1, IC_Handler);              // channel, callback
    MyTim->setInterruptPriority(0, 0); // highest priority
    MyTim->resume();

    // --------------------------------------------------------------------------------------
    pass;
}

void loop()
{
    Serial2.print("PPM: ");
    for (uint8_t i = 0; i < CHANNELS; i++)
    {
        Serial2.print(ppmValues[i]);
        Serial2.print(" ");
    }
    Serial2.println();
    delay(5);
}