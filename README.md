# Syncbox

This repository contains the code that is run on the hardware sync components of the CANDOR system. Specifically, main.c is the program that is loaded onto a TI launchpad kit, LP-MSP430FR2476.

This code should be flashed onto the TI board using code composer studio and requires the MSP430 and the driverlib libraries. 

## Operation
The TI board acts as a signal generator for multiple cameras operating at 120Hz, 60Hz, and 30Hz. It uses a push button to turn the signals on and off.

Pinout:

    Input: 
        One push button or trigger connected to P2.4, active high
    Output:
        120Hz - P1.0, P1.1, P1.2, P1.3, P1.6, P1.7
        60Hz  - P2.0, P2.1, P2.2, P2.5, P2.6, P2.7
        30Hz  - P3.1, P3.2, P3.3
        30Hz (inverted) - P3.5, P3.6, P3.7
        Sync high - P5.1, P5.2
        Pulse out - P5.0

Whenever the button is pressed (high signal to P2.4) all of the signals start, outputting signals to the above mentioned pins. The button can be pressed again to turn the system off. The 120Hz, 60Hz, and 30Hz all rise at the same time after the button is pressed and stay in sync with each other. Additionally, both sync high pins are held high while the system is active and are low when the system is turned off. The pulse out is held high momentarily when the system is turned on and then turns off as soon as the signals are output. The onboard LED1 should be on while the system is active.


If you want to edit the frequencies, here is the msp gpio pinout relative to the cable pins

| Pin on Syncbox cable plug | Hand | Face |
|---------------------------|------|------|
| 1                         | P1.6 | P1.0 |
| 2                         | P1.7 | P1.1 |
| 3                         | P2.1 | P1.2 |
| 4                         | P2.5 | P1.3 |
| 5                         | P2.6 | P2.0 |
| 6                         | P3.5 | P3.2 |


## To change the base frequency from 120Hz (120, 60, 30) to 74Hz (74, 37, 18.5):
- change `TA0CCR0` to be set to 40000 (counts)
- change `TA0CTL` with bit masks: 
```
TA0CTL &= ~0x0080;
TA0CTL |=  0x0040;
``` 
- this sets clk divider to /2
- to change the pins, lines `155` to `172` define which pins get which frequencies
- tweak and check with and oscilloscope as needed


This code was created by Sean McGinnis and edited by Ava Teegarden