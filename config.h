#ifndef CONFIG_H
#define CONFIG_H

// CONFIG1H
#pragma config FOSC    = INTIO67
#pragma config PLLCFG  = OFF
#pragma config PRICLKEN = ON
#pragma config FCMEN   = OFF
#pragma config IESO    = OFF

// CONFIG2L
#pragma config PWRTEN  = OFF
#pragma config BOREN   = OFF

// CONFIG2H
#pragma config WDTEN   = OFF

// CONFIG3H
#pragma config CCP2MX  = PORTC1

// CONFIG4L
#pragma config LVP     = ON

#endif
