#include <xc.h>
#include <stdint.h>
#include "config.h"

#define _XTAL_FREQ 16000000UL

// =========================
// Motor direction pins
// =========================
#define AIN1 LATDbits.LATD0
#define AIN2 LATDbits.LATD1
#define BIN1 LATDbits.LATD2
#define BIN2 LATDbits.LATD3

// =========================
// Ultrasonic sensor pins
// =========================
#define TRIG LATBbits.LATB0
#define ECHO PORTBbits.RB1

// =========================
// Line sensors
// Left   -> RA2
// Middle -> RA1
// Right  -> RB2
//
// Confirmed behavior:
// BLACK = 1
// WHITE = 0
// =========================
#define IR_LEFT  PORTAbits.RA2
#define IR_MID   PORTAbits.RA1
#define IR_RIGHT PORTBbits.RB2

// =========================
// Obstacle stop threshold
// =========================
#define STOP_DISTANCE_IN 5.0

// =========================
// PWM speeds
// =========================
#define BASE_SPEED_LEFT  55
#define BASE_SPEED_RIGHT 58
#define SLIGHT_INNER     35
#define SLIGHT_OUTER     82
#define HARD_INNER       0
#define HARD_OUTER       95
#define CROSS_SPEED      45
#define SEARCH_SPEED     55

// Keeps track of last seen line direction (for line-lost recovery)
static int8_t last_error = 0;

// =========================
// Function Prototypes
// =========================
void GPIO_Init(void);
void PWM_Init(void);
void Timer1_Init(void);
void setLeftSpeed(uint8_t duty);
void setRightSpeed(uint8_t duty);
void forward(void);
void stop_motors(void);
void slight_left(void);
void slight_right(void);
void hard_left(void);
void hard_right(void);
uint8_t sensor_on_line(uint8_t raw);
void line_follow_control(void);
uint16_t measure_echo_us(void);
float get_distance_inches(void);

// =========================
// GPIO Init
// =========================
void GPIO_Init(void)
{
    // Disable analog
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;
    ANSELD = 0x00;
    ANSELE = 0x00;

    // Disable comparators
    CM1CON0 = 0x00;
    CM2CON0 = 0x00;

    // Motor direction outputs
    TRISDbits.TRISD0 = 0; // AIN1
    TRISDbits.TRISD1 = 0; // AIN2
    TRISDbits.TRISD2 = 0; // BIN1
    TRISDbits.TRISD3 = 0; // BIN2

    // Ultrasonic
    TRISBbits.TRISB0 = 0; // TRIG output
    TRISBbits.TRISB1 = 1; // ECHO input

    // Line sensors
    TRISAbits.TRISA2 = 1; // Left
    TRISAbits.TRISA1 = 1; // Middle
    TRISBbits.TRISB2 = 1; // Right

    LATD = 0x00;
    LATBbits.LATB0 = 0;
}

// =========================
// PWM Init
// CCP1 = RC2
// CCP2 = RC1
// =========================
void PWM_Init(void)
{
    TRISCbits.TRISC2 = 0; // CCP1
    TRISCbits.TRISC1 = 0; // CCP2

    // Timer2 ON, prescaler 1:16
    T2CON = 0b00000111;
    PR2 = 255;

    // PWM mode
    CCP1CON = 0b00001100;
    CCP2CON = 0b00001100;
    CCPR1L = 0;
    CCPR2L = 0;
}

// =========================
// Timer1 Init for HC-SR04 echo timing
//
// Fosc = 16 MHz -> instruction clock (Fosc/4) = 4 MHz
// Timer1 clock source = instruction clock (TMR1CS = 00)
// Prescaler = 1:4  ->  4 MHz / 4 = 1 MHz  ->  1 tick = 1 us
//
// NOTE: an earlier version of this file set T1CON = 0b00110000,
// which actually selects a 1:8 prescaler (2 us/tick), not the 1:4
// (1 us/tick) the comments claimed. That mismatch would have made
// get_distance_inches() report roughly double the true distance.
// Fixed below so 1 timer tick == 1 us as get_distance_inches() assumes.
// =========================
void Timer1_Init(void)
{
    T1CON = 0b00100000; // TMR1CS = 00, T1CKPS1:0 = 10 (1:4 prescale), TMR1ON = 0
    TMR1H = 0;
    TMR1L = 0;
}

// =========================
// PWM helpers
// =========================
void setLeftSpeed(uint8_t duty)
{
    CCPR1L = duty;
}

void setRightSpeed(uint8_t duty)
{
    CCPR2L = duty;
}

// =========================
// Motor direction helpers
// =========================
void forward(void)
{
    AIN1 = 1;
    AIN2 = 0;
    BIN1 = 1;
    BIN2 = 0;
}

void stop_motors(void)
{
    AIN1 = 0;
    AIN2 = 0;
    BIN1 = 0;
    BIN2 = 0;
    setLeftSpeed(0);
    setRightSpeed(0);
}

void slight_left(void)
{
    forward();
    setLeftSpeed(SLIGHT_INNER);
    setRightSpeed(SLIGHT_OUTER);
}

void slight_right(void)
{
    forward();
    setLeftSpeed(SLIGHT_OUTER);
    setRightSpeed(SLIGHT_INNER);
}

void hard_left(void)
{
    forward();
    setLeftSpeed(HARD_INNER);
    setRightSpeed(HARD_OUTER);
}

void hard_right(void)
{
    forward();
    setLeftSpeed(HARD_OUTER);
    setRightSpeed(HARD_INNER);
}

// =========================
// Sensor logic
// BLACK = 1, WHITE = 0
// =========================
uint8_t sensor_on_line(uint8_t raw)
{
    return (raw == 1);
}

// =========================
// Line following logic
// =========================
void line_follow_control(void)
{
    uint8_t L = sensor_on_line(IR_LEFT);
    uint8_t M = sensor_on_line(IR_MID);
    uint8_t R = sensor_on_line(IR_RIGHT);

    // 010 = centered
    if (!L && M && !R)
    {
        forward();
        setLeftSpeed(BASE_SPEED_LEFT);
        setRightSpeed(BASE_SPEED_RIGHT);
        last_error = 0;
    }
    // 110 = slight left
    else if (L && M && !R)
    {
        slight_left();
        last_error = -1;
    }
    // 011 = slight right
    else if (!L && M && R)
    {
        slight_right();
        last_error = 1;
    }
    // 100 = hard left
    else if (L && !M && !R)
    {
        hard_left();
        last_error = -2;
    }
    // 001 = hard right
    else if (!L && !M && R)
    {
        hard_right();
        last_error = 2;
    }
    // 111 = wide line / intersection
    else if (L && M && R)
    {
        forward();
        setLeftSpeed(CROSS_SPEED);
        setRightSpeed(CROSS_SPEED);
    }
    // 101 = transition / straddle
    else if (L && !M && R)
    {
        forward();
        setLeftSpeed(CROSS_SPEED);
        setRightSpeed(CROSS_SPEED);
    }
    // 000 = line lost -> search based on last known direction
    else
    {
        if (last_error < 0)
        {
            forward();
            setLeftSpeed(0);
            setRightSpeed(SEARCH_SPEED);
        }
        else if (last_error > 0)
        {
            forward();
            setLeftSpeed(SEARCH_SPEED);
            setRightSpeed(0);
        }
        else
        {
            forward();
            setLeftSpeed(SEARCH_SPEED);
            setRightSpeed(SEARCH_SPEED);
        }
    }
}

// =========================
// Ultrasonic echo timing
// Returns echo pulse width in us (via Timer1), or 0xFFFF on timeout
// =========================
uint16_t measure_echo_us(void)
{
    uint16_t count;

    // Trigger pulse
    TRIG = 0;
    __delay_us(2);
    TRIG = 1;
    __delay_us(10);
    TRIG = 0;

    // Wait for echo to go high
    TMR1H = 0;
    TMR1L = 0;
    PIR1bits.TMR1IF = 0;
    T1CONbits.TMR1ON = 1;

    while (!ECHO)
    {
        if (PIR1bits.TMR1IF)
        {
            T1CONbits.TMR1ON = 0;
            return 0xFFFF;
        }
    }

    // Measure echo high pulse
    TMR1H = 0;
    TMR1L = 0;
    PIR1bits.TMR1IF = 0;

    while (ECHO)
    {
        if (PIR1bits.TMR1IF)
        {
            T1CONbits.TMR1ON = 0;
            return 0xFFFF;
        }
    }

    T1CONbits.TMR1ON = 0;
    count = ((uint16_t)TMR1H << 8) | TMR1L;
    return count;
}

// =========================
// Convert echo time to inches
// HC-SR04: inches ~= us / 148
// =========================
float get_distance_inches(void)
{
    uint16_t echo_us = measure_echo_us();

    if (echo_us == 0xFFFF)
    {
        return 999.0;
    }

    return ((float)echo_us) / 148.0;
}

// =========================
// Main
// =========================
void main(void)
{
    float distance_in;

    // 16 MHz internal oscillator
    OSCCON = 0b01110000;

    GPIO_Init();
    PWM_Init();
    Timer1_Init();

    // Start with known PWM values
    setLeftSpeed(BASE_SPEED_LEFT);
    setRightSpeed(BASE_SPEED_RIGHT);

    while (1)
    {
        distance_in = get_distance_inches();

        if (distance_in <= STOP_DISTANCE_IN)
        {
            stop_motors();
            last_error = 0;
        }
        else
        {
            setLeftSpeed(BASE_SPEED_LEFT);
            setRightSpeed(BASE_SPEED_RIGHT);
            line_follow_control();
        }

        __delay_ms(25);
    }
}
