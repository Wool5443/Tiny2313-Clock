#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdbool.h>

typedef struct
{
    uint8_t hrs;
    uint8_t mins;
    uint8_t secs;
} Time;

typedef enum 
{
    NORMAL,
    SET_ALARM,
    SET_HOURS,
    SET_MINS,
    SET_ALARM_HOURS,
    SET_ALARM_MINS,
    ALARM_RINGING,
} Modes;

typedef enum
{
    NO_BLINK,
    BLINK_HRS,
    BLINK_MIN,
    BLINK_DOT,
} BlinkMode;

static const uint16_t MEGA_TIMER_PODGON = 49911;

static const uint8_t SEGMENT_PINS[] = { 1 << PD4, 1 << PD3, 1 << PD2, 1 << PD6 };
static const uint8_t ALL_SEGMENTS   = 1 << PD4 | 1 << PD3 | 1 << PD2 | 1 << PD6;

static const uint8_t DIGIT_SCHEMES[] = 
{   //bgcfPade
    0b01001000,
    0b01011111,
    0b00111000,
    0b00011001,
    0b00001111,
    0b10001001,
    0b10001000,
    0b01011011,
    0b00001000,
    0b00001001,
};

static const uint8_t F_LETTER = 0b10101010,
                     N_LETTER = 0b10000110,
                     NOTHING  = 0b11111111,
                     POINT    = 0b11110111;

static Time time;

static void initHardware();
void timer1_init(void);

static void drawTime(uint8_t currentSegmen, Time time, BlinkMode blink);

static void printOnDisplay(uint8_t currentDigit, uint8_t segmentData);
static void printOn(uint8_t currentDigit);
static void printOff(uint8_t currentDigit);

int main(void)
{
    initHardware();
    timer1_init();

    uint8_t currentDigit = 0;
    while (true)
    {
        drawTime(currentDigit, time, NO_BLINK);
        currentDigit = (currentDigit + 1) % 4;
        _delay_ms(1);
    }
}

static void initHardware(void)
{
    DDRB = 0xFF;
    DDRD |= ALL_SEGMENTS;
}

void timer1_init(void)
{
    cli();

    TCCR1A = 0x00;
    TCCR1B = (1 << CS12) | (1 << CS10);
    TIFR   |= 1 << TOV1;
    TIMSK  = (1 << TOIE1);

    sei();
}

static void drawTime(uint8_t currentDigit, Time time, BlinkMode blink)
{
    uint8_t segmentData = NOTHING;

    switch (currentDigit)
    {
        case 0:
            if (time.hrs >= 10)
                segmentData = DIGIT_SCHEMES[time.hrs / 10];
            break;
        case 1:
            segmentData = DIGIT_SCHEMES[time.hrs % 10];
            break;
        case 2:
            segmentData = DIGIT_SCHEMES[time.mins / 10];
            break;
        case 3:
            segmentData = DIGIT_SCHEMES[time.mins % 10];
            break;
    }

    printOnDisplay(currentDigit, segmentData);
}

ISR(TIMER1_OVF_vect)
{
    time.secs += 60;

    if (time.secs >= 60)
    {
        time.mins += 1;
        time.secs = 0;
    }

    if (time.mins >= 60)
    {
        time.hrs += 1;
        time.mins = 0;
    }

    if (time.hrs >= 24)
        time.hrs = 0;

    TCNT1 = MEGA_TIMER_PODGON;
}

static void printOnDisplay(uint8_t currentDigit, uint8_t segmentData)
{
	PORTD &= ~(SEGMENT_PINS[0] | SEGMENT_PINS[1] | SEGMENT_PINS[2] | SEGMENT_PINS[3]);
	PORTB = segmentData;
	PORTD |=  SEGMENT_PINS[currentDigit];
}

static void printOn(uint8_t currentDigit)
{
    uint8_t symbols[] = { DIGIT_SCHEMES[0], N_LETTER, NOTHING, NOTHING };
    printOnDisplay(currentDigit, symbols[currentDigit]);
}

static void printOff(uint8_t currentDigit)
{
    uint8_t symbols[] = { DIGIT_SCHEMES[0], F_LETTER, F_LETTER, NOTHING };
    printOnDisplay(currentDigit, symbols[currentDigit]);
}
