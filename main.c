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

typedef enum
{
    PLUS  = (1 << 0),
    SET   = (1 << 1),
    MINUS = (1 << 2)
} Buttons;

static Time time;

static void initHardware();
void timers_init(void);

static void drawTime(Time time, BlinkMode blink);

static void printOnDisplay(uint8_t segmentData);
static void printOn();
static void printOff();

static void allSegmentsOff();

static uint8_t drawingSegmentData[4] = {NOTHING, NOTHING, NOTHING, NOTHING};

int main(void)
{
    initHardware();
    timers_init();

    time.hrs = 13;
    time.mins = 30;

    while (true)
    {
        _delay_ms(10);
        drawTime(time, NO_BLINK);
        OCR0A++;
    }
}

static void initHardware(void)
{
    DDRB = 0xFF;
    DDRD |= ALL_SEGMENTS;
}

void timers_init(void)
{
    cli();

    TCCR1A = 0x00;
    TCCR1B = (1 << CS12) | (1 << CS10); // 1/1024

    TCCR0A = 0x00;
    TCCR0B = (1 << CS01) | (1 << CS00); // 1/1024

    TIFR  |= (1 << TOV1)  | (1 << TOV0)  | (1 << OCF0A);
    TIMSK  = (1 << TOIE1) | (1 << TOIE0) | (1 << OCIE0A);

    OCR0A = 100; // brightness

    sei();
}

static void drawTime(Time time, BlinkMode blink)
{
    if (time.hrs >= 10)
        drawingSegmentData[0] = DIGIT_SCHEMES[time.hrs / 10];
    else
        drawingSegmentData[0] = NOTHING;

    drawingSegmentData[1] = DIGIT_SCHEMES[time.hrs  % 10];
    drawingSegmentData[2] = DIGIT_SCHEMES[time.mins / 10];
    drawingSegmentData[3] = DIGIT_SCHEMES[time.mins % 10];
}

ISR(TIMER1_OVF_vect)
{
    time.secs += 1;

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

// Drawing time

static uint8_t currentDigit = 0;
static uint16_t buttonOvfCounter = 0;

ISR(TIMER0_OVF_vect)
{
    buttonOvfCounter++;
	PORTD |= SEGMENT_PINS[currentDigit];
    PORTB = drawingSegmentData[currentDigit];
}

ISR(TIMER0_COMPA_vect)
{
    allSegmentsOff();
    currentDigit = (currentDigit + 1) % 4;
}

static void printOn()
{
    drawingSegmentData[0] = DIGIT_SCHEMES[0];
    drawingSegmentData[1] = N_LETTER;
    drawingSegmentData[2] = NOTHING;
    drawingSegmentData[3] = NOTHING;
}

static void printOff()
{
    drawingSegmentData[0] = DIGIT_SCHEMES[0];
    drawingSegmentData[1] = F_LETTER;
    drawingSegmentData[2] = F_LETTER;
    drawingSegmentData[3] = NOTHING;
}

/* static uint8_t readButtons() */
/* { */
/*     PORTB = NOTHING; */

/*     uint8_t buttons = 0; */
/*     for(int i = 0; i < 3; i++) */
/*     { */
/*         PORTD &= ~(SEGMENT_PINS[0] | SEGMENT_PINS[1] | SEGMENT_PINS[2] | SEGMENT_PINS[3]); */
/*         _delay_us(10); */
/*         PORTB = segmentData; */
/*         PORTD |=  SEGMENT_PINS[currentDigit]; */
/*         PORTD = SEGMENT_PINS[i]; */

/*     } */

/*     PORTD &= ~(SEGMENT_PINS[0] | SEGMENT_PINS[1] | SEGMENT_PINS[2] | SEGMENT_PINS[3]); */
/*     return buttons; */
/* } */

static void allSegmentsOff() {
	PORTD &= ~(SEGMENT_PINS[0] | SEGMENT_PINS[1] | SEGMENT_PINS[2] | SEGMENT_PINS[3]);
}

