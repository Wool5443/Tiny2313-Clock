#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <string.h>
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
    SAVE_SETTINGS,
    SET_ALARM,
    SET_HOURS,
    SET_MINS,
    SET_ALARM_HOURS,
    SET_ALARM_MINS,
    SET_BRIGHTNESS,
    ALARM_RINGING,
} Modes;

typedef enum
{
    NO_BLINK,
    BLINK_HRS,
    BLINK_MIN,
    BLINK_DOT,
} BlinkMode;

typedef struct 
{
    uint8_t  alarmEnabled;
    Time     alarmTime;
    uint8_t  brightness;
    uint16_t magic;
} EEPROMsettings;

static uint16_t MAGIC = 0xF00D;

static EEPROMsettings settings;

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
                     N_LETTER = 0b10011110,
                     NOTHING  = 0b11111111,
                     POINT    = 0b11110111;

static const uint8_t BUTTON_PIN = PD5;
static const uint8_t BUZZER_PIN = PD0;

static Time time;

static void initHardware();
static void timersInit(void);

static void drawTime(Time time, BlinkMode blink);
static void drawNumber(uint8_t time);

static void printOnDisplay(uint8_t segmentData);
static void printOn();
static void printOff();

static void writeSettings();
static void readSettings();
static void setDefSettings();

static void readButtons();
static void processSlowButtons(uint8_t button);
static void processFastButtons(uint8_t button);

static uint8_t addHours(uint8_t hours);
static uint8_t decHours(uint8_t hours);
static uint8_t addMins(uint8_t mins);
static uint8_t decMins(uint8_t mins);
static uint8_t addBrightness(uint8_t hours);
static uint8_t decBrightness(uint8_t hours);

static uint8_t drawingSegmentData[4] = {NOTHING, NOTHING, NOTHING, NOTHING};
static uint8_t buttonsState = 0xFF;
static uint16_t timer0OvfCounter = 0;

static uint8_t mode = NORMAL;

static const uint16_t ovfPeriod = F_CPU / 64 / 256;

static const uint16_t shortPressTime =    0.1f * ovfPeriod;
static const uint16_t longPressTime =     3.0f * ovfPeriod;
static const uint16_t setAlarmDelay =     1.0f * ovfPeriod;

typedef enum
{
    PLUS  = (1 << 0),
    SET   = (1 << 1),
    MINUS = (1 << 2)
} Buttons;


static void readButtons()
{
    uint8_t newButtonsState = buttonsState;
    static uint8_t oldButtonsState = 0xFF;
    uint8_t button = newButtonsState ^ oldButtonsState;

    if (newButtonsState < oldButtonsState) // unpress button
    {
        if (timer0OvfCounter > longPressTime)
            processSlowButtons(button);
        else if (timer0OvfCounter > shortPressTime)
            processFastButtons(button);
    }

    if (button)
        timer0OvfCounter = 0;
    oldButtonsState = newButtonsState;
}

static void processSlowButtons(uint8_t button)
{
    switch (mode)
    {
    case NORMAL:
        switch (button)
        {
        case SET:
            mode = SET_BRIGHTNESS;
            break;
        }
        break;
    }
}

static void processFastButtons(uint8_t button)
{
    switch (mode)
    {
    case NORMAL:
    case SET_ALARM:
        switch (button)
        {
        case PLUS:  mode = SET_HOURS;       break;
        case MINUS: mode = SET_ALARM_HOURS; break;
        case SET:
            mode = SET_ALARM;
            settings.alarmEnabled = !settings.alarmEnabled;
            break;
        }
        break;
    case SET_HOURS:
        switch (button)
        {
        case PLUS:  time.hrs = addHours(time.hrs); break;
        case MINUS: time.hrs = decHours(time.hrs); break;
        case SET:   mode = SET_MINS;               break;
        }
        break;
    case SET_MINS:
        switch(button)
        {
        case PLUS:  time.mins = addMins(time.mins); break;
        case MINUS: time.mins = decMins(time.mins); break;
        case SET:   mode = SAVE_SETTINGS;           break;
        }
        break;
    case SET_ALARM_HOURS:
        switch (button)
        {
        case PLUS:  settings.alarmTime.hrs = addHours(settings.alarmTime.hrs); break;
        case MINUS: settings.alarmTime.hrs = decHours(settings.alarmTime.hrs); break;
        case SET:   mode = SET_ALARM_MINS;                                     break;
        }
        break;
    case SET_ALARM_MINS:
        switch (button)
        {
        case PLUS:  settings.alarmTime.mins = addMins(settings.alarmTime.mins); break;
        case MINUS: settings.alarmTime.mins = decMins(settings.alarmTime.mins); break;
        case SET:   mode = SAVE_SETTINGS;                                        break;
        }
        break;
    case SET_BRIGHTNESS:
        switch (button)
        {
        case PLUS:  settings.brightness =  addBrightness(settings.brightness); break;
        case MINUS: settings.brightness =  decBrightness(settings.brightness); break;
        case SET:   mode = SAVE_SETTINGS;                                     break;
        }
        break;
    case ALARM_RINGING:
        if (button)
        {
            mode = NORMAL;
        }
        break;
    }
}

int main(void)
{
    initHardware();
    readSettings();

    while (true)
    {
        if(settings.alarmEnabled && time.hrs == settings.alarmTime.hrs && time.mins == settings.alarmTime.mins && time.secs == 0)
            mode = ALARM_RINGING;

        _delay_ms(10);
        readButtons();
        switch (mode)
        {
        case SAVE_SETTINGS:
            writeSettings();
            mode = NORMAL;
            //no break
        case ALARM_RINGING:
        case NORMAL:
        {
            drawTime(time, BLINK_DOT);
            break;
        }
        case SET_ALARM:
            if (settings.alarmEnabled)
                printOn();
            else
                printOff();

            if (timer0OvfCounter > setAlarmDelay)
                mode = SAVE_SETTINGS;

            break;
        case SET_HOURS:
            drawTime(time, BLINK_HRS);
            break;
        case SET_MINS:
            drawTime(time, BLINK_MIN);
            break;
        case SET_ALARM_HOURS:
            drawTime(settings.alarmTime, BLINK_HRS);
            break;
        case SET_ALARM_MINS:
            drawTime(settings.alarmTime, BLINK_MIN);
            break;
        case SET_BRIGHTNESS:
            OCR0A = settings.brightness;
            drawNumber(OCR0A);
            break;
        }
    }
}

static void initHardware(void)
{
    PORTD |= (1 << BUTTON_PIN);
    DDRB = 0xFF;
    DDRD |= ALL_SEGMENTS;
    DDRD |= (1 << BUZZER_PIN);

    timersInit();
}

void timersInit(void)
{
    cli();

    TCCR1A = 0x00;
    TCCR1B = (1 << CS12) | (1 << CS10); // 1/1024

    TCCR0A = 0x00;
    TCCR0B = (1 << CS01) | (1 << CS00); // 1/64

    TIFR  |= (1 << TOV1)  | (1 << TOV0)  | (1 << OCF0A);
    TIMSK  = (1 << TOIE1) | (1 << TOIE0) | (1 << OCIE0A);

    TCNT1 = MEGA_TIMER_PODGON;

    sei();
}

static void drawTime(Time time, BlinkMode blink)
{
    if(blink == BLINK_HRS && timer0OvfCounter & (1 << 8)) {
        drawingSegmentData[0] = NOTHING;
        drawingSegmentData[1] = NOTHING;
    } else {
        if (time.hrs >= 10)
            drawingSegmentData[0] = DIGIT_SCHEMES[time.hrs / 10];
        else
            drawingSegmentData[0] = NOTHING;

        drawingSegmentData[1] = DIGIT_SCHEMES[time.hrs  % 10];

        if(blink == BLINK_DOT && timer0OvfCounter & (1 << 9))
            drawingSegmentData[1] &= POINT;
    }

    if(blink == BLINK_MIN && timer0OvfCounter & (1 << 8)) {
        drawingSegmentData[2] = NOTHING;
        drawingSegmentData[3] = NOTHING;
    } else {
        drawingSegmentData[2] = DIGIT_SCHEMES[time.mins / 10];
        drawingSegmentData[3] = DIGIT_SCHEMES[time.mins % 10];
    }
}

static void drawNumber(uint8_t number) {
    drawingSegmentData[0] = NOTHING;
    if(number >= 100)
        drawingSegmentData[1] = DIGIT_SCHEMES[number / 100 % 10];
    else
        drawingSegmentData[1] = NOTHING;
    if(number >= 10)
        drawingSegmentData[2] = DIGIT_SCHEMES[number / 10 % 10];
    else
        drawingSegmentData[2] = NOTHING;
    drawingSegmentData[3] = DIGIT_SCHEMES[number % 10];
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

ISR(TIMER0_OVF_vect)
{
    currentDigit = (currentDigit + 1) % 5;
    timer0OvfCounter++;

    if(currentDigit == 4) // Read buttons state
    {
        PORTB = NOTHING;
        buttonsState = 0x00;
        uint8_t lastPBState = PORTD | (SEGMENT_PINS[0] | SEGMENT_PINS[1] | SEGMENT_PINS[2] | SEGMENT_PINS[3]);

        for(int button = 0; button < 3; button++)
        {
            PORTD = lastPBState & ~SEGMENT_PINS[button];
            _delay_us(1);
            buttonsState <<= 1;
            buttonsState |= (~PIND & (1 << BUTTON_PIN)) >> BUTTON_PIN;
        }

        return;
    }

    if(mode == ALARM_RINGING && timer0OvfCounter & (1 << 7))
        PORTD ^= (1 << BUZZER_PIN);
    else
        PORTD &= ~(1 << BUZZER_PIN);


	PORTD |= SEGMENT_PINS[currentDigit];
    PORTB = drawingSegmentData[currentDigit];
}

ISR(TIMER0_COMPA_vect)
{
	PORTD &= ~(SEGMENT_PINS[0] | SEGMENT_PINS[1] | SEGMENT_PINS[2] | SEGMENT_PINS[3]);
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

static uint8_t addHours(uint8_t hours)
{
    if(hours == 23)
        return 0;
    return hours + 1;
}

static uint8_t decHours(uint8_t hours)
{
    if(hours == 0)
        return 23;
    return hours - 1;
}

static uint8_t addMins(uint8_t mins)
{
    if(mins == 59)
        return 0;
    return mins + 1;
}

static uint8_t decMins(uint8_t mins)
{
    if(mins == 0)
        return 59;
    return mins - 1;
}

static uint8_t addBrightness(uint8_t brightness)
{
    if(brightness >= 250)
        return brightness;
    if(brightness >= 100)
        return brightness + 10;
    if(brightness >= 50)
        return brightness + 5;
    if(brightness >= 20)
        return brightness + 2;
    return brightness + 1;
}

static uint8_t decBrightness(uint8_t brightness)
{
    if(brightness >= 110)
        return brightness - 10;
    else if(brightness >= 55)
        return brightness - 5;
    else if(brightness >= 22)
        return brightness - 2;
    else if(brightness >= 1)
        return brightness - 1;
    return brightness;
}

static void readSettings() {
    eeprom_read_block(&settings, 0, sizeof(settings));
    if(settings.magic != MAGIC)
        setDefSettings();
    OCR0A = settings.brightness;
}

static void writeSettings() {
    eeprom_write_block(&settings, 0, sizeof(settings));
}

static void setDefSettings() {
    settings.alarmEnabled = false;
    settings.alarmTime.hrs  = 8;
    settings.alarmTime.mins = 0;
    settings.brightness = 100;
    settings.magic = MAGIC;
}

