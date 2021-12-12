#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include "time.h"

#define SENSOR PB1

#define LINE_1 PB3                // PB0 as a LED pin
#define LINE_2 PB2
#define LINE_3 PB4
#define PWM_1 PB0

#define SPIKE_THRESHOLD CLOCK_TIME(10)
#define DEBOUNCE_THRESHOLD CLOCK_TIME(100)
#define SHORT_TOUCH_DURATION CLOCK_TIME(500)
#define LONG_TOUCH_DURATION CLOCK_TIME(3000)

#define LINES _BV(LINE_1) | _BV(LINE_2) | _BV(LINE_3)

#define WRITE_STATE(bit_mask, to_write) {             \
    PORTB = (PORTB & ~(bit_mask)) | (to_write);       \
}                                                     \

#define SBI(port,bit) asm volatile("sbi %0,%1"::"I"(_SFR_IO_ADDR(port)),"I" (bit))
#define CBI(port,bit) asm volatile("cbi %0,%1"::"I"(_SFR_IO_ADDR(port)),"I" (bit))

#define BRIGHTNESS_STEP 32
#define BRIGHTNESS_FADEOUT_STEP 4

// TODO fix magic index of fadeout program in state machine (maybe one more enum?)
// TODO include TOUCH_IN_PROGRESS into time_event structure, cut some program size here

enum sensor_state {
    RELEASED          = 0b00010000,
    TOUCH_IN_PROGRESS = 0b00100000,
    TAP               = 0b00110000,
    SHORT_TOUCH       = 0b01000000,
    LONG_TOUCH        = 0b01010000
};

enum machine_states {
    SLEEP         = 0b00000001,
    WORKING       = 0b00000010,
    SET_PROGRAM   = 0b00000011,
};


volatile uint8_t sensor_state = RELEASED;
uint8_t machine_state = SLEEP;
uint8_t brightness = 128;
uint8_t program_ptr = 0;

volatile time_event touch_event = {0};
time_event_short timer = {0};
uint8_t program_stage = 0;

void all_on_program(void) {
    WRITE_STATE(LINES, LINES);
}

void execute_sequence(uint8_t* stages, uint8_t* delays, uint8_t len) {
    if (!timer.event_activated) {
        START_TIMER(timer, delays[program_stage])
        PORTB = stages[program_stage];
        if (program_stage < len-1) {
            program_stage++;
        } else {
            program_stage=0;
        }
    } else CHECK_TIMER(timer);
}

uint8_t delays[] = {SLOW_CLOCK_TIME(500), SLOW_CLOCK_TIME(500), SLOW_CLOCK_TIME(500)};
void running_light_program(void) {
    uint8_t stages[] = {
            _BV(LINE_1),
            _BV(LINE_2),
            _BV(LINE_3)
    };
    execute_sequence(stages, delays, sizeof(stages)/sizeof(stages[0]));
}

void running_shadow_program(void) {
    uint8_t stages[] = {
            _BV(LINE_1) | _BV(LINE_2),
            _BV(LINE_2) | _BV(LINE_3),
            _BV(LINE_3) | _BV(LINE_1)
    };
    execute_sequence(stages, delays, sizeof(stages)/sizeof(stages[0]));
}
uint8_t local_brightness = 128;
uint8_t direction = BRIGHTNESS_FADEOUT_STEP;
void fadeout(void) {
    WRITE_STATE(LINES, LINES);
    if (!timer.event_activated) {
        START_TIMER(timer, SLOW_CLOCK_TIME(100))
        OCR0A = local_brightness;
        if (UINT8_MAX - local_brightness <= BRIGHTNESS_FADEOUT_STEP ||
        local_brightness <= BRIGHTNESS_FADEOUT_STEP) {
            direction = -direction;
        }
        local_brightness += direction;
    } else {
        CHECK_TIMER(timer);
    }
}

void (*programs[])() = {
        all_on_program,
        running_light_program,
        running_shadow_program,
        fadeout
};
uint8_t program_count = sizeof(programs) / sizeof(void *);

// Touch
ISR(PCINT0_vect){
    UPDATE_EVENT(touch_event)
    sensor_state = TOUCH_IN_PROGRESS;
}

int main(void) {
    /* setup */
    SBI(ACSR, ACD);                                                 // power off comparator
    CBI(ADCSRA, ADEN);                                              // power off ADC
    MCUCR = _BV(SM1);                                               // set power down sleep mode

    // set LINEs 1-3 and PWM as output, SENSOR pin stays input
    DDRB = LINES | _BV(PWM_1);
    TCCR0A = _BV(WGM00) | _BV(WGM01) | _BV(COM0A1) | _BV(COM0A1);   // enable PWM mode (WGM) and using inverse PWM (COMO)

    ENABLE_TIMER()

    /* interrupt */
    // set interrupt mode to pin change interrupt bit in GIMSK register
    GIMSK |= (1 << PCIE);
    SBI(PCMSK, SENSOR);
    sei();                                                          // enable interrupts globally
    WRITE_STATE(LINES, LINES)
    while (1) {
        cli();
        if (touch_event.event_activated == 2 && clock - touch_event.end > DEBOUNCE_THRESHOLD) {
            // Touch sensor was pressed and released, determine type of control input
            // Filter out short spikes
            if (touch_event.end - touch_event.start < SPIKE_THRESHOLD) {
                sensor_state = RELEASED;
            } else if (touch_event.end - touch_event.start < SHORT_TOUCH_DURATION) {
                sensor_state = TAP;
            } else if (touch_event.end - touch_event.start < LONG_TOUCH_DURATION) {
                sensor_state = SHORT_TOUCH;
            } else {
                sensor_state =  LONG_TOUCH;
            }
            touch_event.event_activated = 0;
        }
        sei();

        // State machine
        switch (machine_state | sensor_state) {
            case SLEEP | LONG_TOUCH:
                machine_state = WORKING;
                break;
            case SET_PROGRAM | LONG_TOUCH:                        // go to sleep from set program mode
            case SLEEP | RELEASED:                                // go to sleep from any input except LONG_TOUCH
            case WORKING | LONG_TOUCH:
                machine_state = SLEEP;
                sensor_state = RELEASED;
                touch_event.event_activated = 0;
                WRITE_STATE(LINES, 0);
                MCUCR |= _BV(SE);                                 // enable sleep
                asm("sleep"::);                                   // send processor to sleep

                // an interrupt will wake me up here
                MCUCR ^= _BV(SE);                                 // disable sleep
                break;
            case WORKING | TAP:
                brightness += BRIGHTNESS_STEP;
                // no need to change brightness with fadeout program
                if (program_ptr != 3) OCR0A = brightness;
                break;
            case WORKING | SHORT_TOUCH:
                machine_state = SET_PROGRAM;
                break;
            case SET_PROGRAM | TAP:
                program_ptr = (program_ptr < program_count - 1) ? program_ptr + 1 : 0;
                break;
            case SET_PROGRAM | SHORT_TOUCH:
                machine_state = WORKING;
                timer.event_activated = 0;
        }

        if (machine_state != SLEEP) programs[program_ptr]();

        if (sensor_state != TOUCH_IN_PROGRESS) {
            sensor_state = RELEASED;
        }
    }
}