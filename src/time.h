//
// Created by ysuho on 13-Dec-21.
//
#include <math.h>
#ifndef SENSOR_TIME_H
#define SENSOR_TIME_H

#endif //SENSOR_TIME_H

#define TIMER_CLOCK_PER_MS (float)F_CPU / TIMER_PRESCALER / FUSE_PRESCALER / 1000 / 255

#define FUSE_PRESCALER 1
#define TIMER_PRESCALER 256
#define SLOW_CLOCK_FACTOR 8

#define CLOCK_TIME(time) ({                 \
    (uint16_t)(time * TIMER_CLOCK_PER_MS);\
})

#define SLOW_CLOCK_TIME(time)                 \
    (uint8_t)(time * TIMER_CLOCK_PER_MS / SLOW_CLOCK_FACTOR)

volatile uint16_t clock = 0;
volatile uint8_t slow_clock = 0;

#define TIMER_PRESCALER_BITS() ({                                                \
    int32_t prescaler;                                                           \
    switch(TIMER_PRESCALER) {                                                    \
    case 0:                                                                      \
        prescaler = 0;                                                           \
        break;                                                                   \
    case 1:                                                                      \
        prescaler = _BV(CS00);                                                   \
        break;                                                                   \
    case 8:                                                                      \
        prescaler = _BV(CS01);                                                   \
        break;                                                                   \
    case 64:                                                                     \
        prescaler = _BV(CS00) | _BV(CS01);                                       \
        break;                                                                   \
    case 256:                                                                    \
        prescaler = _BV(CS02);                                                   \
        break;                                                                   \
    case 1024:                                                                   \
        prescaler = _BV(CS02) | _BV(CS00);                                       \
        break;                                                                   \
    }                                                                            \
    prescaler;                                                                   \
})

#define TIME_STRUCTURE(length, type)                                             \
typedef struct {                                                                 \
      volatile  uint##length##_t start;                                          \
      volatile  uint##length##_t end;                                            \
      volatile  uint8_t event_activated;                                         \
}time_event ## type;

TIME_STRUCTURE(16,)
TIME_STRUCTURE(8, _short)


#define UPDATE_EVENT(t_e)                                                        \
    if (t_e.event_activated) {                                                   \
        t_e.event_activated = 2;                                                 \
        t_e.end = _CLOCK(t_e);                                                   \
    } else {                                                                     \
        t_e.event_activated = 1;                                                 \
        t_e.start = _CLOCK(t_e);                                                 \
    }

#define _CLOCK(t_e) \
    ((sizeof(t_e.start) == sizeof(uint16_t)) ? clock : slow_clock)

#define START_TIMER(t_e, clock_cycles)                                           \
    t_e.start = _CLOCK(t_e);                                                     \
    t_e.end = _CLOCK(t_e) + clock_cycles;                                        \
    t_e.event_activated = 1;                                                     \

#define CHECK_TIMER(t_e)({                                                       \
    uint8_t result = 0;                                                          \
    if ((_CLOCK(t_e) >= t_e.end && _CLOCK(t_e) >= t_e.start) ||                  \
    (t_e.end <= _CLOCK(t_e) && _CLOCK(t_e) <= t_e.start)) {                      \
        t_e.event_activated = 0;                                                 \
        /*Expired*/                                                              \
        result = 1;                                                              \
    }                                                                            \
    result;                                                                      \
})

#define ENABLE_TIMER()                                                           \
    /* halt clock  */                                                            \
    GTCCR = _BV(TSM);                                                            \
    /* set prescaler and enable interrupt on timer overflow */                   \
    TCCR0B = TIMER_PRESCALER_BITS();                                             \
    TIMSK0 = _BV(TOIE0);                                                         \
    /* run clock */      \
    GTCCR = 0;

ISR(TIM0_OVF_vect) {
        clock++;
        slow_clock = clock >> (uint8_t)(log(SLOW_CLOCK_FACTOR)/log(2));
}