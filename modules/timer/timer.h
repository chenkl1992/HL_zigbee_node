#ifndef __TIMER_H
#define __TIMER_H

#define TIMER_NUM                                               8

#define TIMER_OPT_MODE_PERIOD                                   0x00
#define TIMER_OPT_MODE_ONCE                                     0x01

typedef enum {
    TIMER_STATE_STOP = 0,
    TIMER_STATE_START,
}timer_state_t;

typedef void (*timer_callback)(void);

typedef struct {
  timer_state_t state;
  struct {
    uint32_t start;
    uint32_t period;
  }tick;
  uint32_t opt;
  timer_callback callback;
}timer_t;

int32_t timer_create(timer_t *p_timer,
                     uint32_t period,
                     uint32_t opt, 
                     timer_callback callback);
void timer_start(timer_t *p_timer);
void timer_stop(timer_t *p_timer);

void timer_sched(void);
uint32_t Time_To_Unix();

#endif