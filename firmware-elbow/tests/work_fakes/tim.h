#ifndef WORK_TEST_TIM_H
#define WORK_TEST_TIM_H
typedef struct { unsigned CCR1, CCR2, CCR3, CCR4; } FakeTimer;
extern FakeTimer timer;
#define TIM1 (&timer)
#endif
