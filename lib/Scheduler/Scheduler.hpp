#ifndef __My_Scheduler____
#define __My_Scheduler____

typedef struct {
    /* period in ticks */
    int period;
    /* ticks until next activation */
    int delay;
    /* function pointer */
    void (*func)(void);
    /* activation counter */
    int exec;
} Sched_Task_t;

#define NT 10


void Sched_Init(void);
int Sched_AddT(void (*f)(void), int d, int p);
void Sched_Schedule(void);
void Sched_Dispatch(void);

#endif




