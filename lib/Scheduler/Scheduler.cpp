#include <Scheduler.hpp>

Sched_Task_t Tasks[NT];

void Sched_Init(void){
    for(int x=0; x<NT; x++)
        Tasks[x].func = 0;
}

int Sched_AddT(void (*f)(void), int d, int p){
    for(int x=0; x<NT; x++){
        if (!Tasks[x].func) {
            Tasks[x].period = p;
            Tasks[x].delay = d;
            Tasks[x].exec = 0;
            Tasks[x].func = f;
            return x;
        }
    }   
    return -1;
}

void Sched_Schedule(void){
    for(int x=0; x<NT; x++) {
        if(Tasks[x].func){
            if(Tasks[x].delay){
                Tasks[x].delay--;
            } else {
                /* Schedule Task */
                Tasks[x].exec++;
                Tasks[x].delay =
                Tasks[x].period-1;
            }
        }
    }
}

void Sched_Dispatch(void){
    for(int x=0; x<NT; x++) {
        if((Tasks[x].func) && (Tasks[x].exec)) {
            Tasks[x].exec=0;
            Tasks[x].func();
            /* Delete task
            * if one-shot */
            if(!Tasks[x].period)
                Tasks[x].func = 0;
        }
    }
}
