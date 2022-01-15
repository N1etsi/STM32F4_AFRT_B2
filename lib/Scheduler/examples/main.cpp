#include <Arduino.h>
#include <Scheduler.hpp>
#include <STM32TimerInterrupt.h>
#include <Wire.h>

#define INTERRUPT_INTERVAL_US 500*1000 // half sec
#define SERIAL_BAUD 115200
#define LED1 2

STM32Timer ITimer0(TIM4); // se for pra mudar o timer, mudar aqui

/*
Isto neste momento pisca um led cada vez que um interrupt é handled mas tbm avança o scheduler
*/
void TimerHandler0()
{
  Sched_Schedule();
}

void Tasks_Init() {
  /* Inicializar cenas das tasks aqui */
  ITimer0.attachInterruptInterval(1000 * 500, TimerHandler0);
}

void TaskExample() {
  digitalWrite(LED1,!digitalRead(LED1));
  //delay(500);  // delay para testar as não-preemptions
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD);
  while(!Serial);

  Tasks_Init();
  Sched_Init();
  //Sched_AddT(/*task*/, /*delay*/, /*period*/); - ORDER OF INSERT = PRIORITY || tempos em incrementos de INTERRUPT_INTERVAL_US
  Sched_AddT(TaskExample, 0, 1); // exemplo: pisca built-in led cada 
  
}

void loop() {
  Sched_Dispatch();
}