// File: TwoTasks.c 

#include <stdio.h>
#include "includes.h"
#include <string.h>
#include "altera_avalon_performance_counter.h"
#define DEBUG 0

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define   TASK_STACKSIZE       2048
OS_STK    task1_stk[TASK_STACKSIZE];
OS_STK    task2_stk[TASK_STACKSIZE];
OS_STK    stat_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define TASK1_PRIORITY      6  // highest priority
#define TASK2_PRIORITY      7
#define TASK_STAT_PRIORITY 12  // lowest priority 

OS_EVENT* sem0;
OS_EVENT* sem1;
 
void printStackSize(char* name, INT8U prio) 
{
  
  OS_STK_DATA stk_data;
    INT8U perr;
  perr = OSTaskStkChk(prio, &stk_data);
  if (perr == OS_NO_ERR) {
    if (DEBUG == 1)
      printf("%s (priority %d) - Used: %d; Free: %d\n", 
       name, prio, stk_data.OSUsed, stk_data.OSFree);
  }
  else
    {
      if (DEBUG == 1)
  printf("Stack Check Error!\n");    
    }
}

/* Prints a message and sleeps for given time interval */
void task1(void* pdata)
{
        INT8U perr; 

  while (1)
    { 

      OSSemPend(sem0,0,&perr);
      printf("Task 0 - State 0\n");
       
      
      PERF_BEGIN(PERFORMANCE_COUNTER_BASE,1);

      OSSemPost(sem1);

      OSSemPend(sem0,0,&perr);
      printf("Task 0 - State 1\n");
      OSTimeDlyHMSM(0, 0, 0, 500);
      OSSemPost(sem0);
       


    }
}

/* Prints a message and sleeps for given time interval */
void task2(void* pdata)
{
        INT8U perr; 
  alt_u64 time = 0;
  int time_list[10]; 
  int sum = 0; 
  int timer_id = -1; 
  int entry = 0;
  int i = 0;
  while (1)
    { 
     //time = perf_get_section_time((void*)PERFORMANCE_COUNTER_BASE,1);
     //printf("Elapsed time %d\n",time);

    OSSemPend(sem1,0,&perr);
    PERF_END(PERFORMANCE_COUNTER_BASE,1);
    time = perf_get_section_time(PERFORMANCE_COUNTER_BASE,1);
    perf_print_formatted_report((void*)PERFORMANCE_COUNTER_BASE, alt_get_cpu_freq(), 1, "Context Switch 1");
    timer_id=(timer_id + 1) % 10; 
    time_list[timer_id] = time;
    printf("timer_id %d\n",timer_id );
    if(timer_id == 9 || entry ){
      sum = 0;
      for(i = 0; i<10;i++)
        sum += time_list[i];
      printf("sum : %d    average %0.6f\n", sum, sum/10.0);
      entry = 1;
    }
      //PERF_RESET(PERFORMANCE_COUNTER_BASE);
     // PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
    PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);
    PERF_RESET(PERFORMANCE_COUNTER_BASE);
    PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);

    printf("time: %d\n",time );
    printf("Task 1 - State 0\n");
    printf("Task 1 - State 1\n");
    OSTimeDlyHMSM(0, 0, 0, 500); 
    OSSemPost(sem0);

    }
}

/* Printing Statistics */
void statisticTask(void* pdata)
{

  while(1)
    {

    }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
  PERF_RESET(PERFORMANCE_COUNTER_BASE);
  PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
  printf("Lab 3 - Two Tasks\n");
  sem0 = OSSemCreate(1);
  sem1 = OSSemCreate(0);
  OSTaskCreateExt
    ( task1,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task1_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK1_PRIORITY,               // Desired Task priority
      TASK1_PRIORITY,               // Task ID
      &task1_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
      );
     
  OSTaskCreateExt
    ( task2,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task2_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK2_PRIORITY,               // Desired Task priority
      TASK2_PRIORITY,               // Task ID
      &task2_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                       
      );  

  if (DEBUG == 1)
    {
      OSTaskCreateExt
  ( statisticTask,                // Pointer to task code
    NULL,                         // Pointer to argument passed to task
    &stat_stk[TASK_STACKSIZE-1],  // Pointer to top of task stack
    TASK_STAT_PRIORITY,           // Desired Task priority
    TASK_STAT_PRIORITY,           // Task ID
    &stat_stk[0],                 // Pointer to bottom of task stack
    TASK_STACKSIZE,               // Stacksize
    NULL,                         // Pointer to user supplied memory (not needed)
    OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
    OS_TASK_OPT_STK_CLR           // Stack Cleared                              
    );
    }  

  OSStart();
  return 0;
}
