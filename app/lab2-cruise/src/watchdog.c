/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description:
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */
#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001
//4.5
#define EXTRA_INPUT_FLAG    (0x1F<<4) // 4 to 9 , surmising buttons actually mapped to those positions

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

#define POSITION_CLEAR_FLAG (0x3f << 12)

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIOTask_Stack[TASK_STACKSIZE];
OS_STK SwitchTask_Stack[TASK_STACKSIZE];
///////for 4.5
OS_STK WatchdogTask_Stack[TASK_STACKSIZE];
OS_STK OverloadTask_Stack[TASK_STACKSIZE];
OS_STK ExtraTask_Stack[TASK_STACKSIZE];

// Task Priorities

#define STARTTASK_PRIO      5
#define VEHICLETASK_PRIO    10
#define CONTROLTASK_PRIO    12
#define BUTTTONIO_TASK_PRIO 14
#define SWITCHTASK_PRIO     16
/////for 4.5
#define WATCHDOGTASK_PRIO 6 // should have highest(except for start) in order to signal at end of hyperperiod
#define EXTRATASK_PRIO    11 // higher prio than other tasks except for start and wtachdog; to stall properly
#define OVERLOADTASK_PRIO 20 // overload low priortity task that runs in the background
// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
/////for 4.5
#define EXTRA_PERIOD  300
#define HYPERPERIOD   300 // lcm of all tasks except watch-d and overload
#define OVERLOAD_PERIOD HYPERPERIOD 
#define WATCHDOGPERIOD_PERIOD  HYPERPERIOD // lcm 
//MACRO to convert ticks to MS; min val 50ms
#define MS_IN_TICKS(ms) ((2*ms)/100) // each tick 50 ms, one period 2 ticks or smth

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_BrakeControl;
OS_EVENT *Mbox_Engine;
OS_EVENT *Mbox_EngineControl;
OS_EVENT *Mbox_Gear;
OS_EVENT *Mbox_Cruise_Control;
OS_EVENT *Mbox_Gas_Pedal;
////for 4.5
OS_EVENT *Mbox_Watchdog_OK_Signal;// maibox for watch dog "ok signal" ; 1 - signal received , 0 - nope
OS_EVENT *Mbox_Util;//mailbox for extra task to set utilization; 
                    //affect computation time with sw4 to sw9 ; 6 bit value 
// Semaphores
OS_EVENT *Vehicle_Sem; 
OS_EVENT *Control_Sem; 
OS_EVENT *ButtonIO_Sem; 
OS_EVENT *SwitchIO_Sem; 
///for 4.5
OS_EVENT *Watchdog_Sem;// sem for watchdog timer period; should be one hyperperiod
OS_EVENT *Overload_Sem;// sem for overload task period;
OS_EVENT *Extra_Sem;// sem for extra task period; same period as other tasks

// SW-Timer
OS_TMR *Vehicle_Timer; 
OS_TMR *Control_Timer; 
OS_TMR *ButtonIO_Timer; 
OS_TMR *SwitchIO_Timer;
/// for 4.5
OS_TMR *Watchdog_Timer; 
OS_TMR *Extra_Timer; 
OS_TMR *Overload_Timer;
// oneshot timer also exists in task extra; to genreate a delay 

OS_TMR_CALLBACK Vehicle_Timer_Callback(void *ptmr, void *parg) {
  INT8U err; 
  // printf("Vechicle task callback\n");
  err = OSSemPost(Vehicle_Sem); 
} 

OS_TMR_CALLBACK Control_Timer_Callback(void *ptmr, void *parg){
  INT8U err; 
  // printf("Control task callback\n");
  err = OSSemPost(Control_Sem); 
} 

OS_TMR_CALLBACK ButtonIO_Timer_Callback(void *ptmr, void *parg){
  INT8U err; 
  // printf("Button task callback\n");
  err = OSSemPost(ButtonIO_Sem); 
} 

OS_TMR_CALLBACK SwitchIO_Timer_Callback(void *ptmr, void *parg){
  INT8U err; 
  // printf("Switch task callback\n");
  err = OSSemPost(SwitchIO_Sem); 
} 

////////////////for 4.5 
OS_TMR_CALLBACK Watchdog_Timer_Callback(void *ptmr, void *parg) {
  INT8U err; 
  // printf("Vechicle task callback\n");
  err = OSSemPost(Watchdog_Sem); 
} 

OS_TMR_CALLBACK Overload_Timer_Callback(void *ptmr, void *parg){
  INT8U err; 
  // printf("Control task callback\n");
  err = OSSemPost(Overload_Sem); 
} 
OS_TMR_CALLBACK Extra_Timer_Callback(void *ptmr, void *parg){
  INT8U err; 
  // printf("Control task callback\n");
  err = OSSemPost(Extra_Sem); 
} 
/*
 * Types
 */
enum active {on = 2, off = 1}; // so stupid


/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
int clicked = 0;

/*
 * Helper functions
 */

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}

static int b2sLUT[] = {
  0x40, //0
  0x79, //1
  0x24, //2
  0x30, //3
  0x19, //4
  0x12, //5
  0x02, //6
  0x78, //7
  0x00, //8
  0x18, //9
  0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
  int tmp = target_vel;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  //INT8U out_sign = 0;

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 | int2seven(0) << 14 | out_high << 7 | out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
  setLed(DE2_PIO_REDLED18_BASE,POSITION_CLEAR_FLAG,0); 
  switch(position){
    case 2000 ... 2400:
     setLed(DE2_PIO_REDLED18_BASE,POSITION_CLEAR_FLAG,1); 
    break;
    case 1600 ... 1999: 
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x01<<12)),1); 
    break;
    case 1200 ... 1599: 
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x03<<12)),1); 
    break;
    case 800 ... 1199: 
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x07<<12)),1); 
     break;
    case 400 ... 799:  
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x0F<<12)),1); 
    break; 
    case 0  ... 399:    
      setLed(DE2_PIO_REDLED18_BASE,(POSITION_CLEAR_FLAG & ~(0x1F<<12)),1); 
    break; 
  }
}

/*
set leds
*/
void setLed(int base_addr,int mask, int val){
  int read_led_data = IORD_ALTERA_AVALON_PIO_DATA(base_addr);
  if(val)
    read_led_data |= mask;
  else
    read_led_data &= ~mask;
  IOWR_ALTERA_AVALON_PIO_DATA(base_addr,read_led_data);
}

/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */
void VehicleTask(void* pdata)
{ 
  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT16S acceleration;  
  INT16U position = 0; 
  INT16S velocity = 0; 
  enum active brake_pedal = off;
  enum active engine = on;

  printf("Vehicle task created!\n");

  while(1)
  {
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

    // OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD);

    // Use soft timer to implement periodic task
    OSSemPend(Vehicle_Sem, 0, &err);  

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
    msg = OSMboxPend(Mbox_Throttle, 1, &err); 
    if (err == OS_NO_ERR) 
      *throttle = *((INT8U*) msg);
    /* Same for the brake signal that bypass the control law */
    msg = OSMboxPend(Mbox_Brake, 1, &err); 
    if (err == OS_NO_ERR) 
      brake_pedal = *((enum active*) msg);
    /* Same for the engine signal that bypass the control law */
    msg = OSMboxPend(Mbox_Engine, 1, &err); 
    if (err == OS_NO_ERR) 
      engine = *((enum active*) msg);


    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80) *throttle = 80;

    // brakes + wind
    if (brake_pedal == off)
    {
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on)
        acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else 
      acceleration = - brake_factor*velocity;

    printf("Position: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);

    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;

    show_velocity_on_sevenseg((INT8S) velocity);
  }
}

/*
 * ButtonIO Task
 */

void ButtonIOTask(void* pdata)
{
  INT8U err;
  void* msg;

  enum active gas_pedal = off;
  enum active top_gear = off;
  enum active brake_pedal = off;
  enum active cruise_control = off;
  enum active prev_cruise_control = off;
  int btn_values = 0;
  printf("ButtonIO Task created! :)\n");

  while(1)
  {

    // Use soft timer to implement periodic task
    OSSemPend(ButtonIO_Sem, 0, &err);
    printf("buttons %d\n",btn_values);
    btn_values = buttons_pressed();
    //brake pedal
    if (btn_values & BRAKE_PEDAL_FLAG){
      brake_pedal = on;
      setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_4,1);
    }
    else {
      brake_pedal = off;
       setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_4,0);
    }
    //cruise control
    if (btn_values & CRUISE_CONTROL_FLAG){
      cruise_control = on;
      //setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_2,1);
    } else {
      cruise_control = off;
    }
    //gas_pedal
    if (btn_values & GAS_PEDAL_FLAG){
      gas_pedal = on;
      setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_6,1);
    }
    else {
      gas_pedal = off;
       setLed(DE2_PIO_GREENLED9_BASE,LED_GREEN_6,0);
    }
    // send messages to mailboxes
    if(cruise_control == on && prev_cruise_control == off){
      err = OSMboxPost(Mbox_Cruise_Control, (void *)&cruise_control);
      
      printf("TRYING TO TOGGLE CRUISE CONTROL\n");
    }
    prev_cruise_control = cruise_control;
    err = OSMboxPost(Mbox_Gas_Pedal, (void *)&gas_pedal);
    err = OSMboxPost(Mbox_Brake, (void *)&brake_pedal);
    err = OSMboxPost(Mbox_BrakeControl, (void *)&brake_pedal);
    printf("Buttons %d\n",buttons_pressed());

  }
}

/*
 * Switch Task
 */

void SwitchTask(void* pdata)
{
  INT8U err;
  void* msg;

  enum active engine = off;
  enum active top_gear = off;
  int leds=0;
  //4.5 
  int util_input = 0; 
  printf("Switch Task created!\n");

  while(1)
  {
    // Use soft timer to implement periodic task
    OSSemPend(SwitchIO_Sem, 0, &err);  

    // Read switches
    int switch_values = switches_pressed();
    printf("switches %d\n",switch_values);
    // Check if engine is one
    if ((switch_values & ENGINE_FLAG)){
      engine = on;
      //setLed(DE2_PIO_REDLED18_BASE,LED_RED_0,1);
    }
    else{
      engine = off;
      //setLed(DE2_PIO_REDLED18_BASE,LED_RED_0,0);
    }
 
    // Check if gear is high
    if (switch_values & TOP_GEAR_FLAG){
      top_gear = on;
          setLed(DE2_PIO_REDLED18_BASE,LED_RED_1,1);
    }
    else{ 
      top_gear = off;
          setLed(DE2_PIO_REDLED18_BASE,LED_RED_1,0);

    }
    // for 4.5
    if (switch_values & EXTRA_INPUT_FLAG)
    {
      util_input = (switch_values & EXTRA_INPUT_FLAG) >> 4; // shift back to get value 
    }
    // send messages to mailboxes
    err = OSMboxPost(Mbox_EngineControl, (void *)&engine);
    err = OSMboxPost(Mbox_Gear, (void *)&top_gear);
    // for 4.5
    err = OSMboxPost(Mbox_Util, (void *)&util_input);

  }
}

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 0; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S current_velocity;
  float error;
  float previous_error;
  float d_error;
  INT16S target_velocity;
  INT16U position = 0; 
  enum active gas_pedal = off;
  enum active brake_pedal = off;
  enum active top_gear = off;
  enum active cruise_control = off; 

  printf("Control Task created!\n");


  float kp = 8;
  float kd = 0.01;
  while(1)
  {
    printf("cruise control %d\n", cruise_control);

    //Read gas pedal value
    msg = OSMboxPend(Mbox_Gas_Pedal, 0, &err);
    gas_pedal = *((enum active*)msg);
    //Read brake value
    msg = OSMboxPend(Mbox_BrakeControl, 0, &err);
    brake_pedal = *((enum active*)msg);
    //Read gear value
    msg = OSMboxPend(Mbox_Gear, 0, &err);
    top_gear = *((enum active*)msg);
    //Read current velocity
    msg = OSMboxPend(Mbox_Velocity, 0, &err);
    current_velocity = *((INT16S*)msg);
        //Read from switch to send to engine
    msg = OSMboxPend(Mbox_EngineControl, 1, &err);
    if(err == OS_NO_ERR){
      if(current_velocity == 0){
        OSMboxPost(Mbox_Engine,msg);
        if(*((enum active*)msg) == on)
          setLed(DE2_PIO_REDLED18_BASE,LED_RED_0,1);
        else if(*((enum active*)msg) == off)
          setLed(DE2_PIO_REDLED18_BASE,LED_RED_0,0);
      }

    }
    //Save target velocity while cruise button is held (if activating cruise)
    msg = OSMboxPend(Mbox_Cruise_Control, 1, &err);

    if (err == OS_NO_ERR){
      printf("HIIIIIIIIIIII\n");
      //Toggle the cruise control value
      if(cruise_control == on){
        cruise_control = off;
        target_velocity = 0;
        //throttle = 0;
        //Turn off cruise control LED
        setLed(DE2_PIO_GREENLED9_BASE, LED_GREEN_2, 0);
      }
      else if (current_velocity >= 20 && top_gear == on){
        target_velocity = current_velocity;
        cruise_control = on;
        //Turn on cruise control LED
        setLed(DE2_PIO_GREENLED9_BASE, LED_GREEN_2, 1);
      }
      clicked = 0;
    }
    
    if(gas_pedal == on)
      throttle = 80;
    else if (cruise_control == off)
      throttle = 0;

    if(cruise_control == on){
      previous_error = error;
      error = (float)current_velocity - target_velocity;
      d_error = ((float)error - previous_error) / 0.3;
      throttle += error*kp + kd*d_error;
    }

    //Remove cruise control if gear is changed to low
    if(top_gear == off || brake_pedal == on){
      cruise_control = off;
      target_velocity = 0;
      setLed(DE2_PIO_GREENLED9_BASE, LED_GREEN_2, 0);
    }
    // current_velocity = (INT16S*) msg;

    // msg = OSMboxPend(Mbox_Gear, 0, &err);
    // top_gear = (enum active*) msg;
    if(position>2400)
      position = 0;
    else
      position = position + current_velocity * VEHICLE_PERIOD / 1000;
    printf("position: %d\n",position );
    show_position(position);
    show_target_velocity(target_velocity);
    printf("TOP GEAR - CONTROL %d\n",top_gear);

    // Here you can use whatever technique or algorithm that you prefer to control
    // the velocity via the throttle. There are no right and wrong answer to this controller, so
    // be free to use anything that is able to maintain the cruise working properly. You are also
    // allowed to store more than one sample of the velocity. For instance, you could define
    //
    // INT16S previous_vel;
    // INT16S pre_previous_vel;
    // ...
    //
    // If your control algorithm/technique needs them in order to function. 

    err = OSMboxPost(Mbox_Throttle, (void *) &throttle);

    // OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);

    // Use softtimer combined with semaphore to create periodic task
    OSSemPend(Control_Sem, 0, &err); 
  }
}
  ////////////////4.5
  // watchdog task - prints overload error if not signaled ok at eof hyperperiod
void WatchdogTask(void* pdata)
{
  printf("WatchdogTask created\n");
  INT8U signal = 0;   // enum did not run = 0 , did run = 1
  INT8U err;
  while(1)
  {// sem initialized as 0; run only at end of hyperperiod
    OSSemPend(Watchdog_Sem,0,&err);
    signal = *((int*)OSMboxPend(Mbox_Watchdog_OK_Signal,1,&err)); // get message
    if(err == OS_NO_ERR){// there was a message --> overload task could run
      printf("NO OVERLOAD, Happy Dog\n");
    }else{ 
    // NON-blocking read, nothing in mailbox --> overload task couldnt run --> overload
      printf("OVERLOAD, Bark ,Bark!\n");
    }
  }
}
  // Overload task - tries tu run in one hyperperiod and signals ok whenever it has run
void OverloadTask(void* pdata)
{
  printf("OverloadTask created\n");
  int ok = 1; 
  INT8U err; 
  while(1)
  {
    OSSemPend(Overload_Sem,0,&err);
  // send ok signal, value doesnt matter, as long as something is sent
    INT8U err = OSMboxPost(Mbox_Watchdog_OK_Signal,(void*)&ok); 
  // necessary to reset mailbox in case it posts later?
  // what if the belated job is scheduled in next period? => false ok signal?
  // ahh it shouldnt be able to run at all since it's fucked/stalled in every hyper period
  }
}
  //  Extra task - a dick task that consumes utilization
  // reads sw 4 to 9 (6 bit value) from SwitchIO task, which sets the dummy loop
  // How to set it? Well ehh let computation time be a ratio of the hyperperiod
  //the 6 bit value sets a timer that runs a perecentage of the hyperperiod e.g.
// +1 in input should be 2% of hyperperiod so... 0 =2%, 1=4% , 2 = 6% ...64/2=32 levels
// max 100% => >=50 
void ExtraTask(void* pdata)
{
  // might have to put timer for
  // average execution time for function and calibrate like in ada lab 1
  printf("ExtraTask created\n");
  int util_input = 0;
  int delay = 0; 
  INT8U err,err2; 
  OS_TMR *Oneshot_Delay_Timer; // used to generate util delay for extra task
  while(1)
  {
    OSSemPend(Extra_Sem,0,&err);
    //mbox for input from swithio task
    util_input = *((int*)OSMboxPend(Mbox_Util,0,&err));// data shifted to lowest bits in switchtask
    util_input = (util_input > 49) ? 49 : util_input; // 100% util greater than input 50; -1 for logic
    // 0.02<-> 1/50 , 2% of hyperperiod * input
    // e.g. input = 20 (40% util), hyper 300, 2% of 300 = 6 => 6*20 = 120 ms , 120/300 = 40%
    delay = MS_IN_TICKS((util_input+1) * (HYPERPERIOD/50));// +1=>cant have delay of 0; if 0, 2%, if 1, 4% etc
    //oneshot timer to generate util delay
    //needs to be local so that timer doesn't run while stalled,hopefully that's how it works
    // 0 ,0 -> no callback and no callback argument passed
    Oneshot_Delay_Timer = OSTmrCreate(delay,0,OS_TMR_OPT_ONE_SHOT,(void*)0,(void*)0,"One-shot Timer",&err2);
    OSTmrStart(Oneshot_Delay_Timer,&err); // start one-shot
      //  os RemainGet returns uint32 of time left; 0 when it has reached its period/deadline
    while(OSTmrRemainGet(Oneshot_Delay_Timer,(void*)&err)!=0);// while there's still time left
    OSTmrStop(Oneshot_Delay_Timer,OS_TMR_OPT_NONE,(void*)0,&err); // stop after while
  }
}
///////////
/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,
        context) < 0)
  {
    printf("No system clock available!n");
  }

  /* 
   * Create and start Software Timer 
   */

  Vehicle_Sem = OSSemCreate(1);
  Control_Sem = OSSemCreate(1);
  ButtonIO_Sem = OSSemCreate(1);
  SwitchIO_Sem = OSSemCreate(1);
  //for 4.5
  Watchdog_Sem = OSSemCreate(0); // should run at hyperperiod
  Overload_Sem = OSSemCreate(1);
  Extra_Sem = OSSemCreate(1);


    Vehicle_Timer = OSTmrCreate(0,
      6,
      OS_TMR_OPT_PERIODIC,
      Vehicle_Timer_Callback,
      (void *)0,
      "Vehicle Task Timer",
      &err);

    Control_Timer = OSTmrCreate(0,
      6,
      OS_TMR_OPT_PERIODIC,
      Control_Timer_Callback,
      (void *)0,
      "Control Task Timer",
      &err);

    ButtonIO_Timer = OSTmrCreate(0,
      2,
      OS_TMR_OPT_PERIODIC,
      ButtonIO_Timer_Callback,
      (void *)0,
      "ButtonIO Task Timer",
      &err);

    SwitchIO_Timer = OSTmrCreate(0,
      2,
      OS_TMR_OPT_PERIODIC,
      SwitchIO_Timer_Callback,
      (void *)0,
      "Switch Task Timer",
      &err);

     //for 4.5 
     Watchdog_Timer = OSTmrCreate(0,
      MS_IN_TICKS(HYPERPERIOD),
      OS_TMR_OPT_PERIODIC,
      Watchdog_Timer_Callback,
      (void *)0,
      "Dog Task Timer",
      &err);

     Overload_Timer = OSTmrCreate(0,
      MS_IN_TICKS(HYPERPERIOD),
      OS_TMR_OPT_PERIODIC,
      Overload_Timer_Callback,
      (void *)0,
      "Overload Task Timer",
      &err);

     Extra_Timer = OSTmrCreate(0,
       MS_IN_TICKS(EXTRA_PERIOD),
      OS_TMR_OPT_PERIODIC,
      Extra_Timer_Callback,
      (void *)0,
      "Extra Task Timer",
      &err);
    
      // Start the timers
      OSTmrStart(Vehicle_Timer,&err);
      OSTmrStart(Control_Timer,&err);
      OSTmrStart(ButtonIO_Timer,&err);
      OSTmrStart(SwitchIO_Timer,&err);
      // for 4.5
      OSTmrStart(Watchdog_Timer,&err);
      OSTmrStart(Overload_Timer,&err);
      OSTmrStart(Extra_Timer,&err);

      printf("error %d\n",err);

  /*
   * Creation of Kernel Objects
   */

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) NULL); /* Empty Mailbox - Velocity */
  Mbox_BrakeControl = OSMboxCreate((void*) NULL); /*Empty Mailbox - Brake*/
  Mbox_Engine = OSMboxCreate((void*) 1); /* Empty Mailbox - Engine */
  Mbox_Gear = OSMboxCreate((void*) 1); /* Empty Mailbox - Gear */
  Mbox_Gas_Pedal = OSMboxCreate((void*) 1); /* Empty Mailbox - Gear */
  Mbox_Cruise_Control = OSMboxCreate((void*) 1); /* Empty Mailbox - Gear */
  // 4.5
  Mbox_Util = OSMboxCreate((void*) 1); // to send switch/ overload data  
  Mbox_Watchdog_OK_Signal = OSMboxCreate((void*) 1); // to send switch/ overload data  
  Mbox_EngineControl = OSMboxCreate((void*)1); 
  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      ButtonIOTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ButtonIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      BUTTTONIO_TASK_PRIO,
      BUTTTONIO_TASK_PRIO,
      (void *)&ButtonIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

    err = OSTaskCreateExt(
      SwitchTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &SwitchTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      SWITCHTASK_PRIO,
      SWITCHTASK_PRIO,
      (void *)&SwitchTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

/*
////////tasks for 4.5
     err = OSTaskCreateExt(
      WatchdogTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &WatchdogTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      WATCHDOGTASK_PRIO,
      WATCHDOGTASK_PRIO,
      (void *)&WatchdogTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

      err = OSTaskCreateExt(
      OverloadTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &OverloadTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      OVERLOADTASK_PRIO,
      OVERLOADTASK_PRIO,
      (void *)&OverloadTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

       err = OSTaskCreateExt(
      ExtraTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ExtraTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      EXTRATASK_PRIO,
      EXTRATASK_PRIO,
      (void *)&ExtraTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);
*/

  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack 
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,  
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  OSStart();

  return 0;
}
