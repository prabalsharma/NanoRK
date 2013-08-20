/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*******************************************************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>


NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);

NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskTwo;
void Task2 (void);

NRK_STK Stack3[NRK_APP_STACKSIZE];
nrk_task_type TaskThree;
void Task3 (void);


NRK_STK Stack4[NRK_APP_STACKSIZE];
nrk_task_type TaskFour;
void Task4 (void);

void nrk_create_taskset();

// Semaphore declaration
nrk_sem_t *semaphore1;
nrk_sem_t *semaphore2;
nrk_sem_t *semaphore3;

int
main ()
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);
  
  nrk_init();
  
  nrk_led_clr(ORANGE_LED);
  nrk_led_clr(BLUE_LED);
  nrk_led_clr(GREEN_LED);
  nrk_led_clr(RED_LED);
  
  nrk_time_set(0,0);
  nrk_create_taskset();
  
// Semaphore initialization
  semaphore1 = nrk_sem_create(1,3);  
  semaphore2 = nrk_sem_create(1,5);
  semaphore3 = nrk_sem_create(1,1);

  nrk_start();
  
  return 0;
}

//To create Task1 and uses the semaphore1
void Task1()
{
  uint16_t counter;
  counter=0;

  printf( "My node's address is %u\r\n",NODE_ADDR );  
  printf( "Task1 PID=%u\r\n",nrk_get_pid());

	 while(1) 
	 {
		nrk_led_toggle(ORANGE_LED);
		printf( "Task1 counter=%u\r\n",counter );
		nrk_wait_until_next_period();
// To halt at 50 and show the stats
  if(counter==50) 
    {
     nrk_stats_display_all();
     nrk_halt();
   }
		counter++;
	 }
}

// To create Task2
void Task2()
{
  int16_t counter;
  printf( "Task2 PID=%u\r\n",nrk_get_pid());
  counter=0;
  while(1) 
  {
    nrk_led_toggle(BLUE_LED);
    printf( "Task2 signed counter=%d\r\n",counter );
    nrk_stats_display_pid(nrk_get_pid());
    nrk_wait_until_next_period();
    counter--;
  }
}

// To create Task3 with the utilization of semaphore1
void Task3()
{
  uint8_t counter = 0;
  printf( "Task3 PID=%d\r\n",nrk_get_pid());
  uint8_t count = 0;
  uint8_t waitCount=0;
  
 while(1) 
  {
        nrk_led_toggle(GREEN_LED);
//        printf( "Task3 counter=%d\r\n",counter );
		if(0==waitCount)
		{
			printf("Task3 CBS_TASK counter %d \r\n",counter);
			counter++;
		}
	   	 waitCount++;
	   if(7==waitCount)
	   {
			waitCount=0;
		}
        nrk_wait_until_next_period();
        
    }

}

// To create Task4 with the utilization of semaphore1
void Task4()
{
  uint8_t counter;
  uint8_t waitCount=0;

  printf( "Task4 PID=%d\r\n",nrk_get_pid());
  counter=0;
   while(1) 
  {
        nrk_led_toggle(RED_LED);
//        printf( "Task4 counter=%d\r\n",counter );
		if(0==waitCount)
		{
			printf("Task4 CBS_TASK counter %d \r\n",counter);
			counter++;
		}
	   	 waitCount++;
	   if(3==waitCount)
	   {
			waitCount=0;
		}
        nrk_wait_until_next_period();
  
    }

}

// Creates tasks-set with
// Task 1 with BASIC_TASK type
// Task 3 and Task 4 with CBS_TASK type
void
nrk_create_taskset()
{
  nrk_task_set_entry_function( &TaskOne, Task1);
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 3;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 6;
  TaskOne.period.nano_secs = 0*NANOS_PER_MS;
  TaskOne.prelevel=1;
  TaskOne.cpu_reserve.secs = 1;
  TaskOne.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

   // nrk_task_set_entry_function( &TaskTwo, Task2); 
   // nrk_task_set_stk( &TaskTwo, Stack2, NRK_APP_STACKSIZE); 
   // TaskTwo.prio = 2; 
   // TaskTwo.FirstActivation = TRUE; 
   // TaskTwo.Type = BASIC_TASK; 
   // TaskTwo.SchType = PREEMPTIVE; 
   // TaskTwo.period.secs = 0; 
   // TaskTwo.period.nano_secs = 250*NANOS_PER_MS; 
   // TaskTwo.PreemptionLevel=1;
   // TaskTwo.cpu_reserve.secs = 0; 
   // TaskTwo.cpu_reserve.nano_secs = 100*NANOS_PER_MS; 
   // TaskTwo.offset.secs = 0;
   // TaskTwo.offset.nano_secs= 0; 
   // nrk_activate_task (&TaskTwo); 
  
  nrk_task_set_entry_function( &TaskThree, Task3); 
   nrk_task_set_stk( &TaskThree, Stack3, NRK_APP_STACKSIZE); 
   TaskThree.prio = 1; 
   TaskThree.FirstActivation = TRUE; 
   TaskThree.Type = CBS_TASK; 
   TaskThree.SchType = PREEMPTIVE; 
   TaskThree.period.secs = 1; 
   TaskThree.period.nano_secs = 0;
   TaskThree.prelevel=2;
   TaskThree.cpu_reserve.secs = 3; 
   TaskThree.cpu_reserve.nano_secs = 50*NANOS_PER_MS; 
   TaskThree.offset.secs = 0; 
   TaskThree.offset.nano_secs= 0; 
    nrk_activate_task (&TaskThree);
   
   nrk_activate_task (&TaskThree); 
   nrk_task_set_entry_function( &TaskFour, Task4); 
   nrk_task_set_stk( &TaskFour, Stack4, NRK_APP_STACKSIZE); 
   TaskFour.prio = 1; 
   TaskFour.FirstActivation = TRUE; 
   TaskFour.Type = CBS_TASK; 
   TaskFour.SchType = PREEMPTIVE; 
   TaskFour.period.secs = 1; 
   TaskFour.period.nano_secs = 0; 
   TaskFour.prelevel=2;
   TaskFour.cpu_reserve.secs = 3; 
   TaskFour.cpu_reserve.nano_secs = 50*NANOS_PER_MS; 
   TaskFour.offset.secs = 0; 
   TaskFour.offset.nano_secs= 0; 
   nrk_activate_task (&TaskFour); 
}

uint8_t kill_stack(uint8_t val)
{	
char bad_memory[10];
uint8_t i;
for(i=0; i<10; i++ ) bad_memory[i]=i;
for(i=0; i<10; i++ ) printf( "%d ", bad_memory[i]);
   printf( "Die Stack %d\r\n",val );
if(val>1) kill_stack(val-1);
return 0;
}
