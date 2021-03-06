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

//creates Task1 and uses semaphore1
void Task1()
{
  nrk_time_t t;
  uint16_t counter;
  counter=0;

  printf( "My node's address is %u\r\n",NODE_ADDR );  
  printf( "Task1 PID=%u\r\n",nrk_get_pid());
  t.secs=30;
  t.nano_secs=0;
	 while(1) 
	 {
		nrk_led_toggle(ORANGE_LED);
		printf( "Task1 counter=%u\r\n",counter );
		nrk_wait_until_next_period();
		counter++;
	 }
}

// creates Task2
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

//creates Task3 and uses semaphore1
void Task3()
{
  uint8_t counter = 0;
  printf( "Task3 PID=%d\r\n",nrk_get_pid());
  uint8_t waitCount = 7;
  
  while(1) 
  {
        nrk_led_toggle(GREEN_LED);
        printf( "Task3 counter=%d\r\n",counter );
		if(0==waitCount)
		{
			nrk_kprintf( PSTR("Task3 accessing semaphore1\r\n"));
			nrk_sem_pend(semaphore1);
			nrk_kprintf( PSTR("\r\n Task3 holding semaphore1 \r\n"));
		}
	   	 waitCount++;
	   if(7==waitCount)
	   {
			nrk_sem_post(semaphore1);
			nrk_kprintf( PSTR("Task3 released semaphore1\r\n"));
			waitCount=0;
		}
        nrk_wait_until_next_period();
        counter++;
    }

}

// creates Task4 and uses semaphore1
void Task4()
{
  uint8_t counter;
  uint8_t waitCount=3;

  printf( "Task4 PID=%d\r\n",nrk_get_pid());
  counter=0;
   while(1) 
  {
        nrk_led_toggle(GREEN_LED);
        printf( "Task4 counter=%d\r\n",counter );
		if(0==waitCount)
		{
			nrk_kprintf( PSTR("Task4 accessing semaphore1\r\n"));
			nrk_sem_pend(semaphore1);
			nrk_kprintf( PSTR("Task4 holding semaphore1\r\n"));
		}
	   	 waitCount++;
	   if(3==waitCount)
	   {
			nrk_sem_post(semaphore1);
			nrk_kprintf( PSTR("Task4 released semaphore1\r\n"));
			waitCount=0;
		}
        nrk_wait_until_next_period();
        counter++;
    }

}

void
nrk_create_taskset()
{
  nrk_task_set_entry_function( &TaskOne, Task1);
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 2;
  TaskOne.period.nano_secs = 500*NANOS_PER_MS;
  TaskOne.prelevel=1;
  TaskOne.cpu_reserve.secs = 1;
  TaskOne.cpu_reserve.nano_secs = 500*NANOS_PER_MS;
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
  
  //sets enrty function for task3
  nrk_task_set_entry_function( &TaskThree, Task3); 
   nrk_task_set_stk( &TaskThree, Stack3, NRK_APP_STACKSIZE); 
   TaskThree.prio = 3; 
   TaskThree.FirstActivation = TRUE; 
   TaskThree.Type = BASIC_TASK; 
   TaskThree.SchType = PREEMPTIVE; 
   TaskThree.period.secs = 1; 
   TaskThree.period.nano_secs = 100*NANOS_PER_MS;
   TaskThree.prelevel=2;
   TaskThree.cpu_reserve.secs = 0; 
   TaskThree.cpu_reserve.nano_secs = 100*NANOS_PER_MS; 
   TaskThree.offset.secs = 0; 
   TaskThree.offset.nano_secs= 0; 
   //activate task3
   nrk_activate_task (&TaskThree); 
   //sets entry function for task3
   nrk_task_set_entry_function( &TaskFour, Task4); 
   nrk_task_set_stk( &TaskFour, Stack4, NRK_APP_STACKSIZE); 
   TaskFour.prio = 4; 
   TaskFour.FirstActivation = TRUE; 
   TaskFour.Type = BASIC_TASK; 
   TaskFour.SchType = PREEMPTIVE; 
   TaskFour.period.secs = 1; 
   TaskFour.period.nano_secs = 0; 
   TaskFour.prelevel=4;
   TaskFour.cpu_reserve.secs = 0; 
   TaskFour.cpu_reserve.nano_secs = 100*NANOS_PER_MS; 
   TaskFour.offset.secs = 0; 
   TaskFour.offset.nano_secs= 0; 
   //  nrk_activate_task (&TaskFour); 
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
