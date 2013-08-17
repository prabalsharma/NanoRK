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
#include <avr/sleep.h>
#include <hal.h>
#include <pcf_tdma.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <power_driver.h>
#include <nrk_eeprom.h>
#include <pkt.h>

// if SET_MAC is 0, then read MAC from EEPROM
// otherwise use the coded value
#define SET_MAC  0x0001

PKT_T	tx_pkt;
PKT_T	rx_pkt;

tdma_info tx_tdma_fd;
tdma_info rx_tdma_fd;

uint8_t rx_buf[TDMA_MAX_PKT_SIZE];
uint8_t tx_buf[TDMA_MAX_PKT_SIZE];

uint32_t mac_address;


nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);


nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();


int main ()
{
  uint16_t div;
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  nrk_time_set (0, 0);

  tdma_task_config();

  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

void rx_task ()
{
  nrk_time_t t;
  uint16_t cnt;
  int8_t v;
  uint8_t len, i;
uint8_t chan;


  cnt = 0;
  nrk_kprintf (PSTR ("Nano-RK Version "));
  printf ("%d\r\n", NRK_VERSION);


  printf ("RX Task PID=%u\r\n", nrk_get_pid ());
  t.secs = 5;
  t.nano_secs = 0;

  chan = 25;
  if (SET_MAC == 0x00) {

    v = read_eeprom_mac_address (&mac_address);
    if (v == NRK_OK) {
      v = read_eeprom_channel (&chan);
    }
    else {
      while (1) {
        nrk_kprintf (PSTR
                     ("* ERROR reading MAC address, run eeprom-set utility\r\n"));
        nrk_wait_until_next_period ();
      }
    }
  }
  else
    mac_address = SET_MAC;

  printf ("MAC ADDR: %x\r\n", mac_address & 0xffff);
  printf ("chan = %d\r\n", chan);



  tdma_init (TDMA_CLIENT, chan, (mac_address));


  while (!tdma_started ())
    nrk_wait_until_next_period ();

  v = tdma_tx_slot_add (mac_address&0xff);

  if (v != NRK_OK)
    nrk_kprintf (PSTR ("Could not add slot!\r\n"));

  while (1) {
    // Update watchdog timer
    // nrk_sw_wdt_update(0);
    v = tdma_recv (&rx_tdma_fd, &rx_buf, &len, TDMA_BLOCKING);
    if (v == NRK_OK) {
     // printf ("src: %u\r\nrssi: %d\r\n", rx_tdma_fd.src, rx_tdma_fd.rssi);
     // printf ("slot: %u\r\n", rx_tdma_fd.slot);
     // printf ("cycle len: %u\r\n", rx_tdma_fd.cycle_size);
      v=buf_to_pkt(&rx_buf, &rx_pkt);
      if((rx_pkt.dst_mac&0xff) == (mac_address&0xff)) 
      {
	// payload 1: Key
	if(rx_pkt.payload[1]==2)
	{
	// payload 2: Outlet Number 
	// payload 3: On/Off
	if(rx_pkt.payload[3]==0) {
		power_socket_disable(rx_pkt.payload[2]);	
		printf( "Disable %d\r\n", rx_pkt.payload[2] );
	}
	if(rx_pkt.payload[3]==1) {
		power_socket_enable(rx_pkt.payload[2]);	
		printf( "Enable %d\r\n", rx_pkt.payload[2] );
	}


	}

      }

      /*      printf ("len: %u\r\npayload: ", len);
      for (i = 0; i < len; i++)
        printf ("%d ", rx_buf[i]);
      printf ("\r\n");

      if(rx_buf[0]==(mac_address&0xff))
      {
	if(rx_buf[2]==0) {
		power_socket_disable(rx_buf[1]);	
		printf( "Disable %d\r\n", rx_buf[1] );
	}
	if(rx_buf[2]==1) {
		power_socket_enable(rx_buf[1]);	
		printf( "Enable %d\r\n", rx_buf[1] );
	}
      }
      */
    }

    //  nrk_wait_until_next_period();
  }

}

uint8_t ctr_cnt[4];

void tx_task ()
{
  uint8_t j, i, val, cnt;
  int8_t len;
  int8_t v;
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;

  //power_socket_disable(0);
  //power_socket_disable(1);
  printf ("tx_task PID=%d\r\n", nrk_get_pid ());

  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that

  while (!tdma_started ())
    nrk_wait_until_next_period ();


  power_init ();

  /*
  for (i = 0; i < 3; i++) {
    power_socket_enable (0);
    power_socket_enable (1);
    nrk_wait_until_next_period ();
    power_socket_disable (0);
    power_socket_disable (1);
    nrk_wait_until_next_period ();
  }
*/
  power_socket_enable (0);
  power_socket_enable (1);

  //nrk_kprintf( PSTR("after outlet on\r\n"));

  // Sample of using Reservations on TX packets
  // This example allows 2 packets to be sent every 5 seconds
  // r_period.secs=5;
  // r_period.nano_secs=0;
  // v=bmac_tx_reserve_set( &r_period, 2 );
  // if(v==NRK_ERROR) nrk_kprintf( PSTR("Error setting b-mac tx reservation (is NRK_MAX_RESERVES defined?)\r\n" ));



  while (1) {


    // For blocking transmits, use the following function call.
    // For this there is no need to register  
    tx_pkt.payload[0] = 1;  // ELEMENTS
    tx_pkt.payload[1] = 1;  // Key
    tx_pkt.payload[2] = (total_secs >> 24) & 0xff;
    tx_pkt.payload[3] = (total_secs >> 16) & 0xff;
    tx_pkt.payload[4] = (total_secs >> 8) & 0xff;
    tx_pkt.payload[5] = (total_secs) & 0xff;
    tx_pkt.payload[6] = freq & 0xff;
    tx_pkt.payload[7] = (rms_voltage >> 8) & 0xff;
    tx_pkt.payload[8] = rms_voltage & 0xff;
    tx_pkt.payload[9] = (rms_current >> 8) & 0xff;
    tx_pkt.payload[10] = rms_current & 0xff;
    tx_pkt.payload[11] = (true_power >> 16) & 0xff;
    tx_pkt.payload[12] = (true_power >> 8) & 0xff;
    tx_pkt.payload[13] = (true_power) & 0xff;
    tx_pkt.payload[14] = tmp_energy.byte[0];
    tx_pkt.payload[15] = tmp_energy.byte[1];
    tx_pkt.payload[16] = tmp_energy.byte[2];
    tx_pkt.payload[17] = tmp_energy.byte[3];
    tx_pkt.payload[18] = tmp_energy.byte[4];
    tx_pkt.payload[19] = tmp_energy.byte[5];
    tx_pkt.payload[20] = socket_0_active;
    tx_pkt.payload[21] = (rms_current2 >> 8) & 0xff;
    tx_pkt.payload[22] = rms_current2 & 0xff;
    tx_pkt.payload[23] = (true_power2 >> 16) & 0xff;
    tx_pkt.payload[24] = (true_power2 >> 8) & 0xff;
    tx_pkt.payload[25] = (true_power2) & 0xff;
    tx_pkt.payload[26] = tmp_energy2.byte[0];
    tx_pkt.payload[27] = tmp_energy2.byte[1];
    tx_pkt.payload[28] = tmp_energy2.byte[2];
    tx_pkt.payload[29] = tmp_energy2.byte[3];
    tx_pkt.payload[30] = tmp_energy2.byte[4];
    tx_pkt.payload[31] = tmp_energy2.byte[5];
    tx_pkt.payload[32] = socket_1_active;
    tx_pkt.payload_len=33;

    tx_pkt.src_mac=mac_address;
    tx_pkt.dst_mac=0;
    tx_pkt.type=APP;

    len=pkt_to_buf(&tx_pkt,&tx_buf );
    if(len>0)
    {
    v = tdma_send (&tx_tdma_fd, &tx_buf, len, TDMA_BLOCKING);
    if (v == NRK_OK) {
      //nrk_kprintf (PSTR ("App Packet Sent\n"));
    }
    } else nrk_wait_until_next_period();

  }

}

void nrk_create_taskset ()
{


  RX_TASK.task = rx_task;
  nrk_task_set_stk (&RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 2;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 1;
  RX_TASK.period.nano_secs = 0;
  RX_TASK.cpu_reserve.secs = 0;
  RX_TASK.cpu_reserve.nano_secs = 0;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk (&TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 2;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 1;
  TX_TASK.period.nano_secs = 0;
  TX_TASK.cpu_reserve.secs = 0;
  TX_TASK.cpu_reserve.nano_secs = 0;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);


  nrk_task_set_entry_function( &event_detector, event_detector_task);
  nrk_task_set_stk( &event_detector, event_detector_stack, EVENT_DETECTOR_STACKSIZE);
  event_detector.prio = 1;
  event_detector.FirstActivation = TRUE;
  event_detector.Type = BASIC_TASK;
  event_detector.SchType = PREEMPTIVE;
  event_detector.period.secs = 1;
  event_detector.period.nano_secs = 0*NANOS_PER_MS;
  event_detector.cpu_reserve.secs = 0;
  event_detector.cpu_reserve.nano_secs = 500*NANOS_PER_MS;
  event_detector.offset.secs = 0;
  event_detector.offset.nano_secs= 0;
  nrk_activate_task (&event_detector);


}
