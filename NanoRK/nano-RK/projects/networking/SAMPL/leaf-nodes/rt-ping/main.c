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
#include <bmac.h>
#include <nrk_error.h>
#include <rt_ping.h>
#include <sampl.h>
#include <pkt_packer.h>

#define REPLY_WAIT_SECS	3


// Full address = 0x01000064
#define MY_SUBNET_MAC_2		1
#define MY_SUBNET_MAC_1		0
#define MY_SUBNET_MAC_0		0
#define MY_MAC			100

#define TXT_DEBUG


nrk_task_type MOBILE_TASK;
NRK_STK mobile_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();


uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];



SAMPL_PEER_2_PEER_PKT_T p2p_pkt;

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

  bmac_task_config ();

  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

void tx_task ()
{
  uint8_t j, i, cnt, error;
  int8_t len;
  int8_t rssi, val;
  uint8_t *local_rx_buf;

  nrk_sig_t tx_done_signal;
  nrk_sig_t rx_signal;
  nrk_sig_mask_t ret;
  nrk_time_t check_period;
  nrk_time_t timeout, start, current;
  nrk_sig_mask_t my_sigs;

  printf ("tx_task PID=%d\r\n", nrk_get_pid ());


  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  bmac_init (26);

  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);


  val =
    bmac_addr_decode_set_my_mac (((uint16_t) MY_SUBNET_MAC_0 << 8) | MY_MAC );
  val = bmac_addr_decode_dest_mac (0xffff);     // broadcast by default
  bmac_addr_decode_enable ();
  bmac_auto_ack_enable();

  nrk_kprintf (PSTR ("bmac_started()\r\n"));
  bmac_set_cca_thresh (-45);


  check_period.secs = 0;
  check_period.nano_secs = 100 * NANOS_PER_MS;
  val = bmac_set_rx_check_rate (check_period);

  // Get and register the tx_done_signal if you want to
  // do non-blocking transmits
  tx_done_signal = bmac_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  rx_signal = bmac_get_rx_pkt_signal ();
  nrk_signal_register (rx_signal);

  cnt = 0;

  while (1) {


    // Build a TX packet by hand...
    p2p_pkt.pkt_type = RT_PING_PKT;
    p2p_pkt.ctrl_flags = LINK_ACK | MOBILE_MASK;   // | DEBUG_FLAG ;  
    p2p_pkt.ack_retry = 0x0f;
    p2p_pkt.ttl = 1;
    p2p_pkt.src_subnet_mac[0] = MY_SUBNET_MAC_0;
    p2p_pkt.src_subnet_mac[1] = MY_SUBNET_MAC_1;
    p2p_pkt.src_subnet_mac[2] = MY_SUBNET_MAC_2;
    p2p_pkt.src_mac = MY_MAC;
    p2p_pkt.last_hop_mac = MY_MAC;
    p2p_pkt.dst_subnet_mac[0] = BROADCAST;
    p2p_pkt.dst_subnet_mac[1] = BROADCAST;
    p2p_pkt.dst_subnet_mac[2] = BROADCAST;
    p2p_pkt.dst_mac = BROADCAST;
    p2p_pkt.buf = tx_buf;
    p2p_pkt.buf_len = P2P_PAYLOAD_START;
    p2p_pkt.seq_num = cnt;
    p2p_pkt.priority = 0;
    p2p_pkt.check_rate = FAST_CHECK_RATE;
    cnt++;



    nrk_led_set (BLUE_LED);

    check_period.secs = 0;
    check_period.nano_secs = 100 * NANOS_PER_MS;
    val = bmac_set_rx_check_rate (check_period);

    // Pack data structure values in buffer before transmit
    pack_peer_2_peer_packet (&p2p_pkt);

    // For blocking transmits, use the following function call.
    val = bmac_tx_pkt (p2p_pkt.buf, p2p_pkt.buf_len);

    check_period.secs = 0;
    check_period.nano_secs = FAST_CHECK_RATE * NANOS_PER_MS;
    val = bmac_set_rx_check_rate (check_period);
#ifdef TXT_DEBUG
    nrk_kprintf (PSTR ("\r\nSent Request:\r\n"));
#endif
    nrk_led_clr (BLUE_LED);
    nrk_led_clr (GREEN_LED);

    // Wait for packets or timeout
    nrk_time_get (&start);
    while (1) {

      timeout.secs = REPLY_WAIT_SECS;
      timeout.nano_secs = 0;

      // Wait until an RX packet is received
      //val = bmac_wait_until_rx_pkt ();
      if(bmac_rx_pkt_ready()==0)
      {
        nrk_set_next_wakeup (timeout);
        my_sigs = nrk_event_wait (SIG (rx_signal) | SIG (nrk_wakeup_signal));
      }


      if (my_sigs == 0)
        nrk_kprintf (PSTR ("Error calling nrk_event_wait()\r\n"));
      if (my_sigs & SIG (rx_signal)) {

        // Get the RX packet 
        local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
        // Check the packet type from raw buffer before unpacking
        if ((local_rx_buf[CTRL_FLAGS] & (DS_MASK | US_MASK)) == 0) {

          // Set the buffer
          p2p_pkt.buf = local_rx_buf;
          p2p_pkt.buf_len = len;
          p2p_pkt.rssi = rssi;
          unpack_peer_2_peer_packet (&p2p_pkt);
#ifdef TXT_DEBUG
          if ((p2p_pkt.dst_subnet_mac[2] == MY_SUBNET_MAC_2 &&
		p2p_pkt.dst_subnet_mac[1] == MY_SUBNET_MAC_1 &&
		p2p_pkt.dst_subnet_mac[0] == MY_SUBNET_MAC_0 &&
		p2p_pkt.dst_mac == MY_MAC ) 
		|| p2p_pkt.dst_mac == BROADCAST) 
	    {
            nrk_led_set (GREEN_LED);
            // Packet arrived and is good to go
            printf ("full mac: %u %u %u %u ", p2p_pkt.src_subnet_mac[0],
                    p2p_pkt.src_subnet_mac[1], p2p_pkt.src_subnet_mac[2],
		    p2p_pkt.src_mac);
            printf ("rx-rssi: %d ", p2p_pkt.rssi);
	    if(p2p_pkt.pkt_type==RT_PING_PKT )
		{
            	  nrk_kprintf (PSTR("RT-PING reply: "));
            	  printf ("  mac: %u ", p2p_pkt.payload[0]);
            	  printf ("  tx-rssi: %d", (int8_t)p2p_pkt.payload[1]);
		}
	   
            nrk_kprintf (PSTR("\r\n"));
          }
#endif

        }
      }

      nrk_time_get (&current);
      if (start.secs + REPLY_WAIT_SECS < current.secs)
        break;
        // Release the RX buffer so future packets can arrive 
        bmac_rx_pkt_release ();
    }
    nrk_kprintf (PSTR ("Done Waiting for response...\r\n"));
    nrk_wait_until_next_period ();
  }

}

void nrk_create_taskset ()
{

  MOBILE_TASK.task = tx_task;
  nrk_task_set_stk (&MOBILE_TASK, mobile_task_stack, NRK_APP_STACKSIZE);
  MOBILE_TASK.prio = 2;
  MOBILE_TASK.FirstActivation = TRUE;
  MOBILE_TASK.Type = BASIC_TASK;
  MOBILE_TASK.SchType = PREEMPTIVE;
  MOBILE_TASK.period.secs = 10;
  MOBILE_TASK.period.nano_secs = 0;
  MOBILE_TASK.cpu_reserve.secs = 1;
  MOBILE_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  MOBILE_TASK.offset.secs = 0;
  MOBILE_TASK.offset.nano_secs = 0;
  nrk_activate_task (&MOBILE_TASK);



}
