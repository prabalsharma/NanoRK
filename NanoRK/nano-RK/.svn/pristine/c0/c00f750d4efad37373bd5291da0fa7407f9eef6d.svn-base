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
 *  Contributing Authors (specific to this file):
 *  Anthony Rowe
 *******************************************************************************/
#ifndef _PCF_TDMA_H
#define _PCF_TDMA_H
#include <include.h>
#include <basic_rf.h>
#include <nrk.h>

/************************************************************************
 Describe protocol here.

 ************************************************************************/

#define TDMA_MAX_PKT_SIZE		116

#ifndef TDMA_TASK_PRIORITY
#define TDMA_TASK_PRIORITY		20
#endif

#define TDMA_HOST	1
#define TDMA_CLIENT	2

#define TDMA_BLOCKING		0
#define TDMA_NONBLOCKING	1

#define TDMA_DEFAULT_SLOT_MS	1
#define TDMA_DEFAULT_SLOTS_PER_CYCLE	1024
#define TDMA_DEFAULT_CONTENSION_SLOTS	0
#define TDMA_PCF_HEADER			23
#define TDMA_MAX_TX_SLOTS		4
#define TDMA_MAX_FWD_CLIENTS	10

#define TDMA_SLOT_LOW			0
#define TDMA_SLOT_HIGH			1
#define TDMA_DST_LOW			2
#define TDMA_DST_HIGH			3
#define TDMA_SRC_LOW			4
#define TDMA_SRC_HIGH			5
#define TDMA_SEQ_NUM_LOW		6
#define TDMA_SEQ_NUM_HIGH		7
#define TDMA_CYCLE_SIZE_LOW		8
#define TDMA_CYCLE_SIZE_HIGH	9
#define TDMA_SLOT_SIZE			10
#define TDMA_CONTENTION_SLOTS 	11
#define TDMA_SRC_LEVEL			12
#define TDMA_SRC_CLIENTS		13
#define TDMA_PKT_TYPE			14
#define TDMA_SRC_TIMESTAMP_0	15
#define TDMA_SRC_TIMESTAMP_1	16
#define TDMA_SRC_TIMESTAMP_2	17
#define TDMA_SRC_TIMESTAMP_3	18
#define TDMA_SRC_TIMESTAMP_4	19
#define TDMA_SRC_TIMESTAMP_5	20
#define TDMA_SRC_TIMESTAMP_6	21
#define TDMA_SRC_TIMESTAMP_7	22

#define TDMA_EVENT_TYPE_SYNC		1
#define TDMA_EVENT_TYPE_TX			2
#define TDMA_EVENT_TYPE_FWD_RX		3
#define TDMA_EVENT_TYPE_FWD_TX		4
#define TDMA_EVENT_TYPE_CONTENTION 	5

#define TDMA_HOST_TIMEOUT	3 //Number of cycles to wait after hearing traffic from non-host node to accept them as pseudo-host
typedef struct {
	uint16_t slot;
	uint16_t cycle_size;
	uint8_t num_contention_slots;
	uint8_t slot_len_ms;
	uint16_t dst;
	uint16_t src;
	uint16_t seq_num;
	int8_t ack_req;
	int16_t rssi;
	uint8_t src_level;
	uint8_t clients;
	uint8_t pkt_type;
	uint64_t timestamp;

} tdma_info;

typedef struct {
	uint8_t type;
	uint16_t slot;
} tdma_schedule_event;

typedef struct {
	int16_t rssi;
	uint8_t level;
	uint8_t clients;
	uint16_t slot;
} tdma_sync_node;

int8_t tdma_tx_reserve_set(nrk_time_t * period, uint16_t pkts);
uint16_t tdma_tx_reserve_get();

nrk_sig_t tdma_rx_pkt_signal;
nrk_sig_t tdma_tx_pkt_done_signal;
nrk_sig_t tdma_enable_signal;

RF_RX_INFO tdma_rfRxInfo;
RF_TX_INFO tdma_rfTxInfo;

int8_t tdma_set_error_callback(void(*fp)(void));
uint8_t tdma_sync_ok();
void tdma_disable();

void tdma_task_config();
int8_t tdma_set_channel(uint8_t chan);
int8_t tdma_send(tdma_info * fd, uint8_t * buf, uint8_t len, uint8_t flags);
int8_t tdma_recv(tdma_info * fd, uint8_t * buf, uint8_t * len, uint8_t flags);
int8_t tdma_rx_pkt_release(void);
int8_t tdma_rx_pkt_set_buffer(uint8_t * buf, uint8_t size);

//int8_t tdma_join();

int8_t tdma_tx_slot_add(uint16_t slot);
//int8_t tdma_rx_slot_add(uint16_t slot);
//int8_t tdma_rx_slot_del(uint16_t slot);
//int8_t tdma_tx_slot_del(uint16_t slot);

nrk_sig_t tdma_get_tx_done_signal();
nrk_sig_t tdma_get_rx_pkt_signal();

int8_t tdma_set_slot_len_ms(uint16_t len);
int8_t tdma_set_slots_per_cycle(uint16_t slots_per_cycle);
int8_t tdma_set_contention_slots(uint8_t cslots_per_cycle);

int8_t tdma_started();
int8_t tdma_init(uint8_t tdma_mode, uint8_t chan, uint16_t my_mac);

int8_t _tdma_rx();
int8_t _tdma_tx();

int8_t _tdma_client_process_event();

uint8_t _tdma_compare_sync_node(tdma_sync_node *node0, RF_RX_INFO *node1);
int8_t _tdma_update_sync_node(tdma_sync_node *node0, RF_RX_INFO *node1);

int8_t _tdma_schedule_sort();
int8_t _tdma_schedule_add_event(tdma_schedule_event *event,
		uint8_t *inserted_slot);
int8_t _tdma_schedule_remove_event(uint16_t slot);
void _tdma_rx_time_sync(void);

#endif
