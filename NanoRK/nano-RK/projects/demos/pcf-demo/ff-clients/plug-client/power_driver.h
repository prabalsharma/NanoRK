#ifndef _POWER_VARS_H_
#define _POWER_VARS_H_

#define VOLTAGE_LOW_THRESHOLD	300
#define CYCLE_UNKNOWN	0
#define CYCLE_LOW	1
#define CYCLE_HIGH	2

#define TRUE_POWER_ON_THRESH	400


#define EVENT_DETECTOR_STACKSIZE  256
NRK_STK event_detector_stack[EVENT_DETECTOR_STACKSIZE];
nrk_task_type event_detector;
void event_detector_task();
nrk_sig_t update_energy_sig;

void event_detector_task();
void power_sample ();
void power_init ();
int8_t power_socket_enable (uint8_t socket);
int8_t power_socket_disable (uint8_t socket);

uint16_t center_chan;
int16_t v,v_last;
int16_t c1,c2;
uint16_t l_v_p2p_low,l_v_p2p_high;
uint16_t  l_c_p2p_low, l_c_p2p_high;
uint16_t  l_c_p2p_low2, l_c_p2p_high2;
uint16_t v_p2p_low,v_p2p_high;
uint16_t c_p2p_low, c_p2p_high;
uint16_t c_p2p_low2, c_p2p_high2;
uint8_t socket_0_active;
uint8_t socket_1_active;
uint16_t c1_center;
uint16_t rms_current;
uint16_t rms_current2;
uint16_t rms_voltage;
uint32_t true_power;
uint32_t true_power2;
//double cummulative_energy;
//double cummulative_energy2;

union
{
	unsigned long long total;
	uint8_t byte[8];  // 0 is the LSB, 7 is the MSB
} cummulative_energy;

union
{
	unsigned long long total;
	uint8_t byte[8];  // 0 is the LSB, 7 is the MSB
} cummulative_energy2;


union
{
unsigned long long total;
uint8_t byte[8];  // 0 is the LSB, 7 is the MSB
} tmp_energy;

union
{
unsigned long long total;
uint8_t byte[8];  // 0 is the LSB, 7 is the MSB
} tmp_energy2;


uint32_t total_secs;

uint16_t ticks;
uint16_t ticks_last;
uint8_t cycle_state;
uint8_t cycle_state_last;
uint16_t cycle_cnt;
uint16_t cycle_avg;
uint16_t freq;
uint8_t cycle_started;

int32_t energy_cycle;
int32_t energy_cycle2;
uint32_t energy_total;
uint32_t energy_total2;
uint32_t current_total;
uint32_t current_total2;
uint32_t current_total_last;
uint32_t current_total2_last;
uint32_t energy_total_last;
uint32_t energy_total2_last;
uint32_t voltage_total_last;

double tmp_d;

uint32_t voltage_total;
uint16_t voltage_avg;

uint32_t freq_total;
uint16_t freq_avg;
uint16_t freq_cnt;
uint8_t triggered;

#endif
