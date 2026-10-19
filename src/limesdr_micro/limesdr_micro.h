#ifndef LIMESDR_MICRO_H
#define LIMESDR_MICRO_H

int initialize_lms7002m_clock_generator();
int LMS64C_protocol_init(void);

extern uint8_t hardware_version;

void limesdr_micro_initialize(void);
int limesdr_micro_set_dcs_clk(struct la9310_hif *hif, uint32_t system_clk_hz);

#endif