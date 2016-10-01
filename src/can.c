#include <string.h>
#include <stdio.h>
#include "cmsis_device.h"
#include "can.h"
#include "timer.h"
#include "datatypes.h"
#include "buffer.h"
#include "crc.h"
#include "vesc.h"

#define VESC_ID     0x01
#define RX_FRAMES_SIZE  100
#define RX_BUFFER_SIZE  1024

static CanRxMsg rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read;
static int rx_frame_write;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
mc_configuration can_mcconf;
volatile mc_values can_values = {
	-1.0, -1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0, -1,-1, FAULT_CODE_NONE,
};

void can_init() {
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    CAN_InitStructure.CAN_Prescaler = 4;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_Init(CAN1, &CAN_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    /*CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0901;
     CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
     CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
     CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;*/
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
    CanRxMsg rxMessage;
    CAN_Receive(CAN1, CAN_FIFO0, &rxMessage);

    rx_frames[rx_frame_write++] = rxMessage;
    if (rx_frame_write == RX_FRAMES_SIZE) {
        rx_frame_write = 0;
    }

}

void commands_process_packet(uint8_t *data, unsigned int len) {
	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;
	int32_t ind = 0;


	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_GET_VALUES:

		ind = 0;
		can_values.temp_mos1 = buffer_get_float16(data, 10.0, &ind);
		can_values.temp_mos2 = buffer_get_float16(data, 10.0, &ind);
		can_values.temp_mos3 = buffer_get_float16(data, 10.0, &ind);
		can_values.temp_mos4 = buffer_get_float16(data, 10.0, &ind);
		can_values.temp_mos5 = buffer_get_float16(data, 10.0, &ind);
		can_values.temp_mos6 = buffer_get_float16(data, 10.0, &ind);
		can_values.temp_pcb = buffer_get_float16(data, 10.0, &ind);
		can_values.current_motor = buffer_get_float32(data, 100.0, &ind);
		can_values.current_in = buffer_get_float32(data, 100.0, &ind);
		can_values.duty_now = buffer_get_float16(data, 1000.0, &ind);
		can_values.rpm = buffer_get_float32(data, 1.0, &ind);
		can_values.v_in = buffer_get_float16(data, 10.0, &ind);
		can_values.amp_hours = buffer_get_float32(data, 10000.0, &ind);
		can_values.amp_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		can_values.watt_hours = buffer_get_float32(data, 10000.0, &ind);
		can_values.watt_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		can_values.tachometer = buffer_get_int32(data, &ind);
		can_values.tachometer_abs = buffer_get_int32(data, &ind);
		can_values.fault_code = (mc_fault_code)data[ind++];

		vesc_calculate_can_data();
		break;

	case COMM_GET_MCCONF:
		ind = 0;
		can_mcconf.pwm_mode = (mc_pwm_mode)data[ind++];
		can_mcconf.comm_mode = (mc_comm_mode)data[ind++];
		can_mcconf.motor_type = (mc_motor_type)data[ind++];
		can_mcconf.sensor_mode = (mc_sensor_mode)data[ind++];

		can_mcconf.l_current_max = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_current_min = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_in_current_max = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_in_current_min = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_abs_current_max = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_min_erpm = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_max_erpm = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_max_erpm_fbrake = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_max_erpm_fbrake_cc = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_min_vin = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_max_vin = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_battery_cut_start = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_battery_cut_end = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_slow_abs_current = data[ind++];
		can_mcconf.l_rpm_lim_neg_torque = data[ind++];
		can_mcconf.l_temp_fet_start = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_temp_fet_end = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_temp_motor_start = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_temp_motor_end = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.l_min_duty = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.l_max_duty = buffer_get_float32(data, 1000000.0, &ind);

		can_mcconf.sl_min_erpm = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.sl_min_erpm_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.sl_max_fullbreak_current_dir_change = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.sl_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.sl_phase_advance_at_br = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.sl_cycle_int_rpm_br = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.sl_bemf_coupling_k = buffer_get_float32(data, 1000.0, &ind);

		memcpy(can_mcconf.hall_table, data + ind, 8);
		ind += 8;
		can_mcconf.hall_sl_erpm = buffer_get_float32(data, 1000.0, &ind);

		can_mcconf.foc_current_kp = buffer_get_float32(data, 1e5, &ind);
		can_mcconf.foc_current_ki = buffer_get_float32(data, 1e5, &ind);
		can_mcconf.foc_f_sw = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_dt_us = buffer_get_float32(data, 1e6, &ind);
		can_mcconf.foc_encoder_inverted = data[ind++];
		can_mcconf.foc_encoder_offset = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_encoder_ratio = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_sensor_mode = (mc_foc_sensor_mode)data[ind++];
		can_mcconf.foc_pll_kp = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_pll_ki = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_motor_l = buffer_get_float32(data, 1e8, &ind);
		can_mcconf.foc_motor_r = buffer_get_float32(data, 1e5, &ind);
		can_mcconf.foc_motor_flux_linkage = buffer_get_float32(data, 1e5, &ind);
		can_mcconf.foc_observer_gain = buffer_get_float32(data, 1e0, &ind);
		can_mcconf.foc_duty_dowmramp_kp = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_duty_dowmramp_ki = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_openloop_rpm = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_sl_openloop_hyst = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_sl_openloop_time = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_sl_d_current_duty = buffer_get_float32(data, 1e3, &ind);
		can_mcconf.foc_sl_d_current_factor = buffer_get_float32(data, 1e3, &ind);
		memcpy(can_mcconf.foc_hall_table, data + ind, 8);
		ind += 8;
		can_mcconf.foc_sl_erpm = buffer_get_float32(data, 1000.0, &ind);

		can_mcconf.s_pid_kp = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.s_pid_ki = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.s_pid_kd = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.s_pid_min_erpm = buffer_get_float32(data, 1000.0, &ind);

		can_mcconf.p_pid_kp = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.p_pid_ki = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.p_pid_kd = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.p_pid_ang_div = buffer_get_float32(data, 1e5, &ind);

		can_mcconf.cc_startup_boost_duty = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.cc_min_current = buffer_get_float32(data, 1000.0, &ind);
		can_mcconf.cc_gain = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.cc_ramp_step_max = buffer_get_float32(data, 1000000.0, &ind);

		can_mcconf.m_fault_stop_time_ms = buffer_get_int32(data, &ind);
		can_mcconf.m_duty_ramp_step = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.m_duty_ramp_step_rpm_lim = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.m_current_backoff_gain = buffer_get_float32(data, 1000000.0, &ind);
		can_mcconf.m_encoder_counts = buffer_get_uint32(data, &ind);
		can_mcconf.m_sensor_port_mode = (sensor_port_mode)data[ind++];
		break;
	/*
	case COMM_GET_DECODED_PPM:
		ind = 0;

		//buffer_append_int32(send_buffer, (int32_t)(servodec_get_servo(0) * 1000000.0), &ind);
		//buffer_append_int32(send_buffer, (int32_t)(servodec_get_last_pulse_len(0) * 1000000.0), &ind);

		break;
	case COMM_GET_DECODED_ADC:
		ind = 0;

		//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_decoded_level() * 1000000.0), &ind);
		//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_voltage() * 1000000.0), &ind);
		//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_decoded_level2() * 1000000.0), &ind);
		//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_voltage2() * 1000000.0), &ind);
		break;*/
	default:
		break;
	}
}


void can_process() {
    static timer_ticks_t can_ticks = 0;
    int32_t ind = 0;
    unsigned int rxbuf_len;
    unsigned int rxbuf_ind;
    uint8_t crc_low;
    uint8_t crc_high;

	if (systicks - can_ticks >= 50) {

		can_request_vesc(COMM_GET_VALUES);

		while (rx_frame_read != rx_frame_write) {
			CanRxMsg rxmsg = rx_frames[rx_frame_read++];

			if (rxmsg.IDE == CAN_Id_Extended) {
				//uint8_t id = rxmsg.ExtId & 0xFF;		id is the sending vesc controller is
				CAN_PACKET_ID cmd = rxmsg.ExtId >> 8;

				/*for(uint8_t i = 0; i < rxmsg.DLC; i++) {
					printf("%02x ", rxmsg.Data[i]);
				}

				putchar('\n');*/

				switch (cmd) {
					case CAN_PACKET_FILL_RX_BUFFER:
						memcpy(rx_buffer + rxmsg.Data[0], rxmsg.Data + 1, rxmsg.DLC - 1);
						break;
					case CAN_PACKET_FILL_RX_BUFFER_LONG:
						rxbuf_ind = (unsigned int)rxmsg.Data[0] << 8;
						rxbuf_ind |= rxmsg.Data[1];
						if (rxbuf_ind < RX_BUFFER_SIZE) {
							memcpy(rx_buffer + rxbuf_ind, rxmsg.Data + 2, rxmsg.DLC - 2);
						}
						break;
					case CAN_PACKET_PROCESS_RX_BUFFER:
						ind = 0;
						// skip (sending controller id) rxmsg.Data[ind++];
						ind++;

						// skip (send true/false) rxmsg.Data[ind++];
						ind++;

						rxbuf_len = (unsigned int)rxmsg.Data[ind++] << 8;
						rxbuf_len |= (unsigned int)rxmsg.Data[ind++];

						if (rxbuf_len > RX_BUFFER_SIZE) {
							break;
						}

						crc_high = rxmsg.Data[ind++];
						crc_low = rxmsg.Data[ind++];

						if (crc16(rx_buffer, rxbuf_len) ==
								((unsigned short) crc_high << 8 | (unsigned short) crc_low)) {

							commands_process_packet(rx_buffer, rxbuf_len);
						}
						break;

					default:
						break;
				}
			}

			if (rx_frame_read == RX_FRAMES_SIZE) {
				rx_frame_read = 0;
			}
		}

		can_ticks = systicks;
	}

}

void can_request_vesc(COMM_PACKET_ID packetId) {

    CanTxMsg canMessage;

    canMessage.ExtId = VESC_ID | (CAN_PACKET_PROCESS_SHORT_BUFFER << 0x08);
    canMessage.RTR = CAN_RTR_DATA;
    canMessage.IDE = CAN_ID_EXT;
    canMessage.DLC = 3;

    canMessage.Data[0] = 0x23;             	// command the vesc the answer CAN ID
    canMessage.Data[1] = 0;      			// no command sending to other VESCS on CAN bus
    canMessage.Data[2] = packetId;       	// the command to execute

    CAN_Transmit(CAN1, &canMessage);

}
