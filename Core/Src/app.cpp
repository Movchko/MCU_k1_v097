#include "app.h"

extern "C" {
#include "backend.h"
}

#include "device_config.h"
#include "device_igniter.hpp"
#include "device_lswitch.hpp"

#include "max31855.h"
#include "stm32h5xx_hal.h"


#include <string.h>

#include "main.h"
#include "mku_cfg_flash.h"

#ifndef DEVICE_MCU_K1
#define DEVICE_MCU_K1 20
#endif



#define MAX_TEMP_SMA_SIZE    10u
#define MAX31855_SWITCH_BLANK_MS 100u
#define MAX_TC_STEP_LIMIT_C  40
#define MAX_TC_VALID_STREAK_REQUIRED 2u
#define MAX_FAULT_MASK_FAULT 0x01u
#define MAX_FAULT_MASK_SCV   0x02u
#define MAX_FAULT_MASK_SCG   0x04u
#define MAX_FAULT_MASK_OC    0x08u


MAX31855_Data t_couple;

MKUCfg g_cfg;
MKUCfg g_saved_cfg;

/* 1: DPT, 2: Igniter1, 3: Igniter2 */
static VDeviceDPT g_dpt(1);
static VDeviceIgniter g_igniter1(2);
static VDeviceIgniter g_igniter2(3);

static uint32_t g_extinguish_deadline_ms[NUM_DEV_IN_MCU];
static uint8_t  g_extinguish_armed[NUM_DEV_IN_MCU];
static uint8_t  g_extinguish_paused[NUM_DEV_IN_MCU];
static uint32_t g_extinguish_remaining_ms[NUM_DEV_IN_MCU];



static uint8_t  g_fire_retry_active = 0;
static uint32_t g_fire_last_send_ms = 0;
static uint8_t  DPT_status = 0;
static int16_t  g_max_temp_sma_buf[MAX_TEMP_SMA_SIZE];
static int32_t  g_max_temp_sma_sum = 0;
static uint8_t  g_max_temp_sma_idx = 0;
static uint8_t  g_max_temp_sma_fill = 0;
static uint32_t g_max_mode_switch_ms = 0u;
static int16_t  g_tc_hist3[3] = {0, 0, 0};
static uint8_t  g_tc_hist3_idx = 0u;
static uint8_t  g_tc_hist3_fill = 0u;
static int16_t  g_tc_prev_filtered = 0;
static uint8_t  g_tc_prev_valid = 0u;
static uint8_t  g_tc_valid_streak = 0u;
static int16_t	g_tc_prev_val = 0;
extern bool isListener;

static uint8_t ResetDelayms = 100;
static uint8_t isReset = 0;

static int16_t Median3(int16_t a, int16_t b, int16_t c)
{
    if (a > b) { int16_t t = a; a = b; b = t; }
    if (b > c) { int16_t t = b; b = c; c = t; }
    if (a > b) { int16_t t = a; a = b; b = t; }
    return b;
}

void MAXReadProcess() {
    if (MAX31855_ReadTemperature(&t_couple) == HAL_OK) {
    	//if(HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin) == GPIO_PIN_RESET)
    	//	t_couple.thermocouple_temp_c = g_max_temp_sma_buf[g_max_temp_sma_idx];

    	//if(t_couple.thermocouple_temp_c > (g_max_temp_sma_buf[g_max_temp_sma_idx] + 100))
    	//	t_couple.thermocouple_temp_c = g_max_temp_sma_buf[g_max_temp_sma_idx];

    	uint8_t valid = 1;

    	if(HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin) == GPIO_PIN_RESET)
    		valid = 0;

	/* После переключения реле в MAX-режим кратковременно подавляем данные термопары. */
	if ((HAL_GetTick() - g_max_mode_switch_ms) < MAX31855_SWITCH_BLANK_MS) {
		valid = 0;
	}

        int16_t tc = (int16_t)t_couple.thermocouple_temp_c;
        int16_t ti = (int16_t)t_couple.internal_temp_c;
        uint8_t fault_mask = 0u;
        if (t_couple.fault) { fault_mask |= MAX_FAULT_MASK_FAULT; }
        if (t_couple.scv)   { fault_mask |= MAX_FAULT_MASK_SCV; }
        if (t_couple.scg)   { fault_mask |= MAX_FAULT_MASK_SCG; }
        if (t_couple.oc)    { fault_mask |= MAX_FAULT_MASK_OC; }

        if(t_couple.fault || t_couple.scv || t_couple.scg || t_couple.oc)
        	valid = 0;

        if (valid) {
            g_tc_hist3[g_tc_hist3_idx] = tc;
            g_tc_hist3_idx++;
            if (g_tc_hist3_idx >= 3u) {
                g_tc_hist3_idx = 0u;
            }
            if (g_tc_hist3_fill < 3u) {
                g_tc_hist3_fill++;
            }

            int16_t tc_filtered = tc;
            if (g_tc_hist3_fill == 3u) {
                tc_filtered = Median3(g_tc_hist3[0], g_tc_hist3[1], g_tc_hist3[2]);
            }

            if (!valid) {
                g_tc_valid_streak = 0u;
            }

            if (valid) {
                if (g_tc_valid_streak < 255u) {
                    g_tc_valid_streak++;
                }
                if (g_tc_valid_streak < MAX_TC_VALID_STREAK_REQUIRED) {
                    valid = 0;
                } else {
                    tc = tc_filtered;
                    g_tc_prev_filtered = tc_filtered;
                    g_tc_prev_valid = 1u;
                }
            }
        } else
        	g_tc_valid_streak = 0u;

        if(valid) {
			if (g_max_temp_sma_fill == MAX_TEMP_SMA_SIZE) {
				g_max_temp_sma_sum -= g_max_temp_sma_buf[g_max_temp_sma_idx];
			} else {
				g_max_temp_sma_fill++;
			}
			g_max_temp_sma_buf[g_max_temp_sma_idx] = tc;
			g_max_temp_sma_sum += tc;
			g_max_temp_sma_idx++;
			if (g_max_temp_sma_idx >= MAX_TEMP_SMA_SIZE) {
				g_max_temp_sma_idx = 0u;
			}
			int16_t tc_avg = tc;
			if (g_max_temp_sma_fill > 0u) {
				tc_avg = (int16_t)(g_max_temp_sma_sum / (int32_t)g_max_temp_sma_fill);
			}
			g_tc_prev_val = tc_avg;
			g_dpt.SetMaxStatus(tc_avg, fault_mask, ti);
        } else
        	g_dpt.SetMaxStatus(g_tc_prev_val, fault_mask, ti);
    }
}


static int8_t App_FindIgniterSlotByMsgId(uint32_t MsgID)
{
    can_ext_id_t id;
    id.ID = MsgID & 0x0FFFFFFFu;

    if ((id.field.d_type & 0x7Fu) != DEVICE_IGNITER_TYPE) {
        return -1;
    }
    if ((id.field.h_adr != g_cfg.UId.devId.h_adr) ||
        ((id.field.zone & 0x7Fu) != (g_cfg.UId.devId.zone & 0x7Fu))) {
        return -1;
    }

    if ((id.field.l_adr & 0x3Fu) == 2u && g_cfg.VDtype[1] == DEVICE_IGNITER_TYPE) {
        return 1;
    }
    if ((id.field.l_adr & 0x3Fu) == 3u && g_cfg.VDtype[2] == DEVICE_IGNITER_TYPE) {
        return 2;
    }
    return -1;
}

extern "C" void RcvStopExtinguishment(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
    (void)MsgData;
    if (is_mine == 0u) {
        return;
    }

    int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
    if (ign_slot < 0) {
        return;
    }

    g_extinguish_armed[(uint8_t)ign_slot] = 0u;
    g_extinguish_paused[(uint8_t)ign_slot] = 0u;
    g_extinguish_remaining_ms[(uint8_t)ign_slot] = 0u;
    SetReplyStopExtinguishment((uint8_t)(ign_slot + 1)); /* slot1->dev2, slot2->dev3 */
}

void RcvReplyStatusFire(uint32_t MsgID,  uint8_t *MsgData, uint8_t bus)
{
	can_ext_id_t id;
	id.ID = MsgID;
	if(id.field.zone == g_cfg.UId.devId.zone)
		g_fire_retry_active = 0;
}


extern "C" void RcvStartExtinguishment(uint32_t MsgID,  uint8_t *MsgData, uint8_t is_mine) {
	if (is_mine == 0u) {
		return;
	}

	int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
	if (ign_slot < 0) {
		return;
	}

	/* payload backend fire: [0]=cmd, [1]=zone, [2]=zone_delay_s, [3]=module_delay_s */
	uint8_t zd = MsgData[2];
	uint8_t md = MsgData[3];
	uint32_t delay_ms = ((uint32_t)zd + (uint32_t)md) * 1000u;

	g_extinguish_deadline_ms[(uint8_t)ign_slot] = HAL_GetTick() + delay_ms;
	g_extinguish_armed[(uint8_t)ign_slot] = 1u;
	g_extinguish_paused[(uint8_t)ign_slot] = 0u;
	g_extinguish_remaining_ms[(uint8_t)ign_slot] = delay_ms;
	SetReplyStartExtinguishment((uint8_t)(ign_slot + 1)); /* slot1->dev2, slot2->dev3 */
}

extern "C" void RcvPauseExtinguishmentTimer(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
	(void)MsgData;
	if (is_mine == 0u) {
		return;
	}

	int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
	if (ign_slot < 0) {
		return;
	}
	if (!g_extinguish_armed[(uint8_t)ign_slot]) {
		return;
	}

	if (!g_extinguish_paused[(uint8_t)ign_slot]) {
		uint32_t now = HAL_GetTick();
		if ((int32_t)(g_extinguish_deadline_ms[(uint8_t)ign_slot] - now) > 0) {
			g_extinguish_remaining_ms[(uint8_t)ign_slot] =
				g_extinguish_deadline_ms[(uint8_t)ign_slot] - now;
		} else {
			g_extinguish_remaining_ms[(uint8_t)ign_slot] = 0u;
		}
		g_extinguish_paused[(uint8_t)ign_slot] = 1u;
	}

	SetReplyPauseExtinguishmentTimer((uint8_t)(ign_slot + 1));
}

extern "C" void RcvResumeExtinguishmentTimer(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
	(void)MsgData;
	if (is_mine == 0u) {
		return;
	}

	int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
	if (ign_slot < 0) {
		return;
	}
	if (!g_extinguish_armed[(uint8_t)ign_slot]) {
		return;
	}

	if (g_extinguish_paused[(uint8_t)ign_slot]) {
		uint32_t now = HAL_GetTick();
		g_extinguish_deadline_ms[(uint8_t)ign_slot] = now + g_extinguish_remaining_ms[(uint8_t)ign_slot];
		g_extinguish_paused[(uint8_t)ign_slot] = 0u;
	}

	SetReplyResumeExtinguishmentTimer((uint8_t)(ign_slot + 1));
}

static void App_DPT_SetResMeasureMode(void)
{
    HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_SET);
}

static void App_DPT_SetMaxMeasureMode(void)
{
    HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_SET);
    g_max_mode_switch_ms = HAL_GetTick();
}

static void VDeviceSetStatus(uint8_t DNum, uint8_t Code, const uint8_t *Parameters)
{
    uint8_t data[7] = {0};
    for (uint8_t i = 0; i < 7; i++) {
        data[i] = Parameters[i];
    }
    SendMessage(DNum, Code, data, 0, BUS_CAN12);
    DeviceDPTLineState state = (DeviceDPTLineState)data[0];
    extern Device BoardDevicesList[];
    if (BoardDevicesList[DNum].d_type == DEVICE_DPT_TYPE) {
        if (((DPT_status != state) || g_fire_retry_active) && (state == DeviceDPTLineState_Fire)) {
            DPT_status = state;
            can_ext_id_t can_id;
            can_id.ID = 0;
        	can_id.field.d_type = BoardDevicesList[DNum].d_type & 0x7F;
        	can_id.field.h_adr = BoardDevicesList[DNum].h_adr;
        	can_id.field.l_adr = BoardDevicesList[DNum].l_adr & 0x3F;
        	can_id.field.zone = BoardDevicesList[DNum].zone & 0x7F;
            uint8_t Data[7] = {(uint8_t)can_id.field.d_type, (uint8_t)can_id.field.l_adr, (uint8_t)can_id.field.zone,0 ,0 ,0, 0};
            SetStatusFire(Data);
            g_fire_retry_active = 1u;
            g_fire_last_send_ms = HAL_GetTick();
        }

    }
}

void App_SendStatus() {
	   uint32_t now = HAL_GetTick();
    /* cmd=0 heartbeat для ППКУ:
     * [0]     секунды с запуска (mod 256)
     * [1..3]  резерв (нули)
     * [4]     CAN flags (используется как can_status_mask в ППКУ)
     * [5]     измеренное U24: шаг 1V
     * [6]     CAN state mask: bits[1:0]=CAN0 (0=OK,1=КЗ,2=ОБРЫВ),
     *                          bits[3:2]=CAN1 (0=OK,1=КЗ,2=ОБРЫВ) */
    uint8_t sec = (uint8_t)(now / 1000u);
    uint8_t status_data[7] = {
        sec,
        0u,
        0u,
        0u,
        (uint8_t)(CAN1_Active | (CAN2_Active << 1)),
        0u,
        App_GetCanStateMask()
    };

    /* U24: вычисляем из ADC-кода канала 11 (internal 24V). */

    const uint32_t VREF_MV = 3300u;
    const uint32_t ADC_MAX = 4095u;
    const uint32_t DIV_K   = 10u;

    uint32_t raw_u24 = ADC_GetU24Filtered();
    uint32_t v_adc_mv = (raw_u24 * VREF_MV) / ADC_MAX;
    uint32_t u24_mv   = v_adc_mv * DIV_K;          /* пересчёт к 24В */
    uint32_t code_1v = u24_mv / 1000u;
    if (code_1v > 255u)
    	code_1v = 255u;

    status_data[5] = (uint8_t)code_1v;

    SendMessage(0, 0, status_data, SEND_NOW, BUS_CAN12);

    /* Позиционный маяк кольца: каждый МКУ раз в секунду шлёт стартовый вес 0 в обе стороны. */
    uint8_t pos_data[7] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};
    SendMessage(0, ServiceCmd_PositionDevice, pos_data, SEND_NOW, BUS_CAN12);
}

void SetHAdr(uint8_t h_adr)
{
    g_cfg.UId.devId.h_adr = h_adr;
    extern uint8_t nDevs;
    extern Device BoardDevicesList[];
    for (uint8_t i = 0; i < nDevs; i++) {
        BoardDevicesList[i].h_adr = g_cfg.UId.devId.h_adr;
    }
    SaveConfig();
}

extern "C" {

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

void DefaultConfig(void)
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    memset(&g_cfg, 0, sizeof(g_cfg));

    g_cfg.UId.UId0 = uid0;
    g_cfg.UId.UId1 = uid1;
    g_cfg.UId.UId2 = uid2;
    g_cfg.UId.UId3 = HAL_GetDEVID();
    g_cfg.UId.UId4 = 1;

    g_cfg.UId.devId.zone  = 0;
    g_cfg.UId.devId.l_adr = 0;

    uint8_t hadr = static_cast<uint8_t>(uid0 & 0xFFu);
    if (hadr == 0u) {
        hadr = static_cast<uint8_t>(uid1 & 0xFFu);
        if (hadr == 0u) {
            hadr = 1u;
        }
    }
    g_cfg.UId.devId.h_adr  = hadr;
    g_cfg.UId.devId.d_type = DEVICE_MCU_K1;

    /* 1 LimitSwitch(DPT-base), 2 Igniter1, 3 Igniter2 */
    g_cfg.VDtype[0] = DEVICE_DPT_TYPE;
    g_cfg.VDtype[1] = DEVICE_IGNITER_TYPE;
    g_cfg.VDtype[2] = DEVICE_IGNITER_TYPE;


    g_cfg.zone_delay = 5;
    g_cfg.module_delay[0] = 0;
    g_cfg.module_delay[1] = 2;
    g_cfg.module_delay[2] = 3;

    DeviceDPTConfig *dpt_cfg = reinterpret_cast<DeviceDPTConfig*>(g_cfg.Devices[0].reserv);
    memset(dpt_cfg, 0, sizeof(DeviceDPTConfig));
    dpt_cfg->mode                  = 0; //DeviceDPTMode_DPT
    dpt_cfg->use_max               = 1;
    dpt_cfg->max_fire_threshold_c  = 60;
    dpt_cfg->state_change_delay_ms = 100;

    DeviceIgniterConfig *ign1_cfg = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[1].reserv);
    ign1_cfg->disable_sc_check     = 1u;
    ign1_cfg->threshold_break_low  = 1000u;
    ign1_cfg->threshold_break_high = 3000u;
    ign1_cfg->burn_retry_count     = 0u;

    DeviceIgniterConfig *ign2_cfg = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[2].reserv);
    ign2_cfg->disable_sc_check     = 1u;
    ign2_cfg->threshold_break_low  = 1000u;
    ign2_cfg->threshold_break_high = 3000u;
    ign2_cfg->burn_retry_count     = 0u;
}






void ResetMCU(void)
{
	isReset = 1;
}

uint32_t GetID(void)
{
    uint32_t id0 = HAL_GetUIDw0();
    uint32_t id1 = HAL_GetUIDw1();
    uint32_t id2 = HAL_GetUIDw2();
    return (id0 ^ id1 ^ id2);
}

void MCU_K1CommandCB(uint8_t Command, uint8_t *Parameters) {
	if(Command == 20) {
		g_cfg.UId.devId.zone = Parameters[0];
		SaveConfig();
	}
}

void CommandCB(uint8_t Dev, uint8_t Command, uint8_t *Parameters)
{
    switch (Dev) {
    case 0: MCU_K1CommandCB(Command, Parameters); break;
    case 1: g_dpt.CommandCB(Command, Parameters); break;
    case 2: g_igniter1.CommandCB(Command, Parameters); break;
    case 3: g_igniter2.CommandCB(Command, Parameters); break;
    default: break;
    }
}


void App_Init(void)
{
    extern Device BoardDevicesList[];
    extern uint8_t nDevs;

    if (!FlashReadConfig(&g_cfg)) {
        DefaultConfig();
        SaveConfig();
    }
    g_saved_cfg = g_cfg;
    SetConfigPtr(reinterpret_cast<uint8_t *>(&g_saved_cfg),
                 reinterpret_cast<uint8_t *>(&g_cfg));

    g_dpt.DeviceInit(&g_cfg.Devices[0]);
    g_dpt.VDeviceSetStatus = VDeviceSetStatus;
    g_dpt.VDeviceSaveCfg   = SaveConfig;
    g_dpt.DPT_SetResMeasureMode = App_DPT_SetResMeasureMode;
    g_dpt.DPT_SetMaxMeasureMode = App_DPT_SetMaxMeasureMode;
    g_dpt.Init();
    g_cfg.VDtype[0] = g_dpt.GetDT();

    g_igniter1.DeviceInit(&g_cfg.Devices[1]);
    g_igniter1.VDeviceSetStatus = VDeviceSetStatus;
    g_igniter1.VDeviceSaveCfg   = SaveConfig;
    g_igniter1.Init();

    g_igniter2.DeviceInit(&g_cfg.Devices[2]);
    g_igniter2.VDeviceSetStatus = VDeviceSetStatus;
    g_igniter2.VDeviceSaveCfg   = SaveConfig;
    g_igniter2.Init();

    nDevs = 1;
    BoardDevicesList[0].zone   = g_cfg.UId.devId.zone;
    BoardDevicesList[0].h_adr  = g_cfg.UId.devId.h_adr;
    BoardDevicesList[0].l_adr  = g_cfg.UId.devId.l_adr;
    BoardDevicesList[0].d_type = DEVICE_MCU_K1;

    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 1; /* DPT-base (actual type from mode) */
        BoardDevicesList[nDevs].d_type = g_dpt.GetDT();
        nDevs++;
    }
    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 2; /* Igniter1 */
        BoardDevicesList[nDevs].d_type = DEVICE_IGNITER_TYPE;
        nDevs++;
    }
    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 3; /* Igniter2 */
        BoardDevicesList[nDevs].d_type = DEVICE_IGNITER_TYPE;
        nDevs++;
    }

    App_DPT_SetResMeasureMode();

    isListener = true;




}

void App_Timer1ms(void)
{
    static uint16_t led_cnt = 0u;
    static uint16_t status_cnt = 0u;
    static uint16_t tmax_cnt = 0u;


    extern Device BoardDevicesList[];
    BoardDevicesList[1].d_type = g_dpt.GetDT();

    if (status_cnt < 1000u) {
        status_cnt++;
    } else {
        status_cnt = 0u;

        App_SendStatus();

    }

    if (led_cnt < 1000u) {
        led_cnt++;
    } else {
        led_cnt = 0u;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
/*
    if (g_fire_retry_active) {
        if ((now - g_fire_last_send_ms) >= 200u) {
            g_fire_last_send_ms = now;
            SetStatusFire();
        }
    }
*/

    for (uint8_t i = 0; i < NUM_DEV_IN_MCU; i++) {
        if (g_extinguish_armed[i] && !g_extinguish_paused[i]) {
        	uint32_t now = HAL_GetTick();
            if ((int32_t)(now - g_extinguish_deadline_ms[i]) >= 0) {
                g_extinguish_armed[i] = 0u;
                g_extinguish_paused[i] = 0u;
                g_extinguish_remaining_ms[i] = 0u;
                uint8_t params[7] = {0,0,0,0,0,0,0};
                if (i == 1u) {
                    g_igniter1.CommandCB(10, params);
                } else if (i == 2u) {
                    g_igniter2.CommandCB(10, params);
                }
            }
        }
    }

    if (tmax_cnt < 100u) {
        tmax_cnt++;
    } else {

    	MAXReadProcess();
        tmax_cnt = 0u;
    }

    App_UpdateCanActivity();

    if (!g_igniter1.IsPwmActive()) {
        uint16_t raw = ADC_GetIgniter1Filtered();
        uint16_t mv = (uint16_t)((uint32_t)raw * 3300u / 4095u);
        g_igniter1.UpdateLineFromAdcMv(mv);
    }
    if (!g_igniter2.IsPwmActive()) {
        uint16_t raw = ADC_GetIgniter2Filtered();
        uint16_t mv = (uint16_t)((uint32_t)raw * 3300u / 4095u);
        g_igniter2.UpdateLineFromAdcMv(mv);
    }

    g_dpt.Timer1ms();
    g_igniter1.Timer1ms();
    g_igniter2.Timer1ms();


    BackendProcess();

    uint16_t pwm1 = g_igniter1.GetPwm();
    if (pwm1 > 0u) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm1);
    } else {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    }

    uint16_t pwm2 = g_igniter2.GetPwm();
    if (pwm2 > 0u) {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2);
    } else {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    }

    // задержка софт-рестарта. нужно чтобы усройство успело широковещательную переслать команду дальше
    if(isReset) {
    	ResetDelayms--;
    	if(ResetDelayms == 0)
    		NVIC_SystemReset();
    }

}



void App_SetDPTAdcValues(uint16_t ch_l, uint16_t ch_h, uint16_t ch_u24)
{
    const uint32_t VREF_MV = 3300u;
    const uint32_t ADC_MAX = 4095u;
    const uint32_t R0_OHM  = 1925u;
    const uint32_t DIV_K   = 2;

    uint32_t v_adc_l_mv = (uint32_t)ch_l * VREF_MV / ADC_MAX;
    uint32_t v_adc_h_mv = (uint32_t)ch_h * VREF_MV / ADC_MAX;
    uint32_t v_adc_u_mv = 5000;//(uint32_t)ch_u24 * VREF_MV / ADC_MAX;

    uint32_t v_line_l_mv = v_adc_l_mv * DIV_K;
    uint32_t v_line_h_mv = v_adc_h_mv * DIV_K;
    uint32_t u24_mv      = v_adc_u_mv * DIV_K;//v_adc_u_mv * DIV_K; // мы всегда в этой версии и далее подаём 5В.

    if (v_line_l_mv == 0u) {
        v_line_l_mv = 1u;
    }

    uint32_t r_line_ohm = 0u;
    if (v_line_h_mv > v_line_l_mv) {
        r_line_ohm = R0_OHM * (v_line_h_mv - v_line_l_mv) / v_line_l_mv;
    }

    int32_t k_scaled = -346;
    k_scaled += (int32_t)(673u * (u24_mv / 10u)) / 1000;
    if (k_scaled < 300)  k_scaled = 300;
    if (k_scaled > 1500) k_scaled = 1500;

    uint32_t r_corr = (uint32_t)((r_line_ohm * (uint32_t)k_scaled + 500u) / 1000u);

#if UNIQ_DEBUG
    r_corr = r_corr / 3;
#endif

    g_dpt.SetAdcValues((uint16_t)r_corr, 0);
}

} /* extern "C" */

