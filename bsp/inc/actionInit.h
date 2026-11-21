/* 上电归位及机构原子操作。ZDT_set 用脉冲数控制步进轴，servo_set 与 Plate 用编号选择 PWM 预设。 */

#ifndef __ACTIONINIT_H
#define __ACTIONINIT_H
#include "main.h"
void action_Init(void);
void Plate(uint16_t num);
void servo_set(uint16_t num);
void ZDT_set(uint8_t id , uint16_t num);
void ZDT_set2(uint8_t id, uint16_t num);
#endif 
