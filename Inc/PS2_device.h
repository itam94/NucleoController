/*
 * PS2_device.h
 *
 *  Created on: 5 wrz 2018
 *      Author: mateu
 */
#include<stdbool.h>
#include "stm32l4xx_hal.h"
#include "stdint.h"
#include "usbd_customhid.h"


#ifndef PS2_DEVICE_H_
#define PS2_DEVICE_H_

int bitnumber;


extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;

extern struct mouseHID_t mouseHID;
extern USBD_HandleTypeDef hUsbDeviceFS;
//extern uint16_t Data_value_GPIO_Port;
//extern uint16_t Data_value_Pin;
bool Sending;
bool Reciving;
int LastRecived;
int LastProcessed;
bool Sended;
bool StreamMode;
bool first;
int bytesCount;
int intellimouseState;
uint8_t dataSendPacket;
uint8_t dataRecivedPacket;
bool isSendingWaiting;
bool preSendingState;
uint8_t Recived;

uint8_t AllRecived[51];
uint8_t AllSended[51];
uint8_t isDataReady;
uint8_t ParityTable[51];
uint32_t start;
uint32_t diff;

void PS2_Initialize();
void PS2_readBit();
void PS2_checkParity();
void PS2_Handle_Reciving();
void PS2_Handle_Sending();
void PS2_setNextDataBit(uint32_t bitnumber);
void PS2_setParityBit();
int  PS2_calculateCount(uint8_t data);
void PS2_SendPackage(uint8_t *data);
void PS2_Handle_Recived();

int8_t setBit(int i);
uint8_t setButtons();

#endif /* PS2_DEVICE_H_ */
