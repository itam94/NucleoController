/*
 * PS2_device.c
 *
 *  Created on: 5 wrz 2018
 *      Author: mateu
 */

#include "PS2_device.h"


void PS2_Initialize(){
	Sending = false;
	Reciving = false;
	LastRecived = 0;
	 LastProcessed = 51;
	Sended = false;
	StreamMode = false;
	first = true;
	bytesCount = 3;
	intellimouseState = 0;
	isSendingWaiting = false;
	preSendingState = false;
	Recived = 0;
	bitnumber = 0;
}
extern uint32_t second;
void PS2_checkParity() {
	unsigned int x = HAL_GPIO_ReadPin(Data_value_GPIO_Port, Data_value_Pin);
	int count = PS2_calculateCount(dataRecivedPacket);


	if ((count + x)%2 == 1 && (StreamMode == true && LastRecived == 2)) {

			  mouseHID.buttons = setButtons();
			  mouseHID.x = setBit(1);
			  mouseHID.y = setBit(2);
			  mouseHID.wheel = setBit(3);
			  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
			  ++second;
	}

}

void PS2_readBit() {
	unsigned int x = HAL_GPIO_ReadPin(Data_value_GPIO_Port, Data_value_Pin);
	dataRecivedPacket = dataRecivedPacket + ((x%2)<<(bitnumber-1));

	if(LastRecived == 50 || (LastRecived == 3 && StreamMode == true)){
		AllRecived[0]=dataRecivedPacket;
	}else{
		AllRecived[LastRecived+1]=dataRecivedPacket;
	}


}

void PS2_Handle_Reciving() {
	if (bitnumber == 0) {
		dataRecivedPacket = 0;
		Reciving = true;
		bitnumber++;

	} else if (bitnumber < 9 && Reciving == true) {
		PS2_readBit();
		bitnumber++;
	} else if (bitnumber == 9) {
		PS2_checkParity();
		bitnumber++;
	} else {
		Reciving = false;
		bitnumber = 0;

			if(LastRecived == 50 || (LastRecived >= 3 && StreamMode == true)){
						LastRecived = 0;
			}else{
				++LastRecived;
			}
			Recived = true;


		if(isSendingWaiting == true){
			PS2_SendPackage(&dataSendPacket);
		}
	}
}

void PS2_setNextDataBit(uint32_t bitnumber){
	uint8_t k = 1;
	if (dataSendPacket & (k << (bitnumber - 1))) {
		HAL_GPIO_WritePin(Data_value_GPIO_Port,Data_value_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(Data_value_GPIO_Port,Data_value_Pin,GPIO_PIN_RESET);
	}

}

void PS2_setParityBit(){
	int count = PS2_calculateCount(dataSendPacket);
	if (count%2 == 0){
		HAL_GPIO_WritePin(Data_value_GPIO_Port,Data_value_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(Data_value_GPIO_Port,Data_value_Pin,GPIO_PIN_RESET);
	}
}



void PS2_SendPackage(uint8_t *data){
	AllSended[LastRecived]= *data;
	dataSendPacket=*data;
	if(Reciving == true && bitnumber>9){
		isSendingWaiting = true;
	}else{
		preSendingState = true;
		HAL_GPIO_WritePin(Clock_host_GPIO_Port,Clock_host_Pin,GPIO_PIN_RESET);
		HAL_TIM_Base_Start_IT(&htim7);
		start = HAL_GetTick();
	}

}

int PS2_calculateCount(uint8_t data){
	int count = 0, i, b = 1;

		for (i = 0; i < 8; i++) {
			if (data & (b << i)) {
				count++;
			}
		}
	return count;
}

void PS2_Handle_Recived(){
	int process;
	 if(LastProcessed >= 50){
		process = 0;
		LastProcessed = 0;
	}else{
		process = LastProcessed++;
	}
	switch(AllRecived[process-1]){
	case 0xFA:if(dataSendPacket == 0xF4){
			StreamMode = true;
			HAL_TIM_Base_Stop_IT(&htim16);
			}
		break;
	case 0xAA:
		break;
	case 0x00:
		break;
	case 0xFE: PS2_SendPackage(&dataSendPacket);
		break;
	case 0xFC: PS2_SendPackage(&dataSendPacket);
		break;
	case 0x03:bytesCount = 4;
		break;
	case 0x04:bytesCount = 4;
		break;

	}
	if(process == LastRecived){
		Recived = false;
	}

}

int8_t setBit(int i){
	switch(i){
	case 1: return AllRecived[i];
		break;
	case 2: return -AllRecived[i];
		break;
	case 3: return -AllRecived[i];
	 break;


	}
	return 0;
}

uint8_t setButtons(){
	uint8_t toSend;
	toSend = AllRecived[0];
	toSend = toSend<<5;

	return  toSend>>5;

}

void PS2_Handle_Sending() {
	if (Reciving == true) {
		bitnumber = 0;
		Reciving = false;
	}else{
		++bitnumber;
		switch(bitnumber){
		case 9:
			PS2_setParityBit();
			break;
		case 10:
			HAL_GPIO_WritePin(Data_value_GPIO_Port,Data_value_Pin,GPIO_PIN_SET);
			break;
		case 11:
			Sending = false;
			bitnumber = 0;
			break;
		default:
			PS2_setNextDataBit(bitnumber);
			break;
		}

	}

}
