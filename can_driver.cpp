/* can.c -- shamelessly stolen from the CANtact development pageL_CAN_ConfigFilter(&hcan, &filter);
 * implementations of can functions defined in can.h
 * Modified extensively by Samuel Ellicott to use hardware registers instead
 * of HAL library.
 */

#include "can_driver.h"

can_driver::can_driver()
{
	// default to 125 kbit/s
	prescaler = 18;
	bs1 = CAN_BS1_13TQ;
	bs2 = CAN_BS2_2TQ;
}

void can_driver::can_init(uint16_t id) {
	UNUSED(id);
	hcan.Instance = CAN1;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.Prescaler = prescaler;
	hcan.Init.TimeSeg1 = bs1;
	hcan.Init.TimeSeg2 = bs2;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Serial.println("HAL CAN INIT FAILED");
		Error_Handler();
	}

	// Serial.printf("CANState %d, CANError %X\r\n", hcan.State, hcan.ErrorCode);
	num_msg = 0;
	bus_state = BUS_OFF;
}

void can_driver::attachInterrupt(HAL_CAN_CallbackIDTypeDef irqType, void (*pCallback)(CAN_HandleTypeDef* _hcan))
{
	HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, pCallback);
}

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef * hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hcan->Instance == CAN1)
	{
		// Serial.println("HAL_CAN_MspInit");
		/* USER CODE BEGIN CAN1_MspInit 0 */

		/* USER CODE END CAN1_MspInit 0 */
		  /* Peripheral clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**CAN GPIO Configuration
		PA11     ------> CAN_RX
		PA12     ------> CAN_TX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* CAN1 interrupt Init */
		HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
		/* USER CODE BEGIN CAN1_MspInit 1 */

		/* USER CODE END CAN1_MspInit 1 */
	}

}

void can_driver::can_enable(void) {
  if (bus_state == BUS_OFF) {
	  if(false)
	  {
		  if(HAL_CAN_Start(&hcan) != HAL_OK)
		  {
			  Serial.println("HAL_CAN_START FAILED");
			  Error_Handler();
		  }
		  return;
	  }

 //    // enable CAN clock
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
 
    // Enter CAN init mode to write the configuration
    hcan.Instance->MCR |= CAN_MCR_INRQ;
    // Wait for the hardware to initilize
    while ((hcan.Instance->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
      ;
    // Exit sleep mode
    hcan.Instance->MCR &= ~CAN_MCR_SLEEP;
    // Setup timing: BS1 and BS2 are set in can_set_bitrate().
    // The prescalar is set to whatever it was set to from can_set_bitrate()
	hcan.Instance->BTR = (uint32_t)(bs1 | bs2 | prescaler - 1U);
 
    hcan.Instance->MCR &= ~CAN_MCR_INRQ; /* Leave init mode */
    /* Wait the init mode leaving */
    while ((hcan.Instance->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
      ;
 
    /* Set FIFO0 message pending IT enable */
    hcan.Instance->IER |= CAN_IER_FMPIE0;

    bus_state = BUS_OK;
  }
                                       
  //HAL_GPIO_WritePin(CAN_EN_GPIO_Port, CAN_EN_Pin, GPIO_PIN_RESET); 
}

void can_driver::can_sleep()
{
	if(HAL_CAN_RequestSleep(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
}

void can_driver::can_wakeup()
{
	if (HAL_CAN_WakeUp(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
}

void can_driver::can_set_bitrate(canBitrate bitrate) {
  // all these values were calculated from the equation given in the reference
  // manual for
  // finding the baudrate. They are calculated from an LibreOffice Calc
  // spreadsheet for a 16MHZ clock

#ifndef STM32F3
  switch (bitrate) {
  case CAN_BITRATE_10K:
    prescaler = 50;
	bs1 = CAN_BS1_13TQ;
	bs2 = CAN_BS2_2TQ;
    break;
  case CAN_BITRATE_20K:
	  prescaler = 25;
	  bs1 = CAN_BS1_13TQ;
	  bs2 = CAN_BS2_2TQ;
    break;
  case CAN_BITRATE_50K:
	  prescaler = 10;
	  bs1 = CAN_BS1_13TQ;
	  bs2 = CAN_BS2_2TQ;
    break;
  case CAN_BITRATE_100K:
	  prescaler = 5;
	  bs1 = CAN_BS1_13TQ;
	  bs2 = CAN_BS2_2TQ;
    break;
  case CAN_BITRATE_125K:
	  prescaler = 18;
	  bs1 = CAN_BS1_13TQ;
	  bs2 = CAN_BS2_2TQ;
    break;
  case CAN_BITRATE_250K:
	  prescaler = 2;
	  bs1 = CAN_BS1_13TQ;
	  bs2 = CAN_BS2_2TQ;
    break;
  case CAN_BITRATE_500K:
	  prescaler = 1;
	  bs1 = CAN_BS1_13TQ;
	  bs2 = CAN_BS2_2TQ;
    break;
  case CAN_BITRATE_1000K:
	  prescaler = 4;
	  bs1 = CAN_BS1_7TQ;
	  bs2 = CAN_BS2_1TQ;
    break;
  }
#endif
}

/**
 * \param id id to filter on 
 *
 * \returns the filter number of the added filter returns \ref CAN_FILTER_ERROR
 * if the function was unable to add a filter.
 */
uint16_t can_driver::can_add_filter_id(uint16_t id) {
    
  CAN_FilterTypeDef filter;
  uint8_t bank_num, fltr_num;
  const int MAX_FILTER = 12;

  // mostly setup filter
  filter.FilterIdLow = 0;
  filter.FilterIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterBank = 0;
  filter.FilterActivation = CAN_FILTER_ENABLE;

  // loop through filter banks to find an empty filter register
  for (fltr_num = bank_num = 0; bank_num < MAX_FILTER; bank_num++) {
    // this is a register currently in use, check if there is any openings
    if (hcan.Instance->FA1R & (1 << bank_num)) {
      // check if bank is using a mask or a id list
      if (hcan.Instance->FM1R & (1 << bank_num)) {
        // id list

        filter.FilterBank = bank_num;

        // if we are here then the first slot has already been filled
        filter.FilterMaskIdLow = (hcan.Instance->sFilterRegister[bank_num].FR1 >> 16) && 0xFFFF;
        filter.FilterIdLow = hcan.Instance->sFilterRegister[bank_num].FR1 && 0xFFFF;

        ++fltr_num;
        // check if second slot has been filled
        if (filter.FilterIdLow == 0) {
          filter.FilterIdLow = id;
          if(HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
		  {
			  Error_Handler();
		  }
          return fltr_num;
        } 
        filter.FilterMaskIdHigh = (hcan.Instance->sFilterRegister[bank_num].FR2 >> 16) && 0xFFFF;
        filter.FilterIdHigh = hcan.Instance->sFilterRegister[bank_num].FR2 && 0xFFFF;

        ++fltr_num;
        // check if third slot has been filled
        if (filter.FilterMaskIdHigh == 0) {
          filter.FilterMaskIdHigh = id;
		  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
		  {
			  Error_Handler();
		  }
          return fltr_num;
        } 

        ++fltr_num;
        // check if fourth slot has been filled
        if (filter.FilterIdHigh == 0) {
          filter.FilterIdHigh = id;
		  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
		  {
			  Error_Handler();
		  }
          return fltr_num;
        } 
        // no empty slots incriment number, should be 4 greater than when
        // started
        ++fltr_num;

      } else {
        // add the number of filters in an id-mask to the filter number
        fltr_num += 2;
      }
    } else {

      filter.FilterIdLow = (id << 5);
      filter.FilterBank = bank_num;

	  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
	  {
		  Error_Handler();
	  }
      // return the filter number
      return fltr_num;
    }
  }

  return CAN_FILTER_ERROR;
}

/**
 * *NOTE:
 * This function takes some finagleing in order for it to work correctly with
 * the %CanNode library.
 * For it to work correctly the returned value from this function should be 
 * passed to CanNode_addFilter() as the id. This lets CanNode_checkForMessages() 
 * know what handler to call if a message using this filter is recieved.*
 * 
 * Example code
 *
 * ~~~~~~~~~~~~ {.c}
 * uint16_t id = can_add_filter_mask(id_to_filter, id_mask);
 * CanNode_addFilter(id, handler);
 * ~~~~~~~~~~~~
 * 
 * \param id base id of the filter mask
 * \param mask mask on top of the base id, 0's are don't cares
 *
 * \returns the filter number of the added filter returns \ref CAN_FILTER_ERROR
 * if the function was unable to add a filter.
 */
uint16_t can_driver::can_add_filter_mask(uint16_t id, uint16_t mask) {
  CAN_FilterTypeDef filter;
  uint8_t bank_num, fltr_num;
  const int MAX_FILTER = 12;

  // mostly setup filter
  filter.FilterIdLow = 0;
  filter.FilterIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank = 0;
  filter.FilterActivation = ENABLE;

  // loop through filter banks to find an empty filter register
  for (fltr_num = bank_num = 0; bank_num < MAX_FILTER; bank_num++) {
    // this is a register currently in use, check if there is any openings
    if (hcan.Instance->FA1R & (1 << bank_num)) {
      // check if bank is using a mask or a id list
      if ((hcan.Instance->FM1R & (1 << bank_num)) == 0) {
        // id mask

        // if we are here then the first slot has been taken. fill it
        // from the registers
        filter.FilterIdLow = hcan.Instance->sFilterRegister[bank_num].FR1;
        filter.FilterIdHigh = (hcan.Instance->sFilterRegister[bank_num].FR1 >> 16);

        // incriment filter index
        ++fltr_num;

        // fill the new slot
        filter.FilterMaskIdLow = (id << 5);
        filter.FilterMaskIdHigh = (mask << 5);

        // configure it
		if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
		{
			Error_Handler();
		}
      } else {
        // add the number of filters in an id-list to the filter number
        fltr_num += 4;
      }
    } else {

      filter.FilterIdLow = (id << 5);
      filter.FilterIdHigh = (mask << 5);
      filter.FilterBank = bank_num;

	  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
	  {
		  Error_Handler();
	  }
      // return the filter number
      return fltr_num;
    }
  }

  return CAN_FILTER_ERROR;
}

CanState can_driver::can_tx(CanMessage *tx_msg, uint32_t timeout) {

	if (true) {
		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.StdId = tx_msg->id;
		TxHeader.RTR = tx_msg->rtr;
		TxHeader.ExtId = tx_msg->id;
		TxHeader.DLC = tx_msg->length;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;
		uint32_t mailBox;

		HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_msg->data.uint8, &mailBox);
		if(status == HAL_OK)
			return BUS_OK;
		Serial.printlnf("HAL_Stat %d, CAN_STAT %#X, CAN_ErrorCode %#X, Mailbox %#X", status, hcan.State, hcan.ErrorCode, mailBox);
		return BUS_BUSY;
	}
  // uint8_t mailbox; // find an empty mailbox
  // for (mailbox = 0; mailbox < 3; ++mailbox) {
  //   // check the status
  //   if (hcan.Instance->sTxMailBox[mailbox].TIR & CAN_TI0R_TXRQ) {
  //     continue;
  //   } else {
  //     break; // found open mailbox
  //   }
  // }
  //
  // // if there are no open mailboxes
  // if (mailbox == 3) {
	 //  Serial.println("Bus busy");
  //   return BUS_BUSY;
  // }
  //
  // // add data to register
  // hcan.Instance->sTxMailBox[mailbox].TIR = (uint32_t)tx_msg->id << 21;
  // if (tx_msg->rtr) {
  //   hcan.Instance->sTxMailBox[mailbox].TIR |= CAN_TI0R_RTR;
  // }
  //
  // // set message length
  // hcan.Instance->sTxMailBox[mailbox].TDTR = tx_msg->length & 0x0F;
  //
  // // clear mailbox and add new data
  // hcan.Instance->sTxMailBox[mailbox].TDHR = 0;
  // hcan.Instance->sTxMailBox[mailbox].TDLR = 0;
  // for (uint8_t i = 0; i < 4; ++i) {
  //   hcan.Instance->sTxMailBox[mailbox].TDHR |= tx_msg->data.bytes[i + 4] << (8 * i);
  //   hcan.Instance->sTxMailBox[mailbox].TDLR |= tx_msg->data.bytes[i] << (8 * i);
  // }
  // // transmit can frame
  // hcan.Instance->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;
  //
  // return BUS_OK;
}

CanState can_driver::can_rx(CanMessage *rx_msg, uint32_t timeout) {
	uint8_t fifoNum = 0;

	//check if there is data in fifo0
	if((hcan.Instance->RF0R & CAN_RF0R_FMP0) == 0){ //if there is no data
		return NO_DATA;
	}

	if(true)
	{
		CAN_RxHeaderTypeDef RxHeader;
		HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(&hcan, 0, &RxHeader, rx_msg->data.uint8);
		if(status == HAL_OK)
		{
			rx_msg->length = RxHeader.DLC;
			rx_msg->id = RxHeader.StdId;
			rx_msg->rtr = RxHeader.RTR;
			return DATA_OK;
		} 
		else
		{
			Serial.print("HAL NOT OK: ");
			Serial.println(status);
			return CanState::DATA_ERROR;
		}
	}
 //
	// //get data from regisers
	// //get the id field
	// rx_msg->id = (uint16_t) (hcan.Instance->sFIFOMailBox[fifoNum].RIR >> 21);
 //
	// //check if it is a rtr message
	// rx_msg->rtr = false;
	// if(hcan.Instance->sFIFOMailBox[fifoNum].RIR & CAN_RI0R_RTR){ 
	// 	rx_msg->rtr = true;
	// }
	//
	// //get data length
	// rx_msg->length = (uint8_t) (hcan.Instance->sFIFOMailBox[fifoNum].RDTR & CAN_RDT0R_DLC);
	//
	// //get filter mask index
	// rx_msg->fmi = (uint8_t) (hcan.Instance->sFIFOMailBox[fifoNum].RDTR >> 8);
 //
	// //get the data
 //    for(uint8_t i=0; i<4; ++i) {
	// 	rx_msg->data.bytes[i+4] = (uint8_t) (hcan.Instance->sFIFOMailBox[fifoNum].RDHR >> (8*i));
	// 	rx_msg->data.bytes[i]   = (uint8_t) (hcan.Instance->sFIFOMailBox[fifoNum].RDLR >> (8*i));
	// }
 //
	// //clear fifo
	// hcan.Instance->RF0R |= CAN_RF0R_RFOM0;
	// --num_msg;
 //
	// return BUS_OK;
}

bool can_driver::is_can_msg_pending() {
	// if(hcan.Instance->ESR & CAN_ESR_BOFF == 1)
	// {
	// 	Serial.println("BUS OFF");
	// }
	// if(hcan.Instance->ESR & CAN_ESR_EPVF == 1)
	// {
	// 	Serial.println("ERROR PASIVE FLAG SET ");
	// }
	// if(hcan.Instance->ESR & CAN_ESR_EWGF == 1)
	// {
	// 	Serial.println("ERROR WARNING FLAG SET");
	// }
	if(hcan.Instance->RF0R & CAN_RF0R_FULL0 == 1)
	{
		Serial.println("FIFO FULL");
	}
	return ((hcan.Instance->RF0R & CAN_RF0R_FMP0) > 0); //if there is no data
}

can_driver canDriver;