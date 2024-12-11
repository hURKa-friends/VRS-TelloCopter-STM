/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00201D2B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */
  LL_I2C_DisableAutoEndMode(I2C1);
  LL_I2C_Enable(I2C1);
  /* USER CODE END I2C1_Init 2 */

}

/* USER CODE BEGIN 1 */
/**
  * @brief  Waits for the TC (Transfer Complete) flag to be set on the specified
  *         I2C peripheral, indicating that the current transfer is complete.
  * @param  I2Cx: pointer to the I2C peripheral (of type I2C_TypeDef) to monitor for the TC flag.
  * @retval I2C_ERR_NONE 	if transfer completes successfully (TC flag set),
  *         I2C_ERR_BERR 	if a bus error is detected,
  *         I2C_ERR_ARLO 	if an arbitration lost error occurs,
  *         I2C_ERR_NACKF 	if a NACK was received during the transfer,
  *         I2C_ERR_TIMEOUT if the timeout is exceeded without TC flag being set.
  */
uint8_t I2C_TC_Wait(const I2C_TypeDef *I2Cx)
{
	uint32_t timeout = 0;
	uint8_t status = I2C_ERR_NONE;
	while(!LL_I2C_IsActiveFlag_TC(I2Cx))
	{
		if (timeout > I2C_US_TIMEOUT)
		{
			status = I2C_ERR_TIMEOUT;
			break;
		}
		if (LL_I2C_IsActiveFlag_BERR(I2Cx))
		{
			status = I2C_ERR_BERR;
			break;
		}
		if (LL_I2C_IsActiveFlag_ARLO(I2Cx))
		{
			status = I2C_ERR_ARLO;
			break;
		}
		if (LL_I2C_IsActiveFlag_NACK(I2Cx))
		{
			status = I2C_ERR_NACKF;
			break;
		}
		timeout++;
	}

	if(status != I2C_ERR_NONE)
		I2C_GenerateStop((I2C_TypeDef *)I2Cx);

	return status;
}

/**
  * @brief  Waits for the TXE (Transmit Empty) flag to be set on the specified I2C peripheral,
  * 		indicating that the transmit buffer is empty and ready for new data transmission.
  * @param  I2Cx: pointer to the I2C peripheral (of type I2C_TypeDef) to monitor for the TXE flag.
  * @retval I2C_ERR_NONE 	if transfer completes successfully (TC flag set),
  *         I2C_ERR_BERR 	if a bus error is detected,
  *         I2C_ERR_ARLO 	if an arbitration lost error occurs,
  *         I2C_ERR_NACKF 	if a NACK was received during the transfer,
  *         I2C_ERR_TIMEOUT if the timeout is exceeded without TXE flag being set.
  */
uint8_t I2C_TXE_Wait(const I2C_TypeDef *I2Cx)
{
	uint32_t timeout = 0;
	uint8_t status = I2C_ERR_NONE;
	while(!LL_I2C_IsActiveFlag_TXE(I2Cx))
	{
		timeout++;
		if (timeout > I2C_US_TIMEOUT)
		{
			status = I2C_ERR_TIMEOUT;
			break;
		}
		if (LL_I2C_IsActiveFlag_BERR(I2Cx))
		{
			status = I2C_ERR_BERR;
			break;
		}
		if (LL_I2C_IsActiveFlag_ARLO(I2Cx))
		{
			status = I2C_ERR_ARLO;
			break;
		}
		if (LL_I2C_IsActiveFlag_NACK(I2Cx))
		{
			status = I2C_ERR_NACKF;
			break;
		}
	}

	if(status != I2C_ERR_NONE)
		I2C_GenerateStop((I2C_TypeDef *)I2Cx);

	return status;
}

/**
  * @brief  Waits for the RXNE (Receive FIFO Not Empty) flag to be set on the specified
  *         I2C peripheral, indicating that data has been received and is ready for reading.
  * @param  I2Cx: pointer to the I2C peripheral (of type I2C_TypeDef) to monitor for the RXNE flag.
  * @retval I2C_ERR_NONE 	if transfer completes successfully (TC flag set),
  *         I2C_ERR_BERR 	if a bus error is detected,
  *         I2C_ERR_ARLO 	if an arbitration lost error occurs,
  *         I2C_ERR_NACKF 	if a NACK was received during the transfer,
  *         I2C_ERR_TIMEOUT if the timeout is exceeded without RXNE flag being set.
  */
uint8_t I2C_RXNE_Wait(const I2C_TypeDef *I2Cx)
{
	uint32_t timeout = 0;
	uint8_t status = I2C_ERR_NONE;
	while(!LL_I2C_IsActiveFlag_RXNE(I2Cx))
	{
		timeout++;
		if (timeout > I2C_US_TIMEOUT)
		{
			status = I2C_ERR_TIMEOUT;
			break;
		}
		if (LL_I2C_IsActiveFlag_BERR(I2Cx))
		{
			status = I2C_ERR_BERR;
			break;
		}
		if (LL_I2C_IsActiveFlag_ARLO(I2Cx))
		{
			status = I2C_ERR_ARLO;
			break;
		}
		if (LL_I2C_IsActiveFlag_NACK(I2Cx))
		{
			status = I2C_ERR_NACKF;
			break;
		}
	}

	if(status != I2C_ERR_NONE)
		I2C_GenerateStop((I2C_TypeDef *)I2Cx);

	return status;
}

/**
  * @brief  Generates a STOP condition on the specified I2C peripheral.
  * @param  I2Cx: pointer to the I2C peripheral (of type I2C_TypeDef) on which
  *         the STOP condition is generated.
  * @retval I2C_ERR_NONE 	if transfer completes successfully (TC flag set),
  *         I2C_ERR_BERR 	if a bus error is detected,
  *         I2C_ERR_ARLO 	if an arbitration lost error occurs,
  *         I2C_ERR_NACKF 	if a NACK was received during the transfer.
  */
uint8_t I2C_GenerateStop(const I2C_TypeDef *I2Cx)
{
	uint8_t status = I2C_ERR_NONE;
	LL_I2C_GenerateStopCondition((I2C_TypeDef *)I2Cx);
	while (!LL_I2C_IsActiveFlag_STOP(I2Cx))
	{
		if (LL_I2C_IsActiveFlag_BERR(I2Cx))
		{
			status = I2C_ERR_BERR;
			break;
		}
		if (LL_I2C_IsActiveFlag_ARLO(I2Cx))
		{
			status = I2C_ERR_ARLO;
			break;
		}
		if (LL_I2C_IsActiveFlag_NACK(I2Cx))
		{
			status = I2C_ERR_NACKF;
			break;
		}
	}
	LL_I2C_ClearFlag_STOP(I2C1);
	return status;
}

/**
* @brief  Reads multiple bytes of data from a specified register on an I2C slave device.
  *         The function initiates a multi-byte read operation by first sending the register address,
  *         followed by a repeated start condition to switch to read mode, and then receiving the
  *         requested number of bytes.
  * @param  slave_address: 		The 7-bit address of the I2C slave device to communicate with.
  * @param  register_address: 	The register address within the slave device to read from.
  *                           	If the transfer involves multiple bytes, the MSB of this address is modified.
  * @param  i2c_rx_data: 		Pointer to the buffer where the received data will be stored.
  * @param  data_count: 		The number of bytes to be read from the slave device.
  * @retval I2C_ERR_NONE 		if transfer completes successfully,
  *         I2C_ERR_BERR 		if a bus error is detected,
  *         I2C_ERR_ARLO 		if an arbitration lost error occurs,
  *         I2C_ERR_NACKF 		if a NACK was received during the transfer,
  *         I2C_ERR_TIMEOUT 	if the timeout is exceeded.
  */
uint8_t I2C1_MultiByteRead(uint8_t slave_address, uint8_t register_address, uint8_t* i2c_rx_data, uint8_t data_count)
{
	uint8_t status = I2C_ERR_NONE;
	if(data_count > 1) // For multi-byte transfer modify REGISTER ADDRESS MSB to 1
		register_address |= 0x80;

	// Step 1: Initiate communication with START bit, SLAVE ADDRESS, WRITE bit
	LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
	if((status = I2C_TXE_Wait(I2C1)))
		return status;

	// Step 2: Transmit the REGISTER ADDRESS
	LL_I2C_TransmitData8(I2C1, register_address);
	if((status = I2C_TC_Wait(I2C1)))
		return status;

	// Step 3: Send REPEATED START with READ bit request
	LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, data_count, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_READ);

	// Step 4: Read DATA COUNT of data bytes
	for (int i = 0; i < data_count; i++)
	{
		if((status = I2C_RXNE_Wait(I2C1))) // Wait until data is available
			return status;
		i2c_rx_data[i] = LL_I2C_ReceiveData8(I2C1);
	}

	// Step 5: Generate STOP condition after all data is received
	I2C_GenerateStop(I2C1);
	return status;
}

/**
  * @brief  Performs a multi-byte write operation to an I2C slave device.
  *         This function writes a register address followed by a specified number of data bytes to the slave.
  *         It handles the preparation of the register address (modifying MSB if needed for multi-byte transfers),
  *         initiates the communication, and ensures that the data is transmitted successfully.
  * @param  slave_address: 		7-bit I2C slave address of the target device.
  * @param  register_address: 	Address of the register to write to, with the MSB modified for multi-byte transfers.
  * @param  i2c_tx_data: 		Pointer to the array of data bytes to transmit.
  * @param  data_count: 		Number of data bytes to transmit after the register address.
  * @retval I2C_ERR_NONE 		if transfer completes successfully,
  *         I2C_ERR_BERR 		if a bus error is detected,
  *         I2C_ERR_ARLO 		if an arbitration lost error occurs,
  *         I2C_ERR_NACKF 		if a NACK was received during the transfer,
  *         I2C_ERR_TIMEOUT 	if the timeout is exceeded.
  */
uint8_t I2C1_MultiByteWrite(uint8_t slave_address, uint8_t register_address, uint8_t* i2c_tx_data, uint8_t data_count)
{
	uint8_t status = I2C_ERR_NONE;
	if(data_count > 1) // For multi-byte transfer modify REGISTER ADDRESS MSB to 1
		register_address |= 0x80;

	// Step 1: Initiate communication with START bit, SLAVE ADDRESS, WRITE bit
	LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, data_count + 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
	if((status = I2C_TXE_Wait(I2C1)))
		return status;

	// Step 2: Transmit the REGISTER ADDRESS
	LL_I2C_TransmitData8(I2C1, register_address);

	// Step 3: Send DATA COUNT of data bytes
	for (int i = 0; i < data_count; i++)
	{
		// Wait until TXIS flag is set.
		if((status = I2C_TXE_Wait(I2C1))) // Wait until TX buffer is empty
			return status;
		LL_I2C_TransmitData8(I2C1, i2c_tx_data[i]);
	}

	if((status = I2C_TC_Wait(I2C1))) // Wait to confirm all data is sent before generating STOP.
		return status;

	// Step 4: Generate STOP condition after all data is sent
	I2C_GenerateStop(I2C1);
	return status;
}
/* USER CODE END 1 */
