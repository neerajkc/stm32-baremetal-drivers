/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 17 Aug 2021
 *      Author: neeraj5k
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;




/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 				100000
#define I2C_SCL_SPEED_FM4K 				400000
#define I2C_SCL_SPEED_FM2K  			200000


/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE        			1
#define I2C_ACK_DISABLE       			0


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2        			0
#define I2C_FM_DUTY_16_9     			1


/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   					( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   				( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB						( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  					( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   					( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 					( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 					( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 					( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 					( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  					( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 					( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 				( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  				RESET
#define I2C_ENABLE_SR   				SET


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);


/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);


/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);





#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
