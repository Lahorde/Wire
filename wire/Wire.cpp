/******************************************************************************
 * @file    Wire.cpp
 * @author  Rémi Pincent - INRIA
 * @date    05 feb. 2015
 *
 * @brief Wire wrapping I2C. It uses hardware twi. Source code modified
 * from nrf51_sdk_path/nrf51822/Source/twi_hw_master.c and Arduino Wire
 * library
 *
 * Project : Wire
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * TODO_revision history
 *****************************************************************************/

#include "Wire.h"
extern "C"{
#include "app_error.h"
}
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "inter_periph_com.h"



TwoWire::TwoWire(NRF_TWI_Type *twi) : 
							_rxBufferIndex(0),
							_rxBufferLength(0),
							_txBufferLength(0),
							_p_twi(twi),
							_transferAddress(0),
							_sdaPin(INVALID_PIN),
							_sclPin(INVALID_PIN),
							_twiStatus(UNINITIALIZED)
{
	//do nothing
}

void TwoWire::begin(uint32_t scl, uint32_t sda, EWireSpeed speed)
{	
	setSpeed(speed);
	_sclPin = arduinoToVariantPin(scl);
	_sdaPin = arduinoToVariantPin(sda);
	bool initialized = twiMasterInit();

	if(initialized)
	{
		_twiStatus = MASTER_IDLE;
	}
	else
	{
		APP_ERROR_CHECK_BOOL(initialized);
	}
}

void TwoWire::beginTransmission( uint8_t address )
{
	_transferAddress = address;
	_twiStatus = MASTER_SEND;
}

void TwoWire::beginTransmission( int address )
{
	beginTransmission( (uint8_t)address );
}

uint8_t TwoWire::endTransmission( uint8_t stop)
{
	uint8_t twi_flag = 0;

	if(_txBufferLength > 0 && twiMasterClearBus() )
	{
		_p_twi->ADDRESS = _transferAddress;
		twi_flag = twiMasterWrite(_txBuffer, _txBufferLength, stop);
	}

	_txBufferLength = 0;
	_twiStatus = MASTER_IDLE;

	return twi_flag;
}

uint8_t TwoWire::endTransmission(void)
{
	uint8_t twi_flag;

	twi_flag = endTransmission(1);

	return twi_flag;
}

void TwoWire::setSpeed(EWireSpeed speed)
{
	uint32_t twiFrequency;
	if( speed == FREQ_100KHZ)
	{
		twiFrequency = TWI_FREQUENCY_FREQUENCY_K100;
	}
	else if( speed == FREQ_250KHZ )
	{
		twiFrequency = TWI_FREQUENCY_FREQUENCY_K250;
	}
	else if( speed == FREQ_400KHZ )
	{
		twiFrequency = TWI_FREQUENCY_FREQUENCY_K400;
	}
	else
	{
		APP_ERROR_CHECK_BOOL(false);
	}
	_p_twi->FREQUENCY = twiFrequency;
}

size_t TwoWire::write(uint8_t data)
{
	if(_twiStatus == MASTER_SEND)
	{	
		if(_txBufferLength >= BUFFER_LENGTH)
		{
			return 0;
		}
		_txBuffer[_txBufferLength++] = data;
		return 1;
	}
	else
	{
		return 0;
	}
}

size_t TwoWire::write(const uint8_t *data, size_t quantity )
{
	if( _twiStatus == MASTER_SEND )
	{
		for( size_t i=0; i<quantity; ++i )
		{	
			if(_txBufferLength >= BUFFER_LENGTH)
			{
				return i;
			}
			_txBuffer[_txBufferLength++] = data[i];
		}
	}
	else
	{
		return 0;
	}

	return quantity;
}

uint8_t TwoWire::requestFrom(uint8_t addr, uint8_t quantity, uint8_t stop)
{
	uint8_t read_num = 0;

	if(quantity > BUFFER_LENGTH)
	{   
		quantity = BUFFER_LENGTH;
	}
	if(quantity > 0 && twiMasterClearBus() )
	{   
		_p_twi->ADDRESS = addr;
		read_num = twiMasterRead(_rxBuffer, quantity, stop);
	}
	_rxBufferIndex = 0;
	_rxBufferLength = quantity;

	return read_num;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity) 
{
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) 
{
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) sendStop);
}

int TwoWire::available(void)
{
	if(_rxBufferIndex <= _rxBufferLength)
	{
		return (_rxBufferLength - _rxBufferIndex);
	}
	else
	{
		return 0;
	}
}

int TwoWire::read(void)
{
	if(_rxBufferIndex < _rxBufferLength)
	{
		return _rxBuffer[_rxBufferIndex++];
	}
	else
	{
		return -1;
	}
}

int TwoWire::peek(void)
{
	if(_rxBufferIndex < _rxBufferLength)
	{
		return _rxBuffer[_rxBufferIndex];
	}
	else
	{
		return -1;
	}
}

void TwoWire::flush(void)
{
	//do nothing
}

bool TwoWire::twiMasterInit(void)
{
	/* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is
       disabled, these pins must be configured in the GPIO peripheral.
	 */
	NRF_GPIO->PIN_CNF[_sclPin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
										  | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
										  | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
										  | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
										  | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->PIN_CNF[_sdaPin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
										  | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
										  | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
										  | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
										  | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);

	_p_twi->EVENTS_RXDREADY = 0;
	_p_twi->EVENTS_TXDSENT  = 0;
	_p_twi->PSELSCL = _sclPin;
	_p_twi->PSELSDA = _sdaPin;
	_p_twi->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

	return twiMasterClearBus();
}

bool TwoWire::twiMasterClearBus(void)
{
	uint32_t twi_state;
	bool     bus_clear;
	uint32_t clk_pin_config;
	uint32_t data_pin_config;

	// Save and disable TWI hardware so software can take control over the pins.
	twi_state = _p_twi->ENABLE;
	_p_twi->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

	clk_pin_config = NRF_GPIO->PIN_CNF[_sclPin];
	NRF_GPIO->PIN_CNF[_sclPin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
										  | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
										  | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
										  | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
										  | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);

	data_pin_config = NRF_GPIO->PIN_CNF[_sdaPin];
	NRF_GPIO->PIN_CNF[_sdaPin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
										  | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
										  | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
										  | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
										  | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);

	setSDA();
	setSCL();
	delayMicroseconds(TWI_DELAY);

	if( (readSCL() == 1) && (readSDA() == 1))
	{
		bus_clear = true;
	}
	else
	{
		uint_fast8_t index;
		bus_clear = false;

		// Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9
		// for slave to respond) to SCL line and wait for SDA come high.
		for( index=18; index--;)
		{
			clrSCL();
			delayMicroseconds(TWI_DELAY);
			setSCL();
			delayMicroseconds(TWI_DELAY);

			if( readSDA() == 1 )
			{
				bus_clear = 1;
				break;
			}

		}
	}

	NRF_GPIO->PIN_CNF[_sclPin] = clk_pin_config;
	NRF_GPIO->PIN_CNF[_sdaPin]  = data_pin_config;

	_p_twi->ENABLE = twi_state;

	return bus_clear;
}

uint8_t TwoWire::twiMasterWrite(uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
	uint32_t timeout = MAX_TIMEOUT_LOOPS;  /* max loops to wait for EVENTS_TXDSENT event*/

	if(data_length == 0)
	{
		/* Return false for requesting data of size 0 */
		return ERROR_UNKNOWN;
	}
	_p_twi->TXD = *data++;
	_p_twi->TASKS_STARTTX = 1;
	while(1)
	{
		while( (_p_twi->EVENTS_TXDSENT == 0) && (--timeout)  && (_p_twi->EVENTS_ERROR == 0) )
		{
			// Do nothing.
		}

		/** NACK received after sending the address (write ‘1’ to clear) */
		if(_p_twi->ERRORSRC &  TWI_ERRORSRC_ANACK_Msk)
		{
			_p_twi->EVENTS_ERROR = 0;
			_p_twi->ERRORSRC |= TWI_ERRORSRC_ANACK_Clear << TWI_ERRORSRC_ANACK_Pos;
			stopTransaction();
			return A_NACK;
		}
		/** NACK received after sending a data byte (write ‘1’ to clear) */
		else if(_p_twi->ERRORSRC & TWI_ERRORSRC_DNACK_Msk)
		{
			_p_twi->EVENTS_ERROR = 0;
			_p_twi->ERRORSRC |= TWI_ERRORSRC_DNACK_Clear << TWI_ERRORSRC_DNACK_Pos;
			stopTransaction();
			return D_NACK;
		}

		if (timeout == 0)
		{
			recoverTwi();
			return ERROR_UNKNOWN;
		}

		_p_twi->EVENTS_TXDSENT = 0;
		if( --data_length == 0)
		{
			break;
		}

		_p_twi->TXD = *data++;
	}
	if(issue_stop_condition)
	{
		stopTransaction();
	}
	return SUCCESS;
}

uint8_t TwoWire::twiMasterRead(uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
	uint8_t twi_ppi_channel;
	uint32_t timeout = MAX_TIMEOUT_LOOPS;
	uint8_t bytes_read = 0;
	volatile void* ppi_task;

	/** RP - 05/02/2015 - refer nrf51 reference manual figure 65 and https://devzone.nordicsemi.com/question/17472/is-it-necessary-to-suspend-twi-for-each-byte-read/ */
	twi_ppi_channel = findFreePPIChannel(UNAVAILABLE_PPI_CHANNEL);

	if(twi_ppi_channel == UNAVAILABLE_PPI_CHANNEL)
	{
		APP_ERROR_CHECK_BOOL(twi_ppi_channel != UNAVAILABLE_PPI_CHANNEL);
		return bytes_read;
	}

	if (data_length == 0)
	{
		/* Return false for requesting data of size 0 */
		return bytes_read;
	}
	else if (data_length == 1 && issue_stop_condition)
	{
		ppi_task = &_p_twi->TASKS_STOP;
	}
	else
	{
		ppi_task = &_p_twi->TASKS_SUSPEND;
	}
	wirePPIChannel(twi_ppi_channel, &_p_twi->EVENTS_BB, ppi_task);

	_p_twi->EVENTS_RXDREADY = 0;
	_p_twi->TASKS_STARTRX = 1;

	while(1)
	{
		while( _p_twi->EVENTS_RXDREADY == 0 && _p_twi->EVENTS_ERROR == 0 && (--timeout))
		{
			//do nothing, just wait
		}
		_p_twi->EVENTS_RXDREADY = 0;

		/** NACK received after sending the address (write ‘1’ to clear) */
		if(_p_twi->ERRORSRC &  TWI_ERRORSRC_ANACK_Msk)
		{
			freePPIChannel(twi_ppi_channel);
			_p_twi->EVENTS_ERROR = 0;
			_p_twi->ERRORSRC |= TWI_ERRORSRC_ANACK_Clear << TWI_ERRORSRC_ANACK_Pos;
			stopTransaction();
			return bytes_read;
		}
		/** NACK received after sending a data byte (write ‘1’ to clear) */
		else if(_p_twi->ERRORSRC & TWI_ERRORSRC_DNACK_Msk)
		{
			freePPIChannel(twi_ppi_channel);
			_p_twi->EVENTS_ERROR = 0;
			_p_twi->ERRORSRC |= TWI_ERRORSRC_DNACK_Clear << TWI_ERRORSRC_DNACK_Pos;
			stopTransaction();
			return bytes_read;
		}

		if( timeout == 0)
		{
			recoverTwi();
			freePPIChannel(twi_ppi_channel);
			return bytes_read;
		}

		*data++ = _p_twi->RXD;
		bytes_read++;

		/* Configure PPI to stop TWI master before we get last BB event */
		if( --data_length == 1 && issue_stop_condition)
		{
			wirePPIChannel(twi_ppi_channel, &_p_twi->EVENTS_BB, &_p_twi->TASKS_STOP);
		}

		if( data_length == 0 )
		{
			break;
		}

		// Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at
		// Product Anomaly Notification document found at
		// https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
		delayMicroseconds(20);
		_p_twi->TASKS_RESUME = 1;
	}

	/* Wait until stop sequence is sent */
	while( _p_twi->EVENTS_STOPPED == 0 )
	{
		//do nothing
	}

	freePPIChannel(twi_ppi_channel);
	return bytes_read;
}

void TwoWire::stopTransaction(void)
{
	/**
	 * From ref manual 27.4
	 * "The TWI master write sequence is stopped when the STOP task is triggered. When the STOP task is triggered,
	 * the TWI master will generate a stop condition on the TWI bus."
	 */
	_p_twi->EVENTS_STOPPED = 0;
	_p_twi->TASKS_STOP     = 1;
	while(_p_twi->EVENTS_STOPPED == 0)
	{
		//do nothing, wait for stop sequence is sent
	}
}

bool TwoWire::recoverTwi(void)
{
	// Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at
	// Product Anomaly Notification document found at
	// https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
	_p_twi->EVENTS_ERROR = 0;
	_p_twi->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
	_p_twi->POWER        = 0;
	delayMicroseconds(5);
	_p_twi->POWER        = 1;
	_p_twi->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

	return twiMasterInit();
}

TwoWire Wire = TwoWire(WIRE_INTERFACE_1);







