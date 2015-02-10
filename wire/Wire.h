/******************************************************************************
 * @file    Wire.h
 * @author  Rémi Pincent - INRIA
 * @date    9 févr. 2015
 *
 * @brief I2C library to be used on nrf51822 soc using Arduino core
 * From :
 *   -https://devzone.nordicsemi.com/documentation/nrf51/4.3.0/html/group__twi__master__example.html
 * and from nrf51_sk_path/nrf51822/Source/twi_master/twi_hw_master.c
 *
 * Project : wire
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * TODO_revision history
 *****************************************************************************/

#ifndef _WIRE_H_
#define _WIRE_H_

#include <stdint.h>
#include <stddef.h>
#include "nrf51.h"
#include "delay.h"
#include "Stream.h"
#include "variant.h"

/** Rx buffer lenmax length - use this name in order to use it with I2CDev */
#define BUFF_MAX_LENGTH  32

class TwoWire : public Stream
{
	/** Types */
public :
	typedef enum{
		FREQ_100KHZ,
		FREQ_250KHZ,
		FREQ_400KHZ
	}EWireSpeed;

	typedef enum  {
		UNINITIALIZED,
		MASTER_IDLE,
		MASTER_SEND,
		MASTER_RECV,
		SLAVE_IDLE,
		SLAVE_RECV,
		SLAVE_SEND
	}TwoWireStatus;

	/** Error codes */
public :
	static const uint8_t SUCCESS = 0;
	/** data too long to fit in transmit buffer */
	static const uint8_t DATA_TOO_LONG = 1;
	/** received NACK on transmit of address */
	static const uint8_t A_NACK = 2;
	/** received NACK on transmit of data */
	static const uint8_t D_NACK = 3;
	/** other error  */
	static const uint8_t ERROR_UNKNOWN = 4;

private :
	/* Max cycles approximately to wait on RXDREADY and TXDREADY event,
	 * This is optimized way instead of using timers, this is not power aware. */
	static const uint32_t MAX_TIMEOUT_LOOPS         =   20000;
	/**  Time to wait when pin states are changed. For fast-mode
	 * the delay can be zero and for standard-mode 4 us delay is sufficient.
	 */
	static const uint32_t TWI_DELAY         =   4;

	uint8_t _rxBuffer[BUFF_MAX_LENGTH];
	uint8_t _rxBufferIndex;
	uint8_t _rxBufferLength;

	uint8_t _txBuffer[BUFF_MAX_LENGTH];
	uint8_t _txBufferLength;

	NRF_TWI_Type *_p_twi;

	uint8_t _transferAddress;
	uint32_t _sdaPin;
	uint32_t _sclPin;
	TwoWireStatus _twiStatus;

	/** Methods */
public :
	TwoWire(NRF_TWI_Type *twi);
	virtual ~TwoWire(){};
	void begin(uint32_t scl_pin = PIN_WIRE_SCL, uint32_t sda_pin = PIN_WIRE_SDA, EWireSpeed speed = FREQ_100KHZ);
	void beginTransmission(uint8_t);
	void beginTransmission(int);
	uint8_t endTransmission(void);
	uint8_t endTransmission(uint8_t);
	void setSpeed(EWireSpeed speed);
	uint8_t requestFrom(uint8_t, uint8_t);
	uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
	uint8_t requestFrom(int, int);
	uint8_t requestFrom(int, int, int);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *, size_t);
	virtual int available(void);
	virtual int read(void);
	virtual int peek(void);
	virtual void flush(void);
	bool twi_master_transfer(uint8_t   address,
			uint8_t * data,
			uint8_t   data_length,
			bool      issue_stop_condition);

	inline size_t write(unsigned long n) { return write((uint8_t)n); }
	inline size_t write(long n) { return write((uint8_t)n); }
	inline size_t write(unsigned int n) { return write((uint8_t)n); }
	inline size_t write(int n) { return write((uint8_t)n); }
	using Print::write;

	/** Methods */
private:
	/**
	 * @brief Function for initializing the twi_master.
	 * @return
	 * @retval false initialization failed.
	 * @retval true initialization successful.
	 */
	bool twiMasterInit(void);

	/**
	 * @brief Function for detecting stuck slaves (SDA = 0 and SCL = 1) and tries to clear the bus.
	 *
	 * @return
	 * @retval false Bus is stuck.
	 * @retval true Bus is clear.
	 */
	bool twiMasterClearBus(void);

	/**
	 * Write given bytes to slave
	 * @param data
	 * @param data_length
	 * @param issue_stop_condition
	 * @return SUCCESS or other error codes
	 */
	uint8_t twiMasterWrite(uint8_t *data, uint8_t data_length, bool issue_stop_condition);

	/**
	 * Read some bytes from slave
	 * @param data
	 * @param data_length
	 * @param issue_stop_condition
	 * @return number of bytes read
	 */
	uint8_t twiMasterRead(uint8_t *data, uint8_t data_length, bool issue_stop_condition);

	/**
	 * Stop a read/write transaction
	 */
	void stopTransaction(void);

	/**
	 * Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at
	 * Product Anomaly Notification document found at
	 * https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
	 * @return
	 * @retval false unable to recover twi.
	 * @retval true recovery successful.
	 */
	bool recoverTwi(void);

	inline void clrSDA(void){NRF_GPIO->OUTCLR = ( 1UL << _sdaPin );}
	inline void clrSCL(void){NRF_GPIO->OUTCLR = ( 1UL << _sclPin );}
	inline void setSDA(void){NRF_GPIO->OUTSET = ( 1UL << _sdaPin );}
	inline void setSCL(void){NRF_GPIO->OUTSET = ( 1UL << _sclPin );}
	inline uint32_t readSCL(void){return (NRF_GPIO->IN >> _sclPin) & 0x1UL;}
	inline uint32_t readSDA(void){return (NRF_GPIO->IN >> _sdaPin) & 0x1UL;}

};

extern TwoWire Wire;


#endif

