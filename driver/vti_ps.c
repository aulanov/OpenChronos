// *************************************************************************************************
//
//	Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/ 
//	 
//	 
//	  Redistribution and use in source and binary forms, with or without 
//	  modification, are permitted provided that the following conditions 
//	  are met:
//	
//	    Redistributions of source code must retain the above copyright 
//	    notice, this list of conditions and the following disclaimer.
//	 
//	    Redistributions in binary form must reproduce the above copyright
//	    notice, this list of conditions and the following disclaimer in the 
//	    documentation and/or other materials provided with the   
//	    distribution.
//	 
//	    Neither the name of Texas Instruments Incorporated nor the names of
//	    its contributors may be used to endorse or promote products derived
//	    from this software without specific prior written permission.
//	
//	  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//	  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//	  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//	  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//	  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//	  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//	  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//	  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//	  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//	  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************
// VTI SCP1000-D0x pressure sensor driver functions
// *************************************************************************************************


// *************************************************************************************************
// Include section

// system
#include "project.h"

// driver
#include "vti_ps.h"
#include "timer.h"
#ifdef FIXEDPOINT
#include "dsp.h"
#endif


// *************************************************************************************************
// Prototypes section
u16 ps_read_register(u8 address, u8 mode);
u8 ps_write_register(u8 address, u8 data);
u8 ps_twi_read(u8 ack);
void twi_delay(void);


// *************************************************************************************************
// Defines section


// *************************************************************************************************
// Global Variable section

// Global flag for proper pressure sensor operation
u8 ps_ok;


// *************************************************************************************************
// Extern section



// *************************************************************************************************
// @fn          ps_init
// @brief       Init pressure sensor I/O
// @param       none
// @return      none
// *************************************************************************************************
void ps_init(void)
{
	volatile u8 success, status, eeprom, timeout;
	
	PS_INT_DIR &= ~PS_INT_PIN;            	// DRDY is input
	PS_INT_IES &= ~PS_INT_PIN;				// Interrupt on DRDY rising edge
	PS_TWI_OUT |= PS_SCL_PIN + PS_SDA_PIN; 	// SCL and SDA are outputs by default
	PS_TWI_DIR |= PS_SCL_PIN + PS_SDA_PIN; 	// SCL and SDA are outputs by default
	
	// Reset global ps_ok flag
	ps_ok = 0;

	// 100msec delay to allow VDD stabilisation
	Timer0_A4_Delay(CONV_MS_TO_TICKS(100));

	// Reset pressure sensor -> powerdown sensor
	success = ps_write_register(0x06, 0x01);   

	// 100msec delay 
	Timer0_A4_Delay(CONV_MS_TO_TICKS(100));

	// Check if STATUS register BIT0 is cleared
	status = ps_read_register(0x07, PS_TWI_8BIT_ACCESS);
	if (((status & BIT0) == 0) && (status != 0)) 
	{
		// Check EEPROM checksum in DATARD8 register 
		eeprom = ps_read_register(0x7F, PS_TWI_8BIT_ACCESS);
		if (eeprom == 0x01) ps_ok = 1;
		else 				ps_ok = 0;
	}
}


// *************************************************************************************************
// @fn          ps_start
// @brief       Init pressure sensor registers and start sampling
// @param       none
// @return      u8		1=Sensor started, 0=Sensor did not start
// *************************************************************************************************
void ps_start(void)
{
	// Start sampling data in ultra low power mode 
	ps_write_register(0x03, 0x0B);  
}



// *************************************************************************************************
// @fn          ps_stop
// @brief       Power down pressure sensor
// @param       none
// @return      none
// *************************************************************************************************
void ps_stop(void)
{
	// Put sensor to standby
	ps_write_register(0x03, 0x00);   
}



// *************************************************************************************************
// @fn          ps_twi_sda
// @brief       Control SDA line
// @param       u8 condition		PS_TWI_SEND_START, PS_TWI_SEND_RESTART, PS_TWI_SEND_STOP
//										PS_TWI_CHECK_ACK
// @return      u8					1=ACK, 0=NACK
// *************************************************************************************************
u8 ps_twi_sda(u8 condition)
{
	u8 sda = 0;
	
	if (condition == PS_TWI_SEND_START)
	{
		PS_TWI_SDA_OUT;			// SDA is output
		PS_TWI_SCL_HI;
		twi_delay();
		PS_TWI_SDA_LO;
		twi_delay();
		PS_TWI_SCL_LO;			// SCL 1-0 transition while SDA=0
		twi_delay();
	}
	else if (condition == PS_TWI_SEND_RESTART)
	{
		PS_TWI_SDA_OUT;			// SDA is output
		PS_TWI_SCL_LO;
		PS_TWI_SDA_HI;
		twi_delay();
		PS_TWI_SCL_HI;
		twi_delay();
		PS_TWI_SDA_LO;
		twi_delay();
		PS_TWI_SCL_LO;
		twi_delay();
	}
	else if (condition == PS_TWI_SEND_STOP)
	{
		PS_TWI_SDA_OUT;			// SDA is output
		PS_TWI_SDA_LO;
		twi_delay();
		PS_TWI_SCL_LO;		
		twi_delay();
		PS_TWI_SCL_HI;			
		twi_delay();
		PS_TWI_SDA_HI;			// SDA 0-1 transition while SCL=1
		twi_delay();
	}
	else if (condition == PS_TWI_CHECK_ACK)
	{
		PS_TWI_SDA_IN;			// SDA is input
		PS_TWI_SCL_LO;		
		twi_delay();
		PS_TWI_SCL_HI;			
		twi_delay();
		sda = PS_TWI_IN & PS_SDA_PIN;
		PS_TWI_SCL_LO;			
	}
	
	// Return value will only be evaluated when checking device ACK
	return (sda == 0);
}



// *************************************************************************************************
// @fn          twi_delay
// @brief       Delay between TWI signal edges. 
// @param       none
// @return      none
// *************************************************************************************************
void twi_delay(void)
{
	asm("	nop");
}


// *************************************************************************************************
// @fn          ps_twi_write
// @brief       Clock out bits through SDA.
// @param       u8 data		Byte to send
// @return      none
// *************************************************************************************************
void ps_twi_write(u8 data)
{
	u8 i, mask;
	
	// Set mask byte to 10000000b
	mask = BIT0<<7;
	
	PS_TWI_SDA_OUT;		// SDA is output
	
	for (i=8; i>0; i--)
	{
		PS_TWI_SCL_LO;	// SCL=0
		if ((data & mask) == mask)
		{
			PS_TWI_SDA_HI; // SDA=1
		}
		else 								
		{
			PS_TWI_SDA_LO; // SDA=0 
		}
		mask = mask >> 1;
		twi_delay();
		PS_TWI_SCL_HI;	// SCL=1
		twi_delay();
	}
	
	PS_TWI_SCL_LO;		// SCL=0
	PS_TWI_SDA_IN;		// SDA is input
} 


// *************************************************************************************************
// @fn          ps_twi_read
// @brief       Read bits from SDA
// @param       u8 ack		1=Send ACK after read, 0=Send NACK after read
// @return      u8			Bits read
// *************************************************************************************************
u8 ps_twi_read(u8 ack)
{
	u8 i;
	u8 data = 0;
	
	PS_TWI_SDA_IN;		// SDA is input
	
	for (i=0; i<8; i++)
	{
		PS_TWI_SCL_LO;			// SCL=0
		twi_delay();
		PS_TWI_SCL_HI;			// SCL=0
		twi_delay();
		
		// Shift captured bits to left 
		data = data << 1; 

		// Capture new bit
		if ((PS_TWI_IN & PS_SDA_PIN) == PS_SDA_PIN) data |= BIT0; 
	}

	PS_TWI_SDA_OUT;			// SDA is output
	
	// 1 aditional clock phase to generate master ACK
	PS_TWI_SCL_LO;			// SCL=0
	if (ack == 1)	PS_TWI_SDA_LO		// Send ack -> continue read
	else			PS_TWI_SDA_HI		// Send nack -> stop read
	twi_delay();
	PS_TWI_SCL_HI;			// SCL=0
	twi_delay();
	PS_TWI_SCL_LO;
	
	return (data);
} 



// *************************************************************************************************
// @fn          as_write_register
// @brief  		Write a byte to the pressure sensor
// @param       u8 address			Register address
//				u8 data			Data to write
// @return      u8					
// *************************************************************************************************
u8 ps_write_register(u8 address, u8 data)
{
  volatile u8 success;

  ps_twi_sda(PS_TWI_SEND_START);			// Generate start condition
  
  ps_twi_write((0x11<<1) | PS_TWI_WRITE); 	// Send 7bit device address 0x11 + write bit '0'
  success = ps_twi_sda(PS_TWI_CHECK_ACK);	// Check ACK from device
  if (!success) return (0);
  
  ps_twi_write(address);					// Send 8bit register address
  success = ps_twi_sda(PS_TWI_CHECK_ACK);	// Check ACK from device
  if (!success) return (0);
	
  ps_twi_write(data);						// Send 8bit data to register
  success = ps_twi_sda(PS_TWI_CHECK_ACK);	// Check ACK from device
 // Slave does not send this ACK
 // if (!success) return (0);
  
  ps_twi_sda(PS_TWI_SEND_STOP);				// Generate stop condition

  return (1);
}


// *************************************************************************************************
// @fn          ps_read_register
// @brief       Read a byte from the pressure sensor
// @param       u8 address		Register address
//				u8	mode		PS_TWI_8BIT_ACCESS, PS_TWI_16BIT_ACCESS
// @return      u16			Register content
// *************************************************************************************************
u16 ps_read_register(u8 address, u8 mode)
{
  u8 success;
  u16 data = 0;

  ps_twi_sda(PS_TWI_SEND_START);			// Generate start condition

  ps_twi_write((0x11<<1) | PS_TWI_WRITE); 	// Send 7bit device address 0x11 + write bit '0'
  success = ps_twi_sda(PS_TWI_CHECK_ACK);	// Check ACK from device
  if (!success) return (0);
  
  ps_twi_write(address);					// Send 8bit register address
  success = ps_twi_sda(PS_TWI_CHECK_ACK);	// Check ACK from device
  if (!success) return (0);

  ps_twi_sda(PS_TWI_SEND_RESTART);			// Generate restart condition

  ps_twi_write((0x11<<1) | PS_TWI_READ); 	// Send 7bit device address 0x11 + read bit '1'
  success = ps_twi_sda(PS_TWI_CHECK_ACK);	// Check ACK from device
  if (!success) return (0);
	
  if (mode == PS_TWI_16BIT_ACCESS)
  {
	  data =  ps_twi_read(1) << 8;			// Read MSB 8bit data from register
	  data |= ps_twi_read(0);				// Read LSB 8bit data from register
  }
  else
  {
	  data = ps_twi_read(0);				// Read 8bit data from register
  }  
  
  ps_twi_sda(PS_TWI_SEND_STOP);				// Generate stop condition

  return (data);
}



// *************************************************************************************************
// @fn          ps_get_pa
// @brief       Read out pressure. Format is Pa. Range is 30000 .. 120000 Pa.
// @param       none
// @return      u32		15-bit pressure sensor value (Pa)
// *************************************************************************************************
u32 ps_get_pa(void)
{
	volatile u32 data = 0;
	
	// Get 3 MSB from DATARD8 register
	data = ps_read_register(0x7F, PS_TWI_8BIT_ACCESS);
	data = ((data & 0x07) << 8) << 8;

	// Get 16 LSB from DATARD16 register
	data |= ps_read_register(0x80, PS_TWI_16BIT_ACCESS);
	
	// Convert decimal value to Pa
	data = (data >> 2);
	
	return (data);
}


// *************************************************************************************************
// @fn          ps_get_temp
// @brief       Read out temperature.
// @param       none
// @return      u16		13-bit temperature value in xx.x°K format
// *************************************************************************************************
#ifndef NO_ALTI
u16 ps_get_temp(void)
{
	volatile u16 data = 0;
	u16 temp = 0;
	u8 is_negative = 0;
	u16 kelvin;
	
	// Get 13 bit from TEMPOUT register
	data = ps_read_register(0x81, PS_TWI_16BIT_ACCESS);
	
	// Convert negative temperatures
	if ((data & BIT(13)) == BIT(13)) 
	{
		// Sign extend temperature
		data |= 0xC000;
		// Convert two's complement
		data = ~data;
		data += 1;
		is_negative = 1;
	}

	temp = data / 2;
	
	// Convert from °C to °K
	if (is_negative)	kelvin = 2732 - temp;
	else				kelvin = temp + 2732;
	
	return (kelvin);
}
#endif
