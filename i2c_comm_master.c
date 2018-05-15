/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////

// Standard C Included Files
#include <string.h>
#include <math.h>
#include <stdio.h>

// SDK Included Files
#include "board.h"
#include "fsl_i2c_master_driver.h"

//from oled.c/////////////////////////////////////////
#include "fsl_spi_master_driver.h"
#include "fsl_clock_manager.h"

/*
 *	Borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
 */
#define SSD1331_COLORORDER_RGB
#define SSD1331_DELAYS_HWFILL		(3)
#define SSD1331_DELAYS_HWLINE		(1)

/*
 *
 */
#define SSD1331_CMD_DRAWLINE 		0x21
#define SSD1331_CMD_DRAWRECT 		0x22
#define SSD1331_CMD_FILL 		0x26
#define SSD1331_CMD_SETCOLUMN 		0x15
#define SSD1331_CMD_SETROW    		0x75
#define SSD1331_CMD_CONTRASTA 		0x81
#define SSD1331_CMD_CONTRASTB 		0x82
#define SSD1331_CMD_CONTRASTC		0x83
#define SSD1331_CMD_MASTERCURRENT 	0x87
#define SSD1331_CMD_SETREMAP 		0xA0
#define SSD1331_CMD_STARTLINE 		0xA1
#define SSD1331_CMD_DISPLAYOFFSET 	0xA2
#define SSD1331_CMD_NORMALDISPLAY 	0xA4
#define SSD1331_CMD_DISPLAYALLON  	0xA5
#define SSD1331_CMD_DISPLAYALLOFF 	0xA6
#define SSD1331_CMD_INVERTDISPLAY 	0xA7
#define SSD1331_CMD_SETMULTIPLEX  	0xA8
#define SSD1331_CMD_SETMASTER 		0xAD
#define SSD1331_CMD_DISPLAYOFF 		0xAE
#define SSD1331_CMD_DISPLAYON     	0xAF
#define SSD1331_CMD_POWERMODE 		0xB0
#define SSD1331_CMD_PRECHARGE 		0xB1
#define SSD1331_CMD_CLOCKDIV 		0xB3
#define SSD1331_CMD_PRECHARGEA 		0x8A
#define SSD1331_CMD_PRECHARGEB 		0x8B
#define SSD1331_CMD_PRECHARGEC 		0x8C
#define SSD1331_CMD_PRECHARGELEVEL 	0xBB
#define SSD1331_CMD_VCOMH 		0xBE


volatile spi_master_state_t		spiMasterState;
volatile spi_master_user_config_t	spiUserConfig;

volatile uint8_t inBuffer[32];
volatile uint8_t payloadBytes[32];

enum _subaddress_index_e
{
    Subaddress_Index_0 = 0x00,
    Subaddress_Index_1 = 0x01,
    Subaddress_Index_2 = 0x02,
    Subaddress_Index_3 = 0x03,
    Subaddress_Index_4 = 0x04,
    Subaddress_Index_5 = 0x05,
    Subaddress_Index_6 = 0x06,
    Subaddress_Index_7 = 0x07,
    Invalid_Subaddress_Index = 50,	//PSM: set to 50==0x32 so we get all the MMA8451Q regs; was implicitly 0x8 in original KSDK example,
    Max_Subaddress_Index
};


/////////////////////////// functions ////////////////////////////////////////////////////////
void
enableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);


	/*	KL03_SPI_MISO	--> PTA6	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	KL03_SPI_MOSI	--> PTA8	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAlt3);

	/*	KL03_SPI_SCK	--> PTA9	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAlt3);


	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 *
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= 50000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
	printf("Calculated baud rate is %ld\n", calculatedBaudRate);
}

//Pre-existing function to write to registers
void
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	/CS (PTA12) low
	 */
	GPIO_DRV_ClearPinOutput(kGpioOC);

	/*
	 *	DC (PTB13) low
	 */
	GPIO_DRV_ClearPinOutput(kGpioDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	GPIO_DRV_SetPinOutput(kGpioOC);

	return;
}

//function to draw a rectangle of given start and end points and colour
void
drawRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t r, uint8_t g, uint8_t b)
{
	writeCommand(0x26);
	writeCommand(0x01);
	writeCommand(0x22);
	writeCommand(x0 & 0xFF);
	writeCommand(y0 & 0xFF);
	writeCommand(x1 & 0xFF);
	writeCommand(y1 & 0xFF);
	writeCommand(r & 0xFF);
	writeCommand(g & 0xFF);
	writeCommand(b & 0xFF);
	writeCommand(r & 0xFF);
	writeCommand(g & 0xFF);
	writeCommand(b & 0xFF);

	OSA_TimeDelay(10);
}


//toggle LED
static void LED_toggle_master(void)
{
    GPIO_DRV_TogglePinOutput(kGpioLED1);
}


//take the data from the registers and put into meaningful x, y and z data based on using in 8g mode
int data_to_decimal(uint16_t data, char sign)
{
	uint16_t integer, decimal, i, n;
   	uint16_t frac[13] ={                   //value of each bit taken from application notes for 8g 
			4,
			2,
			1,
			5000,
			2500,
			1250,
			625,
			313,
			156,
			78,
			39,
			20,
			10
			     } ;
            //determine iteger (by determining the value of each of the first three bits)
	    integer = 0;
	    for(n=0; n<3; n++)
	    {
		data = data <<1;
		if (data > 0x7FFF)
		{    integer = integer + frac[n];
                }
	    }
           //determine decimal	    
	    decimal = 0;
	    for(i=3; i<13; i++)
	    {
		data = data <<1;
		if (data > 0x7FFF)
		{    decimal = decimal + frac[i];
                }
	    }
	    return integer*10000+decimal;  //returns meaningful integer value
}

//write to a given register
void write(uint8_t reg, uint8_t data)
{
    uint8_t cmdBuff[1] = {0xFF};
    uint8_t sendBuff[1] = {0xFF};       // save data sent to i2c slave
    i2c_master_state_t master;
    i2c_status_t returnValue;

    i2c_device_t slave =
    {
        .address = 0x1D,
        .baudRate_kbps = 100
    };

        cmdBuff[0]  = reg;
        sendBuff[0] = data;

        returnValue = I2C_DRV_MasterSendDataBlocking(
                                                    BOARD_I2C_COMM_INSTANCE,
                                                    &slave,
                                                    cmdBuff,
                                                    1,
                                                    sendBuff,
                                                    sizeof(sendBuff),
                                                    500);
        if (returnValue != kStatus_I2C_Success)
        {
            printf("\r\nI2C communication failed, error code: %d", returnValue);
        }
}

//read from a given register
uint8_t read(uint8_t reg)
{
    uint8_t cmdBuff[1] = {0xFF};
    uint8_t receiveBuff[1] = {0xFF};    // save data received from i2c slave
    i2c_master_state_t master;
    i2c_status_t returnValue;

    i2c_device_t slave =
    {
        .address = 0x1D,
        .baudRate_kbps = 100
    };

    cmdBuff[0] = reg;
    returnValue = I2C_DRV_MasterReceiveDataBlocking(
	                                       BOARD_I2C_COMM_INSTANCE,
	                                       &slave,
	                                       cmdBuff,
	                                       1,
	                                       receiveBuff,
	                                       sizeof(receiveBuff),
	                                       500);
    if (returnValue == kStatus_I2C_Success)
    {
    return receiveBuff[0];
    }
    else
    {
	printf("\r\nI2C communication failed, error code: %d", returnValue);
    } 
    

}

//////////////////////////////////////////////main///////////////////////////////////////////////

int main(void)
{
//setup//////////////////////////

//values to be set
    uint16_t time_limit = 30, reset_delay = 5, cycles = 15;


    uint8_t oncount = 0;
    uint16_t i = 0, LED = 0, timer_divide = 0, timer = 0, offtimer=0, size;
    uint16_t xdata, ydata, zdata, xinteger,yinteger, zinteger, value;
    char xsign, ysign, zsign;
    i2c_master_state_t master;


    i2c_device_t slave =
    {
        .address = 0x1D,
        .baudRate_kbps = 100
    };

    hardware_init();
    dbg_uart_init();

    // Configure I2C pins
    configure_i2c_pins(BOARD_I2C_COMM_INSTANCE);    
    OSA_Init();
    GPIO_DRV_Init(0, ledPins);

    // Init I2C module
    I2C_DRV_MasterInit(BOARD_I2C_COMM_INSTANCE, &master);

/////////////////////////based on oled.c//////////////////////////////////////////////

    // Setup SPI pins
    PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3); // MOSI
    PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3); // SCK

    // Setup GPIO pins for OLED
    PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio); // OCS
    PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio); // DC
    PORT_HAL_SetMuxMode(PORTA_BASE, 2u, kPortMuxAsGpio); // RST

    enableSPIpins();


	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(SSD1331_CMD_DISPLAYOFF);		// 0xAE
	writeCommand(SSD1331_CMD_SETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(SSD1331_CMD_STARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(SSD1331_CMD_DISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(SSD1331_CMD_NORMALDISPLAY);	// 0xA4
	writeCommand(SSD1331_CMD_SETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(SSD1331_CMD_SETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(SSD1331_CMD_POWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(SSD1331_CMD_PRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(SSD1331_CMD_CLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(SSD1331_CMD_PRECHARGEA);		// 0x8A
	writeCommand(0x64);
	writeCommand(SSD1331_CMD_PRECHARGEB);		// 0x8B
	writeCommand(0x78);
	writeCommand(SSD1331_CMD_PRECHARGEA);		// 0x8C
	writeCommand(0x64);
	writeCommand(SSD1331_CMD_PRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(SSD1331_CMD_VCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(SSD1331_CMD_MASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(SSD1331_CMD_CONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(SSD1331_CMD_CONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(SSD1331_CMD_CONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(SSD1331_CMD_DISPLAYON);		//--turn on oled panel    


	drawRectangle(0, 0, 95, 63 , 0, 32, 0);


//////////////////////////////////////////////////////////////////////////////////////

    OSA_TimeDelay(500);
    LED_toggle_master();
    OSA_TimeDelay(500);
    LED_toggle_master();
    OSA_TimeDelay(500);
    LED_toggle_master();
    OSA_TimeDelay(500);
    LED_toggle_master();
    GPIO_DRV_WritePinOutput(kGpioLED1, 1);
    LED= 0;

//Start of main code////////////////////

    write(0x2A, 0x00);  //Set to standby mode
    write(0x0E, 0x02);  //Set to 8g
    write(0x2A, 0x01);  //Set to active mode

    printf("\r\n  x integer  yinteger  z integer     Value   Timer ");
    printf("==========================================================");

//    for (n=0; n<1000; n++)
    while(1)
    {
	//Get x data
	    xdata = (read(0x01) <<8)|read(0x02);  //takes values from two 8 bit registers to give one 16 bit variable
	//determine if data is postive or negative, and if negative convert to positive equivelent since only interested in value (2's compliment). (record if positive or negative as my be useful later)
	    if (xdata > 0x7FFF)                   
	    {
		xsign = '-';
		xdata = ~xdata +1;          
	    }
	    else 
	    {
		xsign = '+';
	    } 
	    xinteger = data_to_decimal(xdata, xsign); 

	//Get y data
	    ydata = (read(0x03) <<8)|read(0x04);
	    if (ydata > 0x7FFF)
	    {
		ysign = '-';
		ydata = ~ydata +1;
	    }
	    else 
	    {
		ysign = '+';
	    } 
	    yinteger = data_to_decimal(ydata, ysign);

	//Get z data
	    zdata = (read(0x05) <<8)|read(0x06);
	    if (zdata > 0x7FFF)
	    {
		zsign = '-';
		zdata = ~zdata +1;
	    }
	    else 
	    {
		zsign = '+';
	    } 
	    zinteger = data_to_decimal(zdata, zsign); 

	//Get total value    
	    value = sqrt(xinteger*xinteger + yinteger*yinteger + zinteger*zinteger);

	//Determine when to turn on LED


	    if (value > 20000)
	    {
		oncount = oncount + 1;
	    }

	    i= i+1;
//	    printf(" %d ", i);
	    if (i==10) //checks count every 10 cycles
	    {
		if (oncount > 2)
		{
			GPIO_DRV_WritePinOutput(kGpioLED1, 0);
		   	LED = 30000;
			offtimer = 0;
			//offtimer = 0;
		}	
		if (oncount == 0)
		{
			GPIO_DRV_WritePinOutput(kGpioLED1, 1);
		   	LED = 0;
			//offtimer = 0;
		}
	    i=0;
	    oncount = 0;
	    }


	    
	//Timers

	   if (LED == 30000)
	   {
		timer_divide = timer_divide+1;
		if (timer_divide == cycles)  //cycles= no of cycles in a second
		{
			timer_divide = 0;
			timer = timer +1; 
			size = (timer*95)/time_limit;                      //95 being the value required to fill the whole screen
			drawRectangle(0, 0, size, 63, 32, 0, 0);
		}
	   }

 	if (LED == 0)
	{
	offtimer = offtimer+1;  

	   if(offtimer == reset_delay*cycles)        //restart if timer has been off for a certain amount of time (should be set as variable)
	   {
		timer= 0;
		drawRectangle(0, 0, size, 63, 0, 32, 0);
		offtimer = 0;
	   }
	}

	    //if timer== time_limit flash red and green until button is pressed and then return timer to zero
	   if (timer == time_limit)
	   {
		while(1)
		{
		    drawRectangle(0, 0, size, 63, 32, 0, 0);
		    OSA_TimeDelay(500);
		    drawRectangle(0, 0, size, 63, 0, 32, 0);
		    OSA_TimeDelay(500);
		}
	    }
			    
    	    printf("\r\n     %d         %d        %d,        %d,    %d    ", xinteger, yinteger, zinteger, value, timer);
    }
      printf("\r\n Timer: %d", timer);


}








