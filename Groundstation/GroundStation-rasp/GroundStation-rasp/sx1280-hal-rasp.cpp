/*
By Lagoni 12022019
(c) Lagoni 2019
Not for commercial use
*/
#include "sx1280-hal.h"
#include "bcm2835.h"
#include <stdio.h> // printf

/*!
 * \brief Helper macro to avoid duplicating code for setting dio pins parameters
 */
 /*
#if defined( TARGET_NUCLEO_L476RG )
#define DioAssignCallback( dio, pinMode, callback )                    \
            if( dio != NULL )                                          \
            {                                                          \
                dio->mode( pinMode );                                  \
                dio->rise( this, static_cast <Trigger>( callback ) );  \
            }
#else
#define DioAssignCallback( dio, pinMode, callback )                    \
            if( dio != NULL )                                          \
            {                                                          \
                dio->rise( this, static_cast <Trigger>( callback ) );  \
            }
#endif*/
/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */


#define WaitOnBusy( )          while(    bcm2835_gpio_lev(BUSY) == 1 ){ }

// This code handles cases where assert_param is undefined
#ifndef assert_param
#define assert_param( ... )
#endif

SX1280Hal::SX1280Hal( int nss, int busy, int dio1, int dio2, int dio3, int rst, int txEnablePin, int rxEnablePin, int ledPin)
        :   SX1280( )
{
//	RadioNss = nss; // not used
    RadioReset = rst;
    BUSY = busy;
	DIO1 = dio1;
    TXENPIN = txEnablePin;
	RXENPIN = rxEnablePin;
	LEDPIN = ledPin;
	
//	bcm2835_gpio_fsel(RadioNss, BCM2835_GPIO_FSEL_OUTP);
//  bcm2835_gpio_write(RadioNss, HIGH);

	bcm2835_gpio_fsel(RadioReset, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(RadioReset, HIGH);
	
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); // The default

	if(nss == RPI_BPLUS_GPIO_J8_24){
		printf("Chipselect CS0 used\n");
		bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      	
		bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      
	}else if(nss==RPI_BPLUS_GPIO_J8_26){
		printf("Chipselect CS1 used\n");
		bcm2835_spi_chipSelect(BCM2835_SPI_CS1);                      
		bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);      
	}else{
		printf("Error! Invalid chipselect\n");
	}
	
	bcm2835_gpio_fsel(BUSY, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(DIO1, BCM2835_GPIO_FSEL_INPT);
	
	bcm2835_gpio_fsel(TXENPIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(TXENPIN, LOW);
	
	bcm2835_gpio_fsel(RXENPIN, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(TXENPIN, HIGH);
	
	bcm2835_gpio_fsel(LEDPIN, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(LEDPIN, LOW);
}

SX1280Hal::~SX1280Hal( void )
{
 
};
/*
void SX1280Hal::IoIrqInit( DioIrqHandler irqHandler )
{
    assert_param( RadioSpi != NULL || RadioUart != NULL );
    if( RadioSpi != NULL )
    {
        SpiInit( );
    }
    if( RadioUart != NULL )
    {
        UartInit( );
    }

    BUSY.mode( PullNone );

    DioAssignCallback( DIO1, PullNone, irqHandler );
    DioAssignCallback( DIO2, PullNone, irqHandler );
    DioAssignCallback( DIO3, PullNone, irqHandler );
}
*/
void SX1280Hal::Reset( void )
{
//    noInterrupts();
	printf("Reset...");
    bcm2835_delay(20);
    bcm2835_gpio_write(RadioReset, LOW);
    bcm2835_delay(50);
    bcm2835_gpio_write(RadioReset, HIGH);
    bcm2835_delay(20);
	printf("done\n");
//    interrupts();
}

void SX1280Hal::Wakeup( void )
{
//    noInterrupts();

    //Don't wait for BUSY	here
	uint8_t txbuffer[2];
	printf("Wakeup...");
	txbuffer[0]=(uint8_t)(RADIO_GET_STATUS);
	txbuffer[1]=0;
	bcm2835_spi_transfern((char *)txbuffer,2);
	// Wait for chip to be ready.
    WaitOnBusy( );
	printf("done\n");
//    interrupts();
}

void SX1280Hal::WriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

	uint8_t txbuffer[size+1];

	txbuffer[0]=(uint8_t)command;

	for( uint16_t i = 0; i < size; i++ )
	{
		txbuffer[i+1] = buffer[i];
	}

	bcm2835_spi_transfern((char *)txbuffer,size+1);
   
    if( command != RADIO_SET_SLEEP )
    {
        WaitOnBusy( );
    }
}

void SX1280Hal::ReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );


	if( command == RADIO_GET_STATUS )
	{
		uint8_t txbuffer[3];

		txbuffer[0] = (uint8_t)command;
		txbuffer[1] = 0;
		txbuffer[2] = 0;

		bcm2835_spi_transfern((char *)txbuffer, 3);
		buffer[0] = txbuffer[0];
	}
	else
	{
		uint8_t txbuffer[size+2];

		txbuffer[0] = (uint8_t)command;
		txbuffer[1] = 0;

		for( uint16_t i = 0; i < size; i++ )
		{
			txbuffer[i+2]=0;
		}

		bcm2835_spi_transfern((char *)txbuffer, size+2);

		for( uint16_t i = 0; i < size; i++ )
		{
			buffer[i]=txbuffer[i+2];
		}
	}
	
    WaitOnBusy( );
}

void SX1280Hal::WriteRegister( uint16_t address, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

	uint8_t txbuffer[size+3];
	txbuffer[0]=(uint8_t)(RADIO_WRITE_REGISTER);
	txbuffer[1]=(uint8_t)(( address & 0xFF00 ) >> 8 );
	txbuffer[2]=(uint8_t)(address & 0x00FF);
	
	for( uint16_t i = 0; i < size; i++ )
	{
		txbuffer[i+3]=buffer[i];
	}

	bcm2835_spi_transfern((char *)txbuffer,size+3);

    WaitOnBusy( );
}

void SX1280Hal::WriteRegister( uint16_t address, uint8_t value )
{
    WriteRegister( address, &value, 1 );
}

void SX1280Hal::ReadRegister( uint16_t address, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

	uint8_t txbuffer[size+4];

	for( uint16_t i = 0; i < size+4; i++ )
	{ 
		txbuffer[i] = 0;
	}

	txbuffer[0]=(uint8_t)(RADIO_READ_REGISTER);
	txbuffer[1]=(uint8_t)((address & 0xFF00) >> 8);
	txbuffer[2]=(uint8_t)(address & 0x00FF);
	txbuffer[3]=0;

	bcm2835_spi_transfern((char *)txbuffer, size+4);

	for( uint16_t a = 0; a < size; a++ )
	{
		buffer[a] = txbuffer[a+4];
	}
	
    WaitOnBusy( );
}

uint8_t SX1280Hal::ReadRegister( uint16_t address )
{
    uint8_t data;

    ReadRegister( address, &data, 1 );
    return data;
}

void SX1280Hal::WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    WaitOnBusy( );

	uint8_t txbuffer[size+2];

	txbuffer[0]=(uint8_t)(RADIO_WRITE_BUFFER);
	txbuffer[1]=offset;

	for( uint16_t i = 0; i < size; i++ )
	{
		txbuffer[i+2] = buffer[i];
	}

	bcm2835_spi_transfern((char *)txbuffer, size+2);

    WaitOnBusy( );
}

void SX1280Hal::ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    WaitOnBusy( );
	
	uint8_t txbuffer[size+3];

	for( uint16_t i = 0; i < size+3; i++ )
	{
		txbuffer[i] = 0;
	}

	txbuffer[0]=(uint8_t)(RADIO_READ_BUFFER);
	txbuffer[1]=offset;
	txbuffer[2]=0;
	
	bcm2835_spi_transfern((char *)txbuffer, size+3);

	for( uint16_t a = 0; a < size; a++ )
	{
		buffer[a] = txbuffer[a+3];
	}
	
    WaitOnBusy( );
}

void SX1280Hal::SetLed(uint8_t output){
	 bcm2835_gpio_write(LEDPIN, output);
}

void SX1280Hal::SetTxEnablePin(uint8_t output){
	bcm2835_gpio_write(TXENPIN, output);
}

void SX1280Hal::SetRxEnablePin(uint8_t output){
	bcm2835_gpio_write(RXENPIN, output);
}

uint8_t SX1280Hal::GetDioPinStatus(){
	return  bcm2835_gpio_lev(DIO1);
}