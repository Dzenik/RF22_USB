/*
	rfm23.c
*/

#include <avr/io.h>
#include <util/delay.h>
#include "rfm23.h"

/*
	spi functions
*/


/* init spi */
void rfm23_spi_init() {
	
	// set mosi, select (ss/cs) and clock as output
	RFM23_SPI_DDR = (1 << RFM23_SPI_MOSI) | (1 << RFM23_SPI_SELECT) | (1 << RFM23_SPI_CLOCK);
	
	// set miso as input
	RFM23_SPI_DDR &= ~(1 << RFM23_SPI_MISO);
	
	// select
	rfm23_spi_select();
	
	// enable spi (SPE), set as master (MSTR) and set clock rate (SPR)
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
}

/* write spi */
uint8_t rfm23_spi_write(uint8_t val) {
	
	// fill value into spi data register
	SPDR = val;
	
	// wait
	while (!(SPSR & (1 << SPIF)));
	
	return (uint8_t)SPDR;
	
}

/* select low */
void rfm23_spi_select() {
	
	// when module is selected,
	// set SELECT to LOW
	RFM23_SPI_PORT &= ~(1 << RFM23_SPI_SELECT);
}

/* select high */
void rfm23_spi_unselect() {
	
	// when module is unselected,
	// set SELECT to HIGH
	RFM23_SPI_PORT |= (1 << RFM23_SPI_SELECT);
}


/*
	general functions
*/

/* initialize rfm module */
void rfm23_init() {
	
	// configure nirq port as input
	RFM23_NIRQ_DDR &= ~(1 << RFM23_NIRQ);
	
	// init spi
	rfm23_spi_init();
	
	// read all interrupts
	rfm23_read(RFM23_03h_ISR1);
	rfm23_read(RFM23_04h_ISR2);
	
	// wait for POR 16ms
	_delay_ms(16);
}

/* test read/write */
uint8_t rfm23_test() {
	
	uint8_t reg = 0x05;
	uint8_t value = 0xEE;
	
	// read register
	uint8_t val_orig = rfm23_read(reg);
	
	// write register
	rfm23_write(reg, value);
	
	// read register
	uint8_t val_new = rfm23_read(reg);
	
	// set orig register value
	rfm23_write(reg, val_orig);
	
	// test if the written register value
	// has been read
	if (val_new == value) {
		return 0xFF;
	} else {
		return 0x00;
	}
}

/* software reset module */
void rfm23_reset() {
	
}


/*
	read/write functions
*/

/* write to rfm registers */
void rfm23_write(uint8_t addr, uint8_t val) {
	
	// first bit 1 means write
	addr |= (1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write value
	rfm23_spi_write(val);
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}

/* read from rfm registers */
uint8_t rfm23_read(uint8_t addr) {
	
	// first bit 0 means read
	addr &= ~(1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write dummy value
	uint8_t val = rfm23_spi_write(0x00);
	
	// unselect module and finish operation
	rfm23_spi_unselect();
	
	return val;
}

/* write to rfm registers in burst mode */
void rfm23_write_burst(uint8_t addr, uint8_t val[], uint8_t len) {

	// first bit 1 means write
	addr |= (1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write values
	for (uint8_t i = 0; i < len; i++) {
		rfm23_spi_write(val[i]);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}

/* read from rfm registers in burst mode */
void rfm23_read_burst(uint8_t addr, uint8_t val[], uint8_t len) {
	
	// first bit 0 means read
	addr &= ~(1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// read values
	for (uint8_t i = 0; i < len; i++) {
		val[i] = rfm23_spi_write(0x00);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}


/*
	interrupt functions
*/

/* returns 0xFF when NIRQ-Pin is low = rfm interrupt */
uint8_t rfm23_on_nirq() {
	return !(RFM23_NIRQ_PIN & (1 << RFM23_NIRQ));
}

/* returns 0xFF when interrupt has happened.
   interrupt is set once by rfm23_handle_interrupt()
   and will be reset when this function is called.
   
   on_nirq() (or hardware interrupt) -> handle_interrupt()
   -> on_interrupt()
  */
uint8_t rfm23_on_interrupt() {
	
	// save current status
	uint8_t tmp = RFM23_STATUS;
	
	// if interrupt bit is set
	// -> return 0xFF and reset
	if (tmp & (1 << RFM23_STATUS_INTERRUPT)) {
		
		// reset bit
		RFM23_STATUS &= ~(1 << RFM23_STATUS_INTERRUPT);
		
		return 0xFF;
	}
	
	return 0x00;
}

/* handle the interrupt */
void rfm23_handle_interrupt() {
	
	// read interrupt status register
	// -> nirq pin of the rfm resets
	RFM23_ISR1 = rfm23_read(RFM23_03h_ISR1);
	RFM23_ISR2 = rfm23_read(RFM23_04h_ISR2);
	
	// set interrupt bit
	RFM23_STATUS |= (1 << RFM23_STATUS_INTERRUPT);
	
	// wait some ms
	// dont know why this is needed
	// ... @TODO
	_delay_ms(10);	
}

/* enable interrupts */
void rfm23_enable_interrupt_1(uint8_t ir) {
	rfm23_write(RFM23_05h_ENIR1, ir);
}

void rfm23_enable_interrupt_2(uint8_t ir) {
	rfm23_write(RFM23_06h_ENIR2, ir);
}

/* return saved interrupt status registers */
uint8_t rfm23_get_isr_1() {
	return RFM23_ISR1;
}

uint8_t rfm23_get_isr_2() {
	return RFM23_ISR2;
}


/*
	operating mode functions
*/

/* mode READY */
void rfm23_mode_ready() {
	
	// go to READY mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_XTON);
	
	// wait for module
	_delay_us(200);	
}

/* mode RXON */
void rfm23_mode_rx() {
	
	// go to RXON mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_RXON);
	
	// wait for module
	_delay_us(200);

	// Clear RX Fifo
	rfm23_clear_rxfifo();
}

/* mode TXON */
void rfm23_mode_tx() {
	
	// go to TXON mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_TXON);
	
	// wait for module
	_delay_us(200);

	// Clear TX Fifo
	rfm23_clear_txfifo();
}

/* mode WAKEUP
   module goes into sleep mode and wakes up after
   a defined period of time.
   configuration of wake-up time is necessary.
   time can be set with set_wakeup_time()
   */
void rfm23_mode_wakeup() {
	
	// go to IDLE mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_ENWT);
	
	// read status register to reset
	rfm23_read(RFM23_03h_ISR1);
	rfm23_read(RFM23_04h_ISR2);
	
	// wait for module
	_delay_us(200);	
}


/*
	fifo functions
*/

/* clear rx fifo */
void rfm23_clear_rxfifo() {
	rfm23_write(0x08, 0x02);
	rfm23_write(0x08, 0x00);
}

/* clear tx fifo */
void rfm23_clear_txfifo() {
	rfm23_write(0x08, 0x01);
	rfm23_write(0x08, 0x00);
}


/*
	send & receive functions
*/

/* send data */
void rfm23_send(uint8_t data[], uint8_t len) {
	
	// clear tx fifo
	rfm23_clear_txfifo();
	
	// set packet length
	rfm23_write(0x3e, len);
	
	// write data into fifo
	rfm23_write_burst(0x7f, data, len);

	// send data
	rfm23_write(0x07, 0x09);
}

void rfm23_send_addressed(uint8_t addr, uint8_t data[], uint8_t len) {
	
	// set receiver address
	rfm23_write(0x3b, addr);
	
	// send data
	rfm23_send(data, len);
}

void rfm23_set_address(uint8_t addr) {
	
	// set sender address
	rfm23_write(0x3A, addr);
	
	// check header2 on receive
	rfm23_write(0x40, addr);
	
	// only receive when header2 match
	rfm23_write(0x32, 0x04);
}

/* receive data */
void rfm23_receive(uint8_t data[], uint8_t len) {
	rfm23_read_burst(0x7f, data, len);
}

/* get packet length */
uint8_t rfm23_get_packet_length() {
	return rfm23_read(0x4b);
}


/*
	wait functions
*/

/* wait for IPKSENT interrupt */
/* @TODO */
void rfm23_wait_packet_sent(uint8_t timeout) {
	//printf("wait for packet sent...\n");
	
	//uint8_t current_time = 0;
	
	// handle interrupt
	rfm23_handle_interrupt();
	
	while (!(rfm23_get_isr_1() & (1 << RFM23_03h_ISR1_IPKSENT))/* && !(current_time > timeout)*/) {
		rfm23_handle_interrupt();
	}
}



/*
	other functions
*/

/* get temperature
   temp = return_value * 0.5 - 64
*/
uint8_t rfm23_get_temperature() {
	
	// set adc input and reference
	rfm23_write(0x0f, 0x00 | (1 << 6) | (1 << 5) | (1 << 4));
	
	// set temperature range
	// -64 to 64°C, ADC8 LSB: 0.5°C
	rfm23_write(0x12, 0x00 | (1 << 5));
	
	// adcstart
	rfm23_write(0x0f, 0x00 | (1 << 7));
	
	// wait for adc_done
	while (!rfm23_read(0x0f) & (1 << 7));
	
	// return adc value
	return rfm23_read(0x11);
}

/* wake-up timer */
void rfm23_set_wakeup_time(uint8_t seconds) {
	rfm23_write(0x14, 0x0A);
	rfm23_write(0x15, 0x00);
	rfm23_write(0x16, 8 * seconds); // 1 = 125ms, 8 = 1000ms = 1s
}

/* get rssi
*/
uint8_t rfm23_get_rssi(void) {
  return rfm23_read(0x26);
}

unsigned char SOURCE_ADDRESS = 0xFF; //Source address of board (defaults to 0xFF)
// Initialize the RFM22 for transmitting
void rfm23_setup(void)
{
/*
//=======================//
//Register initialisation//
//=======================//
  Address Read/Write - Function
*/
    _delay_ms(50);
    rfm23_write(0x07, 0x81);  //Reset register values on RFM22B
    _delay_ms(50);

//0x00 R - Device type
//0x01 R - Device version
//0x02 R - Device status
//0x03 R - Interrupt status 1
//0x04 R - Interrupt status 2
//0x05 R/W - Interrupt Enable 1		
    rfm23_write(0x05, 0x00);			//Disable all interrupts
//0x06 R/W - Interrupt Enable 2
    rfm23_write(0x06, 0x00);          // Disable all interrupts
//0x07 R/W - Operating function and control 2
    rfm23_write(0x07, 0x01);              // Set READY mode
//0x08 R/W - Operating function and control 2
//0x09 R/W - Crystal oscillator load capacitance        
    rfm23_write(0x09, 0x7F);              // Cap = 12.5pF
//0x0A R/W - Microcontroller output clock
    rfm23_write(0x0A, 0x05);              // Clk output is 2MHz
// Ensure the antenna can be switched automatically according to transmit and receive
// This assumes GPIO0(out) is connected to TX_ANT(in) to enable tx antenna during transmit
// This assumes GPIO1(out) is connected to RX_ANT(in) to enable rx antenna during receive
    rfm23_write(0x0B, 0x12); // TX state
    rfm23_write(0x0C, 0x15); // RX state

//0x0B R/W - GPIO_0 configuration       
//    rfm23_write(0x0B, 0xF4);              // GPIO0 is for RX data output
//0x0C R/W - GPIO_1 configuration
//    rfm23_write(0x0C, 0xEF);              // GPIO1 is TX/RX data CLK output
//0x0D R/W - GPIO_2 configuration        
    rfm23_write(0x0D, 0x00);              
//0x0E R/W - I/O port configuration
    rfm23_write(0x0E, 0x00);              
//0x0F R/W - ADC configuration        
    rfm23_write(0x0F, 0x70);             
//0x10 R/W - ADC sensor amplifier offset
    rfm23_write(0x10, 0x00);              
//0x11 R - ADC value
//0x12 R/W - Temperature sensor control
    rfm23_write(0x12, 0x00);              
//0x13 R/W - Temperature offset value
    rfm23_write(0x13, 0x00);              
//0x14 R/W - Wake-up timer period 1        
//0x15 R/W - Wake-up timer period 2
//0x16 R/W - Wake-up timer period 3
//0x17 R - Wake-up timer value 1
//0x18 R - Wake-up timer value 2
//0x19 R/W - Low-duty cycle mode duration
//0x1A R/W - Low battery detector threshold
//0x1B R - Battery voltage level
//0x1C R/W - IF filter bandwidth
    rfm23_write(0x1C, 0x88);	//200kbps 
//	rfm23_write(0x1C, 0x83);	//150kbps
//	rfm23_write(0x1C, 0x9A);	//100kbps              
//0x1D R/W - AFC loop gearshift override
    rfm23_write(0x1D, 0x3C);     	//200kbps & 150kbps & 100kbps       
//0x1E R/W - AFC timing control
    rfm23_write(0x1E, 0x02);		//200kbps & 150kbps & 100kbps 
//0x1F R/W - Clock recovery gearshift override
    rfm23_write(0x1F, 0x03);		//200kbps & 150kbps & 100kbps
//0x20 R/W - Clock recovery oversampling ratio        
    rfm23_write(0x20, 0x3C); 		//200kbps  
//	rfm23_write(0x20, 0x50); 		//150kbps
//	rfm23_write(0x20, 0x3C); 		//100kbps           
//0x21 R/W Clock recovery offset 2 
    rfm23_write(0x21, 0x02);      //200kbps 
//	rfm23_write(0x21, 0x01);      //150kbps
//	rfm23_write(0x21, 0x02);      //100kbps       
//0x22 R/W - Clock recovery offset 1
    rfm23_write(0x22, 0x22);      //200kbps
//	rfm23_write(0x22, 0x99);      //150kbps
//	rfm23_write(0x22, 0x22);      //100kbps
//0x23 R/W - Clock recovery offset 0
    rfm23_write(0x23, 0x22);      //200kbps
//	rfm23_write(0x23, 0x9A);      //150kbps 
//	rfm23_write(0x23, 0x22);      //100kbps        
//0x24 R/W - Clock recovery timing loop gain 1
    rfm23_write(0x24, 0x07);      //200kbps & 150 kbps & 100kbps       
//0x25 R/W - Clock recovery timing loop gain 0
    rfm23_write(0x25, 0xFF);		//200kbps & 150 kbps & 100kbps              
//0x26 R - Received signal strength indicator
//0x27 R/W - RSSI threshold for clear channel indicator
//0x28 R - Antenna dicersity register 1       
//0x29 R - Antenna dicersity register 2
//0x2A R/W - AFC limiter
    rfm23_write(0x2A, 0xFF);		//200kbps & 150 kbps & 100kbps
//0x2B R - AFC correction read
//0x2C R/W - OOK counter value 1
    rfm23_write(0x2C, 0x00);
//0x2D R/W - OOK counter value 2
    rfm23_write(0x2D, 0x00);
//0x2E R/W Slicer peak hold
    rfm23_write(0x2E, 0x00);
//0x2F - RESERVED
//0x30 R/W - Data access control              
    rfm23_write(0x30, 0x8C);              
//0x31 R - EzMAC Status
//0x32 R/W - Header control 1        
    rfm23_write(0x32, 0x88);  	//Check header byte 3 and enable broadcast byte (0xFF)          
//0x33 R/W - Header control 2        
    rfm23_write(0x33, 0x10);       //Sets length of header to 1 byte       
//0x34 R/W - Preamble length        
    rfm23_write(0x34, 4);          // 4 nibble = 2 byte preamble
//0x35 R/W - Preamble detection control
    rfm23_write(0x35, 0x20);              
//0x36 R/W - Sync word 3
    rfm23_write(0x36, 0x2D);              
//0x37 R/W - Sync word 2
//0x38 R/W - Sync word 1
//0x39 R/W - Sync word 0
//0x3A R/W - Transmit header 3 	//Transmit header set dynamically in transmit function           
//0x3B R/W - Transmit header 2
//0x3C R/W - Transmit header 1
//0x3D R/W - Transmit header 0
//0x3E R/W - Packet length		//Packet length is set dynamically in transmit function
//0x3F R/W - Check header 3
    rfm23_write(0x3F, SOURCE_ADDRESS);
//0x40 R/W - Check header 2
//0x41 R/W - Check header 1
//0x42 R/W - Check header 0
//0x43 R/W - Header enable 3
    rfm23_write(0x43, 0xFF);             //Check all bits
//0x44 R/W - Header enable 2
//0x45 R/W - Header enable 1
//0x46 R/W - Header enable 0
//0x47 R - Received header 3
//0x48 R - Received header 2
//0x49 R - Received header 1
//0x4A R - Received header 0        
//0x4B R - Received packet length
//0x4C to 0x4E - RESERVED
//0x4F R/W - ADC8 control
//0x50 to 0x5F - RESERVED
//0x58 R/W - Changed whsen setting tx data rate
    rfm23_write(0x58, 0xED);	//200kbps
//	rfm23_write(0x58, 0xC0);	//150kbps & 100kbps
//0x60 R/W - Channel filter coefficient address        
//0x61 - RESERVED
//0x62 R/W - Crystal oscillator / Control test
//0x63 to 0x68 - RESERVED
//0x69 R/W - AGC Override 1
    rfm23_write(0x69, 0x60);	//200kbps & 150kbps & 100kbps
//0x6A to 0x6C - RESERVED
//0x6D R/W - TX power
    rfm23_write(0x6D, 0x00);              // TX power to min
//0x6E R/W - TX data rate 1
    rfm23_write(0x6E, 0x33);	//200kbps
//	rfm23_write(0x6E, 0x26);	//150kbps
//	rfm23_write(0x6E, 0x19);	//100kbps              
//0x6F R/W - TX data rate 0
    rfm23_write(0x6F, 0x33);  //200kbps
//	rfm23_write(0x6F, 0x66);	//150kbps
//	rfm23_write(0x6F, 0x9A);	//100kbps          
//0x70 R/W - Modulcation mode control 1        
    rfm23_write(0x70, 0x0C);              // No manchester code, no data whiting
//0x71 R/W - Modulcation mode control 2
    //rfm23_write(0x71, 0x02);			//Direct mode FSK
    rfm23_write(0x71, 0x22);              // FSK, fd[8]=0, no invert for TX/RX data, FIFO mode
//0x72 R/W - Frequency deviation        
    rfm23_write(0x72, 0x50);	//200kbps             // Frequency deviation setting to 45K=72*625
//0x73 R/W - Frequency offset 1        
    rfm23_write(0x73, 0x00);              // No frequency offset
//0x74 R/W - Frequency offset 2        
    rfm23_write(0x74, 0x00);              // No frequency offset
//0x75 R/W - Frequency band select
    rfm23_write(0x75, 0x53);              // frequency set to 434MHz
//0x76 R/W - Nominal carrier frequency 1
    rfm23_write(0x76, 0x64);              // frequency set to 434MHz
//0x77 R/W - Nominal carrier frequency 0  
    rfm23_write(0x77, 0x00);              // frequency set to 434Mhz
//0x78 - RESERVED
//0x79 R/W - Frequency hopping channel select
    rfm23_write(0x79, 0x00);              // no frequency hopping
//0x7A R/W - Frequency hopping step size
    rfm23_write(0x7A, 0x00);              // no frequency hopping
//0x7B - RESERVED
//0x7C R/W - TX FIFO control 1
//0x7D R/W - TX FIFO control 2
//0x7E R/W - RX FIFO control
//0x7F R/W - FIFO access    

}
