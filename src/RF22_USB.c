#include "RF22_USB.h"

static volatile bool IRQ_TRIGGERED;			//Flag indicating when an IRQ has occured
static volatile long int  DELAY_INTRS = 0;		//Used in timer 2 for timeouts
static bool ALOHA_MODE_FLAG = false;			//Flag to indicate if in byte-by-byte mode or ALOHA mode (default is byte-by-byte)
static bool ALOHA_RECEIVE_FLAG = false; //Flag to determine if a packet has been received in ALOHA mode
static bool TRANSMITTING = false;  //Flag to indicate radio is currently TRANSMITTING
static bool VALID_ACK = false;					//Flag to determine if a valid ACK message has been received
static int SLOT_TIME_ARRAY[] = {0, 519, 1010, 1502, 1993, 2485, 2976, 3304, 4942, 6582};	//Array of preset slot time values (15 ms increments). (Set via trial and error testing)
static int SLOT_TIME = 2485;					//The slot time in timer counts. Default is ~75ms.
static int LEN_SLOT_TIME_ARRAY = sizeof(SLOT_TIME_ARRAY)/sizeof(*(SLOT_TIME_ARRAY));	//Finds the length of the slot time array
static int BACKOFF_LIMIT = 6;					//2^BACKOFF_LIMIT - 1 is the max value of the back-off timer (default value = 6)
static uint8_t MAX_TRANSMISSIONS = 1;			//The maximum number of times the radio will attempt to transmit a packet

static unsigned char SOURCE_ADDRESS = 0xFF; //Source address of board (defaults to 0xFF)

static void IRQ_Handler(unsigned char*);                 //Handles Interrupts
static void API_Handler(int, unsigned char*, unsigned char*, bool);		//Handles API instructions (in API mode)
static void ALOHA_Receive(unsigned char *, bool, unsigned char *);			//Aloha mode receiver
static void ALOHA_Transmit(unsigned char *, unsigned char*, bool);			//Transmits when in Aloha mode
static void Transmit(unsigned char *, bool);								//Writes data to FIFO on RFM22B and sends transmit command
static void ProcessPacket(unsigned char*);									//Reads in data packets for transmission

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = 0,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
    // Local variables
    unsigned char tx_buffer[255];			//Transmission buffer
    unsigned char rx_buffer[255];			//Receiver buffer
    bool API_instruction_flag = false;
	IRQ_TRIGGERED = false;

	SetupHardware();
    LEDs_SetAllLEDs(LEDS_ALL_LEDS);

//	RingBuffer_InitBuffer(&FromHost_Buffer, FromHost_Buffer_Data, sizeof(FromHost_Buffer_Data));

	sei();
	rfm23_mode_rx();

	for (;;)
	{
		IRQ_Handler(rx_buffer);

		if(ALOHA_RECEIVE_FLAG)
		{
			ALOHA_Receive(rx_buffer, API_instruction_flag, rx_buffer);
			ALOHA_RECEIVE_FLAG = 0;
		}

		if(!TRANSMITTING)
		{
			int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
			
			if(!(ReceivedByte < 0))
			{
//				RingBuffer_Insert(&FromHost_Buffer, ReceivedByte);
				API_Handler(ReceivedByte, tx_buffer, rx_buffer, API_instruction_flag);
			}
		}

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the application's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
    LEDs_Init();
	USB_Init();

	/* RFM22 Initialization */
	rfm23_init();
	rfm23_setup();

    // External Interrupt(s) initialization
    // INT0 Mode: Falling Edge
    //Sets pin PE6 to receive neg edge interrupt from RFM22B module
	EICRA = 0x02;
	EICRB = 0x00;
	EIMSK = 0x01;
	EIFR = 0x01;

    //Enable timers and timer overflow interrupts
	TCCR1B = 0x00;
	TIMSK1 = 0x01;
	TCNT1 = 0;

	/* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
//	TCCR0B = (1 << CS02);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
    LEDs_SetAllLEDs(LEDS_ALL_LEDS);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
    LEDs_SetAllLEDs(LEDS_NO_LEDS);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

    LEDs_SetAllLEDs(ConfigSuccess ? LEDS_ALL_LEDS : LEDS_NO_LEDS);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

void IRQ_Handler(unsigned char* rx_buffer)
{
	uint8_t receive_packet_length = 0;

	if(IRQ_TRIGGERED && rfm23_on_nirq())
	{
		rfm23_handle_interrupt();
		IRQ_TRIGGERED = false;

		if(rfm23_get_isr_1() & 0x02)
		{
			rfm23_write(0x07, 0x01); //Set to ready mode
			TRANSMITTING = false;
			rfm23_mode_rx(); //Sets chip back to rx_mode
		}

		else if(rfm23_get_isr_1() & 0x01)
		{
			rfm23_write(0x07, 0x01); //Set to ready mode
			rfm23_receive(rx_buffer, receive_packet_length);
			CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 0x81);	//Identifies packet as received over wireless link

            for(int i = 0; i < (receive_packet_length); i++)	
            {
            //Prints contents of rx_buffer to comms port
                CDC_Device_SendByte(&VirtualSerial_CDC_Interface, rx_buffer[i]);
            }	

            if(ALOHA_MODE_FLAG)
            {
            	ALOHA_RECEIVE_FLAG = true;
            }
            rfm23_mode_rx(); //Sets chip back to rx_mode
		}
	}
}

void API_Handler(int API_instruction, unsigned char* tx_buffer, unsigned char* rx_buffer, bool API_instruction_flag)
{
    int slot_time_index;

    switch(API_instruction){
        case 0x01:	//Reset
            ALOHA_MODE_FLAG = false;
            SOURCE_ADDRESS = 0xff;
            rfm23_write(0x3F, SOURCE_ADDRESS);
            SLOT_TIME = SLOT_TIME_ARRAY[5];
            BACKOFF_LIMIT = 6;
            MAX_TRANSMISSIONS = 1;
            TRANSMITTING = false;
            VALID_ACK = false;
            break;

        case 0x02:	//Clear Aloha mode 
            ALOHA_MODE_FLAG = false;
            break;

        case 0x04:	//Set Aloha mode
            ALOHA_MODE_FLAG = true;
            break;

        case 0x08:	//Set unique source address
            SOURCE_ADDRESS = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
            rfm23_write(0x3F, SOURCE_ADDRESS);
            break;

        case 0x10:	//Aloha transmission mode
                ProcessPacket(tx_buffer);
                API_instruction_flag = true;
                if(ALOHA_MODE_FLAG)
                {
                    ALOHA_Transmit(tx_buffer, rx_buffer, API_instruction_flag);
                }//if
                else
                {
                    TRANSMITTING = true;
                    rfm23_mode_tx();
                    Transmit(tx_buffer, API_instruction_flag);
                }//else
                API_instruction_flag - false;
            break;

        case 0x20:	//Set back-off limit
            BACKOFF_LIMIT = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
            break;

        case 0x40:	//Set slot time from an array of preset values
            slot_time_index = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
            // If a valid address, change the slot time else toggle LED2 to blue
            if(slot_time_index < LEN_SLOT_TIME_ARRAY)
            {
                SLOT_TIME = SLOT_TIME_ARRAY[slot_time_index];
//                PORTC |= (0x01 << 0x00);  //Clears the blue on the RGB LED
            }
            else{
                //Clears LEDs
//                PORTC |= (0x01 << 0x00);
//                PORTE |= (0x03 << 0x00);

//                PORTC &= ~(0x01 << 0x00); //Causes RGB LED to go blue representing an failure to change slot time
            }
            break;
        case 0x80:	//Set Maxmimum Transmissions
            MAX_TRANSMISSIONS = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
            break;
        default:
            break;
    }//switch
}

//================================//
//Function to read in data packets//
//================================//
void ProcessPacket(unsigned char* tx_buffer){
    uint8_t done = 0;
    int tx_count = 0;

    while(!done){
        int16_t received_byte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
        
        if(received_byte >= 0)
        {
            switch(tx_count){
                //Byte after initial control byte contains destination address
                case 0:
                    tx_buffer[tx_count] = received_byte;
                    tx_count++;
                    break;
                //Next byte contains length of data to send
                case 1:
                    tx_buffer[tx_count] = received_byte;
                    tx_count++;
                    break;
                default:
                    if(tx_count < tx_buffer[1]+1){
                        tx_buffer[tx_count] = received_byte;
                        tx_count++;	
                    }
                    else{
                        tx_buffer[tx_count] = received_byte;
                        tx_count = 0;
                        done = 1;
                    }//if
                    break;
            }//switch
        }//if
    }
}

//==========================================//
//Function to transmit packets in Aloha mode//
//==========================================//
void ALOHA_Transmit(unsigned char* tx_buffer, unsigned char * rx_buffer, bool API_instruction_flag)
{
    uint8_t delay_count;
    //Aloha transmission code. NOTE: layer 2 frame starts from tx[4] (the first two bytes are useless, tx[3] is the API identifier)
    int tx_count;
    unsigned char aloha_buffer[255]; //Buffer for the aloha frame
    uint8_t transmissions;
    long long int timeout_count = 0;
    long long int timeout_limit = 0;
    static long long int backoff_number;
    static long int random_number_max;
    unsigned char stats_packet[13];	//Statistics packet returned to higher layers
    static long unsigned int mult;
    
    //Creating the aloha frame (that we wish to send) from the incoming tx_buffer (frame + instruction headers)
    for (tx_count = 0; tx_count < tx_buffer[1]; tx_count++)
    {
        aloha_buffer[tx_count] = tx_buffer[tx_count+2];        
    }//for

    VALID_ACK = false;	//Resets flag
    transmissions = 0; 	//Resets counter
    
    // Resetting the timer here
    DELAY_INTRS = 0;
    TCNT1 = 0;
    TCCR1B = 0x01; //Enables delay timer
    
    //Loop runs while there is not a valid ACK and the maximum number of transmissions is not reached
    while(!VALID_ACK && (transmissions < MAX_TRANSMISSIONS))
    {
        //Transmitting
        TRANSMITTING = true;
        rfm23_mode_tx();
        Transmit(tx_buffer, API_instruction_flag);
        
        //Waits for confirmation of successful transmission
        while(TRANSMITTING)
        {
            IRQ_Handler(rx_buffer);
        }//while(transmitting)

        timeout_count = 256*DELAY_INTRS + TCNT1; //Current value of delay timer
        transmissions +=1;

        //If this is a broadcast message, continue without waiting for ACK
        if(aloha_buffer[0] == 0xFF)
        {
            VALID_ACK = true;
        }//if

        //Waiting for ACK
        timeout_limit = timeout_count + 3220; //Waits here for ~100ms
        while((timeout_count < timeout_limit) && !VALID_ACK)
        {
            IRQ_Handler(rx_buffer);
            if(ALOHA_RECEIVE_FLAG){
                ALOHA_Receive(rx_buffer, API_instruction_flag, aloha_buffer);
                ALOHA_RECEIVE_FLAG = false;
            }//if
            timeout_count = 256*DELAY_INTRS + TCNT1; //Rechecks value of interrupts
        }//while

        //Random back-off timer for retransmissions
        if(!VALID_ACK && (transmissions < MAX_TRANSMISSIONS))
        {
            timeout_count = 256*DELAY_INTRS + TCNT1;
            //Sets the maximum number of slot times to back off for
                if(transmissions < (BACKOFF_LIMIT + 1))
                {
//                    random_number_max = pow(2, transmissions) - 1;
                }
                else
                {
//                    random_number_max = pow(2, BACKOFF_LIMIT+1) - 1;
                }//if(pow...)
        
                //Random number generator (from StackOverflow)
/*                do
                {
                    mult = rand()/(RAND_MAX/(random_number_max + 1) );
                }
                while(mult > random_number_max);
*/                
                backoff_number = mult * SLOT_TIME + timeout_count;
                while(timeout_count < backoff_number)
                {
                    timeout_count = 256*DELAY_INTRS + TCNT1;
                }//while
        }//if(!valid_ACK)
        
    }//while(!valid_ACK && transmissions < MAX_TRANSMISSIONS)

    TCCR1B = 0x00; 	//Stops delay timer
    delay_count = TCNT1; //Reads the timer

    //If no ACK received, set transmissions = 0 to indicate failure to send
    if(!VALID_ACK)
    {
        transmissions = 0;	//Indicates error
    }
    
    //Creates and 'sends' statistics 'packet' to itself
    stats_packet[0] = 0x7E;	//Identifies packet as statistics packet
    stats_packet[1] = SOURCE_ADDRESS;
    stats_packet[2] = SOURCE_ADDRESS;
    stats_packet[3] = 0;
    stats_packet[4] = 6;
    stats_packet[5] = SOURCE_ADDRESS;
    stats_packet[6] = SOURCE_ADDRESS;
    stats_packet[7] = transmissions;
    stats_packet[8] = delay_count;

    for(int n = 0; n < 4; n++)
    {
        stats_packet[9+n] = ((DELAY_INTRS >> (8*n)) & 0xFF); 
    }//for

    //'Sending' statistics 'packet'
    for(int n = 0; n < stats_packet[4] + 7; n++)
    {
        CDC_Device_SendByte(&VirtualSerial_CDC_Interface, stats_packet[n]);
    }//for
}//Aloha_Transmit

void ALOHA_Receive(unsigned char * rx_buffer, bool API_instruction_flag, unsigned char * tx_buffer)
{
	unsigned char ACK[6]; //Creates ACK packet

	if(rx_buffer[0] == 0xFF){return;}

	if(rx_buffer[2] == 0x80)
	{
		if((rx_buffer[2] & 0x70) == ((tx_buffer[2] + 0x10) & 0x70))
		{
			VALID_ACK = true;

		}
	}
	else 
	{
		ACK[0] = rx_buffer[1];
		ACK[1] = SOURCE_ADDRESS;
        ACK[2] = ((0x01 << 0x07) | (rx_buffer[2] + 0x10)); //Sets ACK bit in type field and increments sequence number
        ACK[3] = 0x00;
        ACK[4] = rx_buffer[1];
        ACK[5] = SOURCE_ADDRESS;

        TRANSMITTING = true;
        rfm23_mode_tx();
        API_instruction_flag = false;
        Transmit(ACK, API_instruction_flag);

        while(TRANSMITTING)
        	{IRQ_Handler(rx_buffer);}
	}
}

void Transmit(unsigned char *tx_packet, bool API_instruction_flag)
{
	if (API_instruction_flag)
    {
        rfm23_write(0x3A, tx_packet[0]);
        rfm23_write(0x3E, tx_packet[1]);
   		for (int i=0; i<(tx_packet[1]); i++)
   		{
            //CDC_Device_SendByte(&VirtualSerial_CDC_Interface, tx_packet[i]);
       		rfm23_write(0x7F, tx_packet[i+2]);      //Send payload to the FIFO
   		}//for
    }
    else	//Sending ACKs
    {
        rfm23_write(0x3A, tx_packet[0]);
        rfm23_write(0x3E, tx_packet[3]+6);
        for (int i=0; i<(tx_packet[3]+6); i++)
   		{
            //CDC_Device_SendByte(&VirtualSerial_CDC_Interface, tx_packet[i]);
       		rfm23_write(0x7F, tx_packet[i]);      //Send payload to the FIFO
   		}//for
    }//if
    
    rfm23_write(0x05, 0x04);      // Enable packet sent interrupt
    rfm23_write(0x06, 0x00);
    //TX_MODE_ENABLED = true; //Sets tx_mode flag
    
    rfm23_write(0x07, 0x09);      // Start TX
}

ISR(INT0_vect, ISR_BLOCK)
{
	IRQ_TRIGGERED = true;	
}

ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
    //Timer 2 used to measure time to complete transmission
    DELAY_INTRS++;
}
