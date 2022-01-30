#include "defines.h"                                                 

void timerInit(void)
{
// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 
// Mode: Normal top=FFh
// OC0 output: Disconnected
TCCR0A=0x02;
TCNT0=0x00;
OCR0A=0x00;    
   
   // Timer/Counter 1 initialization
   // Clock source: System Clock
   // Clock value: 250.000 kHz
   // Mode: Normal top=FFFFh
   // OC1A output: Discon.
   // OC1B output: Discon.
   // OC1C output: Discon.
   // Noise Canceler: Off
   // Input Capture on Falling Edge
   // Timer 1 Overflow Interrupt: On
   // Input Capture Interrupt: Off
   // Compare A Match Interrupt: Off
   // Compare B Match Interrupt: Off
   // Compare C Match Interrupt: Off
   TCCR1A=0x00;
   TCCR1B=0x03;
   TCNT1H=0x9E;
   TCNT1L=0x58;
   ICR1H=0x00;
   ICR1L=0x00;
   OCR1AH=0x00;
   OCR1AL=0x00;
   OCR1BH=0x00;
   OCR1BL=0x00;
   OCR1CH=0x00;
   OCR1CL=0x00;
      
   // Timer/Counter 2 initialization
   // Clock source: System Clock
   // Clock value: 7.813 kHz
   // Mode: Normal top=FFh
   // OC2 output: Disconnected
 
   ASSR=0x00;
   TCCR2A=0x05;
   TCNT2=0x00;
   OCR2A=0x00; 
   
   // Timer/Counter 3 initialization
   // Clock source: System Clock
   // Clock value: 15.625 kHz
   // Mode: Normal top=FFFFh
   // Noise Canceler: Off
   // Input Capture on Falling Edge
   // OC3A output: Discon.
   // OC3B output: Discon.
   // OC3C output: Discon.
   // Timer 3 Overflow Interrupt: Off
   // Input Capture Interrupt: Off
   // Compare A Match Interrupt: Off
   // Compare B Match Interrupt: Off
   // Compare C Match Interrupt: Off
   TCCR3A=0x00;
   TCCR3B=0x05;
   TCNT3H=0xE1;
   TCNT3L=0x7B;
   ICR3H=0x00;
   ICR3L=0x00;
   OCR3AH=0x00;
   OCR3AL=0x00;
   OCR3BH=0x00;
   OCR3BL=0x00;
   OCR3CH=0x00;
   OCR3CL=0x00;

   // Timer(s)/Counter(s) Interrupt(s) initialization
   EICRA=0x00;
   EICRB=0x00;
   EIMSK=0x00;

   // Timer/Counter 0 Interrupt(s) initialization
   TIMSK0=0x00;
   // Timer/Counter 1 Interrupt(s) initialization
   TIMSK1=0x00;              
   // Timer/Counter 2 Interrupt(s) initialization  
   TIMSK2=0x00;
   // Timer/Counter 3 Interrupt(s) initialization
   TIMSK3=0x01;         

} 
            
// Timer 1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{ 
      //int temp=0;
      // Reinitialize Timer 1 value
      TCNT1H = 0x9E ;
      TCNT1L = 0x58 ; 
  //    TCNT1H = 0x5b ;
   //   TCNT1L = 0x8c ; 

                      
      ptimer++;
      if(ptimer==60000)ptimer=0; 

    if(I||(revmove))traveltimer++;
          else traveltimer=0;
     
//Check for received data by CAN
     interpreter(); 
                      
     LED_indicator();

     supervisor(); 
     
    if(Eth_number)netstackService();    	// service the network    
    
if(beep_request_on)
        {BUZ=1;
         beep_request_on=0;}
else
if(beep_request)
        {beep_request=0;
         BUZ=0;
         beep_request_on=1;}


}

interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
	// set timer for 10ms interval
      TCNT2 = (unsigned char)-TIMER_INTERVAL;
      // count up on uptime
      UptimeMs += 10;    
      return;
}       

interrupt [TIM3_OVF] void timer3_ovf_isr(void)
{
    //For blinking 7segs. only
    // Reinitialize Timer 3 value
    TCNT3H=0xE1;
    TCNT3L=0x7B;   

    if(!WDRS)
          {
           #asm("cli")
          TIMSK0=0;
          TIMSK1=0;
          TIMSK2=0;
          TIMSK3=0;
          }

    if(WDRS==1)
          {
           #asm("wdr")
           WDRS=0;
          }

    if(beep_en)
          {BUZ=0;
           beep_en=0;}
    else BUZ=1;
    
    if(LED_blink)LED_blink=0;
      else LED_blink=1;    
         
    if(!blink_en)
    {         
         PORTC.0=0;     //led array
         PORTC.1=0;
         PORTC.2=0;
         PORTC.3=0;
         OE_7seg=0;
         return;
    }

    if(!OE_7seg)
    {
        // PORTC.0=1;
         PORTC.1=1;
         PORTC.2=1;
         PORTC.3=1;
         OE_7seg=1;
         return;
    }
    else
     if(OE_7seg)
     {
       //  PORTC.0=0;
         PORTC.1=0;
         PORTC.2=0;
         PORTC.3=0;
         OE_7seg=0;
         return;
     }
} 

interrupt [USART0_RXC] void usart0_rx_isr(void)
{     
 	RecData = UDR0;          
	ReadyFlag =1;          
	ReadyFlag2 =1;     
}         
// USART1 Receiver interrupt service routine
interrupt [USART1_RXC] void usart1_rx_isr(void)
{
   char status,data;
   status=UCSR1A;
   data=UDR1; 
   if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
      if (rx_index < RX_BUFFER_SIZE1) 
      {
        rx_buffer1[rx_index]=data;
        rx_index++;
      }  
      else         
        rx_index=0; 
      if (data==0xFF)
      {            
         if (End_Temp==0)
           End_Temp=1;
         else  
         {
           End_Data=rx_buffer1[2];  
           Address=rx_buffer1[3];  
           READY=1; 
           End_Temp=0;  
           rx_index=0;          
                  //received data by RS422 now in End_Data
                  //and Address
         }  
      }
      else
        End_Temp=0;    
   }   
  
}
// USART1 Transmitter interrupt service routine
interrupt [USART1_TXC] void usart1_tx_isr(void)
{

}

void uartInit(void)
{               
      // USART0 initialization
      // Communication Parameters: 8 Data, 1 Stop, No Parity
      // USART0 Receiver: On      Receive  Int. on
      // USART0 Transmitter: On   Transmit Int. off 
      // USART0 Mode: Asynchronous
      // USART0 Baud rate: 19200
      UCSR0A=0x00;
      UCSR0B=0x98;
      UCSR0C=0x06;
      UBRR0H=0x00;
      UBRR0L=0x33;
           
      
      // USART1 initialization
      // Communication Parameters: 8 Data, 1 Stop, No Parity
      // USART1 Receiver: On
      // USART1 Transmitter: On
      // USART1 Mode: Asynchronous
      // USART1 Baud rate: 56000
      UCSR1A=0x00;
      UCSR1B=0xD8;
      UCSR1C=0x06;
      UBRR1H=0x00;
      UBRR1L=0x11;            //0x33                    
}


void UartSendByte(char c)
{                    
      if( UCSR0A & 0x20 ) 	         // UDRE (USART Data Register Empty)
      {
    	    UDR0 = c;	               // Put data into buffer, sends the data
    	    UCSR0A |= 0x40; 		   // Rest TXC
      }           
}

void ClearScreen(void)
{    
   //   printf("\x1B[2J");
}

interrupt [CAN_IT] void can_isr(void)
{    //handle all possible can interrupts
     can_handel_interrupts();      
}


void initialize(void)
{
      // Declare your local variables here
      // Crystal Oscillator division factor: 1
      #pragma optsize-
      CLKPR=0x80;
      CLKPR=0x00;
      #ifdef _OPTIMIZE_SIZE_
      #pragma optsize+
      #endif
     
// Input/Output Ports initialization
// Port A initialization
// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In 
// State7=P State6=P State5=P State4=P State3=P State2=P State1=P State0=P 
PORTA=0xFF;
DDRA=0xFF;

// Port B initialization
// Func7=Out Func6=Out Func5=Out Func4=Out Func3=In Func2=Out Func1=Out Func0=Out 
// State7=0 State6=0 State5=1 State4=1 State3=P State2=0 State1=0 State0=0 
PORTB=0x38;
DDRB=0xF7;

// Port C initialization
// Func7=Out Func6=Out Func5=Out Func4=Out Func3=Out Func2=Out Func1=Out Func0=Out 
// State7=0 State6=0 State5=0 State4=0 State3=1 State2=1 State1=1 State0=1 
PORTC=0x00;
DDRC=0xFF;

// Port D initialization
// Func7=Out Func6=In Func5=Out Func4=Out Func3=Out Func2=In Func1=In Func0=In 
// State7=0 State6=P State5=0 State4=0 State3=0 State2=P State1=P State0=P 
PORTD=0x47;
DDRD=0xB8;

// Port E initialization
// Func7=In Func6=In Func5=In Func4=In Func3=Out Func2=Out Func1=Out Func0=In 
// State7=P State6=P State5=P State4=P State3=P State2=P State1=0 State0=P 
PORTE=0xFD;
DDRE=0x0C;

// Port F initialization
// Func7=Out Func6=Out Func5=Out Func4=Out Func3=Out Func2=Out Func1=Out Func0=Out 
// State7=0 State6=0 State5=0 State4=0 State3=0 State2=0 State1=0 State0=0 
PORTF=0x00;
DDRF=0xFF;

// Port G initialization
// Func4=Out Func3=Out Func2=Out Func1=Out Func0=Out 
// State4=1 State3=1 State2=1 State1=1 State0=1 
PORTG=0x1F;
DDRG=0x1F;   
   
      timerInit();
      uartInit();
   
      // Analog Comparator initialization
      // Analog Comparator: Off
      // Analog Comparator Input Capture by Timer/Counter 1: Off
      ACSR=0x80;
      ADCSRB=0x00;
   
      // CAN Controller initialization
      // CAN: On
      CANGCON=0x02;
      // CAN Interrupts:
      // Timer Overrun: Off
      // General Errors: Off
      // Frame Buffer: On
      // MOb Errors: Off
      // Transmit: On
      // Receive: On
      // Bus Off: Off
      // All, except Timer Overrun: Off
      CANGIE=0b10110100;
      // MOb0: Enabled, MOb1: Enabled, MOb2: Enabled, MOb3: Enabled, MOb4: Enabled, MOb5: Enabled, MOb6: Enabled, MOb7: Enabled
      CANEN2=0xFF;
      // MOb8: Enabled, MOb9: Enabled, MOb10: Enabled, MOb11: Enabled, MOb12: Enabled, MOb13: Enabled, MOb14: Enabled
      CANEN1=0x7F;
      // MOb0..7 Interrupts: MOb0: On, MOb1: On, MOb2: On, MOb3: On, MOb4: On, MOb5: On, MOb6: On, MOb7: On
      CANIE2=0xFF;
      // MOb8..14 Interrupts: MOb8: On, MOb9: On, MOb10: On, MOb11: On, MOb12: On, MOb13: On, MOb14: On
      CANIE1=0x7F;
      // Highest Interrupt Priority: MOb0
      CANHPMOB=0x00;
      // CAN System Clock: 4000.0 kHz
      CANBT1=0x06;
      // Propagation Time Segement: 1.750 us
      // Re-Sync Jump Width: 0.250 us
      CANBT2=0x0C;
      // Sample Point(s): 3
      // Phase Segment 1: 1.000 us
      // Phase Segment 2: 1.000 us
      CANBT3=0x37;
      // CAN Timer Clock Period: 0.500 us
      CANTCON=0x00;          
      
      // Watchdog Timer Prescaler: OSC/2048k
        #pragma optsize-
        WDTCR=0x1E;
        WDTCR=0x0E;
        #ifdef _OPTIMIZE_SIZE_
        #pragma optsize+
        #endif         
                 

      // I2C Bus initialization
      i2c_init();         

      // LM75 Temperature Sensor initialization
      // thyst: 75°C
      // tos: 80°C
      // O.S. polarity: 0  
      lm75_init(1,75,80,0);

      // DS1307 Real Time Clock initialization
      // Square wave output on pin SQW/OUT: Off
      // SQW/OUT pin state: 0
      rtc_init(0,0,0);   

      w1=rtc_read(0x32);
      w2=rtc_read(0x33);  
      number_of_start=w1+w2*0x0100;  
      
//Setting from DIP-Switch
     
     PORTA = 0xFF ;
     DDRA = 0x00 ;
     PORTG = 0x0F;
     delay_ms(1);

     if(PINA & 0x01)Lifttype='v';                    // hydraulics output      
     else if(PINA & 0x02)Lifttype='h';            // vvvf ouputs
     else  Lifttype='2';                                // 2 speed outputs 
                                                                
     if(PINA & 0x04)Numtype='n';       // normal nomerator and IO  system
     else Numtype='d'; 
     
     Eth_number=((PINA & 0b11100000)>>5) & 0b00000111;       
     
     #ifdef Ethernet_Disable
          Eth_number = 0;
     #endif
     
     
     if(Eth_number)
       {
          IP           =  Eth_number;
          IPADDRESS  = assemble_ipdot(192,168,0,IP);  
          NETMASK    = assemble_ipdot(255,255,255,0);
          GATEWAY    = assemble_ipdot(192,168,0,10); 
          MAIN2       = assemble_ipdot(192,168,0,119);
       }
     
     PORTG=0x1F;
     DDRA=0xFF;           
      
      // Global enable interrupts
      #asm("sei")    
}  
