#include "defines.h"                                                 
#include "90can128.h"

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

  
//irq 4 & irq5 rising edge enable  
   EICRA=0x00;                    //***\\
   EICRB=0x0F;
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
      
      unsigned char init1;
      
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

/*
     sw1  :   VVVF  - Hydrauilc / 2speed
     sw2  :   Hydraulic  - 2 speed
     sw3  :   Normal Numerator - Decreased Numerator
     sw4  : -----
     sw5  :   Grouping service - Single
     sw6/sw7/sw8: last IP  sector

*/
     
     PORTA = 0xFF ;     //dip switch reading
     DDRA = 0x00 ;
     PORTG = 0x0F;
     delay_ms(1);     
     
     init1=PINA;
     init1 &= 0b00000011;
     
     if(init1==1)Lifttype='v';
     else
     if(init1==2)Lifttype='h';
     else
     if(init1==3)Lifttype='d';
     else Lifttype='2';  
     
     if(Lifttype=='h')Hyd_Releveling='y';
       else Hyd_Releveling='n';
     
     
     init1=PINA;  
     init1 = init1>>2;
     init1 &= 0b00000011;
     
     if(init1==1)Numtype='n';        //separated  normal 2 digit numerator and IO  system
     else
     if(init1==2)Numtype='s';        //RS485 decreased mode   (hybrid numerator and IO sys for 7 stairs or lower)
     else
     if(init1==0)Numtype='d';      //CAN decreased mode  (hybrid numerator and IO sys for 7 stairs or lower)
     else Numtype='f';                 //RS485separated  normal 2 digit numerator and IO  system
                                                                
 //    if(PINA & 0x04)Numtype='n';       //separated  normal 2 digit numerator and IO  system
 //    else 
 //    if(PINA & 0x08)Numtype='s';        //RS485 decreased mode   (hybrid numerator and IO sys for 7 stairs or lower)
 //    else Numtype='d';                      //CAN decreased mode  (hybrid numerator and IO sys for 7 stairs or lower)
     
 
     
     if(PINA&0x20)
       {
         if(Lifttype=='v')Dir_delay='y';           //use only in lifttype='v'
       }                                           //remove contu or contd in cont_overlap delay(sec) after remove conts command
     else Dir_delay='n';     
     
     if(PINA&0x80)revision_open_door_en=1;
     else revision_open_door_en=0;
  
     init1=EE_Grouptype;
     if(((init1>'8')||(init1<'2'))&&(init1!='s'))EE_Grouptype=Grouptype;
          else  Grouptype=init1;
   //  if((PINA & 0x10)==0)Grouptype='s'; 

     init1 = EE_Eth_number;
     if(init1>254)EE_Eth_number = Eth_number;
          else  Eth_number = init1;     
          
     init1 = EE_Monitoring;
     if((init1!='y')&&(init1!='n'))EE_Monitoring = Monitoring;
          else  Monitoring = init1;  
     
     init1 = EE_Gateway_lsb;
     if(init1==255)EE_Gateway_lsb = Gateway_lsb;
          else  Gateway_lsb = init1;                                
            
     
     #ifdef Ethernet_Disable
          Eth_number = 0;
     #endif         
     
     #ifdef Numtype_Disable
          Numtype = 'd';
     #endif
    
     if(!Eth_number)
       {
          Monitoring = 'n';
          Grouptype = 's';  
          Gateway_lsb = 0;
        }        
     

// *******************   CAN Controller initialization  *********************

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
      
      
      #if CAN_BAUD_RATE==250
      // CAN System Clock: 4000.0 kHz
      CANBT1=0x06;
      // Propagation Time Segement: 1.750 us
      // Re-Sync Jump Width: 0.250 us
      CANBT2=0x0C;
      // Sample Point(s): 3
      // Phase Segment 1: 1.000 us
      // Phase Segment 2: 1.000 us
      CANBT3=0x37;            
      #endif


      #if CAN_BAUD_RATE==50      
      // CAN System Clock: 500.0 kHz
    CANBT1=0x3E;
     // Propagation Time Segement: 10.000 us
     // Re-Sync Jump Width: 2.000 us
    CANBT2=0x08;
     // Sample Point(s): 3
     // Phase Segment 1: 4.000 us
     // Phase Segment 2: 4.000 us
    CANBT3=0x13;
     #endif
     
  
     // CAN Timer Clock Period: 0.500 us
     CANTCON=0x00;

     if(Eth_number)
       {
          IP           =  Eth_number;
          IPADDRESS  = assemble_ipdot(192,168,0,IP);  
          NETMASK    = assemble_ipdot(255,255,255,0);
          GATEWAY    = assemble_ipdot(192,168,0,Gateway_lsb); 
          MAIN2       = assemble_ipdot(192,168,0,119); 
          BROADCAST  = assemble_ipdot(255,255,255,255);
       }
     
     PORTG=0x1F;
     DDRA=0xFF;           
      
      // Global enable interrupts
      #asm("sei")    
}                                                             

void set_default_values(void)
{
unsigned char sdv1;
unsigned int  sdv2; 
unsigned char sdv3;  
float sdv4;

WDRS=1;
  
sdv1 = EE_Floorsno;
if( (sdv1>64) || sdv1==0 )
        EE_Floorsno = Floorsno;
      else  Floorsno = sdv1;      
      
sdv1 = EE_Navistrategy;
if(!((sdv1=='s')||(sdv1=='d')||(sdv1=='f')))
       EE_Navistrategy = Navistrategy;         //EEProm has an invalid value
      else  Navistrategy = sdv1;
                                   //EEprom has a valid value
        
sdv1 = EE_Parkfloor;
if( (sdv1>Floorsno) || sdv1==0 )
       EE_Parkfloor = Parkfloor; 
      else  Parkfloor = sdv1;           

sdv1 = EE_Firefloor;
if( (sdv1>Floorsno) )
        EE_Firefloor = Firefloor;   
      else  Firefloor = sdv1;      

sdv1 = EE_VIPfloor;
if( (sdv1>Floorsno) )
        EE_VIPfloor = VIPfloor;   
      else  VIPfloor = sdv1;   
      
sdv1 = EE_Contofftd;
if(sdv1>50)
       EE_Contofftd = Contofftd;
     else Contofftd = sdv1;  

sdv1 = EE_Contofftu;
if( sdv1>50 )
       EE_Contofftu = Contofftu;
      else Contofftu = sdv1;        
            
sdv1 = EE_Cont_overlap;
if(sdv1>100)
        EE_Cont_overlap = Cont_overlap;
      else Cont_overlap = sdv1; 
      
sdv1 = EE_Timebreakdelay;
if(sdv1>10)
        EE_Timebreakdelay = Timebreakdelay;
      else Timebreakdelay = sdv1;       
      
if(Lifttype=='v'||Lifttype=='d')Fsflag=1;    

sdv1 = EE_Fsflag;
if((sdv1!=1)&&(sdv1!=0)&&(sdv1!=2))
        EE_Fsflag = Fsflag;
      else Fsflag = sdv1; 
      
sdv1 = EE_Calfs;
if((sdv1!=1)&&(sdv1!=0))
        EE_Calfs = Calfs;
      else Calfs = sdv1;   

sdv1 = EE_Needtocalibration;
if((sdv1!='y')&&(sdv1!='n'))
        EE_Needtocalibration = Needtocalibration;
      else Needtocalibration = sdv1;               
      
 sdv1 = EE_Oneflag;
if((sdv1!=1)&&(sdv1))
        EE_Oneflag = Oneflag;
      else Oneflag = sdv1;   
      
sdv1 = EE_Groundexist;
if(sdv1>6)
        EE_Groundexist = Groundexist;
      else Groundexist = sdv1;     
      
sdv1 = EE_Parksign;
if(sdv1>5)
        EE_Parksign = Parksign;
      else Parksign = sdv1;   
      
sdv1 = EE_Undergroundno;
if(sdv1>5)
        EE_Undergroundno = Undergroundno;
      else Undergroundno = sdv1;                    
   
sdv2 = EE_Calmt;
if( (sdv2>3000) || (sdv2==0) )
        EE_Calmt = Calmt;
      else Calmt = sdv2;

sdv2 = EE_Timepark;
if( (sdv2>3000) )
       EE_Timepark = Timepark;
      else Timepark = sdv2;  
      
sdv2 = EE_Timestndby;
if( (sdv2>3000) )
       EE_Timestndby = Timestndby;
      else Timestndby = sdv2;      
      
sdv2 = EE_Time1cf3;
if( (sdv2>300) || ((sdv2<60)&&(sdv2!=0)) )
        EE_Time1cf3 = Time1cf3;
      else Time1cf3 = sdv2;
      
sdv2 = EE_Time2cf3;
if( (sdv2>300) || ((sdv2<60)&&(sdv2!=0)) )
        EE_Time2cf3 = Time2cf3;
      else Time2cf3 = sdv2;        

sdv2 = EE_Timetravel;
if( (sdv2>60000) || (sdv2<130) )
        EE_Timetravel = Timetravel;
      else Timetravel = sdv2;         

WDRS=1;      
      
sdv2 = EE_Cf1mt;
if( (sdv2>300) || (sdv2<80) )
        EE_Cf1mt = Cf1mt;
      else Cf1mt = sdv2;     
      

sdv2 = EE_Doorwtc;
if( (sdv2>6000) )
      EE_Doorwtc = Doorwtc;
      else Doorwtc = sdv2;  
    
sdv1 = EE_Doornrc;
if( (sdv1>100) )
        EE_Doornrc = Doornrc;
      else Doornrc = sdv1; 

 
sdv1 = EE_Doorsiso;
if( (sdv1>200) || (sdv1==0) )
        EE_Doorsiso = Doorsiso;
      else Doorsiso = sdv1;  

sdv1 = EE_Doorunderl;
if( (sdv1!='y') && (sdv1!=68) && (sdv1!=69))
        EE_Doorunderl = Doorunderl;
      else Doorunderl = sdv1;    
      
sdv1 = EE_Doorclosepark;
if( (sdv1!='y') && (sdv1!='n') )
        EE_Doorclosepark = Doorclosepark;
      else Doorclosepark = sdv1;  
      
sdv1 = EE_Doorparkunderl;     
if( (sdv1!='y') && (sdv1!='n') )
        EE_Doorparkunderl = Doorparkunderl;
      else Doorparkunderl = sdv1;      
      
sdv1 = EE_Car_Call_Erase_En;
if( (sdv1!='y') && (sdv1!='n') )
        EE_Car_Call_Erase_En = Car_Call_Erase_En;
      else Car_Call_Erase_En = sdv1;                
                            
sdv2 = EE_Doordur;
if( (sdv2>600) || (sdv2==0) )
        EE_Doordur = Doordur;
      else Doordur = sdv2; 
      
sdv2 = EE_Doordur_open;
if( (sdv2>600) || (sdv2==0) )
        EE_Doordur_open = Doordur_open;
      else Doordur_open = sdv2;       
 
sdv1 = EE_Doortype;
if( (sdv1!='a') && (sdv1!='s') && (sdv1!='n'))
        EE_Doortype = Doortype;
      else Doortype = sdv1;   
      
sdv1 = EE_Door2type;
if( (sdv1!='a') && (sdv1!='s') && (sdv1!='n'))
        EE_Door2type = Door2type;
      else Door2type = sdv1;         
      
sdv1 = EE_Dooruldelay;
if(sdv1>200)
        EE_Dooruldelay = Dooruldelay;
      else Dooruldelay = sdv1;  
      
sdv1 = EE_Doorcdebouncet;
if(sdv1>50)
        EE_Doorcdebouncet = Doorcdebouncet;
      else Doorcdebouncet = sdv1;  
      
       
if(Lifttype=='2')floor_announce_delay=5;
  else floor_announce_delay=20;

sdv1 = EE_floor_announce_delay;
if(sdv1>40)
       EE_floor_announce_delay = floor_announce_delay;
     else floor_announce_delay = sdv1;  
     
sdv1 = EE_Music_Play_En;
if( (sdv1!='y') && (sdv1!='n') )
        EE_Music_Play_En = Music_Play_En;
      else Music_Play_En = sdv1;              
      
sdv1 = EE_Hyd_S2D_Time;
if(sdv1>250)
       EE_Hyd_S2D_Time = Hyd_S2D_Time;
     else Hyd_S2D_Time = sdv1;   
     
sdv1 = EE_Hyd_Start_Time;
if(sdv1>250)
       EE_Hyd_Start_Time = Hyd_Start_Time;
     else Hyd_Start_Time = sdv1;    
     
sdv1 = EE_Hyd_Stop_Time;
if(sdv1>250)
       EE_Hyd_Stop_Time = Hyd_Stop_Time;
     else Hyd_Stop_Time = sdv1;                
      
sdv1 = EE_SMS_Level;
if(sdv1>2)
        EE_SMS_Level = SMS_Level;
      else SMS_Level = sdv1;  

sdv1 = EE_IVA_comm;
if( (sdv1!='s') && (sdv1!='c') && (sdv1!='p'))
        EE_IVA_comm = IVA_comm;
      else IVA_comm = sdv1;                      
      
for(sdv2=0;sdv2<5;sdv2++)
   {
        sdv1 = EE_Password[sdv2];
        if( (sdv1>'9')||(sdv1<'0') )
           EE_Password[sdv2] = '0';                          
   }                  
        
sdv1 = EE_Maskfloor_Chksum;
if(sdv1>64)EE_Maskfloor_Chksum=0;

sdv3=0;
for(sdv2=0;sdv2<8;sdv2++) 
   {
        sdv1 = EE_Maskfloor[sdv2];
        if(!(sdv1 & 0b00000001))sdv3++;
        if(!(sdv1 & 0b00000010))sdv3++;        
        if(!(sdv1 & 0b00000100))sdv3++;        
        if(!(sdv1 & 0b00001000))sdv3++;        
        if(!(sdv1 & 0b00010000))sdv3++;        
        if(!(sdv1 & 0b00100000))sdv3++;        
        if(!(sdv1 & 0b01000000))sdv3++;        
        if(!(sdv1 & 0b10000000))sdv3++;        
   }
sdv1= EE_Maskfloor_Chksum;
if(sdv1 != sdv3)
        {
                 for(sdv2=0;sdv2<8;sdv2++)EE_Maskfloor[sdv2]=0xFF;   
                 EE_Maskfloor_Chksum = 0;
        }
for(sdv2=0;sdv2<8;sdv2++)
   {
       Maskfloor[sdv2] = EE_Maskfloor[sdv2];
   }    
   
        
/*****/      
   
sdv1 = EE_FirstDoor_Chksum;
if(sdv1>64)EE_FirstDoor_Chksum=0;

sdv3=0;
for(sdv2=0;sdv2<8;sdv2++) 
   {
        sdv1 = EE_FirstDoor[sdv2];
        if(!(sdv1 & 0b00000001))sdv3++;
        if(!(sdv1 & 0b00000010))sdv3++;        
        if(!(sdv1 & 0b00000100))sdv3++;        
        if(!(sdv1 & 0b00001000))sdv3++;        
        if(!(sdv1 & 0b00010000))sdv3++;        
        if(!(sdv1 & 0b00100000))sdv3++;        
        if(!(sdv1 & 0b01000000))sdv3++;        
        if(!(sdv1 & 0b10000000))sdv3++;        
   }
sdv1= EE_FirstDoor_Chksum;
if(sdv1 != sdv3)
        {
                 for(sdv2=0;sdv2<8;sdv2++)EE_FirstDoor[sdv2]=0xFF;   
                 EE_FirstDoor_Chksum = 0;
        }
                 

for(sdv2=0;sdv2<8;sdv2++)
   {
       FirstDoor[sdv2] = EE_FirstDoor[sdv2];
   } 
         
/*****/ 

 
  
sdv1 = EE_SecondDoor_Chksum;
if(sdv1>64)EE_SecondDoor_Chksum=64;

sdv3=0;
for(sdv2=0;sdv2<8;sdv2++) 
   {
        sdv1 = EE_SecondDoor[sdv2];
        if(!(sdv1 & 0b00000001))sdv3++;
        if(!(sdv1 & 0b00000010))sdv3++;        
        if(!(sdv1 & 0b00000100))sdv3++;        
        if(!(sdv1 & 0b00001000))sdv3++;        
        if(!(sdv1 & 0b00010000))sdv3++;        
        if(!(sdv1 & 0b00100000))sdv3++;        
        if(!(sdv1 & 0b01000000))sdv3++;        
        if(!(sdv1 & 0b10000000))sdv3++;        
   }
sdv1= EE_SecondDoor_Chksum;
if(sdv1 != sdv3)
        {
                 for(sdv2=0;sdv2<8;sdv2++)EE_SecondDoor[sdv2]=0x00;   
                 EE_SecondDoor_Chksum = 64;
        }
                 

for(sdv2=0;sdv2<8;sdv2++)
   {
       SecondDoor[sdv2] = EE_SecondDoor[sdv2];
   }  
   
/****/       
   
sdv1 = EE_HalfFloor_Chksum;
if(sdv1>64)EE_HalfFloor_Chksum=0;

sdv3=0;
for(sdv2=0;sdv2<8;sdv2++) 
   {
        sdv1 = EE_HalfFloor[sdv2];
        if(!(sdv1 & 0b00000001))sdv3++;
        if(!(sdv1 & 0b00000010))sdv3++;        
        if(!(sdv1 & 0b00000100))sdv3++;        
        if(!(sdv1 & 0b00001000))sdv3++;        
        if(!(sdv1 & 0b00010000))sdv3++;        
        if(!(sdv1 & 0b00100000))sdv3++;        
        if(!(sdv1 & 0b01000000))sdv3++;        
        if(!(sdv1 & 0b10000000))sdv3++;        
   }
sdv1= EE_HalfFloor_Chksum;
if(sdv1 != sdv3)
        {
                 for(sdv2=0;sdv2<8;sdv2++)EE_HalfFloor[sdv2]=0xFF;   
                 EE_HalfFloor_Chksum = 0;
        }
for(sdv2=0;sdv2<8;sdv2++)
   {
       HalfFloor[sdv2] = EE_HalfFloor[sdv2];
   }  
   
/****/        
sdv1 = EE_HalffloorSpeed;
if((sdv1!=0) &&  (sdv1!='s') && (sdv1!='r') && (sdv1!='b'))
        EE_HalffloorSpeed = HalffloorSpeed;
      else HalffloorSpeed = sdv1; 
      
sdv1 = EE_CA1Second_En;
if( (sdv1!=0) && (sdv1!=1))
        EE_CA1Second_En = CA1Second_En;
      else CA1Second_En = sdv1;    
      
sdv1 = EE_HalffloorDelay;
if( sdv1>200 || sdv1<10 )
        EE_HalffloorDelay = HalffloorDelay;
      else HalffloorDelay = sdv1;   
      
sdv1 = EE_OneMovefloorSpeed;
if((sdv1!=0) &&  (sdv1!='s') && (sdv1!='r') && (sdv1!='b'))
        EE_OneMovefloorSpeed = OneMovefloorSpeed;
      else OneMovefloorSpeed = sdv1; 
      
    
sdv1 = EE_OneMovefloorDelay;
if( sdv1>200 || sdv1<10 )
        EE_OneMovefloorDelay = OneMovefloorDelay;
      else OneMovefloorDelay = sdv1;                 
      
/****/    
      
   
/****/        
sdv1 = EE_Tunnel_Door_en;
if( (sdv1!='s') && (sdv1!='2') && (sdv1!='n'))
        EE_Tunnel_Door_en = Tunnel_Door_en;
      else Tunnel_Door_en = sdv1;  
/****/

sdv1 = EE_Enc_enable;
if( (sdv1!='y') && (sdv1!='n') )
        EE_Enc_enable = Enc_enable;
      else Enc_enable = sdv1;  
      
sdv1 = EE_Force_to_Rescue_En;
if( (sdv1!='y') && (sdv1!='n') )
        EE_Force_to_Rescue_En = Force_to_Rescue_En;
      else Force_to_Rescue_En = sdv1;        
      
sdv1 = EE_manual_floor_naming;
if( (sdv1!='y') && (sdv1!='n') )
        EE_manual_floor_naming = manual_floor_naming;
      else manual_floor_naming = sdv1;      


if(manual_floor_naming=='y')
 {
    for(sdv2=1;sdv2<65;sdv2++)
     {
        sdv1=EE_Floor_names_1[sdv2];
        if(sdv1!=0xFF)Floor_names_1[sdv2]=sdv1;   
        else {Floor_names_1[sdv2]='0'+(sdv2/10);EE_Floor_names_1[sdv2]='0'+(sdv2/10);} 
        sdv1=EE_Floor_names_2[sdv2];
        if(sdv1!=0xFF)Floor_names_2[sdv2]=sdv1;     
        else {Floor_names_2[sdv2]='0'+(sdv2%10);EE_Floor_names_2[sdv2]='0'+(sdv2/10);}   
     }
 }
                           
      
sdv1 = EE_Block_date_en;
if( (sdv1!='y') && (sdv1!='n') )EE_Block_date_en = 'n';


sdv1 = EE_Preopening_en;
if( (sdv1!='s') && (sdv1!='n') && (sdv1!='t') )
        EE_Preopening_en = Preopening_en;
      else Preopening_en = sdv1; 
      

 sdv1 = EE_Expired_date;   
 sdv3 = EE_Expired_month;   
 sdv2 = EE_Start_dayofyear;
  
if(EE_Block_date_en=='y')
{
  if((sdv1==0xFA)&&(sdv3==0xFA)&&(sdv2==0xFAFA))
  { 
      Expired_month=0xFA;
      Expired_date=0xFA;  
      Start_dayofyear=0xFAFA;
  }  
  else
  if((sdv1<=31)&&(sdv3<=12)&&(sdv1!=0)&&(sdv3!=0)&&(sdv2<366))
   { 
      Expired_month=sdv3;
      Expired_date=sdv1;  
      Start_dayofyear=sdv2;
   } 
  else
   { 
      EE_Expired_month=0xFF;
      Expired_month=0xFF;
      EE_Expired_date=0xFF;
      Expired_date=0xFF;
      Start_dayofyear=0xFFFF;
      EE_Start_dayofyear=0xFFFF;   
   }     
}
else
{
   if(sdv1!=0xFF)EE_Expired_date=0xFF;
   if(sdv3!=0xFF)EE_Expired_month=0xFF;
   if(sdv2!=0xFFFF)EE_Start_dayofyear=0xFFFF;  
   Expired_date=0xFF;  
   Expired_month=0xFF;
   Start_dayofyear=0xFFFF;
}
   
 
 

   
}             
