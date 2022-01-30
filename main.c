//#define debug 7         
                                                                                 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>                                                                                               
#include <90can128.h> 
#include <delay.h>

// I2C Bus functions
#asm
   .equ __i2c_port=0x0E                             ;PORTE
   .equ __sda_bit=3
   .equ __scl_bit=2
#endasm

#include <i2c.h>  

// DS1307 Real Time Clock functions
#include <ds1307.h>
#include <lm75.h> 

// include avrlib basics
#include "global.h"
#include "debug.h"
#include "net.h"
#include "debug.h"  
#include "arp.h"
#include "icmp.h"
#include "ip.h"
#include "nic.h"

#include "can.h"

#include "defines.h"
#include "misc.h"   



////////////////////////////// global variables//////////////////////////
//static volatile unsigned long UptimeMs;
unsigned char    RecData=0,side=0,vel=0,Door_Status=0x03;
unsigned char    buffer[100];
unsigned char    DataBuff[10];

bit                   ReadyFlag=0,ReadyFlag2=0;
int                   ElavNum=1;               
unsigned int     node_address=0x00;   //Main Address

unsigned char   hour,min,sec;
unsigned char   floor,w1,w2,w3,w5,w6;
int ptimer_preopening;

struct can_message current_message; 

struct message_serial_format
        {
	  unsigned char source;
	  unsigned char address;
	  unsigned char length;
    	  unsigned char data[8]; 
        }serial_current_message;

struct can_message  can_send_history[15 - TxStaMob];         
extern unsigned int  can_rxfull_mobs;
extern unsigned char cansrshow;           
 
unsigned char rx_buffer1[RX_BUFFER_SIZE1],tx_buffer1[TX_BUFFER_SIZE1],rx_index=0,Ready=0,Ready2=0;
unsigned char rx_buffer2[RX_BUFFER_SIZE1];  
unsigned char rx_buff[RX_BUFFER_SIZE1],SentTimeout=0;
unsigned char tx_wr_index1,tx_rd_index1,tx_counter1;
unsigned char End_Temp=0,Address=0,SerialSource=0 , rs485show=0; 


struct can_long_message long_message_buff,long_message_send_buff; 
unsigned long int ID; 
struct protocol_id_structure_sep sep_id; 
////////////////////////////// Ethernet functions///////////////////////
  
void netstackInit(uint32_t ipaddress, uint32_t netmask, uint32_t gatewayip,char EthAddrOffset);
void netstackUDPIPProcess(unsigned int len, udpip_hdr* packet);
//void netstackTCPIPProcess(unsigned int len, tcpip_hdr* packet);
void netstackIPProcess(unsigned int len, ip_hdr* packet);
void processCommand(u16 len, u08* data);
void serviceLocal(void);                
void netstackService(void);
u08* netstackGetBuffer(void);   
unsigned long int assemble_ipdot(unsigned char,unsigned char,unsigned char,unsigned char);

///////////////////////////// Can Functions/////////////////////////////////////////////    

// External Interrupt 5 service routine
interrupt [EXT_INT5] void ext_int5_isr(void)   //***\\
{
 if(!dir_cal)return;  
 if(!floor_cal)return;  
 
 if(Fsflag==2)
  {
    M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
    if(!CF3)
      {
       if(floor)_int5_en=2; 
         else  _int5_en=1; 
       EIMSK&=(~0x20);
       return;
      }  
    else 
       {   if(floor)_int4_en=2; 
             else   _int4_en=1;        
       }
      
  }    

 
 if(dir_cal!=dir_prev)
   {
    dir_prev=dir_cal;
       
   } 
 else
   {
     if(dir_cal==1)
       {floor_cal++;   
        if(floor_cal>=Floorsno)floor_cal=0;
       }
     if(dir_cal==2)
        {floor_cal--;                   
         if(floor_cal<=1)floor_cal=0;
        }
   }   
   
   EIMSK=0x00;   //disble interrupt   

   if(floor)_int5_en=2; 
     else  _int5_en=1; 
} 

// External Interrupt 4 service routine
interrupt [EXT_INT4] void ext_int4_isr(void)         //***\\
{ 
 if(!dir_cal)return;  
 if(!floor_cal)return;    
    
 
 if(Fsflag==2)                 //jahat dar
  {

    M_INT4 = ((PINE & 0x20)>>5)&0x01 ;
    if(!CF1)
      {
       if(floor)_int4_en=2; 
          else  _int4_en=1;             //not in level state
       EIMSK&=(~0x10);            //disbale current interrupt
       return;
      }  
    else         //in level state
      {
        if(floor)_int5_en=2; 
           else  _int5_en=1;       
      }
      
  }  
  else return;

 
 if(dir_cal!=dir_prev)
   {
    dir_prev=dir_cal;   
   } 
 else
   {
     if(dir_cal==1)
       {floor_cal++;
        if(floor_cal>=Floorsno)floor_cal=0;
        }
     if(dir_cal==2)
        {floor_cal--;                     
         if(floor_cal<=1)floor_cal=0;
        }  
    

   }   
   
   EIMSK=0x00;   //disble interrupts    

   if(floor)_int4_en=2; 
     else  _int4_en=1; 
}

//     Timer 1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{ 
 
     TIMSK1=0x00;                                                   
      ptimer++;
      if(ptimer==60000)ptimer=0; 
      if(ptimer_preopening)ptimer_preopening++;

      if(Eth_number)
      {    
         netstackService();    	// service the network    
      }
     
    
    if(I) traveltimer++;
          else traveltimer=0;                                                              
          
    
     if(!_inp_in_use)interpreter(); 
                      
     LED_indicator();

     supervisor();  

   
if(beep_request_on)
        {BUZ=1;
         beep_request_on=0;}
else
if(beep_request)
        {beep_request=0;
         BUZ=0;
         beep_request_on=1;}    
         
      // Reinitialize Timer 1 value
      TCNT1H = 0x9E ;
      TCNT1L = 0x58 ; 

   TIMSK1=0x01;                
     
} 


interrupt [TIM3_OVF] void timer3_ovf_isr(void)
{
    TCNT3H=0xE1;
    TCNT3L=0x7B;   

    if(!WDRS)
          {
           #asm("cli")
          TIMSK0=0;TIMSK1=0;
          TIMSK2=0;TIMSK3=0;
          }
    if(WDRS==1)
          {
           #asm("wdr")
           WDRS=0;
          }
     
     if(SentTimeout==1)SentTimeout=2;    //To restore Serial variables
     else  if(SentTimeout==2)               // in critical Mode
                         {
                            End_Temp=0;  
                            rx_index=0;   
                            SentTimeout=0;                                
                         }      
                         
    if(_int5_en==1)_int5_en=2;
    else
    if(_int5_en==2)_int5_en=3;
    else if(_int5_en==3)
                         {
                           if(dir_cal)
                              {EIFR|=0x20;
                               EIMSK|=0x20;
                               }
                               _int5_en=0;
                         }       

    if(_int4_en==1)_int4_en=2;
    else
    if(_int4_en==2)_int4_en=3;
    else if(_int4_en==3)
                         {
                           if(dir_cal)
                              {EIFR|=0x10;
                               EIMSK|=0x10;
                               }
                               _int4_en=0;
                          }  


    //if(BUZ==0)BUZ=1;
    
    if(beep_en)
          {BUZ=0;
           beep_en=0;}
    else BUZ=1;
    
    if(LED_blink)LED_blink=0;
      else LED_blink=1;    
         
    if(!blink_en)
    {         
         PORTC.0=0;PORTC.1=0;
         PORTC.2=0;PORTC.3=0;
         OE_7seg=0;
         return;
    }
   if(!OE_7seg)
    {
         PORTC.1=1;PORTC.2=1;
         PORTC.3=1;OE_7seg=1;
         return;
    }
    else
     if(OE_7seg)
     {
         PORTC.1=0;PORTC.2=0;
         PORTC.3=0;OE_7seg=0;
         return;
     } 
} 
      

interrupt [USART0_RXC] void usart0_rx_isr(void)
{     
 	RecData = UDR0;          
	ReadyFlag =1;          
	ReadyFlag2 =1;     
}       
  


interrupt [USART1_RXC] void usart1_rx_isr(void)
{
   char status,data,jtemp,ktemp;
   status=UCSR1A;
   data=UDR1; 
   if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
      
      if(rx_index==0)SentTimeout=1;
      
      if (rx_index < RX_BUFFER_SIZE1) 
        {
           rx_buff[rx_index]=data;
           rx_index++;
        }  
      else  rx_index=0; 
      
      if (data==0xFF)
      {            
         if (End_Temp==0)
                       End_Temp=1;
         else  
         {
           if(rx_index>5)        //validity in end and start of data frame
             {  
               if(rx_buff[3]==node_address)
                      {
                        Ready = 1;    
                        rs485show = 1;
                        Address = rx_buff[3];   
                        SerialSource = rx_buff[2];
                        for(jtemp=0;jtemp<rx_index-5;jtemp++) rx_buffer1[jtemp]=rx_buff[jtemp+4];  
                        for(ktemp=jtemp;ktemp<=RX_BUFFER_SIZE1;ktemp++)rx_buffer1[ktemp]=0;
                       }
             }
               End_Temp=0;  
               rx_index=0;   
               SentTimeout=0;       
         }  
      }
      else
        End_Temp=0;    
   }   
  
}             


interrupt [USART1_TXC] void usart1_tx_isr(void)
{
        if (tx_counter1)
        {
             --tx_counter1;
             UDR1=tx_buffer1[tx_rd_index1];
             if (++tx_rd_index1 == TX_BUFFER_SIZE1) tx_rd_index1=0;
        }
        else
             PORTD.7=0;

}

void transmit_data_422(unsigned int SendData)
{   
  
if (tx_counter1 || ((UCSR1A & DATA_REGISTER_EMPTY)==0))
   {
   tx_buffer1[tx_wr_index1]=SendData;
   if (++tx_wr_index1 == TX_BUFFER_SIZE1) tx_wr_index1=0;
   ++tx_counter1;
   }
else
   {
     PORTD.4=1;
     delay_us(10);  
     UDR1=SendData;    
   }   
   
}             

void transmit_data_485(unsigned int SendData)
{   
    
if (tx_counter1 || ((UCSR1A & DATA_REGISTER_EMPTY)==0))
   {
   tx_buffer1[tx_wr_index1]=SendData;
   if (++tx_wr_index1 == TX_BUFFER_SIZE1) tx_wr_index1=0;
   ++tx_counter1;
   }
else
   {
     PORTD.7=1;
     delay_us(10);  
     UDR1=SendData;    
   }
  
}    

unsigned char RS422_send_message(struct message_serial_format current_message_422)
{
    unsigned char j,waitforsend=0;
    delay_ms(2);    
    while(rx_index)
          {if(++waitforsend>50)return;
           delay_ms(1);}
    transmit_data_422(0xFE);
    transmit_data_422(0xFE);    
    transmit_data_422(current_message_422.source);     
    transmit_data_422(current_message_422.address);  
    for(j=0;j<current_message_422.length;j++)
          transmit_data_422(current_message_422.data[j]);       
    transmit_data_422(0xFF);
    transmit_data_422(0xFF);     
}        

unsigned char RS485_send_message(struct message_serial_format current_message_485)
{
    unsigned char j,waitforsend=0;      

    while(rx_index)
         {if(++waitforsend>50) return;
           delay_ms(1);}                      

    transmit_data_485(0xFE);
    transmit_data_485(0xFE);                                    
    transmit_data_485(current_message_485.source);     
    transmit_data_485(current_message_485.address);  
    for(j=0;j<current_message_485.length;j++)
          transmit_data_485(current_message_485.data[j]);       
    transmit_data_485(0xFF);
    transmit_data_485(0xFF); 
    
    delay_ms(5);        
    return 0;    
} 

void main(void)
{         
struct netEthAddr myEthAddress;  
      
WDRS=1;    //for Watchdog reset         
      
delay_ms(100);          //for starting up   
                               
initialize();
           
errormsg(0);
Write_7Seg(0,0);      

      
      #ifdef debug
      printf("\r\n Start...");      
      #endif
      
WDRS=1;    //for Watchdog reset        
      
  

//rescueviamain();    

       
set_default_values(); 
      
WDRS=1;    //for Watchdog reset      


 if(Eth_number)
        {
          netstackInit(IPADDRESS, NETMASK, GATEWAY,Eth_number);            //Unremark for Using Ethernet
          delay_ms(100);                                              //Unremark for Using Ethernet
         //  nicGetMacAddress(&myEthAddress.addr[0]);            //Unremark for Using Ethernet
 	  myEthAddress.addr[0] =0x30;
  	  myEthAddress.addr[1] =0x46;
  	  myEthAddress.addr[2] =0x46;
 	  myEthAddress.addr[3] =0x49;
  	  myEthAddress.addr[4] =0x43;
  	  myEthAddress.addr[5] =0x45+Eth_number;     
  	  WDRS=1;
        }  


delay_ms(100);

              
WDRS=1;    //for Watchdog reset  
       
if (MCUSR & 8)             //Reset by watchdog
        {
        // Watchdog Reset
          MCUSR&=0xE0;
          errormsg(36);
          ptimer=0;
          while(ptimer<30)
           {
             WDRS=1;    //for Watchdog reset                
             delay_ms(100);
             ptimer++;                         //delay 4 seconds in start
             LED_indicator();
             interpreter(); 
             if(Eth_number)netstackService();    	// service the network
           }    
          errormsg(0); 
        }          

while(1)
{   
                 //service local stuff		
           	 //    serviceLocal();
                                    
  WDRS=1;    //for Watchdog reset  

  ClearAllMailbox();
  
  WDRS=1;    //for Watchdog reset 
  
  delay_ms(100);
    
  sep_id.dest_address = 0x00; 
  sep_id.source_address = node_address;
  sep_id.message_num = 0; 
  sep_id.priority = 0;  
  ID = assemble_id(sep_id);
     
  current_message.datalen=8;
  current_message.ide=1;
  current_message.rb=0;
  current_message.rtr=0;
  current_message.id=ID;
  current_message.data[0]=0;
  current_message.data[1]=0;
  current_message.data[2]=0;
  current_message.data[3]=0;
  current_message.data[4]=0;
  current_message.data[5]=0;
  current_message.data[6]=0;
  current_message.data[7]=0;        
      
  set_mobs_for_receive(8);   
  
  serial_current_message.length = 8;
  serial_current_message.source = node_address;    
  serial_current_message.address = 0x00;  
  serial_current_message.data[0] = 0x00;
  serial_current_message.data[1] = 0x00;
  serial_current_message.data[2] = 0x00; 
  serial_current_message.data[3] = 0x00;
  serial_current_message.data[4] = 0x00;
  serial_current_message.data[5] = 0x00;   
  serial_current_message.data[6] = 0x00;
  serial_current_message.data[7] = 0x00;
 
  opmode=0;            //Not in any operation mode
  standbymf=0;
  parkmf=0;
  firemf=0; 
  VIPmf=0;
  _pf_reform=0;
  _pf_park=0;
  blink_en=0;
  turnoffleds=0;
  _set_numerator=0;     
  set_annunciator=0;  
  arrivedstair=0; 
  _delay_unload_door=0;    
  _delay_park_unload_door=0;   
  ptimer_preopening=0;   
     

  FTOC=0;
  OVLC=0;
  FLTC=0;
  FULC=0;

   
 TIMSK1=0x00;
//Set Timer1 for 100ms and STOP it

 WDRS=1;    //for Watchdog reset 
 
 delay_ms(50);  
 
 if(forcetoreset)
  {
            forcetoreset=0;           
            serial_current_message.length=4;
            serial_current_message.address=0xD2;
            serial_current_message.data[0]='S';
            serial_current_message.data[1]='R';
            serial_current_message.data[2]='s'; 
            serial_current_message.data[3]='t';
            serial_current_message.data[4]=0;
            serial_current_message.data[5]=0;   
            serial_current_message.data[6]=0x00;
            serial_current_message.data[7]=0x00;
            RS485_send_message(serial_current_message);   
            delay_ms(10);
  }  

 _volume_set_rs485=2; 
 sep_id.dest_address = 0xD1;              //Annunciator Board on CAN-BUS
 ID = assemble_id(sep_id); 
 current_message.id = ID;	
 current_message.datalen=2;         
 current_message.data[0]='G';     
 current_message.data[1]='V';  
 can_send_message(current_message);
 delay_ms(50);        

 sep_id.dest_address = 0xC0;              //Annunciator Board  via CABIN BUS
 ID = assemble_id(sep_id); 
 current_message.id = ID;	
 current_message.datalen=4;
 current_message.data[0]='X'; 
 current_message.data[1]=0xD1; 
 current_message.data[2]='G';
 current_message.data[3]='V';
 can_send_message(current_message);
 delay_ms(50);        

 serial_current_message.address=0xD1;
 serial_current_message.length=2;
 serial_current_message.data[0]='G';          
 serial_current_message.data[1]='V';           
 RS485_send_message(serial_current_message);    
 delay_ms(10);    
 
 interpreter();   

 
 if(Numtype=='n')                                       //Normal numerator
 {
    for(mb1=0;mb1<((Floorsno-1)/8+1);mb1++)       //Reset Floor Boards outputs
     {
        WDRS=1;    //for Watchdog reset  
        sep_id.dest_address = 0xF0+mb1;
        ID = assemble_id(sep_id); 
        current_message.id = ID;	
        current_message.datalen=1;        
        current_message.data[0]='R';    //R
        can_send_message(current_message);
        delay_ms(50);
     } 
 
  for(mb1=0;mb1<((Floorsno-1)/16+1);mb1++)        //Reset Extension Board outputs
   {
        WDRS=1;    //for Watchdog reset   
        sep_id.dest_address = 0xE0+mb1;
        ID = assemble_id(sep_id);
        current_message.id = ID;	        
        current_message.datalen=1;         
        current_message.data[0]='R';      //R
        can_send_message(current_message);
        delay_ms(50);
    }           
    
        WDRS=1;    //for Watchdog reset           
        sep_id.dest_address = 0xD0;              //Numerator Board
        ID = assemble_id(sep_id); 
        current_message.id = ID;	
        current_message.datalen=1;         
        current_message.data[0]='R';      //R
        can_send_message(current_message);
        delay_ms(50);            
        
 }  
 else
 if(Numtype=='d')                            //Decreased Numerator
 {
        WDRS=1;    //for Watchdog reset           
        sep_id.dest_address = 0xB0;              //Num-Floor Board
        ID = assemble_id(sep_id); 
        current_message.id = ID;	
        current_message.datalen=1;         
        current_message.data[0]='R';      //R
        can_send_message(current_message);
        delay_ms(50); 

        WDRS=1;    //for Watchdog reset           
        sep_id.dest_address = 0x10;              //Num-Ext Board
        ID = assemble_id(sep_id); 
        current_message.id = ID;	
        current_message.datalen=1;         
        current_message.data[0]='R';      //R
        can_send_message(current_message);
        delay_ms(50);     
}    
 else
 if(Numtype=='s')                         
 {
   serial_current_message.length=1;
   serial_current_message.address=0xD3;
   serial_current_message.data[0]='R';
   RS485_send_message(serial_current_message);   

   sep_id.dest_address = 0xC0;              //Cabin Board
   ID = assemble_id(sep_id); 
   current_message.id = ID;	
   current_message.datalen=1;         
   current_message.data[0]='R';      //R
   can_send_message(current_message);
   delay_ms(50);       
   
 }   
 else
 if(Numtype=='f')                         
  {
    for(mb1=0;mb1<((Floorsno-1)/8+1);mb1++)       //Reset Floor Boards outputs
     { 
        serial_current_message.length=1;
        serial_current_message.address=0xF0+mb1;
        serial_current_message.data[0]='R';
        RS485_send_message(serial_current_message);        
        WDRS=1;    //for Watchdog reset  
        delay_ms(50);
     } 
 
  for(mb1=0;mb1<((Floorsno-1)/16+1);mb1++)        //Reset Extension Board outputs
   {
     sep_id.dest_address = 0xC0;              //Cabin Board
     ID = assemble_id(sep_id); 
     current_message.id = ID;	
     current_message.datalen=3;         
     current_message.data[0]='X';       
     current_message.data[1]=0xE0+mb1;     
     current_message.data[2]='R';      //R      
     can_send_message(current_message);        
     delay_ms(50);
    }           
    
        sep_id.dest_address = 0xC0;              //Numerator
        ID = assemble_id(sep_id); 
        current_message.id = ID;	
        current_message.datalen=3;         
        current_message.data[0]='X';       
        current_message.data[1]=0xD0;     
        current_message.data[2]='R';      //R      
        can_send_message(current_message);        
        delay_ms(50);        
        
        WDRS=1;    //for Watchdog reset           
        serial_current_message.length=1;
        serial_current_message.address=0xD0;
        serial_current_message.data[0]='R';
        RS485_send_message(serial_current_message);  
        delay_ms(50);            
        
 }                
 

 WDRS=1;    //for Watchdog reset 
 interpreter();          
 
 if(IVA_comm=='c')
 {
   sep_id.dest_address = 0xD1;              //Annunciator Board on CAN-BUS
   ID = assemble_id(sep_id); 
   current_message.id = ID;	
   current_message.datalen=1;         
   current_message.data[0]='R';      
   can_send_message(current_message);
   delay_ms(50);        
 }
 else
 if(IVA_comm=='s')
 {
   sep_id.dest_address = 0xC0;              //Annunciator Board  via CABIN BUS
   ID = assemble_id(sep_id); 
   current_message.id = ID;	
   current_message.datalen=3;
   current_message.data[0]='X'; 
   current_message.data[1]=0xD1; 
   current_message.data[2]='R';
   can_send_message(current_message);
   delay_ms(50);        
 }     
 else
 if(IVA_comm=='p')                         //ANnnunciatior Board on RS485
 {
   serial_current_message.address=0xD1;
   serial_current_message.length=1;
   serial_current_message.data[0]='R';          
   RS485_send_message(serial_current_message);    
   delay_ms(10);       
 } 
 _volume_set_rs485=0;
 
if(SMS_Level)
     {
        serial_current_message.address=0xD4;
        serial_current_message.length=8;
        sprintf(serial_current_message.data,SERIAL_NUMBER);
        serial_current_message.data[0]='S';          
        RS485_send_message(serial_current_message);    
        delay_ms(10);
     }    
 

 WDRS=1;    //for Watchdog reset          
 
    
   
 if((Tunnel_Door_en=='s')||(Tunnel_Door_en=='2'))
  {
    sep_id.dest_address = 0xC0;              //Cabin Board
    ID = assemble_id(sep_id); 
    current_message.id = ID;	
    current_message.datalen=3;         
    current_message.data[0]='T';      //send parameter for tunnel door 
    current_message.data[1]='u';
    current_message.data[2]=Tunnel_Door_en;
    can_send_message(current_message);
    delay_ms(50);  
   }  
           
 if(Numtype=='s')
   {
      mb1=1;            
      do
      {
       if(showfloor(mb1,2)=='-')
        {
         mb1=0;
         break;
        } 
       if(mb1>64)break;
       WDRS=1;
       mb1++;
      }while(mb1<=Floorsno); 
      
     sep_id.dest_address = 0xC0;              //Cabin Board
     ID = assemble_id(sep_id); 
     current_message.id = ID;	
     current_message.datalen=3;         
     current_message.data[0]='M';      //send parameter for tunnel door 
     current_message.data[1]='e';
            
     serial_current_message.length=3;
     serial_current_message.address=0xD3;
     serial_current_message.data[0]='M';
     serial_current_message.data[1]='e';
                 
     if(mb1==0)
               {
                    current_message.data[2]='y'; 
                    serial_current_message.data[2]='y';      
               }   
     else
               {
                    current_message.data[2]='n'; 
                    serial_current_message.data[2]='n';               
               } 
     can_send_message(current_message);
     RS485_send_message(serial_current_message);  
     
     delay_ms(20);      
   
   }    
         

// if(!firstchk)RELAYSTBY=1;  else RELAYSTBY=0;        //***//
 RELAYSTBY=1;                                          //***//
 RELAYO=0;
 //if(!I)RELAYC=0;
 if(!(CONTU||CONTD))RELAYC=0;          //open door if not moving
 URA=0;  
 CONTF=0;				//First Stop!!
 CONTU=0;                   
 CONTD=0;
 CONTS=0;
 CONTM=0;
 CONTL=0; 
 if(!firstchk)set_outputs_relay(1);         
 if(firstchk)set_outputs_relay(0);     //don't send in first loop   
 

 WDRS=1;    //for Watchdog reset   

 delay_ms(100);      

if(Eth_number)      //Reset ALL request packet
 {
      if(Monitoring == 'y')
        {    
          
          DataBuff[0]=0xFF;
          DataBuff[1]=0xFF;
          DataBuff[2]=0x01;     //Status Packet
          DataBuff[3]=0xF6;     
          DataBuff[4]=0;        
          DataBuff[5]=0;       
          DataBuff[6]=0;        
          DataBuff[7]=IP;       //IP address 
          for(mb1=0;mb1<=7;mb1++)
             buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+mb1]=DataBuff[mb1]; 
          udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);     
        }
 }

ptimer=10;    
if(!firstchk)ptimer=0;
while(ptimer<20)
{
         WDRS=1;    //for Watchdog reset                
         delay_ms(100);
         ptimer++;                         //delay 4 seconds in start
         LED_indicator();
         interpreter();
         if(Eth_number)netstackService();    	// service the network 
}                   

read_inputs();
if((FEEDBF||FEEDBU||FEEDBD||FEEDBS))     //!FEEDB4BS||
	{if(firstchk)
		{errormsg(32);		//Contcea
		 errormsg(88);} 
	 do{	 
                 WDRS=1;    //for Watchdog reset  	 
		 delay_ms(100);
		 read_inputs();
	    }while(FEEDBS||FEEDBU||FEEDBF||FEEDBD);  
					//Waiting for Conts. to be freed
	}  
	   
 I=0;       //Movement Flag "I" reset

 LED_indicator();
 
 WDRS=1;    //for Watchdog reset    	


 if((Eth_number)&&(firstchk))
        {
          netstackInit(IPADDRESS, NETMASK, GATEWAY,Eth_number);            //Unremark for Using Ethernet
          delay_ms(100);                                              //Unremark for Using Ethernet
         //  nicGetMacAddress(&myEthAddress.addr[0]);            //Unremark for Using Ethernet
 	  myEthAddress.addr[0] =0x30;
  	  myEthAddress.addr[1] =0x46;
  	  myEthAddress.addr[2] =0x46;
 	  myEthAddress.addr[3] =0x49;
  	  myEthAddress.addr[4] =0x43;
  	  myEthAddress.addr[5] =0x45+Eth_number;     
  	  WDRS=1;
        }  
        
 
sep_id.source_address = node_address;
sep_id.message_num = 0;
sep_id.priority = 0;  
current_message.datalen=4;
current_message.data[0]='S';          
current_message.data[1]='E'; 
current_message.data[2]='R';          
current_message.data[3]='n'; 
        
if(Numtype=='n')
{
 sep_id.dest_address = 0xD0;           
 ID = assemble_id(sep_id);
 current_message.id = ID;                                   
 can_send_message(current_message);
 delay_ms(10);  
}
else                
if(Numtype=='d')
{
 sep_id.dest_address = 0xB0;            //num-Floor
 ID = assemble_id(sep_id);
 current_message.id = ID;                                   
 can_send_message(current_message);
 delay_ms(10);     
 sep_id.dest_address = 0x10;            //num-Ext
 ID = assemble_id(sep_id);
 current_message.id = ID;                                   
 can_send_message(current_message);
 delay_ms(10); 
}                  
else
if(Numtype=='s')
{   
serial_current_message.address=0xD3;
serial_current_message.length=4;
serial_current_message.data[0]='S';          
serial_current_message.data[1]='E'; 
serial_current_message.data[2]='R';          
serial_current_message.data[3]='n';    
RS485_send_message(serial_current_message);   

 sep_id.dest_address = 0xC0;            //cabin
 ID = assemble_id(sep_id);
 current_message.id = ID;                                   
 can_send_message(current_message);
 delay_ms(10); 
}    
else       
if(Numtype=='f')
{   
serial_current_message.address=0xD0;
serial_current_message.length=4;
serial_current_message.data[0]='S';          
serial_current_message.data[1]='E'; 
serial_current_message.data[2]='R';          
serial_current_message.data[3]='n';    
RS485_send_message(serial_current_message);   

current_message.datalen=6;    
current_message.data[0]='X';       
current_message.data[1]=0xD0; 
current_message.data[2]='S';          
current_message.data[3]='E'; 
current_message.data[4]='R';          
current_message.data[5]='n'; 
 sep_id.dest_address = 0xC0;            //cabin
 ID = assemble_id(sep_id);
 current_message.id = ID;                                   
 can_send_message(current_message);
 delay_ms(10); 
}   


if(Eth_number)
       {
         if(Monitoring == 'y')_reset_out_request = 0xFF;   //ethernet_enable
         if(Grouptype!='s')Send_Request_via_Eth(0,'E');
        }         
         
WDRS=1;    //for Watchdog reset

ptimer=0;
//******************* First check ***************************    
while(1)
{   
 delay_ms(100); 
 LED_indicator();
 interpreter();    
 if(Eth_number)netstackService();    	// service the network
 read_inputs(); 
 if(forcetoreset)break;  

 WDRS=1;    //for Watchdog reset  
 
 if(Force_to_Rescue_En=='y')
     { 
 
       if((LPC_error<100)&&(LPC_error>90))
        {

         ptimer++;
         if(ptimer==100)  //force to rescue after 10 seconds
          {
                
                if( (!CF1&&Fsflag!=2) || ( !(CF1&&CF3) && Fsflag==2) )
                 {
                   serial_current_message.address=0xD5;
                   serial_current_message.length=3;
                   serial_current_message.data[0]='F';          
                   serial_current_message.data[1]='T'; 
                   serial_current_message.data[2]='R';          
                   serial_current_message.data[3]=0;    
                   RS485_send_message(serial_current_message);          
                   delay_ms(10);
                   needreset=1;
                  }
          }
      
       }
       
     }    

 if(!POWER24)
	{
	 if((firstchk)||(last_display_msg!=51))			//"firstchk" set in initializing() just one time
		errormsg(51);		//alarm & announce if it's first time
         continue;
	}
	
if(!POWER110)
	{if((firstchk)||(last_display_msg!=52))
		errormsg(52);//alarm & announce if it's first time
         continue;
	}   
	
 if(FLT)
	{if((firstchk)||(last_display_msg!=LPC_error))	
	     {
                if(!LPC_error)
                       {
                          ptimer=0;
                          while(ptimer<5)
                            {
                             interpreter();
                             if(LPC_error)break;
                             delay_ms(100);
                             if(Eth_number)netstackService();    	// service the network
                             #asm("WDR")
                             ptimer++;
                            } 
                          ptimer=0;  
                          if(!LPC_error)
                            {
                             if((Lifttype=='v')||(Lifttype=='d'))
                              {
                               errormsg(99);
                               LPC_error=99;
                              } 
                             else 
                              {
                               errormsg(91);
                               LPC_error=91;
                              }                              
                            } 
                            else errormsg(LPC_error);
                       }
                     else errormsg(LPC_error);		//Alarm and announce it
             }		
         continue;
	}	

 if(!SS63)
	{if((firstchk)||(last_display_msg!=63))
		{errormsg(63);		//Alarm and anounce it
		 }		
         continue;
	}
 else
 if(!SS64)
	{if((firstchk)||(last_display_msg!=64))
		{errormsg(64);		//Alarm and announse it
		 }		
         continue;
	}
 else	
 if(!SS65)
	{if((firstchk)||(last_display_msg!=65))
		{errormsg(65);		//Alarm and announce it
		 }		
         continue;
      }
  
 if(FTO)
	{if((firstchk)||(last_display_msg!=82))			
		{errormsg(82);		//Alarm and announce it
		 }		
         continue;
	}
	 
 
 if(CA1 && CAn)
	{if((firstchk)||(last_display_msg!=71))
		{errormsg(71);		//Alarm and announce it
		 }		//"Error in start"
         continue;
	}                                                                                                   

 if((Fsflag==0)||(Fsflag==1))
 {
  if(CF3 && CF1)
	{if((firstchk)||(last_display_msg!=72))
		{errormsg(72);		//Alarm and announce it
		 }		//"Error in Start"
         continue;
	}
 }
 break;
 }          //while(1)  for   first checking
 
 firstchk=0; 				//Clear "firstchk" for not showing	 
 LPC_error =0;
 

 if(needreset)
	{needreset=0;                  //Alarm and announce it  the board must be reset
         blink_en=1;                   //For blinking   
         beep_en=1;  
         
         WDRS=1;    //for Watchdog reset   
         
         sep_id.source_address = node_address;       //For Blinking 'E' at numerator
         sep_id.message_num = 0;
         sep_id.priority = 0;  
         current_message.datalen=2;	
         current_message.data[0]='D';          
         current_message.data[1]='B';   
         
         if(Numtype=='n')
         {
          sep_id.dest_address = 0xD0;           
          ID = assemble_id(sep_id);
          current_message.id = ID;                                   
          can_send_message(current_message);
          delay_ms(10);   
         }
         else    
         if(Numtype=='d')
         {
         sep_id.dest_address = 0xB0;            //numerator
         ID = assemble_id(sep_id);
         current_message.id = ID;                                   
         can_send_message(current_message);
         delay_ms(10);         
         sep_id.dest_address = 0x10;            //numerator
         ID = assemble_id(sep_id);
         current_message.id = ID;                                   
         can_send_message(current_message);
         delay_ms(10);} 
         else
         if(Numtype=='s')
          {   
           serial_current_message.address=0xD3;
           serial_current_message.length=2;
           serial_current_message.data[0]='D';          
           serial_current_message.data[1]='B'; 
           RS485_send_message(serial_current_message);   
           
            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID;                                   
            can_send_message(current_message);
            delay_ms(10);            
         }    
         else
         if(Numtype=='f')
          {   
           serial_current_message.address=0xD0;    //numerator
           serial_current_message.length=2;
           serial_current_message.data[0]='D';          
           serial_current_message.data[1]='B'; 
           RS485_send_message(serial_current_message);   
           
            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID; 
            current_message.datalen=4;  
            current_message.data[0]='X';               //send to numerator via cabin
            current_message.data[1]=0xD0;               
            current_message.data[2]='D';          
            current_message.data[3]='B';                                                 
            can_send_message(current_message);
            delay_ms(10);            
         }                
         
         WDRS=1;    //for Watchdog reset 
	 while(1)
	     {
   	       delay_ms(100);
	       LED_indicator();
	       interpreter(); 
	       if(Eth_number)netstackService();    	// service the network 
	       if(forcetoreset)break;
	       WDRS=1;    //for Watchdog reset 
	      }
	      
	  if(forcetoreset)
	     { 
    	      needreset=0;
	      break;
	     }   
	      
	}     
	
ptimer=0;
while(ptimer<10)
{
         WDRS=1;    //for Watchdog reset  
         delay_ms(100);
         ptimer++;                         //delay 4 seconds in start
         LED_indicator();
         interpreter(); 
         if(Eth_number)netstackService();    	// service the network
}       	



//*********   THIS FUNCTION IS NAVIGATING 
//*********   LIFT IN ON-CAR REVISION MODE   

read_inputs();
if(REVC)
	{
          WDRS=1;    //for Watchdog reset  	  

	  opmode='c';
	  errormsg(83);		            //for display rev c 

          sep_id.source_address = node_address;
          sep_id.message_num = 0;
          sep_id.priority = 0;  
          current_message.datalen=4;	
          current_message.data[0]='S';          //'S' for first digit of numerator
          current_message.data[1]='R'; 
          current_message.data[2]= 0 ;
          current_message.data[3]='n';    
          if(Numtype=='n')
          {
           sep_id.dest_address = 0xD0;            //numerator
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);
          }  
          else               
         if(Numtype=='d')          
          {
           sep_id.dest_address = 0xB0;          
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);            
           sep_id.dest_address = 0x10;          
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);
          }                   
         else
         if(Numtype=='s')
          {   
           serial_current_message.address=0xD3;
           serial_current_message.length=4;
           serial_current_message.data[0]='S';          
           serial_current_message.data[1]='R'; 
           serial_current_message.data[2]=0;          
           serial_current_message.data[3]='n';    
           RS485_send_message(serial_current_message);  

            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID;                                   
            can_send_message(current_message);
            delay_ms(10);
           }      
         else
         if(Numtype=='f')
          {   
           serial_current_message.address=0xD0;
           serial_current_message.length=4;
           serial_current_message.data[0]='S';          
           serial_current_message.data[1]='R'; 
           serial_current_message.data[2]=0;          
           serial_current_message.data[3]='n';    
           RS485_send_message(serial_current_message);  

            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID; 
            current_message.datalen=6;	
            current_message.data[0]='X';       
            current_message.data[1]=0xD0;    
            current_message.data[2]='S';          //'S' for first digit of numerator
            current_message.data[3]='R'; 
            current_message.data[4]= 0 ;
            current_message.data[5]='n';               
                                   
            can_send_message(current_message);
            delay_ms(10);
           }                   	
	}
      WDRS=1;    //for Watchdog reset            				                
	
/***********************************************************/


       revision_c();


/************************************************************/

				//End of Revision_c while loop
 revmove=0;

if(needreset)continue;

WDRS=1;    //for Watchdog reset                             

read_inputs();
if(FEEDBD || FEEDBU)
	{   
         sep_id.source_address = node_address;
         sep_id.message_num = 0;
         sep_id.priority = 0;  
         current_message.datalen=4;		
         current_message.data[0]='S';      
         current_message.data[1]='R'; 
         current_message.data[3]='n';
         if(Numtype=='n') 
          {sep_id.dest_address = 0xD0;          
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);}
         else   
         if(Numtype=='d')
          {sep_id.dest_address = 0xB0;          
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10); 
           sep_id.dest_address = 0x10;          
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);}    
         else
         if(Numtype=='s')
          {   
           serial_current_message.address=0xD3;
           serial_current_message.length=4;
           serial_current_message.data[0]='S';          
           serial_current_message.data[1]='R'; 
           serial_current_message.data[2]=0;          
           serial_current_message.data[3]='n';    
           RS485_send_message(serial_current_message);    
           
            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID;                                   
            can_send_message(current_message);
            delay_ms(10);
         }         
         else
         if(Numtype=='f')
          {   
           serial_current_message.address=0xD0;
           serial_current_message.length=4;
           serial_current_message.data[0]='S';          
           serial_current_message.data[1]='R'; 
           serial_current_message.data[2]=0;          
           serial_current_message.data[3]='n';    
           RS485_send_message(serial_current_message);    
           
            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID;   
            current_message.datalen=6;	
            current_message.data[0]='X';       
            current_message.data[1]=0xD0;    
            current_message.data[2]='S';          //'S' for first digit of numerator
            current_message.data[3]='R'; 
            current_message.data[4]= 0 ;
            current_message.data[5]='n'; 
                                 
            can_send_message(current_message);
            delay_ms(10);
         }             
	
	 CONTU = 0; 
	 CONTD = 0;
	 CONTS = 0; 
	 CONTF = 0;
	 CONTL = 0;
	 CONTM = 0;
	 set_outputs_relay(0);
         while(ptimer<50)
                     {
                         WDRS=1;    //for Watchdog reset                         
                         delay_ms(100);
                         ptimer++;                         //delay 5 seconds in start
                         LED_indicator();
                         interpreter();    
                         if(Eth_number)netstackService();    	// service the network
                     }

	 read_inputs();
	 if(FEEDBD || FEEDBU)
		{errormsg(32);			//contcea
	  	 needreset=1;
	 	 continue;}			//stop when quitting revc
	 					//& when an error occurs go to first and wait
	}         
	
read_inputs();
if(REVC)continue;                           //For Errors occured during REVISION

WDRS=1;    //for Watchdog reset                                  

//********  THIS SUB-PROGRAM IS NAVIGATING 
//********   LIFT IN ON-BOARD REVISION MODE
read_inputs();
if(REVM&&(!REVC))                   
	{
          WDRS=1;    //for Watchdog reset     
                	
   	  opmode='m';                                        
	  errormsg(84);		//for display rev m 
	 
          sep_id.source_address = node_address;
          sep_id.message_num = 0;
          sep_id.priority = 0;  
          current_message.datalen=4;	
          current_message.data[0]='S';          //'S' for first digit of numerator
          current_message.data[1]='R';           
          current_message.data[3]='n'; 
      
          if(Numtype=='n')
          {
           sep_id.dest_address = 0xD0;            //numerator
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);   
          }
          else  
          if(Numtype=='d')
          {
           sep_id.dest_address = 0x10;            //Ext-num
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);   
           sep_id.dest_address = 0xB0;            //Ext-num
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);                                      
          }    
          else   
          if(Numtype=='s')
           {   
             serial_current_message.address=0xD3;
             serial_current_message.length=4;
             serial_current_message.data[0]='S';          
             serial_current_message.data[1]='R'; 
             serial_current_message.data[2]=0;          
             serial_current_message.data[3]='n';    
             RS485_send_message(serial_current_message);     
             
            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID;                                   
            can_send_message(current_message);
             delay_ms(10);
           }        
          else   
          if(Numtype=='f')
           {   
             serial_current_message.address=0xD0;
             serial_current_message.length=4;
             serial_current_message.data[0]='S';          
             serial_current_message.data[1]='R'; 
             serial_current_message.data[2]=0;          
             serial_current_message.data[3]='n';    
             RS485_send_message(serial_current_message);     
             
            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID;  
            current_message.datalen=6;	
            current_message.data[0]='X';       
            current_message.data[1]=0xD0;    
            current_message.data[2]='S';          //'S' for first digit of numerator
            current_message.data[3]='R'; 
            current_message.data[4]= 0 ;
            current_message.data[5]='n'; 
                                             
            can_send_message(current_message);
             delay_ms(10);
           }                         

	}		  

WDRS=1;    //for Watchdog reset            

/***********************************************************/


  revision_m();


/************************************************************/


revmove=0;

WDRS=1;    //for Watchdog reset   
  
read_inputs(); 
if(FEEDBD||FEEDBU||FEEDBS||FEEDBF)
	{ 
         sep_id.source_address = node_address;
         sep_id.message_num = 0;
         sep_id.priority = 0;  
         current_message.datalen=4;		
         current_message.data[0]='S';          //'S' for first digit of numerator
         current_message.data[1]='R'; 
         current_message.data[3]='n'; 
         if(Numtype=='n')
          {sep_id.dest_address = 0xD0;          
          ID = assemble_id(sep_id);
          current_message.id = ID;                                   
          can_send_message(current_message);
          delay_ms(10);}
         else 
         if(Numtype=='d')
          {sep_id.dest_address = 0xB0;          
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10); 
           sep_id.dest_address = 0x10;          
           ID = assemble_id(sep_id);
           current_message.id = ID;                                   
           can_send_message(current_message);
           delay_ms(10);}  
          else   
          if(Numtype=='s')
           {   
             serial_current_message.address=0xD3;
             serial_current_message.length=4;
             serial_current_message.data[0]='S';          
             serial_current_message.data[1]='R'; 
             serial_current_message.data[2]=0;          
             serial_current_message.data[3]='n';    
             RS485_send_message(serial_current_message);   
             
            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID;                                   
            can_send_message(current_message);
            delay_ms(10);
           }               
          else   
          if(Numtype=='f')
           {   
             serial_current_message.address=0xD0;
             serial_current_message.length=4;
             serial_current_message.data[0]='S';          
             serial_current_message.data[1]='R'; 
             serial_current_message.data[2]=0;          
             serial_current_message.data[3]='n';    
             RS485_send_message(serial_current_message);   
             
            sep_id.dest_address = 0xC0;            //cabin
            ID = assemble_id(sep_id);
            current_message.id = ID;     
            current_message.datalen=6;	
            current_message.data[0]='X';       
            current_message.data[1]=0xD0;    
            current_message.data[2]='S';          //'S' for first digit of numerator
            current_message.data[3]='R'; 
            current_message.data[4]= 0 ;
            current_message.data[5]='n'; 
                                          
            can_send_message(current_message);
            delay_ms(10);
           }                 
	
	 CONTU = 0; 
	 CONTD = 0;
	 CONTS = 0; 
	 CONTF = 0; 
	 CONTL = 0;
	 CONTM = 0;
	 set_outputs_relay(0);
	 ptimer=0;
         while(ptimer<5)
                      {
                           WDRS=1;    //for Watchdog reset                         
                           delay_ms(100);
                           ptimer++;                         //delay 0.5 seconds 
                           LED_indicator();
                           interpreter();   
                           if(Eth_number)netstackService();    	// service the network
                      }	
	 read_inputs(); 
	 if(FEEDBD||FEEDBU||FEEDBS||FEEDBF)
		{errormsg(32);		//contcea
	 	 needreset=1;
	 	 continue;}
	} 


if(needreset)continue;         
if(REVC||REVM)continue; 

WDRS=1;    //for Watchdog reset    

RELAYSTBY = 0;
_ack_can_transmit_error = 0xFF;                
	
read_inputs();  
if(Fsflag!=2)
 {
   if((CF1)&&(Needtocalibration=='n'))
       floor=rtc_read(0x31);       //recover last saved floor
 }
else  
  {
   if((CF1)&&(CF3)&&(Needtocalibration=='n'))
       floor=rtc_read(0x31);       //recover last saved floor  
  }

    
      
//enable timer1 overflow interrupt for supervision program
TIMSK1=0x01; 

opmode='d';                   //Calibration (Detection) mode

//****************************** Calibration ************************
if((!floor)||(floor>Floorsno)|| ((!CF1)&&Fsflag!=2) || ( !(CF1&&CF3) && Fsflag==2) )
 {
      errormsg(101);          //CAL
        
      _set_calibration_num=1;
      while(_set_calibration_num)
                         {
                           WDRS=1;    //for Watchdog reset   
                           if(!floor)break;                      
                         }        
      
      blink_en=0;  

      floor=1;
    
      calibration();   
          
      if(!floor)continue;

       recovery(1);                 //Saving last floor=1
 }     
//***************************  end of calibration *********************     



opmode='n';                   //Normal mode

blink_en=0; 
beep_en=0;  
BUZ=1;
errormsg(102);    //Floor display      
errormsg(102);    //Floor display and erase old_display_msg    

NNOD=0;
direction=0;
pathfinder(0,32);
stayinstair=1;
I=0;                        

turnoffallleds=1;
while(turnoffallleds)
                    {
                       WDRS=1;    //for Watchdog reset   
                       if(!floor)break;                       
                    }

 WDRS=1;             //FOR WATCHDOG RESET

 delay_ms(100);   

 WDRS=1;    //for Watchdog reset   

_set_numerator=1;                   //to send the changes in numerator by can in supervision
while(_set_numerator)
                    {
                       WDRS=1;    //for Watchdog reset   
                       if(!floor)break;                       
                    }
set_annunciator=2;                //set annunciator to play WELLCOME note

WDRS=1;             //FOR WATCHDOG RESET

delay_ms(100);  

WDRS=1;    //for Watchdog reset  

if(Eth_number)
        {
          if(Monitoring == 'y')
                _send_move_status=1;            //Ethernet monitoring packet
          if(Grouptype!='s')
                Waiting_List_Manager(0,0,0,0,0,0);            //clear List

         }

LPC_error=0;                 //reset its value 
_OVL_timeout_announuce=0;


floor_cal=floor;                //***\\
dir_cal=0;
dir_prev=0;                     //***\\
EIMSK=0;  
EIFR=0x30;
_int5_en=0;
_int4_en=0;           




                                               
//*******************************************************************
//*********************** Main Loop for Navigation  *****************
//*******************************************************************


while(floor)                               //the MAIN LOOP for navigation
 {            
 
   WDRS=1;    //for Watchdog reset   

   if(RELAYSTBY)
    { RELAYSTBY=0;      			//Turn the Fan and Lights on    
      set_outputs_relay(1);
      ptimer=0;
      do
        {
         WDRS=1;    //for Watchdog reset       
         if(!floor)break;
        }while(ptimer<2);            
    }    
    
   

  if(NNOD)                                      //NNOD-> No Need Opening Door
	{
	  NNOD=0;
	}
  else
        {
          if((!firemf)||(floor==Firefloor))
	     {
	       if(!dooropen())              //while
		{       
                  WDRS=1;    //for Watchdog reset          	  
        	  RELAYC=0;
	          RELAYO=0;                               //Turn on Open Relay for opening the door
        	  set_outputs_relay(1);   
	          if(!floor)break;
                  ptimer=0;
                  while(ptimer<50)
                         {
                           if(!floor)break;
                           WDRS=1;    //for Watchdog reset  
                         }
                  if(!floor)break;
                  errormsg(102);            //Floor display 
		}
	     }
           else 
             {
               if(!doorfire())               //while
		 {    
		  WDRS=1;    //for Watchdog reset    
        	  RELAYC=0;
	          RELAYO=0;                                   //Turn on Open Relay for opening the door
        	  set_outputs_relay(1);                   //By fireman's rules
	          if(!floor)break;                        //Open the door for fire mode
                  ptimer=0;
                  while(ptimer<50)
                         {
                           if(!floor)break;
                           WDRS=1;    //for Watchdog reset  
                         }                  
                  if(!floor)break;
                  
                  errormsg(102);		                  //Floor display
                  WDRS=1;    //for Watchdog reset                    
		 }                                   
              } //end else
         }

if(!floor)break;   



WDRS=1;    //for Watchdog reset  


turnoffleds = floor;        
while(turnoffleds)				//TURN OFF THE LEDS OF THE STAIR     again
	               {   
                         if(!floor)break;
                          WDRS=1;    //for Watchdog reset  
                        }    
                        
ptimer_preopening=0;

set_annunciator = 3;
arrivedstair=0;    
while(set_annunciator)
                     {
                       WDRS = 1;
                       if(!floor)break;          //for resetting
                     }      
                     
                     
if((Expired_date!=0xFF)&&(Expired_month!=0xFF))
 {
    if(block_date())
     {
       RELAYSTBY = 1;	 
       set_outputs_relay(1);      
       while(floor)
        {
            if(Expired_date==0xFF)floor=0;
            WDRS=1;                  
        }
         break;        //go to 
     }
 }                     
                     
                       
ptimer=0;
while((!direction)&&floor)				//Wait for Parking
	{
	 WDRS=1;                                                                    //for Watchdog reset  
	 
	 if((!Parkfloor)||(floor==Parkfloor)||(!Timepark))break;
	                                                                       //Get out if parkfloor is not defined
	 if(ptimer>Timepark)
	                  {parkmf=1;		                                //Set flags if Park time is over
				 _pf_park=1;	                	// and get out when direction is changed
				 while(_pf_park)
                                      {
                                         if(!floor)break;
                                         WDRS=1;                      //for Watchdog reset  
                                      }				 
			  }
	 if(!SS66)ptimer=0;				//If hall-doors opened, reset park timer
	}

if(!floor)break;					      //Get out if a fault had been occured
WDRS=1;                                                      //for Watchdog reset 


ptimer=0; 
while((!direction)&&floor)
      {
	  WDRS=1;    //for Watchdog reset  
	 if(!Timestndby)break;	

	 if(ptimer>Timestndby)        			//Wait for standby function
	                    { 
 	                       standbymf=1;
	                       if(Doorclosepark=='y')
	                            {  
	                                if(!SS66)
	                                     {_set_blink_num='B';
                                               while(_set_blink_num&&floor)WDRS=1;
                                               blink_en=1;}                   //For blinking  on
	                                
	                                while(standbymf&&(!direction))      //waiting for closing hall-doors
	                                    {
                                               WDRS=1;
                                               if(!floor)break;
                                               if(SS66)
	                                           {
	                                             w1=1;                        //temp3 --->  flag for getting out cause of debouncing
	                                             ptimer=0;                //save a copy of ptimer cause we need 2 timer
	                                             while(ptimer<Doorcdebouncet)
	                                               {
	                                                 WDRS=1; 
	                                                 if(!standbymf)break;
	                                                 if(direction)break;
	                                                 if(!SS66)
	                                                        {w1=0;
	                                                         break;}     //Debounce 1 second for ss66 
	                                                 if(!floor)break;
	                                                }
	                                            if(w1)break;
	                                           }	                            
	                                     }  //end of while for 66   
	                                     
                                         if(!floor)break;	                                     
	                                     
                                         _set_blink_num='O';
                                          while(_set_blink_num&&floor)WDRS=1;
                                          blink_en=0;                   //For blinking  off                                        
	                            
	                                 if((!standbymf)||direction)
	                                        {NNOD=1;
	                                         break;}
                                
	                                 if(doorclosestandby())
	                                    {
             	                                RELAYSTBY=1;	      //Go standby mode and get out if Standby time was over
             	                                set_outputs_relay(1);
	                                        break;
	                                    } //end if 
                                          else
                                          if(standbymf&&(!DOO)&&(SS66))       //exit by an Error
                                            {  
                                               ptimer=0;
                                               while(ptimer<50)
                                                    {if(!floor)break;   
                                                     WDRS=1;    //for Watchdog reset  
                                                    }
                                              if(!floor)break;
                                              errormsg(102);                         //Floor display                                    
                                              standbymf=0;
                                              break;
                                            }
	                             standbymf=0;
	                             break;              //if some errors occured during closing the door
	                            }  //end if(doorclose=='y')	                     

	                       RELAYSTBY=1;	      //Go standby mode and get out if Standby time was over  
	                       set_outputs_relay(1);
	                       
      	                       break;
      	                     }   
      	                     
      //      if(!SS66)ptimer=0;              //If hall-doors opened, exit standby & continue            
      } 
                                                                         
WDRS=1;    //for Watchdog reset 

if(!floor)break;

if(!direction&&floor)
{ 
 if((Timestndby)&&(!standbymf))
        {
          if(Doorclosepark!='y')NNOD=1;         //No need to open the door if not in closed door park mode 
          continue;                                    //exit from stand by and open the door 
        }     
}         

WDRS=1;    //for Watchdog reset     

if((!direction)&&Timestndby&&floor)errormsg(0);              //turn off 7-segs   


//wait for close the hall door in standby mode
if((!SS66)&&(Doorclosepark!='y')&&Timestndby)
        {
          while(!SS66)
          {  
           WDRS=1;
           if(!floor)break;  
           if(!standbymf)break;
           if(direction)break;           
           if(SS66)
	     {w1=1;                        //temp3 --->  flag for getting out cause of debouncing
	      ptimer=0;
	      while(ptimer<Doorcdebouncet)
	               {
	                if(!SS66)
	                          {w1=0;
	                           break;}     //Debounce 1 second for ss66 
	                if(!floor)break; 
	                if(!standbymf)break;
	                if(direction)break;
	                WDRS=1;
	               }
	      if(w1)break;
	     }      
	     
	    if(DOO)standbymf=0; 
	     
           }
        } 

// Program stay here in Idle mode (standby or etc.)
 while((!direction)&&floor)
	{ 
         WDRS=1;    //for Watchdog reset  
         if(!standbymf)break;    
         
         if(DOO)standbymf=0;
              
         if(Timestndby)	
          {                              
            WDRS=1;    //for Watchdog reset  
            if(!SS66)     //            if((!SS66)&&(Doorclosepark=='y'))
                     {
                      w1=1;
                      ptimer=0;     
                      do
                         {
                           WDRS=1;    //for Watchdog reset       
                           if(!floor)break;
                           if(SS66)w1=0;
                         }while(ptimer<2);                  //for debounce
                          
                      if(!floor)break;    
		      if(w1==1)standbymf=0;	      //If hall-doors opened, exit standby & continue
                     } 
            
          }  
          
         if(Lifttype=='h')
           {          
              if(Hyd_Releveling=='y')
              {
                if((Fsflag!=2)&&(!CF1))break;
                if(Fsflag==2)
                   if((!CF1)||(!CF3))break;
              }
           }
                 
	}


WDRS=1;

if(!floor)break;

if(Timestndby&&floor)errormsg(102);              //turn on 7-segs						      
 
if(!direction&&floor)
{        
 if((Lifttype=='h')&&(Hyd_Releveling=='y'))
       {
         Releveling();
         if(!floor)break;
         NNOD=0;
         continue;
       }

 if((Timestndby)&&(!standbymf))
        {
          if(Doorclosepark!='y')NNOD=1;         //No need to open the door if not in closed door park mode
        }                                                  //exit from stand by and open the door                                  
 continue;          
}                             

if(RELAYSTBY)
{
   RELAYSTBY=0;      			//Exit standby and Turn the Fan and Lights on   
   set_outputs_relay(1); 
   ptimer=0; 
   do
    {
      WDRS=1;    //for Watchdog reset       
      if(!floor)break;
    }while(ptimer<2);                 
}
   
   WDRS=1;    //for Watchdog reset     
 
if(direction==1)
       {
			if((floor==Floorsno)||CAn)
			                  {mcbycan=1;
			                   while(mcbycan)
                                              {
                                                 if(!floor)break;
                                                 WDRS=1;    //for Watchdog reset  
                                              }				                   
					   continue;}
        }						
							      //It doesn't allow in upper floor to start upward
else 
if(direction==2)
      	{
			if((floor==1)||(CA1 && CA1Second_En==0) || (CA1 && CA1S && CA1Second_En==1))
			               {mcbyca1=1;
					    while(mcbyca1)
                                                  {
                                                        if(!floor)break;
                                                         WDRS=1;    //for Watchdog reset  
                                                  }						    
					    continue;}
	}
else 
        {        
                         direction=0;
                         continue;
        };	
							      //It doesn't allow in lower floor to start downward

WDRS=1;    //for Watchdog reset  


while(OVLC||FTOC||FLTC)					      //;;;;;Before closing door, wating for normal condition 
	{
	  if(!floor)break;
	  WDRS=1;    //for Watchdog reset  
	 }
if(!floor)break;

while(floor)
   {     
     WDRS=1;    //for Watchdog reset       
     if(!doorclose())
        {
          if(!direction)stayinstair=1;
          if(!floor)break;   
          RELAYO=0;
          RELAYC=0;
          URA=0;  
          delay_ms(50);
          set_outputs_relay(1);
          
          blink_en=0;
          
          ptimer=0;
          while(ptimer<50)
                           {
                             if(!floor)break;
                             WDRS=1;    //for Watchdog reset  
                           }
          if(!floor)break;                                  
          
          errormsg(102);                 //Floor display
          
          if(!floor)break;
          if(!direction)break;
        }
     else break;
   }

if(!floor)break;
if(!direction)continue;             
	
if(OVLC||FTOC||FLTC)continue;					      //;;;Open the cabin door,if conditions aren't suitable after closing door

stayinstair=0;						// Stayinstair  means that requests  
							      // of this floor should be accepted

/* 'I' means that safety-serie have been completed to move*/

I=1;
if(direction==1)
               {
			if((floor==Floorsno)||CAn)
					   {
					    mcbycan=1;
				   	    while(mcbycan) 
                              	               {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }
							      //Open the door if start's direction is invalid after closing the door		
					    continue;}
							      //Check the direction cause it may be changed during door closing.
		}
else 
if(direction==2)
               {
			if((floor==1)||(CA1 && CA1Second_En==0) || (CA1 && CA1S && CA1Second_En==1))
					   {mcbyca1=1;
					    while(mcbyca1)
                              	               {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }					    
					    		      //Open the door if start's direction is invalid after closing the door
					    continue;}
		}				            //Check the direction cause it may be changed during door closing.
else 
                {        
                         direction=0;
                         I=0;
                         stayinstair=1;  
                         continue;
                }


if(FEEDBU||FEEDBD||FEEDBF||FEEDBS)
                               {
                                 contce=1;
				 while(contce)
                              	               {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }				 
				}
							      //Movement's starting steps:
if(!floor)break;					      //1. Check the conts. to be freed.

WDRS=1;    //for Watchdog reset          
                                                

      
_set_numerator=1;          //to send the changes in numerator by can in supervision         

arrivedstair=0;
if(Music_Play_En=='y')set_annunciator=1;                //set annunciator to play music

recovery(3);        //Erasing last saved floor
recovery(4);                 //To increase number of start    

dir_cal=direction;             //***\\
dir_prev=direction;
 
if(direction==1)
	{                                   
	  if(Lifttype=='h')
	   {
	     CONTS=1; 
	     set_outputs_relay(0);			      //CONTS for Motor
	     w1=0;  
	     w2=0;
	     ptimer=0;
             do
		  {
		    WDRS=1;    //for Watchdog reset  
		    if((ptimer>20)&&(!FEEDBS))   // 2 seconds for receiving its feedback 
			       {contnca=1;
		  	        while(contnca)
                              	               {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }		  	        
			        break;} 
	            if(!floor)break;	
	            
	            if((ptimer>=Hyd_S2D_Time)&&(!w1))
	              {
	               CONTL=1;  
	               set_outputs_relay(0);			      //contl for delta   
	               w1=1;  
	               if(w2)break;
	              }
	                 
	            
	            if((ptimer>=Hyd_Start_Time)&&(!w2))
	              {
	               CONTF=1;
	               CONTU=1;
	               set_outputs_relay(0);   
	               w2=1;
	               if(w1)break;  
	              }	            
	            			       
                    if(ptimer>200)break; 
                                               	     
                  }while(1);	
                  
                  	      
             if(!floor)break;
          
	   }  
	  else
	   {
   	     CONTU=1;				      //2. Trigger one of the direction conts.
	     CONTD=0;
   	     set_outputs_relay(0);			      //CONTU for direction UP	     
	   }
	   

	  if((Lifttype=='2')||(Lifttype=='h'))
	  {
	   ptimer=0;
           do
		  {
		    WDRS=1;    //for Watchdog reset  
		    if(ptimer>10)
			       {contnca=1;
		  	        while(contnca)
                              	               {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }		  	        
			        break;} 
	            if(!floor)break;				       
                                               	      //3. Ensure  it to be triggered. Waiting  
                  }while(!FEEDBU);		      // one second for receiving its feedback
           } //end lifttype
	}
else 
if(direction==2)
	{
	 CONTD=1;				            //CONTD for direction Down
	 CONTU=0;
	 set_outputs_relay(0);  
	 if(Lifttype=='2')
	   {		
		ptimer=0;
		do
		  {
			WDRS=1;    //for Watchdog reset  
			 if(ptimer>10)
			            {contnca=1;
				     while(contnca)
                              	               {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }				     
				     break;}  
			 if(!floor)break;
		  }while(!FEEDBD);
            }
	}  
else 
        {
           direction=0;
           I=0;
           stayinstair=1;  
           continue;        
        }



if(!floor)break;   


	
CONTM=0;
if(Lifttype!='h')CONTS=0;
CONTF=1;   

w3=floor;  //w3 is copy  of floor value before ++ or --
pulse_counter=0;
w1=0;
if(direction==1)
 {
   if(AL[0]==(floor+1))w1=1;
 }
else
if(direction==2)
 {
   if(AL[0]==(floor-1))w1=1;
 } 

if(w1==1)          //movement is between 2 consecutive floors
 {
   w1=0;
   if(Fsflag==2)
    {
      if(direction==1)
          {
           if(!halffloorstatus(floor))w1='h';    //2 means halffloorspeed must be applied
          }
      else
      if(direction==2)
         {
            if(!halffloorstatus(floor-1))w1='h';  //2 means halffloorspeed must be applied
         }
     }
     
    if(w1!='h')
     {
        if(OneMovefloorSpeed!=0)w1='o';          //3 means OneMovefloorSpeed must be applied  
     }       
 
 }
 
if(Lifttype=='h')w1=0; 

if(w1)
 {
   if(Lifttype=='2'){CONTS=1;CONTF=0;CONTL=0;} 
   else
   if(w1=='h')         //halffloorspeed
     {
         if(HalffloorSpeed=='s'){CONTS=1;CONTF=0;CONTL=0;}
         else if(HalffloorSpeed=='b'){CONTS=1;CONTF=1;CONTL=0;}
         else if(HalffloorSpeed=='r'){CONTS=0;CONTF=0;CONTL=1;}
         else {CONTS=0;CONTF=1;CONTL=0;}  
         pulse_counter=0xFF;             //for checking HalffloorDelay      
         if(floor)floor=AL[0]; 
     } 
   else  
   if(w1=='o')         //OneMovefloorSpeed
     {
         if(OneMovefloorSpeed=='s'){CONTS=1;CONTF=0;CONTL=0;}
         else if(OneMovefloorSpeed=='b'){CONTS=1;CONTF=1;CONTL=0;}
         else if(OneMovefloorSpeed=='r'){CONTS=0;CONTF=0;CONTL=1;}
         else {CONTS=0;CONTF=1;CONTL=0;}   
         pulse_counter=0xFE;             //for checking OneMovefloorDelay    
         if(floor)floor=AL[0];
     }   
     
   errormsg(102);                           //Floor  display
  _set_numerator=1;          //to send the changes in numerator by can in supervision  
 
 };
if(!floor)break; 
set_outputs_relay(0);  

if(Fsflag==2 && pulse_counter<0xFE)
    {
      if(direction==1)
          {
           if(!halffloorstatus(floor))   
               {floor++;
                errormsg(102);                           //Floor  display
                _set_numerator=1;}          //to send the changes in numerator by can in supervision  
          }
      else
      if(direction==2)
         {
            if(!halffloorstatus(floor-1))
               {floor--;
                errormsg(102);                           //Floor  display
                _set_numerator=1;}          //to send the changes in numerator by can in supervision  
         }
     }
     



ptimer=0;
do
 {
     
        if(Lifttype=='2' && CONTF==0 && CONTS==1)break;
        
        WDRS=1;    //for Watchdog reset  		  
        if(ptimer>20)
	           {contnca=1;
	      	     while(contnca)
                 	               {   
                                          if(!floor)break;
                                          WDRS=1;    //for Watchdog reset  
                                       }		      	     
		     break;}
 }while(!FEEDBF);				//5. Wait for its feedback
if(!floor)break;

//recovery(3);        //Erasing last saved floor
//recovery(4);                 //To increase number of start    


//dir_cal=direction;             //***\\
//dir_prev=direction;
  
ptimer=0;
do
{
    WDRS=1;    //for Watchdog reset      
    if(!floor)break;       
    if(!Timebreakdelay)break;
    if(ptimer>Timebreakdelay)
            {
              feedb4bse=1;	
	      while(feedb4bse)
        	           {   
                             if(!floor)break;
                             WDRS=1;    //for Watchdog reset  
                           }	      
   	      break;
  	    }	
}while(!FEEDB4BS);                 //Check Break  feedback

if(Eth_number)
  if(Monitoring == 'y')
      _send_move_status=1;

WDRS=1;    //for Watchdog reset  

if(!floor)break;   


_set_numerator_CA1CAn=0;	
if(Fsflag==2)                                     
  {                                               //Fsflag==2
     
    while(1)
     {
        
        if(pulse_counter==0xFF || pulse_counter==0xFE)break;       //break out from Pulse-Counting in halffloor    
        

     	ptimer=0;					      //Here now count CF3 pulses to arrive to the destination
        do{
               WDRS=1;    //for Watchdog reset  
               if((CAn&&(direction==1))||(CA1&&(direction==2)))break;
               if( (ptimer>Time2cf3) && (Time2cf3!=0) )        
       		            { if(!floor)break;
       		               timecf3e=1;
     			       while(timecf3e)
			                       {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }
	        	      break;}
              M_INT3 = ((PINE & 0x10)>>4)&0x01 ; 
              M_INT4 = ((PINE & 0x20)>>5)&0x01 ;  
              delay_ms(1);
              if((CF3)&&(CF1))break;
	     }while(floor);				//Wait 3 seconds to turn on both of CF3  and 1CF
	    
        if(!floor)break;
       
        if(CAn&&(direction==1)&&floor)
                {floor=Floorsno;     
                 _set_numerator_CA1CAn=1;
                 break;}
        if(CA1&&(direction==2)&&floor)
                {floor=1;       
                 _set_numerator_CA1CAn=1;
                 break;}  
                   
	if(!floor)break;  
	 
	WDRS=1;
        delay_ms(20);        
       
	ptimer=0; 
	if(Lifttype=='2')ptimer=30;		      //Here now count CF3 pulses to arrive to the destination
	do{
                WDRS=1;    //for Watchdog reset  
                if((CAn&&(direction==1))||(CA1&&(direction==2)))break;
		if((ptimer>70)&& (Time1cf3!=0))
		            { if(!floor)break;
		               timecf3e=1;
			       while(timecf3e)
			                       {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }
			       break;}
                 M_INT3 = ((PINE & 0x10)>>4)&0x01 ;   
                 M_INT4 = ((PINE & 0x20)>>5)&0x01 ;  
                 delay_ms(1);
                 if((!CF3)&&(!CF1))break;
	    }while(floor);				//Wait 3 seconds to turn off the CF3  and 1CF
	    
       if(!floor)break;
       
       if(CAn&&(direction==1)&&floor)
                {floor=Floorsno;  
                 _set_numerator_CA1CAn=1;
                 break;}
       if(CA1&&(direction==2)&&floor)
                {floor=1;    
                 _set_numerator_CA1CAn=1;
                 break;}    
                           
	if(!floor)break;                                 //CF3=0
       
        WDRS=1;    //for Watchdog reset  
        

        if(direction==1)                                                //w3 is copy  of floor value before ++ or --
             {
               if(halffloorstatus(w3)==0 && halffloorstatus(w3+1)==1)w1=0;  //w1 is number of pulses between floors
               else
               if(halffloorstatus(w3)==1 && halffloorstatus(w3+1)==0)w1=2;
               else w1=1;
               if(w3<Floorsno)w3++;
             } 
        else 
        if(direction==2)
             {
               if(halffloorstatus(w3-1)==0 && halffloorstatus(w3-2)==1)w1=0; 
               else
               if(halffloorstatus(w3-1)==1 && halffloorstatus(w3-2)==0)w1=2;
               else w1=1;  
               if(w3>1)w3--;   
             }   
          
nextflag:  
         if(w1==0)continue; 
         w1--;
	
	 delay_ms(50);   
	
	 WDRS=1;    //for Watchdog reset  
	
	
	 ptimer=0;
	 do{
                WDRS=1;   
                if((CAn&&(direction==1))||(CA1&&(direction==2)))break;		
		if( (ptimer>Time1cf3) && (Time1cf3!=0) )
		                  {
		                   timecf3e=1;
				   while(timecf3e)
				   	       {   
                                                     if(!floor)break;
                                                      WDRS=1; 
                                               }
				   break;}
                M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
                M_INT4 = ((PINE & 0x20)>>5)&0x01 ;                
                				   
                delay_ms(1);
                
                if(!floor)break;
                
                if((direction==1)&&(CF3))
                   {
                    floor++;
                    break;
                   }
                if((direction==2)&&(CF1))
                    {
                     floor--;
                     break; 
                    }               
                                  				//Wait as long as time1cf3 to turn on the CF3
	    }while(floor);			              
	    
        if(!floor)break;   
        
        EIFR=0x30;
        EIMSK=0x30;
                               //***\\ enable both of interrupts
	    
       if(CAn&&(direction==1)&&floor)
                {
                 floor=Floorsno;   
                 _set_numerator_CA1CAn=1;
                 break;}
       if(CA1&&(direction==2)&&floor)
                {floor=1;   
                 _set_numerator_CA1CAn=1;
                 break;}     
  
       errormsg(102);                           //Floor  display


       _set_numerator=1;          //to send the changes in numerator by can in supervision    


       if(Eth_number)
       if(Monitoring == 'y')
         _send_move_status=1;            //Ethernet monitoring packet
	
       if(floor==AL[0])break;
	
       if(floor==Floorsno)
	     {
	            if(!CAn)canins=1;
		    while(canins)		   //Cabin in highest stair but CaN is still turned off
		  	           {   
                                     if(!floor)break;
                                     WDRS=1;    //for Watchdog reset  
                                    }				    
      	            break;
             }
       if(floor==1)
	     {
	         if(!CA1)ca1ins=1;
	          while(ca1ins)		   //Cabin in lowest stair but Ca1 is still turned off
	   	           {   
                                      if(!floor)break;
                                       WDRS=1;    //for Watchdog reset  
                            }			          
		  break;
	     }     
	     
       WDRS=1;    //for Watchdog reset  
	
       delay_ms(50);   
	
       WDRS=1;    //for Watchdog reset  

       ptimer=0;
       do{
              WDRS=1;    //for Watchdog reset                                    
              if((CAn&&(direction==1))||(CA1&&(direction==2)))break;	                    
              if(ptimer>30)
	             {timecf3e=1;
	              while(timecf3e)
	  	   	           {   
                                       if(!floor)break;
                                       WDRS=1;    //for Watchdog reset  
                                    }			      	                  
	               break;}
  	      M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
  	      M_INT4 = ((PINE & 0x20)>>5)&0x01 ;  
  	      delay_ms(1);
  	      if((!CF3)&&(!CF1))break;                  //***\\    &&<--||
	  }while(floor);      //Wait 2 second to turn off the CF3 or 1CF
	             
        if(!floor)break;
                           
        if(CAn&&(direction==1)&&floor)
                  {floor=Floorsno;   
                   _set_numerator_CA1CAn=1;
                   break;}
        if(CA1&&(direction==2)&&floor)
                  {floor=1;
                   _set_numerator_CA1CAn=1;
                   break;}                 

	ptimer=0;                        //minimum delay between 2 sensed
	while(ptimer<2)
	          {
                       WDRS=1;    //for Watchdog reset  
                       if((CAn&&(direction==1))||(CA1&&(direction==2)))break;
                       if(!floor)break;                     //wait 0.2 sec to make non-sense about CF3
		  }           

                          
         WDRS=1;  
         
         
         goto nextflag;
         
         
         
	 
      }    //end while(1
 
   }      //end if(fsflag==2

else	

							      //Here, the motor starts to move and go up or down
while(1)                //First of Dorandazi Loop
{							     
	if(pulse_counter==0xFF || pulse_counter==0xFE)break;       //break out from Pulse-Counting 
	
	WDRS=1;    //for Watchdog reset  
	ptimer=0;					      //Here now count CF3 pulses to arrive to the destination
	do{
                WDRS=1;    //for Watchdog reset  
                if((CAn&&(direction==1))||(CA1&&(direction==2)))break;
		if(ptimer>30)         
		            { if(!floor)break;
		               timecf3e=1;
			       while(timecf3e)
			                       {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }
			       break;}
                 M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
                 delay_ms(1);
	    }while(CF3&&floor);				//Wait one second to turn off the CF3 
	    
       if(!floor)break;
       
       if(CAn&&(direction==1)&&floor)
                {floor=Floorsno;
                 _set_numerator_CA1CAn=1;
                 break;}
       if(CA1&&(direction==2)&&floor)
                {floor=1;  
                 _set_numerator_CA1CAn=1;
                 break;}                 
                                                                //It's for ending the last pulse, it will be passed in start 
	if(!floor)break;                                 //CF3=0
       
        WDRS=1;    //for Watchdog reset  
	
	delay_ms(50);   
	
	WDRS=1;    //for Watchdog reset  
	
	ptimer=0;
	do{
                WDRS=1;    //for Watchdog reset  
                if((CAn&&(direction==1))||(CA1&&(direction==2)))break;		
		if( (ptimer>Time1cf3) && (Time1cf3!=0) )
		                  {
		                   timecf3e=1;
				   while(timecf3e)
				   	       {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }
				   break;}
                M_INT3 = ((PINE & 0x10)>>4)&0x01 ;				   
                delay_ms(1);
                                  				//Wait as long as time1cf3 to turn on the CF3
	    }while((!CF3)&&floor);			//time1cf3-> time from 1CF to 1st CF3
	    
        if(!floor)break;	    
	    
       if(CAn&&(direction==1)&&floor)
                {
                 floor=Floorsno; 
                 _set_numerator_CA1CAn=1;
                 break;}
       if(CA1&&(direction==2)&&floor)
                {floor=1;                
                 _set_numerator_CA1CAn=1;
                 break;}                 
	    
	                                                        //CF3=1
        WDRS=1;    //for Watchdog reset  

        delay_ms(20);                                                        //Here, we got ONE Rising-Edge
        
	if(!floor)break;
	if((!Fsflag)&&(!Oneflag))
                   {	                                       //Counting with 2nd Flag - Used 2 Flags for each stair
	                  ptimer=0;
	                  do{    
	                          WDRS=1;    //for Watchdog reset  
                                  if((CAn&&(direction==1))||(CA1&&(direction==2)))break;	                  
		                  if(ptimer>30)           
		                         {timecf3e=1;
		      	                  while(timecf3e)
				   	       {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                               }		      	                  
			                  break;}
                                  M_INT3 = ((PINE & 0x10)>>4)&0x01 ;			                  
                                  delay_ms(1);
	                    }while(CF3&&floor);	       //Wait one second to turn off the CF3
	                    
                          if(!floor)break;
                          
                          if(CAn&&(direction==1)&&floor)
                                {floor=Floorsno; 
                                 _set_numerator_CA1CAn=1;
                                 break;}
                          if(CA1&&(direction==2)&&floor)
                                {floor=1;               
                                 _set_numerator_CA1CAn=1;
                                 break;}                 
                                                               //We got One PULSE completely
	                  if(!floor)break;
		                                            //wait as long as time2cf3 to turn on the CF3
			           		             //time2cf3-> time from 1st CF3 to 2nd CF3
			  ptimer=0;                        //minimum delay between 2 sensed
			  while(ptimer<2)
			          {
                                   WDRS=1;    //for Watchdog reset  
                                   if((CAn&&(direction==1))||(CA1&&(direction==2)))break;
			           if(!floor)break;                     //wait 0.2 sec to make non-sense about CF3
			          }            
			  ptimer=0;
			  do{
                               WDRS=1;    //for Watchdog reset  
                              if((CAn&&(direction==1))||(CA1&&(direction==2)))break;			  
			      if( (ptimer>Time2cf3) && (Time2cf3 != 0) )
			                         {timecf3e=1;
			    	       		  while(timecf3e)
				   	           {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                                   }			    	       		  
					          break;}
                              M_INT3 = ((PINE & 0x10)>>4)&0x01 ;					          
                              delay_ms(1);
 		  	     }while((!CF3)&&floor);  
 		  	     
                          if(!floor)break;
                          
                          if(CAn&&(direction==1)&&floor)
                                {floor=Floorsno;   
                                 _set_numerator_CA1CAn=1;
                                 break;}
                          if(CA1&&(direction==2)&&floor)
                                {floor=1;                
                                 _set_numerator_CA1CAn=1;
                                 break;}                 

                          WDRS=1;    //for Watchdog reset   		          
 		          delay_ms(20);
		    };			
							      	
	if(!floor)break;
	
	switch(direction)
	             {
				case 1 : 
				      floor++;
				      break;
				case 2 :
				      floor--;
				      break;
		      }; 
		      

EIFR=0x20;
EIMSK=0x20;        //enable interrupt 1cf
 
           //***\\
		      


errormsg(102);                           //Floor  display


_set_numerator=1;          //to send the changes in numerator by can in supervision    


if(Eth_number)
   if(Monitoring == 'y')
      _send_move_status=1;            //Ethernet monitoring packet
	
	if(floor==AL[0])break;
	
	if(floor==Floorsno)
	               {
			    if(!CAn)canins=1;
			    while(canins)		   //Cabin in highest stair but CaN is still turned off
				   	           {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                                   }				    
			    break;}
	if(floor==1)
	               {
		          if(!CA1)ca1ins=1;
		          while(ca1ins)		   //Cabin in lowest stair but Ca1 is still turned off
				   	           {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                                   }			          
		          break;}

	if((Fsflag)&&(!Oneflag))                     //Counting with 1st flag - Used 2 flags per stair
	                  {
	                    ptimer=0;
	                    do{
                                  WDRS=1;    //for Watchdog reset                                    
                                  if((CAn&&(direction==1))||(CA1&&(direction==2)))break;	                    
		                  if(ptimer>30)
		                         {timecf3e=1;
		      	                  while(timecf3e)
				   	           {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                                   }			      	                  
			                  break;}
  	                          M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
  	                          delay_ms(1);
	                      }while(CF3&&floor);      //Wait 1 second to turn off the CF3 
	                      
                           if(!floor)break;
                           
                           if(CAn&&(direction==1)&&floor)
                                {floor=Floorsno;
                                 _set_numerator_CA1CAn=1;
                                 break;}
                           if(CA1&&(direction==2)&&floor)
                                {floor=1;                
                                 _set_numerator_CA1CAn=1;
                                 break;}                 

			  ptimer=0;                        //minimum delay between 2 sensed
			  while(ptimer<2)
			          {
                                   WDRS=1;    //for Watchdog reset  
                                   if((CAn&&(direction==1))||(CA1&&(direction==2)))break;
			           if(!floor)break;                     //wait 0.2 sec to make non-sense about CF3
			          }           
  			     
 			 ptimer=0;
			 do{
                                  WDRS=1;    //for Watchdog reset  
                                  if((CAn&&(direction==1))||(CA1&&(direction==2)))break;			     
	                          if( (ptimer>Time2cf3) && (Time2cf3!=0) )
	                               {timecf3e=1;
				        while(timecf3e)
				   	           {   
                                                     if(!floor)break;
                                                      WDRS=1;    //for Watchdog reset  
                                                   }					        
			                break;}
                                  M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
                                  delay_ms(1);
                           }while((!CF3)&&floor);     //Wait as long as time2cf3 to turn on the CF3   
                                
                          if(!floor)break;
                          
                          if(CAn&&(direction==1)&&floor)
                                {floor=Floorsno;  
                                 _set_numerator_CA1CAn=1;
                                 break;}
                          if(CA1&&(direction==2)&&floor)
                                {floor=1;                
                                 _set_numerator_CA1CAn=1;
                                 break;}                 
                                 
                          WDRS=1;    //for Watchdog reset  
                          
                          delay_ms(20);
	                }                          
};      //End of  DORANDAZI loop 


if(!floor)break;



errormsg(102);                           //Floor  display

if(_set_numerator_CA1CAn)
{
  _set_numerator_CA1CAn=0;
  _set_numerator=1;  
}    

if(pulse_counter<0xFE)       //this section is not run in OneMoveFloor movement
 {
  if(Lifttype!='h')CONTS=1;  //Turn off Fast Contactor     

  CONTF=0;
  set_outputs_relay(0); 
  if(Lifttype=='2')
   {
     ptimer=0;                                          //Turn on Slow Contactor
     do{                                                //Make the motor speed slow
	  WDRS=1;    //for Watchdog reset  
	  if(ptimer>10){
			if(!FEEDBS)contnca=1;
		 	if(FEEDBF)contcea=1;
			while((contnca||contcea))
	   	           {   
                                if(!floor)break;
                                WDRS=1;    //for Watchdog reset  
                           }			
			break;
		     }    
      	  if(!floor)break;   
        }while(FEEDBF||(!FEEDBS));
    }
 }
if(!floor)break;       

if(Lifttype=='h')
{
 ptimer=0;                                          //Turn on Slow Contactor
 do{                                                //Make the motor speed slow
	WDRS=1;    //for Watchdog reset  
	if(ptimer>20){
		 	if(FEEDBF)contcea=1;
			while((contnca||contcea))
	   	           {   
                                if(!floor)break;
                                WDRS=1;    //for Watchdog reset  
                           }			
			break;
		     }    
	if(!floor)break;   
  }while(FEEDBF);
}
if(!floor)break;   


if(Fsflag==2 && pulse_counter<0xFE)     //detect if 2 level flags on the slow down path
 {
        if(direction==1) //up
         {
           if(halffloorstatus(floor-1)==0)pulse_counter=0xFD;    //0xFD means Not need to change speed 
         }                                                       //and just 2 levels pulse must be sensed 
        else
        if(direction==2) //down
         {
           if(halffloorstatus(floor)==0)pulse_counter=0xFD;  
           if(CA1Second_En && CA1)pulse_counter=0xFD; 
         }         
 }



ptimer=0;                                       //Reset timer for seeking 1CF
ptimer_preopening=0;


w3=0;     //state flag for level sensor
w5=0;     //counter reg
w1=0;     //state flag for SLOW DOWN
while(1)                        //waiting here to Arrive stair
   {
        delay_ms(1);    
        
        if(Fsflag==2)
         {

            if(pulse_counter>=0xFD && w3<0xFF)
              {
                 if(w3==0 && CF1 && CF3)                              //bypass the first 1cf pulse
                      {
                        w3=1;         //w3=1 means First level sensor pulse is now ON and it is necessary to wait for TURN OFF
                      }  
                 if(w3==1)
                     {
                       if(!CF1 && !CF3)w5++; 
                        else w5=0;
                       if(w5>200)w3=0xFF;       //w6=0xFF means First level sensor pulse is now OFF and waiting for Last pulse
                     }     
              }         
             else if(CF1 && CF3)break;       //get out when 2nd level pulse is sensed
             
         }  
        else
         {
            if(pulse_counter>=0xFD && w3<0xFF)
              {
                 if(w3==0 && CF1)                             
                      {
                        w3=1;            //w6=1 means First level sensor pulse is now ON and it is necessary to wait for TURN OFF
                      }  
                 if(w3==1)
                     {
                       if(!CF1)w5++; 
                        else w5=0;
                       if(w5>200)w3=0xFF;      
                     }     
              }         
             else if(CF1)break;             //get out when 2nd level pulse is sensed
         }
        
        M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
        M_INT4 = ((PINE & 0x20)>>5)&0x01 ;    
            
        WDRS=1;                     //WATCHDOG	
	if(!floor)break;
	if( (ptimer>Cf1mt) && (Cf1mt!=0) )
	            {time1cf=1;
			 while(time1cf)
	                  {   
                              if(!floor)break;
                              WDRS=1;    //for Watchdog reset  
                          }  			 
			 break;}   
			                   //Ptimer is not reset when preopening,
	
	if(!arrivedstair)
	{
	 if(ptimer>floor_announce_delay)
	   {
                arrivedstair=1;                          //for annuncing stair
                set_annunciator=1;      
	   }
    }
         
        if(Preopening_en=='s')
         {

           if((Doortype=='a')&&(ptimer_preopening==0))
            {
                if(direction==1)
                  {
                    if(DZU&&(!DZD))
                     {
                      if(!NNOD)
                       {
                        URA=1; 
                        RELAYO=1;
                        RELAYC=0;
                        set_outputs_relay(1); 
                        ptimer_preopening=1;
                       }
                     }
                  }
                else  
                if(direction==2)
                  {
                    if(DZD&&(!DZU))
                     {
                       if(!NNOD)
                        {
                          URA=1;
                          RELAYO=1;
                          RELAYC=0;
                          set_outputs_relay(1); 
                          ptimer_preopening=1;                     
                        }
                     
                     }
                  
                  }

            }

       
         } //end of if(Preopening_en=='s')
         
       
       if(pulse_counter>=0xFE && w1==0)
         {
    	  if(pulse_counter==0xFF)
	   {
	      if(ptimer>HalffloorDelay)w1=1;
	   } 
	  else    
    	  if(pulse_counter==0xFE)
	   {
	      if(ptimer>OneMovefloorDelay)w1=1;
	   } 
         }          
         
       if(w1==1)
         {
           w1=0xFF; 
           CONTL=0;
           CONTM=0;
           CONTF=0;
           CONTS=1;
           set_outputs_relay(0);          
         } 
     

   }   //while(1)                            
   

 

if(!floor)break;       

if(CAn&&(direction==1)&&floor)floor=Floorsno;
if(CA1&&(direction==2)&&floor)
 {
   if(Fsflag!=2)floor=1;
   else
   {
     if(CA1Second_En==0)floor=1;
     else 
      {
        if(!CA1S)floor=2;
        else floor=1;
      }
   }
 } 
errormsg(102);                           //Floor  display    


switch(direction)
      {
case 1:
	w1=0;
        while((w1<Contofftu))
	               {  WDRS=1;    //for Watchdog reset    
	                  delay_ms(100);
                         if(!floor)break;  
                         w1++;
                       }  	
	break;			                 //For correct leveling
	
case 2:
	w1=0;
        while((w1<Contofftd))
	               { 
	                 WDRS=1;    //for Watchdog reset    
	                 delay_ms(100);
                         if(!floor)break;          
                         w1++;
                       }  
	break;
     
     };

if(!floor)break;

I=0;    

                                          //STOP in stair fir Lift
CONTF=0;
CONTM=0;
if((Lifttype=='d')||(Lifttype=='2'))
     {CONTU=0;
      CONTD=0;}
      
if(Lifttype=='h')
 {
  CONTD=0;    
  if(direction==1)  //upward
   {
    if(Hyd_Stop_Time<51)                    //0~50 delay
     {                                      //1st pump 2nd valves
      CONTL=0;
      CONTS=0;         //turn the motor off
      set_outputs_relay(0);  
      ptimer=0;
      while(ptimer<Hyd_Stop_Time)
       {
         WDRS=1;
         if(!floor)break;
       }                       
      if(!floor)break;
      CONTU=0;       
     }
    else
    if((Hyd_Stop_Time>99)&&(Hyd_Stop_Time<151))      //100~150 means 0~-50 delay (negative)
     {                                               //1st valve 2nd pump
      CONTU=0;     //turn the valves off
      set_outputs_relay(0);   
      ptimer=100;
      while(ptimer<Hyd_Stop_Time)
       {
         WDRS=1;
         if(!floor)break;
       }                       
      if(!floor)break;
      CONTL=0;
      CONTS=0;         //turn the motor off     
     }
   } //if(direction

 }
else
{
 CONTL=0;
 CONTS=0;
}

set_outputs_relay(0);               

if(Lifttype=='v')ptimer=0;               //2 sec for 2 speed              
else
if(Lifttype=='d')ptimer=20;               //3 sec for delta vvvf
   else  ptimer=30;                         //5 sec for vvvf for STOP
do
  {
 	    if(!floor)break;
	    if(ptimer>50)
	             {contcea=1;    
			  while(contcea)
                               {
                                    WDRS=1;    //for Watchdog reset   
                                    if(!floor)break;                        //delay 0.5 seconds 
                               }				  
			  break;}            //Wait one second for sense feedbacks
            WDRS=1;    //for Watchdog reset  	
            
       if(Dir_delay=='y')
       {
        if((ptimer>Cont_overlap))
         {
                CONTU=0;
                CONTD=0;
                set_outputs_relay(0); 
         }
        }            
            		  
  }while(FEEDBD || FEEDBS || FEEDBU || FEEDBF);     
if(!floor)break;  
if(Lifttype=='v')
    { CONTU=0;
       CONTD=0;
      set_outputs_relay(0);}

if(!floor)break;

stayinstair = 1;                                          //for blinking numerator and anuncing floor name


turnoffleds = AL[0];        
while(turnoffleds)				//TURN OFF THE LEDS OF THE STAIR
	               {   
                         if(!floor)break;
                          WDRS=1;    //for Watchdog reset  
                       }   


_pf_reform = 1;
//while(_pf_reform&&floor);		            //REFORM & CORRECTION OF AL & DIRECTION   

EIFR=0x30;
EIMSK=0x00;
floor_cal=floor;
dir_cal=0;
dir_prev=0;             //***\\

recovery(1);                                              //Saving last floor=1 

if(!arrivedstair)
 {
                arrivedstair=1;
                set_annunciator=1;                //set annunciator to play music
                while(set_annunciator)
	             {   
                   if(!floor)break;
                   WDRS=1;    //for Watchdog reset  
                 }   
 }
arrivedstair=0;
 
 

_set_numerator=1;                                   //to send the changes in numerator by can in supervision
while(_set_numerator)
	               {   
                         if(!floor)break;
                          WDRS=1;    //for Watchdog reset  
                       }      

                       
if(!floor)break;     		
}	//End of MainLoop of program
     		                               
} //end of while(1)
         
	
}     //end of main()          
//************************ END MAIN ********************

unsigned long int assemble_ipdot(unsigned char IP_part1 , unsigned char  IP_part2,
                                    unsigned char IP_part3 , unsigned char IP_part4 )
{
     return   ( ( (unsigned long int)(IP_part1) <<24  ) | ( (unsigned long int)(IP_part2) <<16 )
                  | ( (unsigned long int)(IP_part3) <<8   ) | ( (unsigned long int)(IP_part4) ) );
 }                             


void netstackInit(uint32_t ipaddress, uint32_t netmask, uint32_t gatewayip,char EthAddrOffset)
{
	// init network device driver
	#ifdef NETSTACK_DEBUG
	printf("Initializing Network Device\r\n");
	#endif
	nicInit(EthAddrOffset);
	// init ARP
	#ifdef NETSTACK_DEBUG
	printf("Initializing ARP cache\r\n");
	#endif
	arpInit();
	// init addressing
	#ifdef NETSTACK_DEBUG
	printf("Initializing Addressing\r\n");
	#endif
	ipSetConfig(ipaddress, netmask, gatewayip);
}   

void netstackUDPIPProcess(unsigned int len, udpip_hdr* packet)
{
	u16 payloadlen=0;
	u08* payloaddata=0;
	u16 i;

	payloadlen = htons(packet->udp.udplen);       // get UDP payload length
	payloadlen -= 8;                              // subtract header
	
	// get UDP payload data
	payloaddata = &((unsigned char*)packet)[IP_HEADER_LEN+UDP_HEADER_LEN];
	
      //debugPrintHexTable(len, (unsigned char*)packet);

	if(packet->udp.destport == HTONS(CONTROL_PORT))
	{		
	    processCommand(payloadlen, payloaddata);  // command packet
	    
	}
	else if(packet->udp.destport == HTONS(LOOPBACK_PORT))
	{
		// loopback - return packet to sender
		udpSend(htonl(packet->ip.srcipaddr), LOOPBACK_PORT, payloadlen, payloaddata);
	}
}


void netstackService(void)
{
	unsigned int len,i=0;
	unsigned char NetBuffer[NETSTACK_BUFFERSIZE];
	
	struct netEthHeader* ethPacket;   
	
       
        if(Grouptype!='s')  
          { 
             if(_reqexistinq)
               {                     
                   if(WL[0].ReqExpireTime)if(++WL[0].ReqExpireTime==Eth_Timeout_For_Rec)Waiting_List_Manager(0,0,0,'f',0,0);
                   if(WL[1].ReqExpireTime)if(++WL[1].ReqExpireTime==Eth_Timeout_For_Rec)Waiting_List_Manager(0,0,0,'f',0,0);
                   if(WL[2].ReqExpireTime)if(++WL[2].ReqExpireTime==Eth_Timeout_For_Rec)Waiting_List_Manager(0,0,0,'f',0,0);
                   if(WL[3].ReqExpireTime)if(++WL[3].ReqExpireTime==Eth_Timeout_For_Rec)Waiting_List_Manager(0,0,0,'f',0,0);                                               
               }  

          }	
	

         if(--Eth_Send_IGP_Timeout==0)
                        {
                         Send_InGroupPresent_Packet(); 
                         Eth_Send_IGP_Timeout=300;
                         return;
                        }
         if(--Eth_Rec_IGP_Timeout==0) 
                        {
                           netstackInit(IPADDRESS, NETMASK, GATEWAY,Eth_number); 
                           Eth_Rec_IGP_Timeout=320;       
                           return;
                        }
                        

	
	// look for a packet                                      
	len = nicPoll(NETSTACK_BUFFERSIZE, NetBuffer);
	if(len)
	{
		ethPacket = (struct netEthHeader*)&NetBuffer[0];
		if(ethPacket->type == htons(ETHTYPE_IP))
		{
			// process an IP packet
			// add the source to the ARP cache
			// also correctly set the ethernet packet length before processing?
			arpIpIn((struct netEthIpHeader*)&NetBuffer[0]);	
			netstackIPProcess( len-ETH_HEADER_LEN, (ip_hdr*)&NetBuffer[ETH_HEADER_LEN] );
			#ifdef NETSTACK_DEBUG
			printf("NET Rx: IP packet\r\n");
			#endif
		        //arpPrintTable();
		}
		else if(ethPacket->type == htons(ETHTYPE_ARP))
		{
			// process an ARP packet
			arpArpIn(len, ((struct netEthArpHeader*)&NetBuffer[0]) );
			#ifdef NETSTACK_DEBUG
			printf("NET Rx: ARP packet\r\n");
			#endif		
		}
		#if NET_DEBUG >= 5
		printf("Received packet len: %d\r\n", len);
//		printfu16(htons(ethPacket->type));
//		printfCRLF();
		printf("Packet Contents\r\n");
		printf("------------------------------------------------------\r\n");		
		for (i=0;i<len;i++)
		{
		   if (i!=0)
		   {
		     if ((i%16)==0 )
		       printf("\r\n");    
		   }    		
         	   if ( (*(NetBuffer+i))<16 )
		     printf("%d",0);
		   printf("%x ",*(NetBuffer+i));
		   if(i==(len-1))         
		      printf("\r\n");    		  
		}    		
//		debugPrintHexTable(len, NetBuffer);
		#endif   
	}                
	return ;
}

void netstackIPProcess(unsigned int len, ip_hdr* packet)
{
	// check IP addressing, stop processing if not for me and not a broadcast
	if( (htonl(packet->destipaddr) != ipGetConfig()->ip) &&
		(htonl(packet->destipaddr) != (ipGetConfig()->ip|ipGetConfig()->netmask)) &&
		(htonl(packet->destipaddr) != 0xFFFFFFFF) ) 
		return;

	// handle ICMP packet
	if( packet->proto == IP_PROTO_ICMP )
	{
	      icmpIpIn((icmpip_hdr*)packet);
              #ifdef NETSTACK_DEBUG
		printf("NET Rx: ICMP/IP packet\r\n");
		//icmpPrintHeader((icmpip_hdr*)packet);
	      #endif
	}
	else if( packet->proto == IP_PROTO_UDP )
	{
		netstackUDPIPProcess(len, ((udpip_hdr*)packet) );
		#ifdef NETSTACK_DEBUG
		printf("NET Rx: UDP/IP packet\r\n");
		//debugPrintHexTable(NetBufferLen-14, &NetBuffer[14]);
		#endif
	}
/*	else if( packet->proto == IP_PROTO_TCP )
	{
		netstackTCPIPProcess(len, ((tcpip_hdr*)packet) );
		#ifdef NETSTACK_DEBUG
		   printf("NET Rx: TCP/IP packet\r\n");
		#endif

	}*/
	else
	{
		#ifdef NETSTACK_DEBUG
		  printf("NET Rx: IP packet\r\n");
		#endif
	}
}  
         

void processCommand(u16 len, u08* data)
{    
      unsigned char i=0,pc1=0,pc2=0;   
     // printf("Rec Eth = %2x %2x %2x %2x %2x %2x %2x %2x\r\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
      switch(data[0])
      	{           
	case 'F':
	      if(data[1]>Floorsno)break;                   //turn off led
         if(Numtype=='n')
	   {
         if(Navistrategy=='s')
          {
           pc2=(data[1]%8);
           if(!pc2)pc2=8;
           pc2*=2; 
           sep_id.source_address = node_address;
           sep_id.dest_address = 0xF0+((data[1]-1)/8);
           sep_id.message_num = 0;
           sep_id.priority = 0;    
           ID = assemble_id(sep_id); 
           current_message.id = ID;	
           current_message.ide=1;
           current_message.rb=0;
           current_message.rtr=0;           
           current_message.datalen=2;
           current_message.data[0]='F';
           current_message.data[1]=pc2;
           can_send_message(current_message);  
           delay_ms(10);
           pc2--;
           current_message.data[1]=pc2;                         
           can_send_message(current_message); 
           delay_ms(10);                    
	  }
	  else 
	  if((Navistrategy=='f')||(Navistrategy=='d'))
	   {
	    pc2=(data[1]%16);                //sv1=> index of dest_address    //floor
            if(!pc2)pc2=16;       
        
            sep_id.source_address = node_address;
            sep_id.dest_address = 0xF0+((data[1]-1)/16);
            sep_id.message_num = 0;
            sep_id.priority = 0;  
            ID = assemble_id(sep_id); 
            current_message.id = ID;   
            current_message.ide=1;
            current_message.rb=0;
            current_message.rtr=0;           
            current_message.datalen=2;	
            current_message.data[0]='F';
            current_message.data[1]=pc2;
            can_send_message(current_message);
            delay_ms(10); 
           }  
          } 
          else                //else for Numtype=='n'
          if(Numtype=='d')
            {     
              sep_id.source_address = node_address;          
              sep_id.message_num = 0;
              sep_id.priority = 0;             
              sep_id.dest_address = 0xB0;
              ID = assemble_id(sep_id);       //turn off led on num-floor board
              current_message.id = ID;  
              current_message.ide=1;
              current_message.rb=0;
              current_message.rtr=0;           
              current_message.datalen=2;           	
              current_message.data[0]='F';
              current_message.data[1]=data[1];
              can_send_message(current_message);
              delay_ms(10);                       
             } 	 
          else
          if(Numtype=='s')
             {   
              serial_current_message.address=0xD3;
              serial_current_message.length=2;
              serial_current_message.data[0]='F';          
              serial_current_message.data[1]=data[1]; 
              RS485_send_message(serial_current_message);
              delay_ms(10);           
             }        
         else                       
         if(Numtype=='f')
	   {
         if(Navistrategy=='s')
          {
           pc2=(data[1]%8);
           if(!pc2)pc2=8;
           pc2*=2; 
           serial_current_message.address=0xF0+((data[1]-1)/8);
           serial_current_message.length=2;  
           serial_current_message.data[0]='F';          
           serial_current_message.data[1]=pc2; 
           RS485_send_message(serial_current_message);
           delay_ms(10);                
           pc2--;
           serial_current_message.data[1]=pc2;                          
           RS485_send_message(serial_current_message);
           delay_ms(10);                     
	  }
	  else 
	  if((Navistrategy=='f')||(Navistrategy=='d'))
	   {
	    pc2=(data[1]%16);                //sv1=> index of dest_address    //floor
            if(!pc2)pc2=16;       
        
            serial_current_message.address=  0xF0+((data[1]-1)/16);
            serial_current_message.length=2;  
            serial_current_message.data[0]='F';          
            serial_current_message.data[1]=pc2; 
            RS485_send_message(serial_current_message);
            delay_ms(10);  
           }  
          }           

             if(Monitoring == 'y')_reset_out_request=data[1];        //ethernet_enable       
	break;
	
	case 'E':
	        if(data[1]!=0xFE)break;                       //turn off all out led
	        if(data[2]!=0xFE)break;
                if(Numtype=='n')
                  {
                for(pc1=0;pc1<((Floorsno-1)/8+1);pc1++)       //Reset Floor Boards outputs
                   {
                     sep_id.source_address = node_address;
                     sep_id.dest_address = 0xF0+pc1;
                     sep_id.message_num = 0;
                     sep_id.priority = 0;  
                     sep_id.message_num = 0;
                     sep_id.priority = 0;                      
                     ID = assemble_id(sep_id); 
                     current_message.id = ID;
                     current_message.datalen=1;
                     current_message.ide=1;
                     current_message.rb=0;
                     current_message.rtr=0;           
                     current_message.data[0]='R';
                     can_send_message(current_message);
                     delay_ms(10); 
                   }
                  } 
                  else 
                  if(Numtype=='d')
                  {       
                    sep_id.source_address = node_address;                   
                    sep_id.dest_address = 0xB0;              //Num-Floor Board
                    sep_id.message_num = 0;
                    sep_id.priority = 0;                     
                    ID = assemble_id(sep_id);   
                    current_message.id = ID;                   
                    current_message.ide=1;
                    current_message.rb=0;
                    current_message.rtr=0;                    
                    current_message.datalen=1;
                    current_message.data[0]='R';
                    can_send_message(current_message);   //Turn off cabin relays  
                    delay_ms(10); 
                  }            
                 else
                if(Numtype=='s')
                  {   
                    serial_current_message.address=0xD3;
                    serial_current_message.length=1;
                    serial_current_message.data[0]='R';          
                    RS485_send_message(serial_current_message);     
                    delay_ms(10);           //************  
                  }         
                  else
               if(Numtype=='f')
                  {
                  for(pc1=0;pc1<((Floorsno-1)/8+1);pc1++)       //Reset Floor Boards outputs
                   {
                    serial_current_message.address=0xF0+pc1;
                    serial_current_message.length=1;
                    serial_current_message.data[0]='R';          
                    RS485_send_message(serial_current_message);     
                    delay_ms(10);    
                   }
                  }                               
       
                if(Monitoring == 'y')_reset_out_request = 0xFF;   //ethernet_enable
           
	break;     
	
	case 'G':
	        if(data[1]>Floorsno)break;                       //turn off all out led
	        if((data[2]!=2)&&(data[2]!=4))break;
                         if(Numtype=='d')
                               {
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0xB0;
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;  
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;  
                                   current_message.datalen=2;
                                   current_message.ide=1;
                                   current_message.rb=0;
                                   current_message.rtr=0;
                                   current_message.id=ID;
                                   current_message.data[0]='O';
                                   current_message.data[1]=data[1];                                                                      
                                   can_send_message(current_message);  
                                   delay_ms(5);    
                                }   
                       else
                       if(Numtype=='s')
                               {
                                   serial_current_message.address=0xD3; 
                                   serial_current_message.length=2;
                                   serial_current_message.data[0]='O'; 
                                   serial_current_message.data[1]=data[1];           
                                   RS485_send_message(serial_current_message);     
                                   delay_ms(10);   
                                }   
                       else                       
                       if(Numtype=='n')
                               { 
                              if((Navistrategy=='f')||(Navistrategy=='d'))
                                {
                                   pc1 = (data[1] - 1) / 16;     
                                   pc2 = data[1] % 16;
                                   if(!pc2)pc2=16;
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0xF0 + pc1; 
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;  
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;  
                                   current_message.datalen=2;
                                   current_message.ide=1;
                                   current_message.rb=0;
                                   current_message.rtr=0;
                                   current_message.data[0]='O'; 
                                   current_message.data[1]=pc2;                                     
                                   can_send_message(current_message);  
                                   delay_ms(5);                                      
                                 }
                              else if(Navistrategy=='s')
                                 {
                                   pc1 = (data[1] - 1) / 8;      
                                   pc2 = data[1] % 8;
                                   if(!pc2)pc2=8;
                                   pc2 *= 2;
                                   if(data[2] == 4)pc2--;        //2
                                   
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0xF0 + pc1; 
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;  
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;  
                                   current_message.datalen=2;
                                   current_message.ide=1;
                                   current_message.rb=0;
                                   current_message.rtr=0;
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;    
                                   current_message.data[0]='O'; 
                                   current_message.data[1]=pc2;
                                   can_send_message(current_message);  
                                   delay_ms(5);                                       
                                 }   
                        }                     
                     else      
                  if(Numtype=='f')
                         { 
                              if((Navistrategy=='f')||(Navistrategy=='d'))
                                {
                                   pc1 = (data[1] - 1) / 16;     
                                   pc2 = data[1] % 16;
                                   if(!pc2)pc2=16;
                                   serial_current_message.address=0xF0 + pc1; 
                                   serial_current_message.length=2;
                                   serial_current_message.data[0]='O'; 
                                   serial_current_message.data[1]=pc2;           
                                   RS485_send_message(serial_current_message);     
                                   delay_ms(10);                                       
                                 }
                              else if(Navistrategy == 's')
                                 {
                                   pc1 = (data[1] - 1) / 8;      
                                   pc2 = data[1] % 8;
                                   if(!pc2)pc2=8;
                                   pc2 *= 2;
                                   if(data[2] == 4)pc2--;   //2
                                   
                                   serial_current_message.address=0xF0 + pc1; 
                                   serial_current_message.length=2;
                                   serial_current_message.data[0]='O'; 
                                    serial_current_message.data[1]=pc2;           
                                   RS485_send_message(serial_current_message);     
                                   delay_ms(10);                                    
                                 }      
                              else return;                                  
                           }                                
                                                                     
                                
                       else return;                                  
	break;	

	
	case 'C':                                                  //fulfill the command
	     if((data[2]!=2)&&(data[2]!=4))break;
             if(pathfinder(data[1],data[2]))
             {
                          beep_request=1;        //blink cabin led
                         if(Numtype=='d')
                               {
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0x10;
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;  
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;  
                                   current_message.datalen=2;
                                   current_message.ide=1;
                                   current_message.rb=0;
                                   current_message.rtr=0;
                                   current_message.id=ID;
                                   current_message.data[0]='B';
                                   current_message.data[1]=data[1];                                                                      
                                   can_send_message(current_message);  
                                   delay_ms(5);    
                                   sep_id.dest_address = 0xB0; 
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;    
                                   current_message.data[0]='O'; 
                                   can_send_message(current_message);  
                                   delay_ms(5);                                          
                                }      
                      else   
                      if(Numtype=='s')
                               {
                                   serial_current_message.address=0xD3; 
                                   serial_current_message.length=2;
                                   serial_current_message.data[0]='O'; 
                                   serial_current_message.data[1]=data[1];           
                                   RS485_send_message(serial_current_message);     
                                   delay_ms(1);
                                   sep_id.dest_address = 0xC0;            //cabin  
                                   sep_id.source_address = node_address;
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;                                     
                                   ID = assemble_id(sep_id);
                                   current_message.id = ID;  
                                   current_message.datalen=2;
                                   current_message.data[0]='B';                                 
                                   current_message.data[1]=data[1];                     
                                   can_send_message(current_message);
                                   delay_ms(10);                                    
                               }
                      else                      
                      if(Numtype=='n')
                            {
                              if((Navistrategy=='f')||(Navistrategy=='d'))
                                {
                                   pc1 = (data[1] - 1) / 16;     
                                   pc2 = data[1] % 16;
                                   if(!pc2)pc2=16;
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0xE0 + pc1;
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;  
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;  
                                   current_message.datalen=2;
                                   current_message.ide=1;
                                   current_message.rb=0;
                                   current_message.rtr=0;
                                   current_message.id=ID;
                                   current_message.data[0]='B';
                                   current_message.data[1]=pc2;                                                                      
                                   can_send_message(current_message);  
                                   delay_ms(5);    
                                   sep_id.dest_address = 0xF0 + pc1; 
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;    
                                   current_message.data[0]='O'; 
                                   can_send_message(current_message);  
                                   delay_ms(5);                                      
                                 }
                              else if(Navistrategy=='s')
                                 {
                                   pc1 = (data[1] - 1 ) / 16;     
                                   pc2 = data[1] % 16;
                                   if(!pc2)pc2=16;
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0xE0 + pc1;
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;  
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;  
                                   current_message.datalen=2;
                                   current_message.ide=1;
                                   current_message.rb=0;
                                   current_message.rtr=0;
                                   current_message.id=ID;
                                   current_message.data[0]='B';
                                   current_message.data[1]=pc2;                                                                      
                                   can_send_message(current_message);  
                                   delay_ms(5); 
                                   
                                   pc1 = (data[1] - 1) / 8;      
                                   pc2 = data[1] % 8;
                                   if(!pc2)pc2=8;
                                   pc2 *= 2;
                                   if(data[2] == 4)pc2--;  ///2
                                      
                                   sep_id.dest_address = 0xF0 + pc1; 
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;    
                                   current_message.data[0]='O'; 
                                   current_message.data[1]=pc2;
                                   can_send_message(current_message);  
                                   delay_ms(5);                                       
                                 }     
                                 else return;
                            }
                      else
                     if(Numtype=='f')
                            {
                              if((Navistrategy=='f')||(Navistrategy=='d'))
                                {
                                   pc1 = (data[1] - 1) / 16;     
                                   pc2 = data[1] % 16;
                                   if(!pc2)pc2=16;
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0xC0;
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;  
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;  
                                   current_message.datalen=4;
                                   current_message.ide=1;
                                   current_message.rb=0;
                                   current_message.rtr=0;
                                   current_message.id=ID; 
                                   current_message.data[0]='X';
                                   current_message.data[1]=0xE0 + pc1;                                    
                                   current_message.data[2]='B';
                                   current_message.data[3]=pc2;                                                                      
                                   can_send_message(current_message);  
                                   delay_ms(5);    
                                   
                                   serial_current_message.address=0xF0 + pc1; 
                                   serial_current_message.length=2;
                                   serial_current_message.data[0]='O'; 
                                    serial_current_message.data[1]=pc2;           
                                   RS485_send_message(serial_current_message);     
                                   delay_ms(10);    
                                 }
                              else if(Navistrategy=='s')
                                 {
                                   pc1 = (data[1] - 1 ) / 16;     
                                   pc2 = data[1] % 16;
                                   if(!pc2)pc2=16;  
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0xC0;
                                   sep_id.message_num = 0;
                                   sep_id.priority = 0;  
                                   ID = assemble_id(sep_id); 
                                   current_message.id = ID;  
                                   current_message.datalen=4;
                                   current_message.ide=1;
                                   current_message.rb=0;
                                   current_message.rtr=0;
                                   current_message.id=ID; 
                                   current_message.data[0]='X';
                                   current_message.data[1]=0xE0 + pc1;                                    
                                   current_message.data[2]='B';
                                   current_message.data[3]=pc2;                                                                      
                                   can_send_message(current_message);  
                                   delay_ms(5);                                     
                                   
                                   
                                   pc1 = (data[1] - 1) / 8;      
                                   pc2 = data[1] % 8;
                                   if(!pc2)pc2=8;
                                   pc2 *= 2;
                                   if(data[2] == 4)pc2--;     //2    
                                   serial_current_message.address=0xF0 + pc1; 
                                   serial_current_message.length=2;
                                   serial_current_message.data[0]='O'; 
                                    serial_current_message.data[1]=pc2;           
                                   RS485_send_message(serial_current_message);     
                                   delay_ms(5);                                       
                                     
                                 }   
                                                          
                                    else return;
                                    
                               }  
                       else return;                                     
                 
                    DataBuff[0]='G';
                    DataBuff[1]= data[1];   //request
                    DataBuff[2]= data[2];   //request type
                    DataBuff[3]= 0;        //Evalue
                    DataBuff[4]= 0;     //  previous station Evalue
                    DataBuff[5]= 0;    //unique id
                    DataBuff[6]= IP;                      
                    
        	     for(i=0;i<=7;i++)
                    buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
                    udpSend(assemble_ipdot(192,168,0,data[6]), CONTROL_PORT, 7, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);                    

              }  // turn on led input and in cabin
	     

		break;	
	case 'R':                                            //receive a request and reply it
	//   data[1] = Request
	//   data[2] = Request Type
	//   data[3] = Evalue
	//   data[4] = 0
	//   data[5] = Unique ID
	//   data[6] = IP    
	
	           
	      if(Waiting_List_Manager(data[1],data[2], 0,'e',0,data[5])==0xFF)break;
	           //check if request existing request in List,
	           
	           
	            

               pc1 = Evaluation(data[1],data[2]);   
         
               if((stayinstair==1)&&(floor==data[1]))
                     {
                       pc1=0xFF;      //turn off out led
                       pathfinder(data[1],data[2]);               //for exit from standby
                     }
                            
  
                  //  printf("pc1=%d\r\n",pc1);
                    
                    DataBuff[0]='S';
                    DataBuff[1]= data[1];   //request
                    DataBuff[2]= data[2];   //request type
                    DataBuff[3]= pc1;        //Evalue
                    DataBuff[4]= data[3];     //  previous station Evalue
                    DataBuff[5]= data[5];    //unique id
                    DataBuff[6]= IP;                      
                    
        	     for(i=0;i<=7;i++)
                    buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
                    udpSend(assemble_ipdot(192,168,0,data[6]), CONTROL_PORT, 7, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);                    

		break;
	case 'S':                                                 //receive replied request and reform  WL
	//   data[1] = Request
	//   data[2] = Request Type
	//   data[3] = Evalue
	//   data[4] = previous station Evalue
	//   data[5] = Unique ID
	//   data[6] = IP   
	
	if(data[1]>Floorsno)break;
	if((data[2]==4)&&(Navistrategy!='s'))return;      
	if((data[2]!=2)&&(data[2]!=4))return;
	
                      
        if(data[3]==0xFF)               //if stay instair at another station
	        {
                  Waiting_List_Manager(data[1],data[2], 0,'d',0,data[5]); //clear the request
	          break;}         
	          
                           
        pc1=Waiting_List_Manager(data[1],data[2]             //req,reqtype
                                                  , data[3],'c'                // evalue, action
                                                  , data[6],data[5]);    //IP , unique id
        
        if(pc1<4)
                 {     
                             WL[pc1].ReqExpireTime=10; 
                             Waiting_List_Manager(0,0 , 0,'f', 0,0);                             
                 }
             
	break;   
		 
	case 0xFF:
	   if (data[1]==0xF1)
	     {     
 	         DataBuff[0]=0xFE;
                 DataBuff[1]=0xF2;
                 DataBuff[2]=floor;     //floor;     
 	         DataBuff[3]=Floorsno;        //No. of Floors
                 DataBuff[4]=1;         // Min of floors  
                 DataBuff[5]=IP;               
        	     for(i=0;i<=5;i++)
                    buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
                 udpSend(GATEWAY, CONTROL_PORT, 6, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		

	     }
	     if (data[1]=='C')
	     {     
               pathfinder(data[2],2);
	     }	        
	     
	     if((data[1]==0xFF)&&(data[2]==0xFF)&&(data[3]==0xFF)&&
	          (data[4]=='I')&&(data[5]=='G')&&(data[6]=='P'))
	                {
	                  Eth_Rec_IGP_Timeout=320;
	           //       Eth_disconnect=0;
	                }
        
	 break;
		
	default:

		break;
	}  
}

          

void Send_door_stat(void)
{       
unsigned char i;   

if(!Eth_number)return;
if(Monitoring != 'y')return;


  if (RELAYO==1)                    //OPENNING            
     Door_Status=0x00;	            
  else
  if (RELAYC==1)                 
    {Door_Status=0x01;              //CLOSING
     if (RELAYCed==1)                    
           Door_Status=0x03;} 

if (RELAYOed==1)                 
   Door_Status=0x02;	            //OPENED
   

   DataBuff[0]=0xFF;
   DataBuff[1]=0xFF;
   DataBuff[2]=0x01;          //Status Packet
   DataBuff[3]=0xF2;          //Door Packet
   DataBuff[4]=Door_Status;   //Door_Status
   DataBuff[5]=IP ;            //IP address
   DataBuff[6]=floor;             
   DataBuff[7]=0;           
   for(i=0;i<=7;i++)
      buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
   udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		
                   
}               

void Send_move_stat(void)
{       
unsigned char i;    

if(!Eth_number)return;
if(Monitoring != 'y')return;

   if((!I)||(!floor))
   {
    side=0;
    vel=0;
   }        
   else
   {
     if (CONTF==1) vel=1; 
     else if ((CONTS==1)||(CONTL==1)||(CONTM==1)) vel=2;           //SLOW
     else vel=0;     
     
     if (direction==1) side=1;                   //UP
     else if (direction==2) side=2;          //DOWN
     else side=0;
   }           
 
   DataBuff[0]=0xFF;
   DataBuff[1]=0xFF;
   DataBuff[2]=0x01;    //Status Packet
   DataBuff[3]=0xF1;    //Moving Status
   DataBuff[4]=side;    //Up or Down
   DataBuff[5]=vel ;    //High or Low
   DataBuff[6]=floor;   //Floor number
   DataBuff[7]=IP;       //IP address 
   for(i=0;i<=7;i++)
      buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
   udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		
              
}
 
 void Send_requests_stat_2(unsigned char Req)
 {
   unsigned char i;  
   
  if(!Eth_number)return;
  if(Monitoring != 'y')return;

	DataBuff[0]=0xFF;
   	DataBuff[1]=0xFF;
   	DataBuff[2]=0x01;     //Status Packet
   	DataBuff[3]=0xF2;     //   4   //Outsid cabin Request
   	DataBuff[4]=Req;      //floor to stop
   	DataBuff[5]=1;        //ReqDir;   //wants to go down=0 or up=1
   	DataBuff[6]=0;        
   	DataBuff[7]=IP;       //IP address 
   	for(i=0;i<=7;i++)
      		buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
   	udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		

 }	  
 
 void Send_requests_stat_4(unsigned char Req)
 {
   unsigned char i;
   
   if(!Eth_number)return;
   if(Monitoring != 'y')return;   

	DataBuff[0]=0xFF;
   	DataBuff[1]=0xFF;
   	DataBuff[2]=0x01;     //Status Packet
   	DataBuff[3]=0xF4;     //Outsid cabin Request
   	DataBuff[4]=Req;      //floor to stop
   	DataBuff[5]=0;        //ReqDir;   //wants to go down=0 or up=1
   	DataBuff[6]=0;        
   	DataBuff[7]=IP;       //IP address 
   	for(i=0;i<=7;i++)
      		buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
   	udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		

 }
 
 
   void Send_requests_stat_1(unsigned char Req)
 {
   unsigned char i;  
   
   if(!Eth_number)return;
   if(Monitoring != 'y')return;   
   
   	DataBuff[0]=0xFF;
   	DataBuff[1]=0xFF;
   	DataBuff[2]=0x01;       //Status Packet
   	DataBuff[3]=0xF3;       //Cabin Request
   	DataBuff[4]=Req;        //floor to stop 1
   	DataBuff[5]=0;           //floor to stop 2
   	DataBuff[6]=0;          //floor to stop 3
  	DataBuff[7]=IP;         //IP address 
   	for(i=0;i<=7;i++)
      		buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
   	udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		

 }	
 
 void Reset_out_request_stat(unsigned char ResetRequest)
 {  
   unsigned char i=0;  
   
   if(!Eth_number)return;
   if(Monitoring != 'y')return;   
   

     if(ResetRequest==0xFF)
      { 
    //Turn Off LED//
          DataBuff[0]=0xFF;
          DataBuff[1]=0xFF;
          DataBuff[2]=0x01;     //Status Packet
          DataBuff[3]=0xF6;     //Reset ALL request packet
          DataBuff[4]=0;        
          DataBuff[5]=0;       
          DataBuff[6]=0;        
          DataBuff[7]=IP;       //IP address 
          for(i=0;i<=7;i++)
             buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
          udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);     
          return;    
      }
    //Turn Off LED//
      DataBuff[0]=0xFF;
      DataBuff[1]=0xFF;
      DataBuff[2]=0x01;     //Status Packet
      DataBuff[3]=0xF5;     //Reset Outside request packet
      DataBuff[4]=floor;    //floor
      DataBuff[5]=0 ;       //
      DataBuff[6]=0;        //
      DataBuff[7]=IP;       //IP address 
      for(i=0;i<=7;i++)
        buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
         udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		

 }	       

 void Send_InGroupPresent_Packet(void)
 {
   unsigned char i; 
   
  if(!Eth_number)return;

	DataBuff[0]=0xFF;
   	DataBuff[1]=0xFF;
   	DataBuff[2]=0xFF;     //Status Packet
   	DataBuff[3]=0xFF;     //   4   //Outsid cabin Request
   	DataBuff[4]='I';      //floor to stop
   	DataBuff[5]='G';        //ReqDir;   //wants to go down=0 or up=1
   	DataBuff[6]='P';        
   	DataBuff[7]=IP;       //IP address 
   	for(i=0;i<=7;i++)
      		buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
   	udpSend(BROADCAST, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		

 } 
 
 
 void Send_Request_via_Eth(struct Waiting_List_ob  Waiting_List_req, unsigned char RequestType)
{
   unsigned char i;           
   
   if(!Eth_number)return;       

         if(RequestType=='C')
                 {        //send to one station
          	
                     	DataBuff[0]='C';  //R or S or C
                	DataBuff[1]=Waiting_List_req.Req;
                	DataBuff[2]=Waiting_List_req.ReqType;                //reqtype
                	DataBuff[3]=0;                 //ReqEval
                	DataBuff[4]=0;        //ReqExpireTime
                	DataBuff[5]=0;           //ReqUniqueID
                	DataBuff[6]=IP;        
                  
   	                for(i=0;i<=7;i++)
      	                          	buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
                      	udpSend(assemble_ipdot(192,168,0,Waiting_List_req.ReqIP), CONTROL_PORT, 7, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);   
                 }
           else
           if(RequestType=='R')
                 {                      //maybe send to many    *********
          	                         // it must be send to any online station
                     	DataBuff[0]='R';  //R or S or C
                	DataBuff[1]=Waiting_List_req.Req;
                	DataBuff[2]=Waiting_List_req.ReqType;                //reqtype
                	DataBuff[3]=Waiting_List_req.ReqEval;                 //ReqEval
                	DataBuff[4]=0;                                                  //ReqExpireTime
                	DataBuff[5]=Waiting_List_req.ReqUniqueID;           //ReqUniqueID
                	DataBuff[6]=IP;        
                  
   	                for(i=0;i<=7;i++)
      	                          	buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
                      	udpSend(BROADCAST, CONTROL_PORT, 7, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);   
                 }    
           else
           if(RequestType=='F')          //turn off out led
                 {                      
          	                         // it must be send to any online station
                     	DataBuff[0]='F';  //R or S or C
                	DataBuff[1]=turnoffleds;
                	DataBuff[2]=0;              
                	DataBuff[3]=0;            
                	DataBuff[4]=0;                                                  
                	DataBuff[5]=0;       
                	DataBuff[6]=IP;        
                  
   	                for(i=0;i<=7;i++)
      	                          	buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
                      	udpSend(BROADCAST, CONTROL_PORT, 2, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);   
                 }     
           else
           if(RequestType=='E')
                 {                      
          	                         // it must be send to any online station
                     	DataBuff[0]='E';  //R or S or C
                	DataBuff[1]=0xFE;
                	DataBuff[2]=0xFE;        
                	DataBuff[3]=0;               
                	DataBuff[4]=0;                                                  
                	DataBuff[5]=0;   
                	DataBuff[6]=IP;        
                  
   	                for(i=0;i<=7;i++)
      	                          	buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
                      	udpSend(BROADCAST, CONTROL_PORT, 3, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);   
                 }     

}

                
void recovery(char rwe)
{    

char rec[4];  
//lastsavedfloor in 31H
//Number of Starts in 32H-33H

if(rwe==1)
     { 
            rtc_write(0x31,floor);
            return;
     }
     
if(rwe==2)
     {   
            floor=rtc_read(0x31);     
            return;
     }            
                 
if(rwe==3)
     {  
            rtc_write(0x31,0);    
            return;
     }

if(rwe==4)
      {             
            rec[0]=rtc_read(0x32);
            rec[1]=rtc_read(0x33);  
            number_of_start=rec[0]+rec[1]*0x0100;
    
            number_of_start++;   
            rec[0]=number_of_start % 0x0100;
            rec[1]=number_of_start / 0x0100;            
            rtc_write(0x32,rec[0]);
            rtc_write(0x33,rec[1]);
            
            return;
      }

if(rwe==5)
      {                   
            rtc_write(0x32,0);
            rtc_write(0x33,0);

            return;
      }                                    

}  


void LED_indicator(void)
{
    unsigned char tov1=0x00;
    if(FUL&&(!LED_blink))tov1 |=0x10;
    if(FUL&&(LED_blink))tov1 &= ~0x10;                    //for blinking in FUL mode
    
//    if(DOO&&(!LED_blink))tov1 |= 0x80;
//    if(DOO&&(LED_blink))tov1 &= ~0x80;               //for blinking in DO 
    
    if((get_mob_free()==0xFF)&&(!LED_blink))tov1 |= 0x04;
    if((get_mob_free()==0xFF)&&(LED_blink))tov1 &= ~0x04;               //for blinking in move-to-park 
    
    if(floor&&(opmode=='n')&&(!LED_blink))tov1|=0x01; 
    if(floor&&(opmode=='n')&&(LED_blink))tov1&=~0x01;      //for blinking in moving   
    
    if((opmode=='n')&&(!I))tov1 |=0x01;                                 //RUN led
    if((!floor)&&(opmode!='n')&&(opmode!='m')&&(opmode!='c'))tov1|= 0x02;
    
    if(can_rxfull_mobs)tov1 &= ~0x04;
    if(cansrshow)tov1 |= 0x04;
    if(!can_rxfull_mobs)cansrshow  = 0;
    if(rs485show)tov1 |= 0x04;
    if(!Ready)rs485show=0;
        
    if(standbymf)tov1 |=0x08;
    if(OVL)tov1 |=0x10;
    if((opmode=='m')||(opmode=='c'))tov1 |= 0x20; 

    if(RELAYC&&(!LED_blink))tov1 |= 0x40; 
    if(RELAYC&&(LED_blink))tov1 &= ~0x40;     
    
    if(RELAYO)tov1 |= 0x40; 
    
    if(DOO)tov1 |=0x80;
    
    Write_7Seg(0,tov1);
}
      
                   

void set_outputs_relay(unsigned char output_select)
{  
unsigned char tempcont=0,carout=0;  

if(CONTU)
      SO1=1;                                         
 else SO1=0;
if(CONTD)
      SO2=1;
 else SO2=0;
if(CONTF)
      SO3=1;
 else SO3=0;
if(CONTS)
      SO4=1;
 else SO4=0;
if(CONTL)
      SO5=1;
 else SO5=0;
 if(CONTM)
      SO6=1;
 else SO6=0;      
 if(RELAYSTBY)
      SO7=1;
 else SO7=0; 
  if(URA)
      SO8=1;
 else SO8=0;  
  if(RELAYC)
      SO9=1;
 else SO9=0;
   if(RELAYO)
      SO10=1;
 else SO10=0;

if(output_select==1)                 //for sending Cabin data by Ethernet and CAN
 {
   if(!floor)
       {
        carout=0x00;
        if(URA)carout|=0x80;
        if(RELAYSTBY)carout|=0x10;
        
        if(Tunnel_Door_en=='2')
         {
           if(RELAYC)carout|=0x48 ;        //c1=1 c2=1
           if((!RELAYC)&&(!RELAYO)&&(!I))               //c=0 o=0
             {
             if(floor_cal)
              {
                tempcont=Tunnel_door_find(floor_cal);
                if(tempcont==1)carout|=0x08;                     //only 1st door is 00 2nd is close
                if(tempcont==2)carout|=0x40;                     //only 2nd door is 00 1st is close
              }                
             }  
                                      
           if(RELAYO)             
           {
             if(floor_cal)
              {
                tempcont=Tunnel_door_find(floor_cal);
                if(tempcont==1)carout|=0x28;                     //only 1st door is open 2nd is close
                if(tempcont==2)carout|=0x44;                     //only 2nd door is open 1st is close
                if(tempcont==3)carout|=0x24;                     //both of doors are open
              }
           }
         }
        else
        {
         if(RELAYC)carout|=0x40;
         if(RELAYO)carout|=0x20;        
        }
       
        sep_id.source_address = node_address;
        sep_id.dest_address = 0xC0;              //Cabin Board
        sep_id.message_num = 0;
        sep_id.priority = 0;  
        ID = assemble_id(sep_id); 
        current_message.id = ID; 
        current_message.datalen=2;            	
        current_message.data[0]='O';
        current_message.data[1]=carout;  
        can_send_message(current_message);   //turn off led in cabin      
        delay_ms(10);
        return;
       }
  _set_output_car=1;
  while(_set_output_car&&floor)WDRS=1;       
  
  if(Eth_number)
     {
       if(Monitoring == 'y')_send_door_status=1;            //Ethernet Monitornig packet
     }
 }
}     

unsigned char showfloor(unsigned char numfloor,unsigned char digit)
{ 

   if(manual_floor_naming=='y')
    {
      if(numfloor>64 || numfloor<1)return 0;
      if(digit==2)return Floor_names_1[numfloor];                //digit==2 ==> LEFT  7SEG
      if(digit==1)return Floor_names_2[numfloor];                //digit==1 ==> RIGHT 7SEG
      return 0;
    }



   if(digit==1)
    {
     if(numfloor==1)
                  {
                     if((Undergroundno==1)&&(Parksign!=0))
                     { 
                      if(Parksign==1)return 'P';
                      if(Parksign==2)return 'B';  
                      if(Parksign==3)return 'B'; 
                      if(Parksign==4)return 'G'; 
                      if(Parksign==5)return 'G';   //BPG
                      return 0;         
                     }          
                     if((Undergroundno>1)&&(Parksign==3))return 'B'; 
                  }
                           
     
     if(numfloor==2)
       {
         if((Undergroundno==2)&&(Parksign==3))return 'P'; 
         if((Undergroundno==2)&&(Parksign==5))return 'P';
 
       }
     
     if(Parksign==4)   
       {
          if(Undergroundno==numfloor)return 'G';     
          if((Undergroundno==2)&&(numfloor==1))return 'P';
          if(numfloor<Undergroundno)return ('0'+Undergroundno-numfloor); 
          
       } 
       
     if(Parksign==5)   
       {
          if(Undergroundno==numfloor)return 'G'; 
          if(Undergroundno==(numfloor+1))return 'P';
          if((Undergroundno==3)&&(numfloor==1))return 'B';    
          if(Undergroundno>(numfloor+1))return ('0'+Undergroundno-numfloor-1); 
          
       }            
     if(numfloor<=Undergroundno)return ('0'+Undergroundno-numfloor+1);
     if((Groundexist!=0)&&(numfloor==(Undergroundno+1)))
                 {
                      switch(Groundexist)
                       {
                         case 1:
                           return 'G';
                         break;
                         case 2:
                           return '0';
                         break;
                         case 3:
                           return 'L';
                         break;
                         case 4:
                           return 'B';
                         break;
                         case 5:
                           return 'P'; 
                         break;  
                         case 6:
                           return 'H'; 
                         break;                                                                                                                          
                       }
                      return 0;                                       
                 }
     if(Groundexist!=0)return ('0'+(numfloor-1-Undergroundno)%10);
        else return ('0'+(numfloor-Undergroundno)%10);
    }
    
   if(digit==2)
    {
     
     if(numfloor==1)
         {
           if((Undergroundno==1)&&(Parksign!=0))return 0; 
           if((Undergroundno>1)&&(Parksign==3))return 0;   
         } 
     if(numfloor==2)
         {
           if((Undergroundno==2)&&(Parksign==3))return 0;
           if((Undergroundno==2)&&(Parksign==4))return 0;
         }       
     
     if(Parksign==4)
       {
        if(Undergroundno==numfloor)return 0;
        if((Undergroundno==2)&&(numfloor==1))return 0;
       } 
       
     if(Parksign==5)
       {
          if(Undergroundno==numfloor)return 0; 
          if(Undergroundno==(numfloor+1))return 0;
          if((Undergroundno==3)&&(numfloor==1))return 0; 
       }       
       
                 
      
         
     if(numfloor<=Undergroundno)
      {
       switch(Parksign)
         {
          case 1:
          case 3:
          case 4:
           return 'P';
          break;
          case 0:
           return '-';
          break;
          case 2: 
          case 5:
           return 'B';
          break;
         }
       }
       
     if((Groundexist!=0)&&(numfloor==(Undergroundno+1)))return 0;  
     if(Groundexist!=0)
       {if((numfloor-1-Undergroundno)/10)return '0'+((numfloor-1-Undergroundno)/10);
        else return 0;}
     else 
       {if((numfloor-Undergroundno)/10)return '0'+((numfloor-Undergroundno)/10);
        else return 0;}     

    }
return 0;
}

char block_date(void)
{
   char date,month,year;
   char sec,min,hour,block=0;
   unsigned int exp_dayofyear=0,todayofyear=0;
   
   #ifdef  BLOCK_DATE_DISABLE
       return 0;
   #endif
   
   if(Expired_date==0xFF)return 0;
   if(Expired_month==0xFF)return 0; 
   if(Start_dayofyear==0xFFFF)return 0; 
   
   if((Expired_date==0xFA)&&(Expired_month==0xFA)&&(Start_dayofyear==0xFAFA))block= 1;
   
   rtc_get_time(&sec,&min,&hour);
   rtc_get_date(&date,&month,&year);

   WDRS=1; 
   
   
   if((date>31)||(date<1)||(month>12)||(month<1))block=1;
   
   if((sec==_sec_saved)&&(min==_min_saved)&&(hour==_hour_saved))
   {
     ptimer=0;
     while(ptimer<20)
      {
       WDRS=1;
       if(!floor)break;
      }
     rtc_get_time(&sec,&min,&hour);
     if((sec==_sec_saved)&&(min==_min_saved)&&(hour==_hour_saved))block=1;
   }
   
    _sec_saved=sec;
    _min_saved=min;
    _hour_saved=hour;
    
    if((date==Expired_date)&&(month==Expired_month))block=1;
    
    exp_dayofyear=calcdayofyear(Expired_date,Expired_month);    
    todayofyear=calcdayofyear(date,month);
    
    if(exp_dayofyear>=Start_dayofyear)
     {
        if((todayofyear>=exp_dayofyear)||(todayofyear<Start_dayofyear))block=1;
     }
    else
     {
        if((todayofyear>=exp_dayofyear)&&(todayofyear<Start_dayofyear))block=1;
     }
    
    if(block)
          {
             EE_Expired_date=0xFA;
             EE_Expired_month=0xFA; 
             EE_Start_dayofyear=0xFAFA;
             Expired_date=0xFA;
             Expired_month=0xFA;
             Start_dayofyear=0xFAFA;
             errormsg(103);              //BLC 
             blink_en=1;
             beep_en=1;
             turnoffallleds=1;
               while(turnoffallleds)
                  {
                    WDRS=1;
                    if(!floor)break;
                  }
             return 1;
          }
    else return 0;

}    

unsigned int calcdayofyear(unsigned char calc_date,unsigned char calc_month)
{

   unsigned int cdoy1=0;                    
                       
   if(calc_month>0)cdoy1=calc_date;
   if(calc_month>1)cdoy1+=31;
   if(calc_month>2)cdoy1+=28;                         
   if(calc_month>3)cdoy1+=31;
   if(calc_month>4)cdoy1+=30;
   if(calc_month>5)cdoy1+=31;
   if(calc_month>6)cdoy1+=30;
   if(calc_month>7)cdoy1+=31;
   if(calc_month>8)cdoy1+=31;
   if(calc_month>9)cdoy1+=30;
   if(calc_month>10)cdoy1+=31;
   if(calc_month>11)cdoy1+=30;
                  
   return cdoy1;

} 



