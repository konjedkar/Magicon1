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



/*! \file enc28j60.c \brief Microchip ENC28J60 Ethernet Interface Driver. */
//*****************************************************************************
//
// File Name	: 'enc28j60.c'
// Title		: Microchip ENC28J60 Ethernet Interface Driver
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
// Description	: This driver provides initialization and transmit/receive
//	functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
// This chip is novel in that it is a full MAC+PHY interface all in a 28-pin
// chip, using an SPI interface to the host processor.
//
//*****************************************************************************

#include <stdio.h>
#include <stdlib.h>

#include "global.h"
#include "enc28j60.h"


// include configuration
//#include "ax88796conf.h"


/* SPI Control Register */
#define    SPIE       7
#define    SPE        6
#define    DORD       5
#define    MSTR       4
#define    CPOL       3
#define    CPHA       2
#define    SPR1       1
#define    SPR0       0

/* SPI Status Register */
#define    SPIF       7
#define    WCOL       6
#define    SPI2X      0


#ifndef outb
	#define	outb(addr, data)  addr = (data)
#endif
#ifndef inb
	#define	inb(addr)	  (addr)
#endif
#ifndef BV
	#define BV(bit)		  (1<<(bit))
#endif
#ifndef cbi
	#define cbi(reg,bit)	  reg &= ~(BV(bit))
#endif
#ifndef sbi
	#define sbi(reg,bit)	  reg |= (BV(bit))
#endif

#define MIN(a,b)			((a<b)?(a):(b))

u08 Enc28j60Bank;
u16 NextPacketPtr;



void nicInit(char Eth)
{
	enc28j60Init(Eth);
}

void nicSend(unsigned int len, unsigned char* packet)
{
	enc28j60PacketSend(len, packet);
}

unsigned int nicPoll(unsigned int maxlen, unsigned char* packet)
{
	return enc28j60PacketReceive(maxlen, packet);
}

void nicGetMacAddress(u08* macaddr)
{
	// read MAC address registers
	// NOTE: MAC address in ENC28J60 is byte-backward
	*macaddr++ = enc28j60Read(MAADR5);
	*macaddr++ = enc28j60Read(MAADR4);
	*macaddr++ = enc28j60Read(MAADR3);
	*macaddr++ = enc28j60Read(MAADR2);
	*macaddr++ = enc28j60Read(MAADR1);
	*macaddr++ = enc28j60Read(MAADR0);
}

void nicSetMacAddress(u08* macaddr)
{
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60Write(MAADR5, *macaddr++);
	enc28j60Write(MAADR4, *macaddr++);
	enc28j60Write(MAADR3, *macaddr++);
	enc28j60Write(MAADR2, *macaddr++);
	enc28j60Write(MAADR1, *macaddr++);
	enc28j60Write(MAADR0, *macaddr++);
}
/*
void nicRegDump(void)
{
	enc28j60RegDump();
}


void ax88796SetupPorts(void)
{
#if NIC_CONNECTION == MEMORY_MAPPED
	// enable external SRAM interface - no wait states
	sbi(MCUCR, SRE);
//	sbi(MCUCR, SRW10);
//	sbi(XMCRA, SRW00);
//	sbi(XMCRA, SRW01);
//	sbi(XMCRA, SRW11);
#else
	// set address port to output
	AX88796_ADDRESS_DDR = AX88796_ADDRESS_MASK;

	// set data port to input with pull-ups
	AX88796_DATA_DDR = 0x00;
	AX88796_DATA_PORT = 0xFF;

	// initialize the control port read and write pins to de-asserted
	sbi( AX88796_CONTROL_PORT, AX88796_CONTROL_READPIN );
	sbi( AX88796_CONTROL_PORT, AX88796_CONTROL_WRITEPIN );
	// set the read and write pins to output
	sbi( AX88796_CONTROL_DDR, AX88796_CONTROL_READPIN );
	sbi( AX88796_CONTROL_DDR, AX88796_CONTROL_WRITEPIN );
#endif
	// set reset pin to output
	sbi( AX88796_RESET_DDR, AX88796_RESET_PIN );
}
*/

u08 enc28j60ReadOp(u08 op, u08 address)
{
	u08 data;

	// assert CS
	ENC28J60_CONTROL_PORT &= ~(1<<ENC28J60_CONTROL_CS);

	// issue read command
	SPDR = op | (address & ADDR_MASK);
	while(!(SPSR & (1<<SPIF)));
	// read data
	SPDR = 0x00;
	while(!(SPSR & (1<<SPIF)));
	// do dummy read if needed
	if(address & 0x80)
	{
		SPDR = 0x00;
		while(!(inb(SPSR) & (1<<SPIF)));
	}
	data = SPDR;

	// release CS
	ENC28J60_CONTROL_PORT |= (1<<ENC28J60_CONTROL_CS);

	return data;
}

void enc28j60WriteOp(u08 op, u08 address, u08 data)
{
	// assert CS
	ENC28J60_CONTROL_PORT &= ~(1<<ENC28J60_CONTROL_CS);

	// issue write command
	SPDR = op | (address & ADDR_MASK);
	while(!(SPSR & (1<<SPIF)));
	// write data
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));

	// release CS
	ENC28J60_CONTROL_PORT |= (1<<ENC28J60_CONTROL_CS);
}

void enc28j60ReadBuffer(u16 len, u08* data)
{
	// assert CS
	ENC28J60_CONTROL_PORT &= ~(1<<ENC28J60_CONTROL_CS);

	// issue read command
	SPDR = ENC28J60_READ_BUF_MEM;
	while(!(SPSR & (1<<SPIF)));
	while(len--)
	{
		// read data
		SPDR = 0x00;
		while(!(SPSR & (1<<SPIF)));
		*data++ = SPDR;
	}
	// release CS
	ENC28J60_CONTROL_PORT |= (1<<ENC28J60_CONTROL_CS);
}

void enc28j60WriteBuffer(u16 len, u08* data)
{
	unsigned char i=0;
	// assert CS
	ENC28J60_CONTROL_PORT &= ~(1<<ENC28J60_CONTROL_CS);

	// issue write command
	SPDR = ENC28J60_WRITE_BUF_MEM;
	while(!(SPSR & (1<<SPIF)));
//	printf("Sent Packet:\r\n");
	while(len--)
	{
		// write data
		SPDR = *data++;
//		if (i!=0)
//		{
//		  if ((i%16)==0)
//		    printf("\r\n");
//		}
//		if ( (*(data-1))<16 )
//		  printf("%d",0);
//		printf("%x ",*(data-1));
//		i++;
//                if (len==0)
//		    printf("\r\n");
		while(!(SPSR & (1<<SPIF)));
	}
	// release CS
	ENC28J60_CONTROL_PORT |= (1<<ENC28J60_CONTROL_CS);
}

void enc28j60SetBank(u08 address)
{
	// set the bank (if needed)
	if((address & BANK_MASK) != Enc28j60Bank)
	{
		// set the bank
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
		Enc28j60Bank = (address & BANK_MASK);
	}
}

u08 enc28j60Read(u08 address)
{
	// set the bank
	enc28j60SetBank(address);
	// do the read
	return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

void enc28j60Write(u08 address, u08 data)
{
	// set the bank
	enc28j60SetBank(address);
	// do the write
	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

u16 enc28j60PhyRead(u08 address)
{
	u16 data;

	// Set the right address and start the register read operation
	enc28j60Write(MIREGADR, address);
	enc28j60Write(MICMD, MICMD_MIIRD);

	// wait until the PHY read completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY);

	// quit reading
	enc28j60Write(MICMD, 0x00);

	// get data value
	data  = enc28j60Read(MIRDL);
	data |= enc28j60Read(MIRDH);
	// return the data
	return data;
}

void enc28j60PhyWrite(u08 address, u16 data)
{
	// set the PHY register address
	enc28j60Write(MIREGADR, address);

	// write the PHY data
	enc28j60Write(MIWRL, data);
	enc28j60Write(MIWRH, data>>8);

	// wait until the PHY write completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY);
}

void enc28j60Init(char Eth)
{
        unsigned int cntr;
	// initialize I/O
	sbi(ENC28J60_CONTROL_DDR, ENC28J60_CONTROL_CS);
	sbi(ENC28J60_CONTROL_PORT, ENC28J60_CONTROL_CS);

	// setup SPI I/O pins
	sbi(PORTB, 1);	// set SCK hi
	sbi(DDRB, 1);	// set SCK as output
	cbi(DDRB, 3);	// set MISO as input
	sbi(DDRB, 2);	// set MOSI as output
	sbi(DDRB, 0);	// SS must be output for Master mode to work
	// initialize SPI interface
	// master mode
	sbi(SPCR, MSTR);
	// select clock phase positive-going in middle of data
	cbi(SPCR, CPOL);
	// Data order MSB first
	cbi(SPCR,DORD);
	// switch to f/4 2X = f/2 bitrate
	cbi(SPCR, SPR0);
	cbi(SPCR, SPR1);
	sbi(SPSR, SPI2X);
	// enable SPI
	sbi(SPCR, SPE);

	// perform system reset
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	// check CLKRDY bit to see if reset is complete
	delay_ms(50);
	//for (cntr=0;cntr<1000;cntr++);
	//while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));   //********

	// do bank 0 stuff
	// initialize receive buffer
	// 16-bit transfers, must write low byte first
	// set receive buffer start address
	NextPacketPtr = RXSTART_INIT;
	enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXSTH, RXSTART_INIT>>8);
	// set receive pointer address
	enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);
	// set receive buffer end
	// ERXND defaults to 0x1FFF (end of ram)
	enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
	enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
	// set transmit buffer start
	// ETXST defaults to 0x0000 (beginnging of ram)
	enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
	enc28j60Write(ETXSTH, TXSTART_INIT>>8);

	// do bank 2 stuff
	// enable MAC receive
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// bring MAC out of reset
	enc28j60Write(MACON2, 0x00);
	// enable automatic padding and CRC operations
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN/*|MACON3_FULDPX*/);
//	enc28j60Write(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// set inter-frame gap (non-back-to-back)
	enc28j60Write(MAIPGL, 0x12);
	enc28j60Write(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
	enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);
	enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);

	// do bank 3 stuff
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60Write(MAADR5, ENC28J60_MAC0);
	enc28j60Write(MAADR4, ENC28J60_MAC1);
	enc28j60Write(MAADR3, ENC28J60_MAC2);
	enc28j60Write(MAADR2, ENC28J60_MAC3);
	enc28j60Write(MAADR1, ENC28J60_MAC4);
	enc28j60Write(MAADR0, ENC28J60_MAC5+Eth);

	// no loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);

	// switch to bank 0
	enc28j60SetBank(ECON1);
	// enable interrutps
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
	// enable packet reception
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

	enc28j60PhyWrite(PHLCON, 0x05ca);

	// setup duplex ----------------------

	// Disable receive logic and abort any packets currently being transmitted
	enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS|ECON1_RXEN);

	{
		u16 temp,temp1;
		// Set the PHY to the proper duplex mode
		temp = enc28j60PhyRead(PHCON1);
		temp |= PHCON1_PDPXMD;
		enc28j60PhyWrite(PHCON1, temp);

		// Set the MAC to the proper duplex mode
		temp = enc28j60Read(MACON3);
		temp1 = ~MACON3_FULDPX;
		temp &= temp1;
//		temp = temp & 0xFE;
		enc28j60Write(MACON3, temp);

	}

	// Set the back-to-back inter-packet gap time to IEEE specified
	// requirements.  The meaning of the MABBIPG value changes with the duplex
	// state, so it must be updated in this function.
	// In full duplex, 0x15 represents 9.6us; 0x12 is 9.6us in half duplex
	//enc28j60Write(MABBIPG, DuplexState ? 0x15 : 0x12);

	// Reenable receive logic
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

	// setup duplex ----------------------

}

void enc28j60PacketSend(unsigned int len, unsigned char* packet)
{
	// Set the write pointer to start of transmit buffer area
	enc28j60Write(EWRPTL, TXSTART_INIT);
	enc28j60Write(EWRPTH, TXSTART_INIT>>8);
	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write(ETXNDL, (TXSTART_INIT+len));
	enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);

	// write per-packet control byte
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

	// copy the packet into the transmit buffer
	enc28j60WriteBuffer(len, packet);

	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

unsigned int enc28j60PacketReceive(unsigned int maxlen, unsigned char* packet)
{
	u16 rxstat;
	u16 len;
	u08 kk;

	kk=enc28j60Read(EIR);
	kk=kk & 0x40; //EIR_PKTIF;

	// check if a packet has been received and buffered
//      if( !(enc28j60Read(EIR) & EIR_PKTIF) )
        if(kk==0)
  	  return 0;

	// Make absolutely certain that any previous packet was discarded
	//if( WasDiscarded == FALSE)
	//	MACDiscardRx();

	// Set the read pointer to the start of the received packet
	enc28j60Write(ERDPTL, (NextPacketPtr));
	enc28j60Write(ERDPTH, (NextPacketPtr)>>8);
	// read the next packet pointer
	NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	NextPacketPtr |= (u16)enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the packet length
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= (u16)enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the receive status
	rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	rxstat |=(u16)enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	// limit retrieve length
	// (we reduce the MAC-reported length by 4 to remove the CRC)
//	printf("Packet Length: %d\r\n",len);
	len = MIN(len, maxlen);

	// copy the packet from the receive buffer
	enc28j60ReadBuffer(len, packet);

	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out
	enc28j60Write(ERXRDPTL, (NextPacketPtr));
	enc28j60Write(ERXRDPTH, (NextPacketPtr)>>8);

	// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

	return len;

}
//void enc28j60ReceiveOverflowRecover(void)
//{
	// receive buffer overflow handling procedure

	// recovery completed
//}

/*void enc28j60RegDump(void)
{
//	unsigned char macaddr[6];
//	result = ax88796Read(TR);

//	printf("Media State: ");
//	if(!(result & AUTOD))
//		printf("Autonegotiation\r\n");
//	else if(result & RST_B)
//		printf("PHY in Reset   \r\n");
//	else if(!(result & RST_10B))
//		printf("10BASE-T       \r\n");
//	else if(!(result & RST_TXB))
//		printf("100BASE-T      \r\n");

	printf("RevID: 0x%x\r\n", enc28j60Read(EREVID));

	printfProgStrM("Cntrl: ECON1 ECON2 ESTAT  EIR  EIE\r\n");
	printfProgStrM("         ");
	printfu08(enc28j60Read(ECON1));
	printfProgStrM("    ");
	printfu08(enc28j60Read(ECON2));
	printfProgStrM("    ");
	printfu08(enc28j60Read(ESTAT));
	printfProgStrM("    ");
	printfu08(enc28j60Read(EIR));
	printfProgStrM("   ");
	printfu08(enc28j60Read(EIE));
	printfCRLF();

	printfProgStrM("MAC  : MACON1  MACON2  MACON3  MACON4  MAC-Address\r\n");
	printfProgStrM("        0x");
	printfu08(enc28j60Read(MACON1));
	printfProgStrM("    0x");
	printfu08(enc28j60Read(MACON2));
	printfProgStrM("    0x");
	printfu08(enc28j60Read(MACON3));
	printfProgStrM("    0x");
	printfu08(enc28j60Read(MACON4));
	printfProgStrM("   ");
	printfu08(enc28j60Read(MAADR5));
	printfu08(enc28j60Read(MAADR4));
	printfu08(enc28j60Read(MAADR3));
	printfu08(enc28j60Read(MAADR2));
	printfu08(enc28j60Read(MAADR1));
	printfu08(enc28j60Read(MAADR0));
	printfCRLF();

	printfProgStrM("Rx   : ERXST  ERXND  ERXWRPT ERXRDPT ERXFCON EPKTCNT MAMXFL\r\n");
	printfProgStrM("       0x");
	printfu08(enc28j60Read(ERXSTH));
	printfu08(enc28j60Read(ERXSTL));
	printfProgStrM(" 0x");
	printfu08(enc28j60Read(ERXNDH));
	printfu08(enc28j60Read(ERXNDL));
	printfProgStrM("  0x");
	printfu08(enc28j60Read(ERXWRPTH));
	printfu08(enc28j60Read(ERXWRPTL));
	printfProgStrM("  0x");
	printfu08(enc28j60Read(ERXRDPTH));
	printfu08(enc28j60Read(ERXRDPTL));
	printfProgStrM("   0x");
	printfu08(enc28j60Read(ERXFCON));
	printfProgStrM("    0x");
	printfu08(enc28j60Read(EPKTCNT));
	printfProgStrM("  0x");
	printfu08(enc28j60Read(MAMXFLH));
	printfu08(enc28j60Read(MAMXFLL));
	printfCRLF();

	printfProgStrM("Tx   : ETXST  ETXND  MACLCON1 MACLCON2 MAPHSUP\r\n");
	printfProgStrM("       0x");
	printfu08(enc28j60Read(ETXSTH));
	printfu08(enc28j60Read(ETXSTL));
	printfProgStrM(" 0x");
	printfu08(enc28j60Read(ETXNDH));
	printfu08(enc28j60Read(ETXNDL));
	printfProgStrM("   0x");
	printfu08(enc28j60Read(MACLCON1));
	printfProgStrM("     0x");
	printfu08(enc28j60Read(MACLCON2));
	printfProgStrM("     0x");
	printfu08(enc28j60Read(MAPHSUP));
	printfCRLF();

	delay_ms(25);
} */



/*! \file net.c \brief Network support library. */
//*****************************************************************************
//
// File Name	: 'net.c'
// Title		: Network support library
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
//*****************************************************************************

#include "avrlibtypes.h"
#include "global.h"
//#include "rprintf.h"

#include "net.h"

uint16_t htons(uint16_t val)
{
	return (val<<8) | (val>>8);
}

uint32_t htonl(uint32_t val)
{
	return (htons(val>>16) | (uint32_t)htons(val&0x0000FFFF)<<16);
}


//uint16_t netChecksum(uint16_t *data, uint16_t len)
uint16_t netChecksum(uint8_t *data, uint16_t len)
{
    register uint32_t sum = 0;
    uint16_t temp=0;

    for (;;) {
        if (len < 2)
            break;
		temp=*(data+1);
		temp=temp<<8;
		temp=temp+(*(data));
		sum +=temp;
		//sum += *((uint16_t *)data);
		data+=2;
        len -= 2;
    }
    if (len)
          sum +=(*(data));
    //    sum += *(uint8_t *) data;

    while ((len = (uint16_t) (sum >> 16)) != 0)
        sum = (uint16_t) sum + len;

    return (uint16_t) sum ^ 0xFFFF;
}
 /*
void netPrintEthAddr(struct netEthAddr* ethaddr)
{
	rprintfu08(ethaddr->addr[0]);
	rprintfChar(':');
	rprintfu08(ethaddr->addr[1]);
	rprintfChar(':');
	rprintfu08(ethaddr->addr[2]);
	rprintfChar(':');
	rprintfu08(ethaddr->addr[3]);
	rprintfChar(':');
	rprintfu08(ethaddr->addr[4]);
	rprintfChar(':');
	rprintfu08(ethaddr->addr[5]);
}

void netPrintIPAddr(uint32_t ipaddr)
{
	rprintf("%d.%d.%d.%d",
		((unsigned char*)&ipaddr)[3],
		((unsigned char*)&ipaddr)[2],
		((unsigned char*)&ipaddr)[1],
		((unsigned char*)&ipaddr)[0]);
}


void netPrintEthHeader(struct netEthHeader* eth_hdr)
{
	rprintfProgStrM("Eth Packet Type: 0x");
	rprintfu16(eth_hdr->type);

	rprintfProgStrM(" SRC:");
	netPrintEthAddr(&eth_hdr->src);
	rprintfProgStrM("->DST:");
	netPrintEthAddr(&eth_hdr->dest);
	rprintfCRLF();
}

void netPrintIpHeader(struct netIpHeader* ipheader)
{
	rprintfProgStrM("IP Header\r\n");
	rprintf("Ver     : %d\r\n", (ipheader->vhl)>>4);
	rprintf("Length  : %d\r\n", htons(ipheader->len));
	if(ipheader->proto == IP_PROTO_ICMP)
		rprintfProgStrM("Protocol: ICMP\r\n");
	else if(ipheader->proto == IP_PROTO_TCP)
		rprintfProgStrM("Protocol: TCP\r\n");
	else if(ipheader->proto == IP_PROTO_UDP)
		rprintfProgStrM("Protocol: UDP\r\n");
	else
		rprintf("Protocol: %d\r\n", ipheader->proto);

	rprintfProgStrM("SourceIP: "); netPrintIPAddr(htonl(ipheader->srcipaddr));	rprintfCRLF();
	rprintfProgStrM("Dest  IP: "); netPrintIPAddr(htonl(ipheader->destipaddr));	rprintfCRLF();
}

void netPrintTcpHeader(struct netTcpHeader* tcpheader)
{
	rprintfProgStrM("TCP Header\r\n");
	rprintf("Src Port: %d\r\n", htons(tcpheader->srcport));
	rprintf("Dst Port: %d\r\n", htons(tcpheader->destport));
	rprintfProgStrM("Seq Num : 0x"); rprintfu32(htonl(tcpheader->seqno));	rprintfCRLF();
	rprintfProgStrM("Ack Num : 0x"); rprintfu32(htonl(tcpheader->ackno));	rprintfCRLF();
	rprintfProgStrM("Flags   : ");
	if(tcpheader->flags & TCP_FLAGS_FIN)
		rprintfProgStrM("FIN ");
	if(tcpheader->flags & TCP_FLAGS_SYN)
		rprintfProgStrM("SYN ");
	if(tcpheader->flags & TCP_FLAGS_RST)
		rprintfProgStrM("RST ");
	if(tcpheader->flags & TCP_FLAGS_PSH)
		rprintfProgStrM("PSH ");
	if(tcpheader->flags & TCP_FLAGS_ACK)
		rprintfProgStrM("ACK ");
	if(tcpheader->flags & TCP_FLAGS_URG)
		rprintfProgStrM("URG ");
	rprintfCRLF();
}
*/

/*! \file ip.c \brief IP (Internet Protocol) Library. */
//*****************************************************************************
//
// File Name	: 'ip.c'
// Title		: IP (Internet Protocol) Library
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
//*****************************************************************************


#include "avrlibtypes.h"
#include "global.h"
//#include "rprintf.h"

#include "net.h"
#include "nic.h"
#include "arp.h"
#include "ip.h"

struct ipConfig IpMyConfig;	///< Local IP address/config structure


void ipSetConfig(uint32_t myIp, uint32_t netmask, uint32_t gatewayIp)
{
	struct netEthAddr ethaddr;

	// set local addressing
	IpMyConfig.ip = myIp;
	IpMyConfig.netmask = netmask;
	IpMyConfig.gateway = gatewayIp;

	// set ARP association
	nicGetMacAddress(ethaddr.addr);
	arpSetAddress(&ethaddr, myIp);
}

struct ipConfig* ipGetConfig(void)
{
	return &IpMyConfig;
}

// deprecated
/*
uint32_t ipGetMyAddress(void)
{
	return IpMyConfig.ip;
}
*/

void ipSend(uint32_t dstIp, uint8_t protocol, uint16_t len, uint8_t* data)
{
	// make pointer to ethernet/IP header
	struct netEthIpHeader* ethIpHeader;

	// move data pointer to make room for headers
	data -= ETH_HEADER_LEN+IP_HEADER_LEN;
	ethIpHeader = (struct netEthIpHeader*)data;

	//debugPrintHexTable(len+ETH_HEADER_LEN+IP_HEADER_LEN, data);

	// adjust length to add IP header
	len += IP_HEADER_LEN;

	// fill IP header
	ethIpHeader->ip.destipaddr = HTONL(dstIp);
	ethIpHeader->ip.srcipaddr = HTONL(IpMyConfig.ip);
	ethIpHeader->ip.proto = protocol;
	ethIpHeader->ip.len = htons(len);
	ethIpHeader->ip.vhl = 0x45;
	ethIpHeader->ip.tos = 0;
	ethIpHeader->ip.ipid = 0;
	ethIpHeader->ip.ipoffset = 0;
	ethIpHeader->ip.ttl = IP_TIME_TO_LIVE;
	ethIpHeader->ip.ipchksum = 0;

	// calculate and apply IP checksum
	// DO THIS ONLY AFTER ALL CHANGES HAVE BEEN MADE TO IP HEADER
	ethIpHeader->ip.ipchksum = netChecksum(&ethIpHeader->ip, IP_HEADER_LEN);

	// add ethernet routing
	// check if we need to send to gateway
	if( (dstIp & IpMyConfig.netmask) == (IpMyConfig.ip & IpMyConfig.netmask) )
	{
		arpIpOut(ethIpHeader,0);					// local send
	  //	printf("Sending IP packet on local net\r\n");
	}
	else
	{
		arpIpOut(ethIpHeader,IpMyConfig.gateway);	// gateway send
	//	printf("Sending IP packet to gateway\r\n");
	}

	// adjust length to add ethernet header
	len += ETH_HEADER_LEN;

	// debug
	//debugPrintHexTable(ETH_HEADER_LEN, &data[0]);
	//debugPrintHexTable(len-ETH_HEADER_LEN, &data[ETH_HEADER_LEN]);

	// send it
	nicSend(len, data);
}

void udpSend(uint32_t dstIp, uint16_t dstPort, uint16_t len, uint8_t* data)
{
	// make pointer to UDP header
	struct netUdpHeader* udpHeader;
	// move data pointer to make room for UDP header
	data -= UDP_HEADER_LEN;
	udpHeader = (struct netUdpHeader*)data;
	// adjust length to add UDP header
	len += UDP_HEADER_LEN;
	// fill UDP header
	udpHeader->destport = HTONS(dstPort);
	udpHeader->srcport  = HTONS(dstPort);
	udpHeader->udplen = htons(len);
	udpHeader->udpchksum = 0;

///	debugPrintHexTable(UDP_HEADER_LEN, (uint8_t*)udpPacket);

	ipSend(dstIp, IP_PROTO_UDP, len, (uint8_t*)udpHeader);
}
/*
void ipPrintConfig(struct ipConfig* config)
{
	rprintfProgStrM("IP Addr : "); netPrintIPAddr(config->ip);		rprintfCRLF();
	rprintfProgStrM("Netmask : "); netPrintIPAddr(config->netmask);	rprintfCRLF();
	rprintfProgStrM("Gateway : "); netPrintIPAddr(config->gateway);	rprintfCRLF();
}
*/
/*! \file icmp.c \brief ICMP Protocol Library. */
//*****************************************************************************
//
// File Name	: 'icmp.c'
// Title		: ICMP (Internet Control Message Protocol) Protocol Library
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
//*****************************************************************************

#include "global.h"
#include "net.h"
#include "nic.h"
#include "arp.h"
#include "icmp.h"

//#include "rprintf.h"
#include "debug.h"

//extern void nicSend(unsigned int len, unsigned char* packet);

// global variables


// functions
void icmpInit(void)
{
}

void icmpIpIn(icmpip_hdr* packet)
{
	// check ICMP type
	switch(packet->icmp.type)
	{
	case ICMP_TYPE_ECHOREQUEST:
		// echo request
		icmpEchoRequest(packet);
		break;
	default:
		break;
	}
}

void icmpEchoRequest(icmpip_hdr* packet)
{
	uint32_t tempIp;

	// change type to reply
	packet->icmp.type = ICMP_TYPE_ECHOREPLY;
	// recalculate checksum
	packet->icmp.icmpchksum = 0;
	packet->icmp.icmpchksum = netChecksum((u08*)&packet->icmp, htons(packet->ip.len)-IP_HEADER_LEN);
//	printf("CheckSum: %d\r\n",packet->icmp.icmpchksum);
	// return to sender
	tempIp = packet->ip.destipaddr;
	packet->ip.destipaddr = packet->ip.srcipaddr;
	packet->ip.srcipaddr = tempIp;
	// add ethernet routing
	arpIpOut((struct netEthIpHeader*)(((u08*)packet)-ETH_HEADER_LEN), 0);

	// debugging
//	#if NET_DEBUG >= 2
//		icmpPrintHeader(packet);
		//debugPrintHexTable(htons(packet->ip.len), (u08*)packet);
//	#endif

	// send it (packet->ip.len+ETH_HEADER_LEN
	nicSend(htons(packet->ip.len)+ETH_HEADER_LEN, (((u08*)packet)-ETH_HEADER_LEN));
}
/*
#ifdef ICMP_DEBUG_PRINT
void icmpPrintHeader(icmpip_hdr* packet)
{
	rprintfProgStrM("ICMP Packet:\r\n");
	// print source IP address
	rprintfProgStrM("SrcIpAddr: ");	netPrintIPAddr(htonl(packet->ip.srcipaddr));	rprintfCRLF();
	// print dest IP address
	rprintfProgStrM("DstIpAddr: ");	netPrintIPAddr(htonl(packet->ip.destipaddr));	rprintfCRLF();
	// print type
	rprintfProgStrM("Type   : ");
	switch(packet->icmp.type)
	{
	case ICMP_TYPE_ECHOREQUEST:		rprintfProgStrM("ECHO REQUEST"); break;
	case ICMP_TYPE_ECHOREPLY:		rprintfProgStrM("ECHO REPLY"); break;
	default:						rprintfProgStrM("UNKNOWN"); break;
	}
	rprintfCRLF();
	// print code
	rprintfProgStrM("Code   : 0x");	rprintfu08(packet->icmp.icode);			rprintfCRLF();
}
#endif
 */
/*! \file arp.c \brief ARP Protocol Library. */
//*****************************************************************************
//
// File Name	: 'arp.c'
// Title		: ARP Protocol Library
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
//*****************************************************************************

#include "global.h"
#include "net.h"
#include "nic.h"
#include "arp.h"

//#define ARP_DEBUG

/// Single ARP table entry/record
struct ArpEntry
{
	uint32_t ipaddr;			///< remote-note IP address
	uint8_t time;				///< time to live (in ARP table); this is decremented by arpTimer()
	struct netEthAddr ethaddr;	///< remote-node ethernet (hardware/mac) address
};

struct ArpEntry ArpMyAddr;		///< my local interface information (IP and MAC address)
struct ArpEntry ArpTable[ARP_TABLE_SIZE];	///< ARP table of matched IP<->MAC associations


void arpInit(void)
{
	u08 i;
	// initialize all ArpTable elements to unused
	for(i=0; i<ARP_TABLE_SIZE; i++)
	{
		ArpTable[i].ipaddr = 0;
		ArpTable[i].time = 0;
	}
}

void arpSetAddress(struct netEthAddr* myeth, uint32_t myip)
{
	// set local address record
	//ArpMyAddr.ethaddr = *myeth;\
	ArpMyAddr.ethaddr.addr[0] = myeth->addr[0];
	ArpMyAddr.ethaddr.addr[1] = myeth->addr[1];
	ArpMyAddr.ethaddr.addr[2] = myeth->addr[2];
	ArpMyAddr.ethaddr.addr[3] = myeth->addr[3];
	ArpMyAddr.ethaddr.addr[4] = myeth->addr[4];
	ArpMyAddr.ethaddr.addr[5] = myeth->addr[5];
	ArpMyAddr.ipaddr = myip;
}

void arpArpIn(unsigned int len, struct netEthArpHeader* packet)
{
	#ifdef ARP_DEBUG
	printf("Received ARP Request\r\n");
//	arpPrintHeader( &packet->arp );
	#endif

	// for now, we just reply to requests
	// need to add ARP cache
	if(   //	(packet->arp.dipaddr == HTONL(ArpMyAddr.ipaddr)) &&
	      	(packet->arp.opcode == htons(ARP_OPCODE_REQUEST)) )
	{

		// in ARP header
		// copy sender's address info to dest. fields
		packet->arp.dhwaddr = packet->arp.shwaddr;
		packet->arp.dipaddr = packet->arp.sipaddr;
		// fill in our information
	       	packet->arp.shwaddr.addr[0] = ArpMyAddr.ethaddr.addr[0];
	       	packet->arp.shwaddr.addr[1] = ArpMyAddr.ethaddr.addr[1];
	       	packet->arp.shwaddr.addr[2] = ArpMyAddr.ethaddr.addr[2];
	       	packet->arp.shwaddr.addr[3] = ArpMyAddr.ethaddr.addr[3];
	       	packet->arp.shwaddr.addr[4] = ArpMyAddr.ethaddr.addr[4];
	       	packet->arp.shwaddr.addr[5] = ArpMyAddr.ethaddr.addr[5];

		packet->arp.sipaddr = HTONL(ArpMyAddr.ipaddr);
		// change op to reply
		packet->arp.opcode = htons(ARP_OPCODE_REPLY);

		// in ethernet header
		packet->eth.dest = packet->eth.src;
		//packet->eth.src  = ArpMyAddr.ethaddr;
		packet->eth.src.addr[0] = ArpMyAddr.ethaddr.addr[0];
		packet->eth.src.addr[1] = ArpMyAddr.ethaddr.addr[1];
		packet->eth.src.addr[2] = ArpMyAddr.ethaddr.addr[2];
		packet->eth.src.addr[3] = ArpMyAddr.ethaddr.addr[3];
		packet->eth.src.addr[4] = ArpMyAddr.ethaddr.addr[4];
		packet->eth.src.addr[5] = ArpMyAddr.ethaddr.addr[5];

		// send reply!
		nicSend(len, (unsigned char*)packet);
		#ifdef ARP_DEBUG
		printf("Sending ARP Reply\r\n");
//		arpPrintHeader( &packet->arp );
		#endif

	}
}

void arpIpIn(struct netEthIpHeader* packet)
{
	int8_t index;

	// check if sender is already present in arp table
	index = arpMatchIp(HTONL(packet->ip.srcipaddr));
	if(index != -1)
	{
		// sender's IP address found, update ARP entry
		ArpTable[index].ethaddr = packet->eth.src;
		// and we're done
		return;
	}

	// sender was not present in table,
	// must add in empty/expired slot
	for(index=0; index<ARP_TABLE_SIZE; index++)
	{
		if(!ArpTable[index].time)
		{
			// write entry
			ArpTable[index].ethaddr = packet->eth.src;
			ArpTable[index].ipaddr = HTONL(packet->ip.srcipaddr);
			ArpTable[index].time = ARP_CACHE_TIME_TO_LIVE;
			// and we're done
			return;
		}
	}

	// no space in table, we give up
}

void arpIpOut(struct netEthIpHeader* packet, uint32_t phyDstIp)
{
	int index;
	// check if destination is already present in arp table
	// use the physical dstIp if it's provided, otherwise the dstIp in packet
	if(phyDstIp)
		index = arpMatchIp(phyDstIp);
	else
		index = arpMatchIp(HTONL(packet->ip.destipaddr));
	// fill in ethernet info
	if(index != -1)
	{
		// ARP entry present, fill eth address(es)
		packet->eth.src.addr[0] = ArpMyAddr.ethaddr.addr[0];
		packet->eth.src.addr[1] = ArpMyAddr.ethaddr.addr[1];
		packet->eth.src.addr[2] = ArpMyAddr.ethaddr.addr[2];
		packet->eth.src.addr[3] = ArpMyAddr.ethaddr.addr[3];
		packet->eth.src.addr[4] = ArpMyAddr.ethaddr.addr[4];
		packet->eth.src.addr[5] = ArpMyAddr.ethaddr.addr[5];

		packet->eth.dest = ArpTable[index].ethaddr;
		packet->eth.type = HTONS(ETHTYPE_IP);
	}
	else
	{
		// not in table, must send ARP request
		packet->eth.src.addr[0] = ArpMyAddr.ethaddr.addr[0];
		packet->eth.src.addr[1] = ArpMyAddr.ethaddr.addr[1];
		packet->eth.src.addr[2] = ArpMyAddr.ethaddr.addr[2];
		packet->eth.src.addr[3] = ArpMyAddr.ethaddr.addr[3];
		packet->eth.src.addr[4] = ArpMyAddr.ethaddr.addr[4];
		packet->eth.src.addr[5] = ArpMyAddr.ethaddr.addr[5];
		// MUST CHANGE, but for now, send this one broadcast
		packet->eth.dest.addr[0] = 0xFF;
		packet->eth.dest.addr[1] = 0xFF;
		packet->eth.dest.addr[2] = 0xFF;
		packet->eth.dest.addr[3] = 0xFF;
		packet->eth.dest.addr[4] = 0xFF;
		packet->eth.dest.addr[5] = 0xFF;
		packet->eth.type = HTONS(ETHTYPE_IP);
	}
}

void arpTimer(void)
{
	int index;
	// this function meant to be called on a regular time interval

	// decrement time-to-live for all entries
	for(index=0; index<ARP_TABLE_SIZE; index++)
	{
		if(ArpTable[index].time)
			ArpTable[index].time--;
	}
}

int arpMatchIp(uint32_t ipaddr)
{
	uint8_t i;

	// check if IP address is present in arp table
	for(i=0; i<ARP_TABLE_SIZE; i++)
	{
		if(ArpTable[i].ipaddr == ipaddr)
		{
			// IP address found
			return i;
		}
	}

	// no match
	return -1;
}
/*
#ifdef ARP_DEBUG_PRINT
void arpPrintHeader(struct netArpHeader* packet)
{
	printfProgStrM("ARP Packet:\r\n");
	//debugPrintHexTable(60, (unsigned char*)&packet);
	// print operation type
	printfProgStrM("Operation   : ");
	if(packet->opcode == htons(ARP_OPCODE_REQUEST))
		printfProgStrM("REQUEST");
	else if(packet->opcode == htons(ARP_OPCODE_REPLY))
		printfProgStrM("REPLY");
	else
		printfProgStrM("UNKNOWN");
	printfCRLF();
	// print source hardware address
	printfProgStrM("SrcHwAddr   : ");	netPrintEthAddr(&packet->shwaddr);	printfCRLF();
	// print source protocol address
	printfProgStrM("SrcProtoAddr: ");	netPrintIPAddr(HTONL(packet->sipaddr));	printfCRLF();
	// print target hardware address
	printfProgStrM("DstHwAddr   : ");	netPrintEthAddr(&packet->dhwaddr);	printfCRLF();
	// print target protocol address
	printfProgStrM("DstProtoAddr: ");	netPrintIPAddr(HTONL(packet->dipaddr));	printfCRLF();
}


void arpPrintTable(void)
{
	uint8_t i;

	// print ARP table
	printfProgStrM("Time    Eth Address    IP Address\r\n");
	printfProgStrM("---------------------------------------\r\n");
	for(i=0; i<ARP_TABLE_SIZE; i++)
	{
		printfu08(ArpTable[i].time);
		printfProgStrM("   ");
		netPrintEthAddr(&ArpTable[i].ethaddr);
		printfProgStrM("  ");
		netPrintIPAddr(ArpTable[i].ipaddr);
		printfCRLF();
	}
}
#endif
*/
//#define debug   7    // Unremark it if debug needed

#include <90can128.h>
#include <stdio.h>
#include <delay.h>

#include "can.h"

//*****************************************************
// TxTimeOut            :Indicates Transmit fails in demanding time
// RxBufferFull         :All Reacieve Buffers Full
// FrameBufferComplete  :Frame Buffere Recieve Completed
//*****************************************************

bit TxTimeOut=0,RxBufferFull=0,FrameBufferComplete=0;

unsigned int  can_rxfull_mobs=0,Received_Add=0,request_data=0;
unsigned int command_type , command_data;
unsigned char cansrshow;
struct can_message  last_rec_message;

//Changes The Current MOB Page
void set_mob_page(unsigned char page){
    CANPAGE=(CANPAGE&0x0f)|((page&0x0f)<<4);
}

//Changes The Current MOB Data Index in Current MOB Page
void set_mob_index(unsigned char ind){
   CANPAGE=(CANPAGE&0xf8)|(ind&0x07);
}

//Clears All Mail Box
void ClearAllMailbox (void){
  unsigned char num_channel, num_data;
  for (num_channel = 0; num_channel < 15; num_channel++){
    set_mob_page(num_channel);
    CANSTMOB = 0;
    CANCDMOB = 0x10;
    CANIDT4  = 0;
    CANIDT3  = 0;
    CANIDT2  = 0;
    CANIDT1  = 0;
    CANIDM4  = 0;
    CANIDM3  = 0;
    CANIDM2  = 0;
    CANIDM1  = 0;
    mob_index_auto_inc;
    for (num_data = 0; num_data < 8; num_data++){
      CANMSG = 0;
    }
  }

}

//Config Current Mob For Desired Function As Defined Follow
#define mob_disable 0
#define mob_transmit 1
#define mob_receive 2
#define mob_frame_rec 3
void config_mob(unsigned char config){
	CANCDMOB=(CANCDMOB&0x3f)|(config<<6);
}


// Handels All CAN Interrupts Except CAN timer OverFlow
void can_handel_interrupts(){
   unsigned char interrupter_mob_no;
   unsigned int CANSIT,CANIE;
   CANSIT=((unsigned int)CANSIT1<<8)+CANSIT2;
   CANIE=((unsigned int)CANIE1<<8)+CANIE2;
   //while(CANGIT&0x80){  //if still interrupt flag exist
      if(CANSIT){
         for(interrupter_mob_no=0;interrupter_mob_no<15;interrupter_mob_no++){
            if(((CANSIT>>interrupter_mob_no)&0x1)&&((CANIE>>interrupter_mob_no)&0x1)){
               #ifdef debug
               printf("MOB # %d has interrupt\n\r",interrupter_mob_no);
               #endif
               set_mob_page(interrupter_mob_no);
               if((CANSTMOB&0x01)&&((CANGIE&0x08)||(CANGIE&0x80))){
                  #ifdef debug
                  printf("Acknowledgment error\n\r");
                  #endif
                  if( (interrupter_mob_no >= TxStaMob)&&(interrupter_mob_no <= TxEndMob))
                         _ack_can_transmit_error =  interrupter_mob_no;
                  CANSTMOB&=~(0x01);
               }
               if((CANSTMOB&0x02)&&((CANGIE&0x08)||(CANGIE&0x80))){
                  #ifdef debug
                  printf("Form error\n\r");
                  #endif
                  if( (interrupter_mob_no >= TxStaMob)&&(interrupter_mob_no <= TxEndMob))
                         _ack_can_transmit_error =  interrupter_mob_no;
                  CANSTMOB&=~(0x02);
               }
               if((CANSTMOB&0x04)&&((CANGIE&0x08)||(CANGIE&0x80))){
                  #ifdef debug
                  printf("CRC error\n\r");
                  #endif
                  if( (interrupter_mob_no >= TxStaMob)&&(interrupter_mob_no <= TxEndMob))
                         _ack_can_transmit_error =  interrupter_mob_no;
                  CANSTMOB&=~(0x04);
               }
               if((CANSTMOB&0x08)&&((CANGIE&0x08)||(CANGIE&0x80))){
                  #ifdef debug
                  printf("Stuff error interrupt\n\r");
                  #endif
                  CANSTMOB&=~(0x08);
               }
               if((CANSTMOB&0x10)&&((CANGIE&0x08)||(CANGIE&0x80))){
                  #ifdef debug
                  printf("Bit error (in transmition)\n\r");
                  #endif
                  if( (interrupter_mob_no >= TxStaMob)&&(interrupter_mob_no <= TxEndMob))
                         _ack_can_transmit_error =  interrupter_mob_no;
                  CANSTMOB&=~(0x10);
               }
               if((CANSTMOB&0x20)&&((CANGIE&0x20)||(CANGIE&0x80))){
                  #ifdef debug
                  printf("Receive OK\n\r");
                  #endif
                  can_rxfull_mobs|=(1<<interrupter_mob_no);  //setting apprpriate rxful bit 1
                  config_mob(mob_disable);                   //Disable until reading its contents
                  if(can_rxfull_mobs==0x7fff){               //All buffer full
                     RxBufferFull=1;
                  }
                  CANSTMOB&=~(0x20);
                  cansrshow=1;

               }
               if((CANSTMOB&0x40)&&((CANGIE&0x10)||(CANGIE&0x80))){
                  #ifdef debug
                  printf("Transmit OK\n\r");
                  #endif
                  CANSTMOB&=~(0x40);
                  config_mob(mob_disable); // disabling (freeing) MOB
                  cansrshow=1;

               }
            }
         }
      }
      if(CANGIT&0x7f){
         if((CANGIT&0x01)&&((CANGIE&0x02)||(CANGIE&0x80))){
            #ifdef debug
            printf("Acknowledgment error interrupt\n\r");
            #endif
            CANGIT|=0x01;
         }
         if((CANGIT&0x02)&&((CANGIE&0x02)||(CANGIE&0x80))){
            #ifdef debug
            printf("Form error interrupt\n\r");
            #endif
            CANGIT|=0x02;
         }
         if((CANGIT&0x04)&&((CANGIE&0x02)||(CANGIE&0x80))){
            #ifdef debug
            printf("CRC error interrupt\n\r");
            #endif
            CANGIT|=0x04;
         }
         if((CANGIT&0x08)&&((CANGIE&0x02)||(CANGIE&0x80))){
            #ifdef debug
            printf("Stuff error interrupt\n\r");
            #endif
            CANGIT|=0x08;
         }
         if((CANGIT&0x10)&&((CANGIE&0x04)||(CANGIE&0x80))){
            #ifdef debug
            printf("Frame buffer receive interrupt\n\r");
            #endif
            FrameBufferComplete=1;
           // read_frame_buffer();
            CANGIT|=0x10;
         }
         /*
         if((CANGIT&0x20)&&(CANGIE&0x01)){
            #ifdef debug
            printf("CAN timer overrun interrupt\n\r");
            #endif
            CANGIT|=0x20;
         }
         */
         if((CANGIT&0x40)&&((CANGIE&0x40)||(CANGIE&0x80))){
            #ifdef debug
            printf("Bus off interrupt. CAN entered in bus off mode.\n\r");
            #endif
            CANGIT|=0x40;
         }
      }
   //}
}

//****************************************************************
// Parameters: ID long int , RTRTAG char 1 bit used , RBTAG char 2 bit used
// Returns: N/A
// Function: Stores ID in Corrent MOB IDTAG
//***************************************************************

void set_id_tag(unsigned long int id,unsigned char rtrtag,unsigned char rbtag){
	unsigned char i,*ptr;
	unsigned long int storable_id;
	storable_id=id;
	storable_id<<=1;
	storable_id+=rtrtag;
	storable_id<<=2;
	storable_id+=rbtag;
	ptr=& storable_id;
	for(i=0;i<4;i++){
		*((&CANIDT4)+i)=*(ptr+i);
	}
}

//****************************************************************
// Parameters: *id a pointer to the id_structure_sep type place to store readed id
// Returns: N/A
// Function: Stores readed ID from Corrent MOB IDTAG to the address specified by *id
//
//
//***************************************************************
void get_id_tag(struct id_structure_sep *id){
	unsigned char i,*ptr;
	unsigned long int stored_id;
	ptr=&stored_id;
	for(i=0;i<4;i++){
		*(ptr+i)=*((&CANIDT4)+i);
	}
	id->id0_28=stored_id>>3;
	id->rtrtag=(stored_id>>2)&0x00000001;
	id->rbtag=stored_id&0x00000003;
}

//****************************************************************
// Parameters: *target is a pointer to the can_message type place to store readed MOB
// Returns: N/A
// Function: Stores readed Corrent MOB  to the address specified by *target
//
//
//***************************************************************
void read_mob(struct can_message *target){ //reads current mob
	unsigned char i;
	struct id_structure_sep temp_id;
	target->status=CANSTMOB;
	target->datalen=CANCDMOB&0x0f;
	target->ide=(CANCDMOB&0x10)>>4;
	get_id_tag(&temp_id);
	target->rtr=temp_id.rtrtag;
	target->rb=temp_id.rbtag;
	target->id=temp_id.id0_28;
	target->time=CANSTM;

	#ifdef debug
   printf("datalen=%2x\n\r",CANCDMOB&0x0f);
   printf("ide    =%2x\n\r",(CANCDMOB&0x10)>>4);
   printf("rb     =%2x\n\r",temp_id.rbtag);
   printf("rtr    =%2x\n\r",temp_id.rtrtag);
   printf("id     =%4x%4x\n\r",(int)(temp_id.id0_28>>16),(int)temp_id.id0_28);
   printf("time   =%2x\n\r",CANSTM);
   printf("data[] =");
   #endif
	no_mob_index_auto_inc;
	for(i=0;i<8;i++){
		set_mob_index(i);
		target->data[i]=CANMSG;
		#ifdef debug
		printf("%2x",CANMSG);
		#endif

		CANMSG=0;
	}
}


//****************************************************************
// Parameters: source is the can_message type variable that desired message stored in it
// Returns: N/A
// Function: Fills the contents of MOB with desired message from SOURCE
//
//
//***************************************************************

void write_mob(struct can_message source){
	unsigned char i;
	//CANSTMOB=source.status;
	CANCDMOB=(CANCDMOB&0xe0)|((source.ide&0x01)<<4)|(source.datalen&0x0f);

	set_id_tag(source.id,source.rtr,source.rb);
	CANSTM=source.time;
	no_mob_index_auto_inc;
	for(i=0;i<8;i++){
		set_mob_index(i);
		CANMSG=source.data[i];
	}
}


//****************************************************************
// Parameters: mask is long int variabel
// Returns: N/A
// Function: Stores MASK in Corrent MOB MASK fields
//
//
//***************************************************************
void set_mask(unsigned long int mask){
	unsigned char i,*ptr;
	ptr=&mask;
	for(i=0;i<4;i++){
		*((&CANIDM4)+i)=*(ptr+i);
	}

}
//****************************************************************
// Parameters: N/A
// Returns: Readed Mask Fields Completely
// Function: Reades MASK fields from Corrent MOB and return it as a long int
//
//
//***************************************************************
unsigned long int get_mask(){
	unsigned char i,*ptr;
	unsigned long int mask;
	ptr=&mask;
	for(i=0;i<4;i++){
		*(ptr+i)=*((&CANIDM4)+i);
	}
	return mask;
}

//****************************************************************
// Parameters: N/A
// Returns: first free MOB number finded and "0xff" if no free MOB
// Function: Searches for any free MOB (Disable Mode)
//
//
//***************************************************************

unsigned char get_mob_free(void){
    unsigned char mob_number, page_saved;
    page_saved = CANPAGE;
    for (mob_number = TxStaMob; mob_number <= TxEndMob; mob_number++){
        set_mob_page(mob_number);
        if ((CANCDMOB & 0xC0) == 0x00){ //! Disable configuration
            CANPAGE = page_saved;
            return (mob_number);
        }
    }
    CANPAGE = page_saved;
    return (NO_MOB);
}

//****************************************************************
// Parameters: SOURCE is the source structure for sending message
// Returns: Used MOB number if successfull "0xff" if fail
// Function: tries to send Message Stored in SOURCE structure by current MOB
//
//***************************************************************
unsigned char can_send_message(struct can_message source){
   unsigned char mob_number;
   mob_number=get_mob_free();
   if(mob_number!=0xff)
   {
      set_mob_page(mob_number);
      write_mob(source);
      config_mob(mob_transmit);
      can_send_history[mob_number-TxStaMob] = source;
   }
   return(mob_number);
}




//****************************************************************
// Parameters: TARGET is the address of can_structure for saving readed message
// Returns: MOB Number if successfull, FF if no MOB has data
// Function: Reads Recieved Message from highest priority Mob (0 has highest priority)
// and ready it for recieve again.
//***************************************************************
unsigned char read_received_message(struct can_message *target){
   unsigned char i;
   if(!can_rxfull_mobs){
      return(NO_MOB);
   }
   for(i = RxStaMob; i <= RxEndMob; i++){
      if(can_rxfull_mobs&(1<<i)){
         set_mob_page(i);
         read_mob(target);
         can_rxfull_mobs&=~(1<<i);                //setting appropriate rxful bit 0
         config_mob(mob_receive);                  //making ready it for recieve again
         RxBufferFull=0;
         return(i);
      }
   }
}

//****************************************************************
// Parameters: mob_numbers is the number of needed MOB for setting as recieve
// Returns: 1 if the desired numbers of MOBs Was free and setted for recieve
//          0 if not enough free MOB finded
// Function: Searches for free MOBs and count them . set them for recieve if
//           needed quantity of free MOBs finded.
//
//***************************************************************
unsigned char set_mobs_for_receive(unsigned char mob_numbers){
   unsigned char mob_number, page_saved,free_mob_counter=0;
    page_saved = CANPAGE;
    for (mob_number = RxStaMob; mob_number <= RxEndMob; mob_number++){
        set_mob_page(mob_number);
        if ((CANCDMOB & 0xC0) == 0x00){ //! Disable configuration
            free_mob_counter++;
        }
    }
    if(free_mob_counter>=mob_numbers){
      for (mob_number = RxStaMob; mob_number < mob_numbers; mob_number++){
        set_mob_page(mob_number);
        if ((CANCDMOB & 0xC0) == 0x00){ //! Disable configuration
            config_mob(mob_receive);
        }
      }
      CANPAGE = page_saved;
      return(1);
    }
    CANPAGE = page_saved;
    return (0);
}



//****************************************************************
// Parameters: sep_id is a structure of separated protocol id
// Returns: assembeled id to a long int variable ,can be used for
//          other functions in driver layer.
// Function: assemles separated protocol id to a compact one
//           without rtr and rb tags.
//***************************************************************
unsigned long int assemble_id(struct protocol_id_structure_sep sep_id){
   unsigned long int id=0;
   id=sep_id.priority;
   id<<=11;
   id+=sep_id.source_address;
   id<<=11;
   id+=sep_id.dest_address;
   id<<=4;
   id+=sep_id.message_num;
   return id;
}
//****************************************************************
// Parameters: sep_id is a pointer to a structure of separated protocol id.
//             id is a long int contains ID used by driver layer functions.
// Returns: N/A
// Function: splits id to a protocol id structure usable by protocol layer funtctions.
//
//***************************************************************
void split_id(unsigned long int id,struct protocol_id_structure_sep *sep_id){
   sep_id->priority=id>>26;
   sep_id->source_address=(id>>15)&0x000007ff;
   sep_id->dest_address=(id>>4)&0x000007ff;
   sep_id->message_num=id&0x0000000f;
}

//****************************************************************
// Parameters: mob_numbers is the number of needed MOB for Frame Buffer recieve.
//             dest_addres is the address of the nodes wants to send multi messages
// Returns: 1 if the desired numbers of MOBs Was free and setted for frame buffer recieve
//          0 if not enough free MOB finded
// Function: Searches for free MOBs and count them . set them for frame buffer recieve if
//           needed quantity of free MOBs finded.
//
//***************************************************************
/*unsigned char set_mobs_for_frame_buffer_rec(unsigned char mob_numbers,unsigned int dest_address){
   struct protocol_id_structure_sep filter_id,sep_mask_id;
   unsigned long int storable_filter,storable_mask;
   unsigned char mob_number, page_saved,free_mob_counter=0,mob_config;

   page_saved = CANPAGE;
   for (mob_number = RxStaMob; mob_number <= RxEndMob; mob_number++){
       set_mob_page(mob_number);
       mob_config=(CANCDMOB & 0xC0)>>6;
       if ((mob_config == mob_disable)||(mob_config == mob_receive)){ //Disable or Recieve
           free_mob_counter++;
       }
   }
   #ifdef debug
   printf("free_mob_counter=%d \n\r",free_mob_counter);
   #endif
   if(free_mob_counter==mob_numbers){
      filter_id.dest_address=node_address;
      filter_id.source_address=dest_address;
      filter_id.priority=0;
      filter_id.message_num=0;
      storable_filter=assemble_id(filter_id);

      sep_mask_id.dest_address=0x7ff;
      sep_mask_id.source_address=0x7ff;
      sep_mask_id.priority=0;
      sep_mask_id.message_num=0;
      storable_mask=assemble_id(sep_mask_id);
      storable_mask<<=3;

      for (mob_number = RxStaMob; mob_number < RxEndMob; mob_number++){
         set_mob_page(mob_number);
         mob_config=(CANCDMOB & 0xC0)>>6;
         if ((mob_config == mob_disable)||(mob_config == mob_receive)){ //! Disable configuration
            config_mob(mob_frame_rec);
            set_mask(storable_mask);
            set_id_tag(storable_filter,0,0);

         }
      }
      CANPAGE = page_saved;
      return(1);
   }
   CANPAGE = page_saved;
   return (0);
}*/

//****************CAN MULTI MESSAGE REQUEST FRAME SPECIFICATION*****************
//     ID     D0  D1  D2  D3  D4  D5  D6           D7
// ________________________________________________________________
// |        |   |   |   |   |   |   |   |                          |
// |   ID   | S   E   N   D   R   E   Q      Nmberof Messages      |
// |________|______________________________________________________|

//****************CAN MULTI MESSAGE READY FRAME SPECIFICATION*****************
//     ID     D0  D1  D2  D3  D4  D5  D6           D7
// _________________________________________
// |        |   |   |   |   |   |   |   |   |
// |   ID   | R   E   C   R   E   A   D   Y |
// |________|_______________________________|

//****************CAN MULTI MESSAGE NOT READY FRAME SPECIFICATION*****************
//     ID     D0  D1  D2  D3  D4  D5  D6           D7
// _________________________________________
// |        |   |   |   |   |   |   |   |   |
// |   ID   | N   O   T   R   E   A   D   Y |
// |________|_______________________________|

/*
flash unsigned char SEND_REQUEST[]="SENDREQ",MULTY_MESSAGE_READY[]="RECREADY",MULTY_MESSAGE_NOT_READY[]="NOTREADY";
bit FrameBufferWaiting=0,RecieverReadyWating=0,RecieverNotReady=0;
bit FrameBufferWaitingTimeout=0,RecieverReadyWatingTimeout=0;
unsigned int FrameBufferRecieveTime=0,FrameBufferSendTime=0;
#define MaxFrameBufferRecieveTimeOut   500 //ms
#define MaxFrameBufferSendTimeOut      500 //ms
#define WAIT_RECIEVE_DELAY 10 //ms
*/
//****************************************************************
// Parameters: destination is the destination node address that the
//                            multi messages will be sent to.
//             number_of_messages is the quantity of messages will be sent
//                                for informing destination.
// Returns: N/A
// Function: Sends A MultiMessage Request to destination node.And retry it
//           for 4 times if no free MOB exist for Sending
//
//***************************************************************
/*
void send_multi_message_request(unsigned int destination,unsigned char Number_of_messages){
   struct can_message tx_message;
   struct protocol_id_structure_sep sep_id;
   unsigned char i;
   sep_id.dest_address=destination;
   sep_id.source_address=node_address;
   sep_id.priority=6;
   sep_id.message_num=0;
   printf("SEND MULTY REQUEST");
   tx_message.datalen=8;
   tx_message.ide=1;
   tx_message.rb=0;
   tx_message.rtr=0;
   tx_message.id=assemble_id(sep_id);
   tx_message.data[0]='S';
   tx_message.data[1]='E';
   tx_message.data[2]='N';
   tx_message.data[3]='D';
   tx_message.data[4]='R';
   tx_message.data[5]='E';
   tx_message.data[6]='Q';
   tx_message.data[7]=Number_of_messages;
   TxTimeOut=1;
   for(i=0;i<4;i++){
      if(can_send_message(tx_message)!=0xff){
         TxTimeOut=0;
         i=4;
      }
      delay_ms(Tx_Time/4);
   }
}*/

//****************************************************************
// Parameters: destination is the destination node address that the
//                            Ready or not ready message will be send to.
//             ready is the ready or not ready indicator
// Returns: N/A
// Function: Sends A MultiMessage READY or NOTREADY to destination node.And retry it
//           for 4 times if no free MOB exist for Sending
//
//***************************************************************
//#define Send_Ready 1
//#define Send_NotReady 0
/*
void send_message_ready_notready(unsigned int destination,unsigned char ready){
   struct can_message tx_message;
   struct protocol_id_structure_sep sep_id;
   unsigned char i;
   sep_id.dest_address=destination;
   sep_id.source_address=node_address;
   sep_id.priority=6;
   sep_id.message_num=0;

   tx_message.datalen=8;
   tx_message.ide=1;
   tx_message.rb=0;
   tx_message.rtr=0;
   tx_message.id=assemble_id(sep_id);
   if(ready){
      tx_message.data[0]='R';
      tx_message.data[1]='E';
      tx_message.data[2]='C';
   }
   else{
      tx_message.data[0]='N';
      tx_message.data[1]='O';
      tx_message.data[2]='T';
   }
   tx_message.data[3]='R';
   tx_message.data[4]='E';
   tx_message.data[5]='A';
   tx_message.data[6]='D';
   tx_message.data[7]='Y';
   TxTimeOut=1;
   for(i=0;i<4;i++){
      if(can_send_message(tx_message)!=0xff){
         TxTimeOut=0;
         i=4;
      }
      delay_ms(Tx_Time/4);
   }
}  */


//****************************************************************
// Parameters: N/A
// Returns: N/A
// Function: Assembeles Recieved Frame buffer data from MOBS to long_message_buff
//           global variable. It called in Frame Buffer Recieved Interrupt.
//
//***************************************************************
/*
void read_frame_buffer(){
   unsigned char mob_number,page_saved,min_message_number=0xff,reading_mob,data_ptr=0;
   unsigned char i,found,mob_config;
   struct protocol_id_structure_sep message_id;
   struct id_structure_sep stored_id;
   unsigned long int temp_id;
   struct can_message message;

   page_saved = CANPAGE;
   do{
      found=0;
      for(mob_number = RxStaMob; mob_number <= RxEndMob; mob_number++){
         set_mob_page(mob_number);
         mob_config=(CANCDMOB & 0xC0)>>6;
         if (mob_config == mob_frame_rec){
            found=1;
            get_id_tag(&stored_id);
            temp_id=stored_id.id0_28;
            split_id(temp_id,&message_id);
            if(message_id.message_num<min_message_number){
               min_message_number=message_id.message_num;
               reading_mob=mob_number;
            }
         }
      }
      if(found){
         set_mob_page(reading_mob);
         read_mob(&message);
         long_message_buff.datalen+=message.datalen;
         long_message_buff.id=message.id;
         for(i=0;i<message.datalen;i++){
            long_message_buff.data[data_ptr+i]=message.data[i];
         }
         data_ptr+=message.datalen;
         can_rxfull_mobs&=~(1<<reading_mob);  //setting appropriate rxful bit 0
         config_mob(mob_receive);             //making ready it for recieve again
         RxBufferFull=0;
      }
   }while(found);
   CANPAGE=page_saved;
}  */

//****************************************************************
// Parameters: N/A
// Returns: N/A
// Function: Segmentizes the Long messages from long_message_send_buff
//           global variable to multi messages and send them this function
//           only can called by "send_long_message" function
//
//***************************************************************
/*
void send_multy_messages(){
   unsigned char i,remaining_datalen,ptr=0,message_num=0;
   struct can_message tx_message;
   struct protocol_id_structure_sep sep_id;
   remaining_datalen=long_message_send_buff.datalen;
   tx_message.ide=long_message_send_buff.ide;
   tx_message.rb=0;
   tx_message.rtr=0;
   while(remaining_datalen){
      if(remaining_datalen>8){
         tx_message.datalen=8;
         remaining_datalen-=8;
      }
      else{
         tx_message.datalen=remaining_datalen;
         remaining_datalen=0;
      }
      split_id(long_message_send_buff.id,&sep_id);
      sep_id.message_num=message_num;
      tx_message.id=assemble_id(sep_id);
      for(i=0;i<tx_message.datalen;i++){
         tx_message.data[i]=long_message_send_buff.data[ptr];
         ptr++;
      }
      can_send_message(tx_message);
      message_num++;
      delay_ms(WAIT_RECIEVE_DELAY);
   }
}*/

//****************************************************************
// Parameters: destination is recipiant node that is ready for recieving Multi messages
// Returns: N/A
// Function: Sends A long message as multi messages to destination.
//           message data and datalen must be filled into "long_message_send_buff"
//           before running this function
//***************************************************************
/*
void send_long_message(unsigned int destination){
   unsigned char number_of_messages;
   struct protocol_id_structure_sep id;
   id.dest_address=destination;
   id.source_address=node_address;
   id.priority=4;
   id.message_num=0;
   long_message_send_buff.id=assemble_id(id);
   number_of_messages=long_message_send_buff.datalen/8;
   if(long_message_send_buff.datalen%8) number_of_messages++;
   send_multi_message_request(destination,number_of_messages);
   RecieverReadyWating=1;
   RecieverReadyWatingTimeout=0;
   FrameBufferSendTime=0;
}

#define SENDREQ   1
#define RECREADY  2
#define NOTREADY  3
#define NOMULTY   0
*/
void message_detector(){
   unsigned char i,message_type;
   struct can_message rec_message;
   struct protocol_id_structure_sep extracted_id;
   read_received_message(&rec_message);
   split_id(rec_message.id,&extracted_id);
   #ifdef debug
   printf("rec_message.datalen=%2x\n\r",rec_message.datalen);
   printf("rec_message.ide    =%2x\n\r",rec_message.ide);
   printf("rec_message.rb     =%2x\n\r",rec_message.rb);
   printf("rec_message.rtr    =%2x\n\r",rec_message.rtr);
   printf("rec_message.id     =%8x\n\r",rec_message.id);
   printf("Source_Address     =%2x\n\r",extracted_id.source_address);
   printf("Destination_Address=%2x\n\r",extracted_id.dest_address);
   printf("rec_message.time   =%2x\n\r",rec_message.time);
   printf("rec_message.data[] =%2x %2x %2x %2x %2x %2x %2x %2x\n\r",rec_message.data[0],rec_message.data[1]
   ,rec_message.data[2],rec_message.data[3],rec_message.data[4],rec_message.data[5],rec_message.data[6],rec_message.data[7]);
   printf("rec_message.data[] =%c %c %c %c %c %c %c %c\n\r",rec_message.data[0],rec_message.data[1]
   ,rec_message.data[2],rec_message.data[3],rec_message.data[4],rec_message.data[5],rec_message.data[6],rec_message.data[7]);
   #endif

   last_rec_message = rec_message;

   command_type = rec_message.data[0];
   command_data = rec_message.data[1];
   request_data = rec_message.data[2];
   Received_Add = extracted_id.source_address;

   /*
   for(i=0;i<7;i++){
      if(rec_message.data[i]!=SEND_REQUEST[i]){
         break;
      }
   }
   if(i==7){
      message_type=SENDREQ;
      #ifdef debug
      printf("message_type=SENDREQ\n\r");
      #endif
   }
   else{
      for(i=0;i<8;i++){
         if(rec_message.data[i]!=MULTY_MESSAGE_READY[i]){
            break;
         }
      }
      if(i==8){
         message_type=RECREADY;
         #ifdef debug
         printf("message_type=RECREADY\n\r");
         #endif
      }
      else{
         for(i=0;i<8;i++){
            if(rec_message.data[i]!=MULTY_MESSAGE_NOT_READY[i]){
               break;
            }
         }
         if(i==8){
            message_type=NOTREADY;
            #ifdef debug
            printf("message_type=NOTREADY\n\r");
            #endif
         }
      }
   }
   switch(message_type){
      case SENDREQ:
         if(set_mobs_for_frame_buffer_rec(rec_message.data[7],extracted_id.source_address)){
            send_message_ready_notready(extracted_id.source_address,Send_Ready);
            FrameBufferWaiting=1;
            FrameBufferWaitingTimeout=0;
            FrameBufferRecieveTime=0;
         }
         else{
            send_message_ready_notready(extracted_id.source_address,Send_NotReady);
            FrameBufferWaiting=0;
         }
         break;
      case RECREADY:
         if(RecieverReadyWating){
            send_multy_messages();
            RecieverReadyWating=0;
            RecieverReadyWatingTimeout=0;
            FrameBufferSendTime=0;
         }
         break;
      case NOTREADY:
         RecieverReadyWating=0;
         RecieverReadyWatingTimeout=0;
         RecieverNotReady=1;
         FrameBufferSendTime=0;
         break;
   } */
}



void can_send_quick(unsigned char data_address, unsigned char data_lenght, unsigned char data_delay,
                     unsigned char candata1 ,      unsigned char candata2,      unsigned char candata3     ,
                    unsigned char candata4 ,      unsigned char candata5,      unsigned char candata6     ,
                    unsigned char candata7 ,       unsigned char candata8  )

{
char i,mob_number;
unsigned long int temp_lid=0;

   //source_address=0x00 num=0 priority=0

   temp_lid=data_address;
   temp_lid<<=4;



   mob_number=get_mob_free();
   if(mob_number!=0xff)
   {
      set_mob_page(mob_number);
      CANCDMOB=(CANCDMOB&0xe0)|(0b00010000)|(data_lenght&0x0f);     //(source.ide&0x01)<<4

      set_id_tag(temp_lid,0,0);
      CANSTM=0;
      no_mob_index_auto_inc;

      switch(data_lenght)
       {
         case 8:
          set_mob_index(7);
          CANMSG=candata8;
         case 7:
          set_mob_index(6);
          CANMSG=candata7;
         case 6:
          set_mob_index(5);
          CANMSG=candata6;
         case 5:
          set_mob_index(4);
          CANMSG=candata5;
         case 4:
          set_mob_index(3);
          CANMSG=candata4;
         case 3:
          set_mob_index(2);
          CANMSG=candata3;
         case 2:
          set_mob_index(1);
          CANMSG=candata2;
         case 1:
          set_mob_index(0);
          CANMSG=candata1;
         break;
       }

      config_mob(mob_transmit);
   }

  if(data_delay!=0)delay_ms(data_delay);

}

void send_save_ok_ack(unsigned char ack_l)
{
   can_send_quick(Received_Add,5,0,'S',last_rec_message.data[1],last_rec_message.data[2],((ack_l==2)?0:last_rec_message.data[3]),1,0,0,0);
}


/*          ****************PATH FINDER******************
  This function sorts the request list attending to new requset and type
       of the services that chosen by various parts of program.
            *********************************************               */

char pathfinder(char request, char sertype)
{
//Defining local variables
unsigned char pf1 = 0 , pf2 = 0 , pf3 = 0 , pf4 = 0 , pf5=0;
unsigned char temp_al[64] = {0};

if(sertype==32)
{
	pf2=0;
	beep_request=0;
	while(AL[pf2]){AL[pf2]=0;
		       _al[pf2]=0;
		       pf2++;};
	return 0;

}

if(Navistrategy=='d')
{
/////Down-Collective Movement Strategy
if(sertype==4)return 0;
else
if(sertype==2)
 {
   pf5=Undergroundno;
   if(Groundexist!=0)pf5++;
   if(request>pf5)sertype=4;
 }
}




if(!floor)return 0;
if(opmode!='n')return 0;
if(request>Floorsno)return 0;



#ifndef  BLOCK_DATE_DISABLE
if((Expired_date==0xFA)&&(Expired_date==0xFA))return 0;
#endif

switch(sertype)
{
//Sertype=0x01 means Car Request
case 1 :
	if(!request)return 0;
	if(OVLC||FTOC||FLTC)return 0;
	if(firemf&&AL[0])return 0;
        if(VIPmf&&AL[0])return 0;
//	if(!floorvalidity(request))return 0;
	break;

//Sertype=0x04 means Down-directed Hall Request
case 4 :

//Sertype=2 means Up-directed Hall Request
// or none-directed Hall Request
case 2 :
	if(!request)return 0;
	if(FTOC||OVLC||FULC||FLTC||firemf||VIPmf)return 0;
	if(!floorvalidity(request))return 0;
	 break;

//Sertype=0x08 is for clearing the recent request
case 8 :
	for(pf1=1;pf1<64;pf1++)
	      {AL[pf1-1]=AL[pf1];
	       _al[pf1-1]=_al[pf1];}
	AL[63]=0;
	_al[63]=0;
	if(parkmf)parkmf=0;
	if(!AL[0])
	   {direction=0;
	    return 0;}
	if(AL[0]>floor)direction=1;
	if(AL[0]<floor)direction=2;
	return 0;
	break;

//Sertype=0x10 means delete hall call
case 16:
	for(pf1=0;AL[pf1]!=0;pf1++)
		{pf3=_al[pf1];
		 if((pf3&0x01)&&((pf3&0x02)||(pf3&0x04)))_al[pf1]=1;}
	pf2=0;
	pf3=0;
	while(AL[pf2])
	    {if(_al[pf2]==1){temp_al[pf3]=AL[pf2];
			     AL[pf2]=0;
			     pf3++;};
	     _al[pf2]=0;
	     pf2++;
	    };
	pf2=0;
	while(temp_al[pf2]){_al[pf2]=1;
			    AL[pf2]=temp_al[pf2];
			    pf2++;};
	for(pf1=pf2;pf1<64;pf1++)
			{AL[pf1]=0;
			 _al[pf1]=0;};

	if(!AL[0])direction=0;
	else if(AL[0]>floor)direction=1;
	else if(AL[0]<floor)direction=2;
	return 0;
	break;

//Sertype=0x20 means delete the all of AL list
case 32:

	break;

case 64 :
	for(pf1=1;pf1<64;pf1++)
	      {
	        if(AL[pf1-1]==0){pf2=0;break;}
	        if(AL[pf1-1]==request && (_al[pf1-1]&0x01))
	         {
	           pf2=_al[pf1-1];
	           if(pf2&0xFE)
	             {
	               _al[pf1-1] &= 0xFE;
	               return 2;     //no turn off leds
	             }

	           pf2=pf1;
	           break;
	         }
	      }

	if(pf2==0)return 0;

	for(pf1=pf2;pf1<64;pf1++)
	      {AL[pf1-1]=AL[pf1];
	       _al[pf1-1]=_al[pf1];}
	AL[63]=0;
	_al[63]=0;

	//if(pf2!=1)return 1;

	if(!AL[0]){direction=0;return 1;}
	if(AL[0]>floor)direction=1;
	if(AL[0]<floor)direction=2;
	return 1;
	break;

default :
	return 0;

};			//switch end

if(standbymf)
 {
   standbymf=0;
   if(request==floor)return 0;          //for exit from standby mode
 }

if(stayinstair&&(request==floor))
        {
         if(_hall_button_during_doorclose=='a')
                          _hall_button_during_doorclose=1;
         return 0;
        }

if(firemf)
{
	if(AL[0])return 0;		//just one request at time
	else
	     {AL[0]=request;
      	     _al[0]=sertype;
		if(request>floor)direction=1;
		if(request<floor)direction=2;
		return 1;
		};
}
else
if(VIPmf)
{
	if(AL[0])return 0;		//just one request at time
	else
	     {AL[0]=request;
      	     _al[0]=sertype;
		if(request>floor)direction=1;
		if(request<floor)direction=2;
		return 1;
		};
}
if(parkmf)
{
	if(!AL[0])          		//place request if list was empty
		{AL[0]=request;
		 _al[0]=sertype;
		 if(request>floor)direction=1;
		 if(request<floor)direction=2;
		 return 1;};

	if(request==Parkfloor)
		{parkmf=0;
		 NNOD=0;
		 return 1;};		//request is same park floor

	if((!I)&&stayinstair)
		{
		 parkmf=0;
		 AL[0]=request;
		 _al[0]=sertype;
		 if(request>floor)direction=1;
	  	 if(request<floor)direction=2;
		 return 1;};		//not moving yet

	if(AL[0]==floor)
		{AL[1]=request;
		 _al[1]=sertype;
		 NNOD=1;
		 parkmf=0;
		 return 1;};		//being stop in stair

	if(((request>floor)&&(direction==1))||((request<floor)&&(direction==2)))
		{AL[0]=request;
		 _al[0]=sertype;
		 parkmf=0;
		 return 1;};		//request in same path

	if((request<=floor)&&(direction==1))
		{AL[0]=floor+1;
		 _al[0]=1;
  		 AL[1]=request;
		 _al[1]=sertype;
		 NNOD=1;
		 parkmf=0;
		 return 1;};

	if((request>=floor)&&(direction==2))
		{AL[0]=floor-1;
		 _al[0]=1;
  		 AL[1]=request;
		 _al[1]=sertype;
		 NNOD=1;
		 parkmf=0;
		 return 1;};

};	//end of park mode service

if(!AL[0])
	{AL[0]=request;
	 _al[0]=sertype;
	 if(request<floor)direction=2;
	 if(request>floor)direction=1;
	 return 1;};

//if((Navistrategy=='f')||((Navistrategy=='d')&&(sertype==2)))
if(Navistrategy=='f')
{
	pf1=0;
	while(AL[pf1])
	{
		if(AL[pf1]==request)
			{if(_al[pf1]==3)return 0;
			 if(sertype!=_al[pf1])
				{_al[pf1]=3;
			 	 return 1;};
			 if(sertype==_al[pf1])return 0;
			}
		pf1++;
		if(pf1==64)break;
	};			//seeking for request in List
};

/*
if(Navistrategy=='d')
{
/////Down-Collective Movement Strategy
/////
if(direction==2)
	{if(request<floor)
		{pf1=0;
		 while(AL[pf1])
			{if(request>=AL[pf1])break;
			 pf1++;
			 if(AL[pf1-1]<AL[pf1])break;
			};
		}
	 else if(request>=floor)
		{pf1=0;
		 while(AL[pf1])
			{pf1++;
			 if(AL[pf1]>AL[pf1-1])break;
			};
		if(sertype==1)
			{
			  while(AL[pf1])
			    {pf2=_al[pf1];
			     if(!(pf2&0x01))break;
			     if(request<=AL[pf1])break;
			     pf1++;
			    };
			}

		else if(sertype==2)
			{
			  do{pf2=_al[pf1];
			     if(!(pf2&0x01))break;
			     pf1++;
			    }while(AL[pf1]);
			  while(AL[pf1])
				{if(request>=AL[pf1])break;
				 pf1++;};
			};
		};
	};

if(direction==1)
	{if((request>floor)&&(sertype==1))
		{pf1=0;
		 if(AL[0]==floor)pf1=1;
      	   while(AL[pf1])
			{pf2=_al[pf1];
			 if(!(pf2&0x01))break; 		//if end of car call, break
			 if(request<=AL[pf1])break;
			 pf1++;
			 if(AL[pf1]<AL[pf1-1])break;         //*******
			};
		}
	  else
		{pf1=0;
		 if(AL[0]==floor)pf1=1;
		 pf2=_al[pf1];

		while((pf2&0x01)&&AL[pf1]>floor)
		   {pf1++;
		    pf2=_al[pf1];
		   };

		 while(AL[pf1])
			{if(request>=AL[pf1])break;
			 pf1++;
			};
		};
	};

pf2=pf1;
while(AL[pf2])
	{if(AL[pf2]==request)
		{sertype|=_al[pf2];
		 while(AL[pf2]||_al[pf2])
			{AL[pf2]=AL[pf2+1];
			 _al[pf2]=_al[pf2+1];
			 pf2++;
			 if(pf2==63)break;
		 	};
		 AL[63]=0;
		_al[63]=0;
		break;
		};
	pf2++;
	};

};
     */

if(Navistrategy=='f')
{
/////Full-Collective Movement Strategy
/////
if(direction==1)          //upward
	{if(request>floor)
			  {pf1=0;
			   while(request>AL[pf1])
				{pf1++;
				 if(!AL[pf1])break;
				 if(AL[pf1]<AL[pf1-1])break;};
			   };
	if(request<=floor)
			  {pf1=0;
			   while(AL[pf1])
				  {pf1++;
			 	   if(!AL[pf1])break;
				   if(AL[pf1]<AL[pf1-1])break;};
			   if(AL[pf1])
				  {while(AL[pf1])
					  {if(AL[pf1]<request)break;
					   pf1++;};
				  };
			  };
	};

if(direction==2)
	{if(request<floor)
			  {pf1=0;
			   while(request<AL[pf1])
				{pf1++;
				 if(!AL[pf1])break;
				 if(AL[pf1]>AL[pf1-1])break;};
			  };
	 if(request>=floor)
			   {pf1=0;
			    while(AL[pf1])
				    {pf1++;if(!AL[pf1])break;
				     if(AL[pf1]>AL[pf1-1])break;};
			    if(AL[pf1])
				 {while(AL[pf1])
					  {if(AL[pf1]>request)break;
					   pf1++;};
				 };
			   };
	};
pf2=0;
while(pf2<=pf1)
	{if(AL[pf2]==request)
		{_al[pf2]|=sertype;
		 return 1;
		};
	 pf2++;
	};
while(AL[pf2])
	{if(AL[pf2]==request)
		{sertype|=_al[pf2];
		 while(AL[pf2])
			{AL[pf2]=AL[pf2+1];
			 _al[pf2]=_al[pf2+1];
			 pf2++;
			 if(pf2==63)break;
		 	};
		 AL[63]=0;
		_al[63]=0;
		};
	pf2++;
	};
};

//if(Navistrategy=='s')
if((Navistrategy=='s')||(Navistrategy=='d'))
{
/////Collective-Selective Movement Strategy
/////
if(direction==1)
{
	if(request>floor)
		{
		 pf1=0;
		 if(AL[0]==floor)pf1=1; ////
		 if((sertype==1)||(sertype==2))
			{
			 while(AL[pf1])
			            {
			             pf2=_al[pf1];
				       if(request<AL[pf1])break;
					 if(!((pf2&0x01)||(pf2&0x02)))break;
					 if((AL[pf1+1]<AL[pf1])&&AL[pf1+1])
					                              {pf1++;
									       break;};
									   		//*********
					 pf1++;
				      };
			};
		};

	if(((request<=floor)&&(sertype==1))||(sertype==4))
		{
		 pf1=0;
		 if(AL[0]==floor)pf1=1;  ////
		 while(AL[pf1])
			{
			 pf2=_al[pf1];
			 if(((pf2&0x01)||(pf2&0x02))&&(AL[pf1]>floor))
				                                    {pf1++;
				                                     continue;};
			 break;
			};
		 while(AL[pf1])
			{
			 pf2=_al[pf1];
			 if(request>AL[pf1])break;
			 if(!((pf2&0x01)||(pf2&0x04)))break;
			 pf1++;
			};
		};

	if((request<=floor)&&(sertype==2))
		{
		 pf1=0;
		 if(AL[0]==floor)pf1=1;                    ////
		 while(AL[pf1])
			{
			 if(_al[pf1]==2)
			 if(AL[pf1]<=floor)break;
			 pf1++;
			};
		while(AL[pf1])
			{
			 if(request<=AL[pf1])break;
			 pf1++;
			};
		};
};
if(direction==2)
{
	if(request<floor)
		{
		 pf1=0;
		 if(AL[0]==floor)pf1=1;                    ////
		 if((sertype==1)||(sertype==4))
			{
			 while(AL[pf1])
			            {
			             pf2=_al[pf1];
				       if(request>AL[pf1])break;
					 if(!((pf2&0x01)||(pf2&0x04)))break;
					 if(AL[pf1+1]>AL[pf1])
					                  {pf1++;
					 		       break;};
					 pf1++;
				      };
			};
		};

	if(((request>=floor)&&(sertype==1))||(sertype==2))
		{
		 pf1=0;
		 if(AL[0]==floor)pf1=1;              ////
		 while(AL[pf1])
			{
			 pf2=_al[pf1];
			 if(((pf2&0x01)||(pf2&0x04))&&(AL[pf1]<floor))
				                        {pf1++;
				                         continue;};
			 break;
			};
		 while(AL[pf1])
			{
			 pf2=_al[pf1];
			 if(request<AL[pf1])break;
			 if(!((pf2&0x01)||(pf2&0x02)))break;
			 pf1++;
			};
		};

	if((request>=floor)&&(sertype==4))
		{
		 pf1=0;
		 if(AL[0]==floor)pf1=1;              ////
		 while(AL[pf1])
			{
			 if(_al[pf1]==4)
			 if(AL[pf1]>=floor)break;
			 pf1++;
			};
		while(AL[pf1])
			{
			 if(request>=AL[pf1])break;
			 pf1++;
			};
		};
};
pf2=0;
while(pf2<=pf1)
	 {
	  if(AL[pf2]==request)
		{
		 _al[pf2]|=sertype;
		 return 1;
		};
	  pf2++;
	 };
while(AL[pf2])
	{
	 if(AL[pf2]==request)
		{
		  sertype|=_al[pf2];
		  while(AL[pf2])
			{
			 AL[pf2]=AL[pf2+1];
			 _al[pf2]=_al[pf2+1];
			 pf2++;
			 if(pf2==63)break;
		  	};
		  AL[63]=0;
		 _al[63]=0;
		};
	 pf2++;
	};

};
//end of collective selective

//place request in the list at pf1-th row of it
pf2=62;
for(pf2=62;pf2>=pf1;pf2--)
	{
	 if(pf2==255)break;
	 pf3=AL[pf2];
	 pf4=_al[pf2];
	 AL[pf2+1]=pf3;
	 _al[pf2+1]=pf4;
	};
AL[pf1]=request;
_al[pf1]=sertype;
return 1;
}


/**********************  Evaluation  **************************
      THIS PROGRAM SCORES A REQUEST FOR PLACEMENT IN
          ANSWER LIST WITH ATTENTION TO ITS TYPE
             IT IS FOLLOWING N.C. ALGORITHM
                  FOR SCORING REQUESTS
*******************************************************/

char Evaluation(unsigned char stair,unsigned char reqtype)
{
unsigned char diststraight,ev3,disttotal,stopsno,ev4=0;
unsigned char extremestair,extremestairL,extremestairH;
unsigned char requeststairs=0;

if(FTOC||OVLC||FULC||firemf||VIPmf)return 0;          //Return 0 in special case
if(!floor)return 0;
if(!stair)return 0;
if((reqtype!=2)&&(reqtype!=4))return 0;
if(opmode!='n')return 0;
if(stair>Floorsno)return 0;
if(!floorvalidity(stair))return 0;

#ifndef  BLOCK_DATE_DISABLE
if((Expired_date==0xFA)&&(Expired_date==0xFA))return 0;
#endif

if(floor>=stair)diststraight=floor-stair;      //Calculate the straight distance
if(floor<stair)diststraight=stair-floor;

for(ev3=0;AL[ev3];ev3++)requeststairs++;

if(requeststairs>Floorsno)return 0;
requeststairs *= 2;

for(ev3=0;AL[ev3];ev3++)
     if(AL[ev3]==stair)
       {
         ev4=2*(Floorsno);
         if(ev4>=(ev3*3))ev4-=(ev3*3);           //ev4 is evaluated when request is exist in list
         break;
       }

if(parkmf)
         {
          if((stair>floor)&&(direction==1)||(stair<floor)&&(direction==2)||(!direction))
                 {
                  return 2*(Floorsno)-diststraight;
                 }
          else return 2*(Floorsno)-diststraight-1-1;      //first for stop, second for redirection
         }

if(!direction)return 2*(Floorsno)-diststraight;
                                                //Calculating the Extreme-Stairs
extremestairH=floor;
extremestairL=floor;
for(ev3=0;AL[ev3];ev3++)
            {
             if(AL[ev3]>extremestairH)extremestairH=AL[ev3];
             if(AL[ev3]<extremestairL)extremestairL=AL[ev3];
            }
if(direction==1){extremestair=extremestairH;}
else if (direction==2)extremestair=extremestairL;

//Calculating un-straigt distance
if(extremestair>=stair)disttotal=extremestair-stair;
else if(extremestair<stair)disttotal=stair-extremestair;   //disttotal is d'


// Collective-Selective Strategy
if(Navistrategy=='s')
        {
         if(!(reqtype==2||reqtype==4))return 0;         //Only accept the hall call

         //Car goes up toward request, in the same direction
         if((direction==1)&&(reqtype==2)&&(stair>floor))
                {
                  ev3 = 2*(Floorsno)-diststraight+1;
                  if(ev3>requeststairs)ev3 -=  requeststairs; else ev3=1;
                  if(ev4>ev3) return ev4;
                  return ev3;
                };
         //Car goes down toward request, in the same direction
         if((direction==2)&&(reqtype==4)&&(stair<floor))
                {
                  ev3 = 2*(Floorsno)-diststraight+1;
                  if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                  if(ev4>ev3) return ev4;
                  return ev3;
                };

         //Car goes up toward request, but request is in other side
         if((direction==1)&&(reqtype==4)&&(stair>floor))
                {
                 if(stair<extremestair)
                    {
                         ev3 = 2*(Floorsno-disttotal)-diststraight+1;
                         if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                         if(ev4>ev3) return ev4;
                         return ev3;
                    }
                 ev3 = 2*(Floorsno)-diststraight+1;
                 if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
                }

         //Car goes down toward request, but request is in other side
         if((direction==2)&&(reqtype==2)&&(stair<floor))
                {
                 if(stair>extremestair)
                    {
                         ev3 = 2*(Floorsno-disttotal)-diststraight+1;
                         if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                         if(ev4>ev3) return ev4;
                         return ev3;
                    }
                 ev3 = 2*(Floorsno)-diststraight+1;
                 if(ev3>requeststairs)ev3 -=  requeststairs;
                 return ev3;
                }

         //Car goes up backward request, and request is in other side
         if((direction==1)&&(reqtype==4)&&(stair<=floor))
                {
                 ev3 = 2*(Floorsno-disttotal)+diststraight;
                 if(ev3>requeststairs)ev3 -=  requeststairs; else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
                }
         //Car goes down backward request, and request is in other side
         if((direction==2)&&(reqtype==2)&&(stair>=floor))
                {
                 ev3 = 2*(Floorsno-disttotal)+diststraight;
                 if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
                }

         //Car goes up backward request, in the same direction
         if((direction==1)&&(reqtype==2)&&(stair<=floor))
                {
                 if(extremestairL>floor)
                    {
                         ev3 = 2*(Floorsno-disttotal)-diststraight-1;
                         if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                         if(ev4>ev3) return ev4;
                         return ev3;
                    }
                 ev3 = 2*(Floorsno-disttotal-extremestairL+floor)+diststraight-2;
                 if(ev3>requeststairs)ev3 -=  requeststairs; else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
                }
         //Car goes down backward request,in the same direction
         if((direction==2)&&(reqtype==4)&&(stair>=floor))
                {
                 if(extremestairH<floor)
                    {
                         ev3 = 2*(Floorsno-disttotal)+diststraight-1;
                         if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                         if(ev4>ev3) return ev4;
                         return ev3;
                    }
                 ev3 = 2*(Floorsno-disttotal-floor+extremestairH)+diststraight-2;
                 if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
                }
         }

// Collective-Down Strategy
else if(Navistrategy=='d')
         {



          //Car goes up toward request,existing a higher request in list.
          if((direction==1)&&(stair>floor)&&(extremestair>stair))
            {
                 ev3 = 2*(Floorsno-disttotal)-diststraight-1;
                 if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
            }

          //Car goes up toward request, not existing a higher request in list.
          if((direction==1)&&(stair>floor)&&(extremestair<=stair))
            {
                 ev3 = 2*Floorsno-diststraight;
                 if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
            }

          //Car goes up backward request.
          if((direction==1)&&(stair<=floor))
            {
                 ev3 = 2*(Floorsno-disttotal)+diststraight-1;
                 if(ev3>requeststairs)ev3 -=  requeststairs; else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
            }


          //Car goes down toward request.
          if((direction==2)&&(stair<floor))
            {
                 ev3 = 2*Floorsno-diststraight;
                 if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
            }

          //Car goes down backward request,
          if((direction==2)&&(stair>=floor))
                {
                 //existing a higher request in list.
                 if(extremestairH>stair)
                    {
                         ev3 = 2*(Floorsno-extremestairH+extremestairL)+diststraight-2;
                         if(ev3>requeststairs)ev3 -=  requeststairs;    else ev3=1;
                         if(ev4>ev3) return ev4;
                         return ev3;
                    }
                 //not existing a higher request in list
                 if(extremestairH<=stair)
                    {
                         ev3 = 2*(Floorsno-disttotal)+diststraight-1;
                         if(ev3>requeststairs)ev3 -=  requeststairs;  else ev3=1;
                         if(ev4>ev3) return ev4;
                         return ev3;
                    }

                }
         }

//Full-Collective Strartegy
if(Navistrategy=='f')
        {
        //Car goes toward request.
        if(((direction==1)&&(stair>floor))||((direction==2)&&(stair<floor)))
            {
                 ev3 = 2*Floorsno-diststraight;
                 if(ev3>requeststairs)ev3 -=  requeststairs;   else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
            }

        //Car goes backward request.
        if(((direction==1)&&(stair<=floor))||((direction==2)&&(stair>=floor)))
            {
                 ev3 = 2*(Floorsno-disttotal)+diststraight-1;
                 if(ev3>requeststairs)ev3 -=  requeststairs;   else ev3=1;
                 if(ev4>ev3) return ev4;
                 return ev3;
            }

        }

else return 0;
}

unsigned char floorvalidity(unsigned char floorvalid)
{
  unsigned char fv1=0,fv2=0,fv3,fv4;
  fv1 = floorvalid % 8  ;
  if(!fv1)fv1=8;
  fv1--;

  fv4= (floorvalid - 1) / 8;

  fv2 = Maskfloor[fv4];
  if(!((fv2>>fv1) & 0x01))return 0;
     else return 1;
}

void setfloormask(unsigned char floormasked,unsigned char floorcond,unsigned char maskno)
{
  unsigned char sfm1=0,sfm2=0,sfm3,sfm4;

  if(!floormasked)return;

  sfm4 = (floormasked-1)/8;

  sfm1= floormasked % 8 ;

  if(!sfm1)sfm1=8;

  if(!floorcond)
         {
           sfm3=0xFE;
           for(sfm2=1;sfm2<sfm1;sfm2++)
             {
                sfm3 <<= 1;
                sfm3 |= 0x01;
             }

           switch(maskno)
            {
             case 0:
             Maskfloor[sfm4] &= sfm3;
             EE_Maskfloor[sfm4] = Maskfloor[sfm4];
             break;
             case 1:
             FirstDoor[sfm4] &= sfm3;
             EE_FirstDoor[sfm4] = FirstDoor[sfm4];
             break;
             case 2:
             SecondDoor[sfm4] &= sfm3;
             EE_SecondDoor[sfm4] = SecondDoor[sfm4];
             break;
             case 3:
             HalfFloor[sfm4] &= sfm3;
             EE_HalfFloor[sfm4] = HalfFloor[sfm4];
             break;
            }
           delay_ms(1);
         }
      else
         {
           sfm3=0x01;
           for(sfm2=1;sfm2<sfm1;sfm2++)
             {
                sfm3 <<= 1;
                sfm3 &= 0xFE;
             }

           switch(maskno)
            {
             case 0:
               Maskfloor[sfm4] |= sfm3;
               EE_Maskfloor[sfm4] = Maskfloor[sfm4];
             break;
             case 1:
               FirstDoor[sfm4] |= sfm3;
               EE_FirstDoor[sfm4] = FirstDoor[sfm4];
             break;
             case 2:
               SecondDoor[sfm4] |= sfm3;
               EE_SecondDoor[sfm4] = SecondDoor[sfm4];
             break;
             case 3:
               HalfFloor[sfm4] |= sfm3;
               EE_HalfFloor[sfm4] = HalfFloor[sfm4];
             break;
            }

           delay_ms(1);
         }


}



unsigned char halffloorstatus(unsigned char floorvalid)
{
  unsigned char hfs1=0,hfs2=0,hfs3,hfs4;
  hfs1 = floorvalid % 8  ;
  if(!hfs1)hfs1=8;
  hfs1--;

  hfs4= (floorvalid - 1) / 8;

  hfs2 = HalfFloor[hfs4];
  if(!((hfs2>>hfs1) & 0x01))return 0;
     else return 1;
}


char Waiting_List_Manager(unsigned char  Request_number,unsigned char  Request_type
                                                  , unsigned char Evalue,unsigned char  action
                                                  , unsigned char IPreceived,unsigned char  UniqueIDreceived)

{
      char i;
      unsigned int wlm1;
      unsigned char pc1,pc2;

            //1 return the index of placed request in WL and 0xFF if was not succesful
            //2 place the request in list and update nearestrequest
            //3 compare the received request and update received Eval and IP in WL[]
            //4 find the expired-time request related to nearestrequest

if(!Eth_number)return;
if(Grouptype=='s')return;

if(action=='w')   //place in balank row
 {
   i=0;
    while(WL[i].Req)
          {i++;
           if(i==4)return 0xFF;}   // unsucessful

    _reqexistinq=1;


     WL[i].Req = Request_number;
     WL[i].ReqType = Request_type;
     WL[i].ReqEval = Evalue;
     WL[i].ReqExpireTime = 1;
     WL[i].ReqUniqueID = UniqueID++;
     WL[i].ReqIP = IP;
     WL[i].ReqResponse=0;

     return i;                          //return row
 }

if(action=='e')                 //check for
 {
     for(i=0;i<4;i++)
       {

          if((WL[i].Req==Request_number)&&(WL[i].ReqType==Request_type))
           {
            if( ((IPreceived<IP)&&(Evalue>0)) || (WL[i].ReqEval==0) )
             {                        //clear request if
                WL[i].Req = 0;
                WL[i].ReqType = 0;
                WL[i].ReqEval = 0;
                WL[i].ReqExpireTime =0;
                WL[i].ReqUniqueID = 0;
                WL[i].ReqIP = 0;
                WL[i].ReqResponse=0;
                for(i=0;i<4;i++)
                      if(WL[i].ReqExpireTime)return 0xFF;
                _reqexistinq=0;
                return 0xFF;
             }
           }
       }

     return 0;

 }

if(action=='f')  //find
 {
     for(i=0;i<4;i++)
       {
          if(WL[i].ReqExpireTime>=10)
            {
               if(WL[i].ReqIP==IP)
                 {
                    if(pathfinder(WL[i].Req,WL[i].ReqType))
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
                                   current_message.data[1]=WL[i].Req;
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
                                   sep_id.source_address = node_address;
                                   sep_id.dest_address = 0xC0;
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
                                   current_message.data[1]=WL[i].Req;
                                   can_send_message(current_message);
                                   delay_ms(5);
                                   serial_current_message.length = 2;
                                   serial_current_message.source = node_address;
                                   serial_current_message.address = 0xD3;
                                   serial_current_message.data[0] = 'O';
                                   serial_current_message.data[1] = WL[i].Req;
                                   RS485_send_message(serial_current_message);
                                   delay_ms(5);
                                }
                      else
                      if(Numtype=='n')
                            {
                              if((Navistrategy== 'f' ) || (Navistrategy== 'd' ))
                                {
                                   pc1 = (WL[i].Req - 1) / 16;
                                   pc2 = WL[i].Req % 16;
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
                                   pc1 = (WL[i].Req - 1 ) / 16;
                                   pc2 = WL[i].Req % 16;
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

                                   pc1 = (WL[i].Req - 1) / 8;
                                   pc2 = WL[i].Req % 8;
                                   if(!pc2)pc2=8;
                                   pc2 *= 2;
                                   if(WL[i].ReqType == 4)pc2--;          //2

                                   sep_id.dest_address = 0xF0 + pc1;
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
                              if((Navistrategy== 'f' ) || (Navistrategy== 'd' ))
                                {
                                   pc1 = (WL[i].Req - 1) / 16;
                                   pc2 = WL[i].Req % 16;
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

                                   serial_current_message.length = 2;
                                   serial_current_message.source = node_address;
                                   serial_current_message.address = 0xF0 + pc1;
                                   serial_current_message.data[0] = 'O';
                                   serial_current_message.data[1] = pc2;
                                   RS485_send_message(serial_current_message);
                                   delay_ms(5);

                                 }
                              else if(Navistrategy=='s')
                                 {
                                   pc1 = (WL[i].Req - 1 ) / 16;
                                   pc2 = WL[i].Req % 16;
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

                                   pc1 = (WL[i].Req - 1) / 8;
                                   pc2 = WL[i].Req % 8;
                                   if(!pc2)pc2=8;
                                   pc2 *= 2;
                                   if(WL[i].ReqType == 4)pc2--;    //2

                                   serial_current_message.length = 2;
                                   serial_current_message.source = node_address;
                                   serial_current_message.address = 0xF0 + pc1;
                                   serial_current_message.data[0] = 'O';
                                   serial_current_message.data[1] = pc2;
                                   RS485_send_message(serial_current_message);
                                   delay_ms(5);

                                 }

                               }
                        }   // if(pathfinder



                  }  //if)WL[i]
              else Send_Request_via_Eth(WL[i],'C');


               WL[i].Req = 0;
               WL[i].ReqType = 0;
               WL[i].ReqEval = 0;
               WL[i].ReqExpireTime = 0;
               WL[i].ReqUniqueID = 0;
               WL[i].ReqIP = 0;
               WL[i].ReqResponse=0;
            }         //if(WL[i].ReqExpireTime

       }       //for

     for(i=0;i<4;i++)
      if(WL[i].ReqExpireTime)return;
     _reqexistinq=0;
    return;
  }



if(action=='c')     //compare  and replace  a better  item in waiting list
  {
     for(i=0;i<4;i++)
       {

          if((WL[i].ReqUniqueID==UniqueIDreceived)&&(WL[i].Req==Request_number)
                &&(WL[i].ReqType==Request_type))
            {
                if(Evalue>WL[i].ReqEval)
                  {
                    WL[i].ReqEval = Evalue;
                    WL[i].ReqIP = IPreceived;
                  }
                WL[i].ReqResponse++;
                break;
            }
       }

      if(i==4)return 0xFF;    //request is not in table may be expired or else
      if(WL[i].ReqResponse==(Grouptype - '0' - 1))return i;
         else return 0xFE;             //not enough response


  }

if(action=='d')     //delete the item matched in list
  {
     for(i=0;i<4;i++)
       {

          if((WL[i].ReqUniqueID==UniqueIDreceived)&&(WL[i].Req==Request_number)
                &&(WL[i].ReqType==Request_type))
            {
                WL[i].Req = 0;
                WL[i].ReqType = 0;
                WL[i].ReqEval = 0;
                WL[i].ReqExpireTime =0;
                WL[i].ReqUniqueID = 0;
                WL[i].ReqIP = 0;
                WL[i].ReqResponse=0;
                break;
            }
       }


     for(i=0;i<4;i++)
      if(WL[i].ReqExpireTime)return;
     _reqexistinq=0;
     return;

  }

if(action==0)     //delete the item matched in list
  {
     for(i=0;i<4;i++)
       {
                WL[i].Req = 0;
                WL[i].ReqType = 0;
                WL[i].ReqEval = 0;
                WL[i].ReqExpireTime =0;
                WL[i].ReqUniqueID = 0;
                WL[i].ReqIP = 0;
                WL[i].ReqResponse=0;
       }
     _reqexistinq=0;
  }

}
/*   Error-Message & 7segments display function  */
#include "defines.h"
#include <90can128.h>

void errormsg(unsigned char errorno)
{
unsigned char  date , month , year;
unsigned char  em1 , em2 , em3=0, em4=0;

switch(errorno)
{
case 0:
      //Clear LCD message
      Write_7Seg(1,0);
      Write_7Seg(2,0);
      Write_7Seg(3,0);
      em3=1;             //To disable saving Error in EEPROM
      break;

case 31:
	//write on LCD
	//Conts. are closed before closing. (contce)

	//Show "E31" on 7segs.
      Write_7Seg(3,'1');
      Write_7Seg(2,'3');
      Write_7Seg(1,'E');
      em4=1;
      break;

case 32:
	//write on LCD
	//Conts. are already closed after turn off bobines. (contcea)

	//Show "E32" on 7segs.
      Write_7Seg(3,'2');
      Write_7Seg(2,'3');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 33:
	//write on LCD
	//Conts. are not be closed until 1 sec after triggering. (contnca)

	//Show "E33" on 7segs.
      Write_7Seg(3,'3');
      Write_7Seg(2,'3');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 34:
	//write on LCD
	//Break feedback is not shown. (feedb4bse)

	//Show "E34" on 7segs.
      Write_7Seg(3,'4');
      Write_7Seg(2,'3');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 35:
	//write on LCD
	//Maximum Travelling Time is elapsed

	//Show "E35" on 7segs.
      Write_7Seg(3,'5');
      Write_7Seg(2,'3');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 36:
	//write on LCD
	//Watch dog reset

	//Show "E35" on 7segs.
      Write_7Seg(3,'6');
      Write_7Seg(2,'3');
      Write_7Seg(1,'E');
	break;

case 37:
	//write on LCD
	//Watch dog reset

	//Show "E35" on 7segs.
      Write_7Seg(3,'7');
      Write_7Seg(2,'3');
      Write_7Seg(1,'E');
	break;

case 41:
	//write on LCD
	//One of the hall-doors is open. (hdoorct)

	//Show "E41" on 7segs.
      Write_7Seg(3,'1');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
	break;

case 42:
	//write on LCD
	//Cabin door can not be closed after a few try. (doorcnc)

	//show "E42" on 7segs.
      Write_7Seg(3,'2');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
	break;

case 43:
	//write on LCD
	//Cabin door can not be opened after a few try. (doorcno)

	//show "E43" on 7segs.
      Write_7Seg(3,'3');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
	break;

case 44:
	//write on LCD
	//Cabin door contact is already closed (is probably bridged). (doorlcc)

	//show "E44" on 7segs.
      Write_7Seg(3,'4');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
	break;

case 45:
	//write on LCD
	//Car is not in valid area to open the door. (carniva)

	//show "E45" on 7segs.
      Write_7Seg(3,'5');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
	break;

case 46:
	//write on LCD
	//URA does not work. (uraminw)

	//show "E46" on 7segs.
      Write_7Seg(3,'6');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
	break;

case 47:
	//write on LCD
	//Door lock contact is not shown. (doorlcns)

	//show "E47" on 7segs.
      Write_7Seg(3,'7');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
	break;

case 48:
	//write on LCD
	//Door lock contact is probably bridged. (doorlcs)

	//show "E48" on 7segs.
      Write_7Seg(3,'8');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
	break;

case 49:
	//write on LCD
	//Door closing error in Revision

	//show "E49" on 7segs.
      Write_7Seg(3,'9');
      Write_7Seg(2,'4');
      Write_7Seg(1,'E');
         em3=1;
	break;

case 51:
	//write on LCD
	//Power 24v is turned off

	//show "E51" on 7segs.
      Write_7Seg(3,'1');
      Write_7Seg(2,'5');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 52:
	//write on LCD
	//Power 110v is turned off

	//show "E52" on 7segs.
      Write_7Seg(3,'2');
      Write_7Seg(2,'5');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 63:
	//write on LCD
	//Serie stop is cut on top of hoistway

	//show "E63" on 7segs.
      Write_7Seg(3,'3');
      Write_7Seg(2,'6');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 64:
	//write on LCD
	//Serie stop is cut at down of hoistway

	//show "E64" on 7segs.
      Write_7Seg(3,'4');
      Write_7Seg(2,'6');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 65:
	//write on LCD
	//Serie stop is cut on the car

	//show "E65" on 7segs.
      Write_7Seg(3,'5');
      Write_7Seg(2,'6');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 66:
	//write on LCD
	//Serie stop is cut on hall doors during movement

	//show "E66" on 7segs.
      Write_7Seg(3,'6');
      Write_7Seg(2,'6');
      Write_7Seg(1,'E');
      em4=1;
	break;


case 69:
	//write on LCD
	//Serie stop is cut on car door during movement

	//show "E69" on 7segs.
      Write_7Seg(3,'9');
      Write_7Seg(2,'6');
      Write_7Seg(1,'E');
      em4=1;
	break;


case 68:
	//write on LCD
	//Serie stop is cut on hall-door lock contact during movement

	//show "E68" on 7segs.
      Write_7Seg(3,'8');
      Write_7Seg(2,'6');
      Write_7Seg(1,'E');
      em4=1;
	break;


case 71:
	//write on LCD
	//Turning on CA1 & CAn simultaneously.

	//show "E71" on 7segs.
      Write_7Seg(3,'1');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
      em4=1;
	break;


case 72:
	//write on LCD
	//Turning on 1CF & CF3 simultaneously. (cf1cf3)

	//show "E72" on 7segs.
      Write_7Seg(3,'2');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
      em4=1;
	break;


case 73:
	//write on LCD
	//Fault in phase-sequence. (motordi)

	//show "E73" on 7segs.
      Write_7Seg(3,'3');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
	break;


case 74:
	//write on LCD
	//Time for to sense of 1CF is elapsed. (time1cf)

	//show "E74" on 7segs.
      Write_7Seg(3,'4');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
	break;

case 75:
	//write on LCD
	//Time for to sense of CF3 is elapsed. (timecf3e)

	//show "E75" on 7segs.
	Write_7Seg(3,'5');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
	break;

case 76:
	//write on LCD
	//Car arrived to final stair but CAn isn't shown. (canins)

	//show "E76" on 7segs.
      Write_7Seg(3,'6');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
	break;

case 77:
	//write on LCD
	//Car arrived to lowest stair but CAn isn't shown. (ca1ins)

	//show "E77" on 7segs.
      Write_7Seg(3,'7');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
	break;

case 78:
	//write on LCD
	//Maximum Travel-time is over. (calmto)

	//show "E78" on 7segs.
      Write_7Seg(3,'8');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
	break;

case 79:
	//write on LCD
	//CAn doesn't allow to start upward. (mcbycan)

	//show "E79" on 7segs.
      Write_7Seg(3,'9');
      Write_7Seg(2,'7');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 80:
	//write on LCD
	//CA1 doesn't allow to start down-ward. (mcbyca1)

	//show "E80" on 7segs.
      Write_7Seg(3,'0');
      Write_7Seg(2,'8');
      Write_7Seg(1,'E');
      em4=1;
	break;

case 81:
	//write on LCD
	//Overload is active

	//show "OVL" on 7segs.
      Write_7Seg(3,'L');
      Write_7Seg(2,'V');
      Write_7Seg(1,'O');
      break;

case 82:
	//write on LCD
	//FTO is active.

	//show "FTO" on 7segs.
      Write_7Seg(3,'O');
      Write_7Seg(2,'T');
      Write_7Seg(1,'F');
      break;

case 83:
	//write on LCD
	//On-Car Revision Mode.

	//show "P13" on 7segs.
      Write_7Seg(3,'C');
      Write_7Seg(2, 0 );
      Write_7Seg(1,'R');
      em3=1;             //To disable saving Error in EEPROM
	break;

case 84:
	//write on LCD
	//On-Panel Revision Mode

	//show "P14" on 7segs.
      Write_7Seg(3,'P');
      Write_7Seg(2, 0 );
      Write_7Seg(1,'R');
      em3=1;             //To disable saving Error in EEPROM
	break;

case 85:
	//write on LCD
	//Fire Mode

	//show "P15" on 7segs.
      Write_7Seg(3,'R');
      Write_7Seg(2,'1');
      Write_7Seg(1,'F');
	break;

case 86:
	//write on LCD
	//Parking Mode

	//show "P16" on 7segs.
      Write_7Seg(3,'R');
      Write_7Seg(2,'A');
      Write_7Seg(1,'P');
      em3=1;             //To disable saving Error in EEPROM
	break;

case 87:
	//Write on LCD
	//Board must be reset.

	//Show "P17" on 7segs.
      Write_7Seg(3,'5');
      Write_7Seg(2,'E');
      Write_7Seg(1,'R');
      em3=1;             //To disable saving Error in EEPROM
	break;

case 88:
	//Write on LCD
	//Error in Strat
      em3=1;             //To disable saving Error in EEPROM
	break;

case 89:
	//write on LCD
	//LIFTER Mode

	//show "LIF" on 7segs.
      Write_7Seg(3,'F');
      Write_7Seg(2,'1');
      Write_7Seg(1,'L');
	break;

case 90:
	//write on LCD
	//VIP Mode

	//show "LIF" on 7segs.
      Write_7Seg(3,'P');
      Write_7Seg(2,'1');
      Write_7Seg(1,'V');
	break;

case 101:

        Write_7Seg(3,'L');
        Write_7Seg(2,'A');
        Write_7Seg(1,'C');
        em3=1;
        break;

case 102:
        Write_7Seg(1,'F');
        Write_7Seg(2, showfloor(floor,2));
        Write_7Seg(3, showfloor(floor,1));
        em3=1;
        break;

case 103:
        Write_7Seg(1, 'B');
        Write_7Seg(2, 'L');
        Write_7Seg(3, 'C');
        em3=1;
        break;

case 104:
	//write on LCD
      Write_7Seg(3,'D');
      Write_7Seg(2,'N'  );
      Write_7Seg(1,'E');
      em3=1;
	break;

case 105:
        Write_7Seg(1,'L');
        Write_7Seg(2, showfloor(floor,2));
        Write_7Seg(3, showfloor(floor,1));
        em3=1;
        break;

case 91:
	//Write on LCD
      Write_7Seg(3,'1');
      Write_7Seg(2,'9');
      Write_7Seg(1,'E');
      em4=1;
      break;

case 92:
	//Write on LCD
      Write_7Seg(3,'2');
      Write_7Seg(2,'9');
      Write_7Seg(1,'E');
      em4=1;
      break;

case 93:
	//Write on LCD
      Write_7Seg(3,'3');
      Write_7Seg(2,'9');
      Write_7Seg(1,'E');
      em4=1;
      break;

case 94:
	//Write on LCD
      Write_7Seg(3,'4');
      Write_7Seg(2,'9');
      Write_7Seg(1,'E');
      em4=1;
      break;

case 95:
	//Write on LCD
      Write_7Seg(3,'5');
      Write_7Seg(2,'9');
      Write_7Seg(1,'E');
      em4=1;
      break;

case 96:
	//Write on LCD
      Write_7Seg(3,'6');
      Write_7Seg(2,'9');
      Write_7Seg(1,'E');
      em4=1;
      break;

case 97:
	//Write on LCD
      Write_7Seg(3,'7');
      Write_7Seg(2,'9');
      Write_7Seg(1,'E');
      em4=1;
      break;

case 99:
	//Write on LCD
      Write_7Seg(3,'9');
      Write_7Seg(2,'9');
      Write_7Seg(1,'E');
      em4=1;
      break;

default:
      em3=1;             //To disable saving Error in EEPROM
      break;
}

old_display_msg   = last_display_msg;
last_display_msg = errorno;

if(SMS_Level && (last_SMS_msg!=last_display_msg))
 {
        serial_current_message.address=0xD4;
        serial_current_message.length=3;
        if((!em3)&&(SMS_Level==1)&&(opmode=='n'))serial_current_message.data[0]='E';   //Show and sending SMS
        else
        if((!em3)&&(SMS_Level==2)&&(em4)&&(opmode=='n'))serial_current_message.data[0]='E';
        else
           serial_current_message.data[0]='P';          //Only Show not sending SMS
        serial_current_message.data[1]= last_display_msg;
        serial_current_message.data[2]= em3 | firstchk;
        if(last_display_msg==102)
                {
                   serial_current_message.length=4;
                   serial_current_message.data[2]=showfloor(floor,1);
                   serial_current_message.data[3]=showfloor(floor,2);
                }
        RS485_send_message(serial_current_message);
        last_SMS_msg = last_display_msg;
 }

if(!em3)
 {
 //Now saving the error in eeprom



      /*
      1- write time and date in eeprom memory
      2- write code of error, attending *ptr
      3- increase write *ptr
      4- a. Write a message to lcd.
         b. Show a message on 7segs.
         c. Send a message via ethernet .
      */



      em1 = EEWriteIndex   ;

      if(firstchk)
           {
            if((ErrorHistory[em1].ErrorNum) == errorno)
              return;
           }

      if(!firstchk)beep_en=1;
      if(errorno==99)beep_en=0;

      rtc_get_time ( &hour , &min   , &sec  )       ;
      rtc_get_date ( &date , &month , &year )      ;

      EENumberoferrors++;

      if(++em1>=200){em1 = 1 ;     //Clear if it had an invlaid value
                           if(EEWriteIndexOV==0xFF)EEWriteIndexOV=0x00;
                         }


      if((ErrorHistory[em1-1].ErrorNum) != errorno)
            {
               EEWriteIndex = em1   ;
               ErrorHistory[em1].ErrorNum = errorno   ;          //New Error
               ErrorHistory[em1].ErrorQty = 1         ;
            }
      else
            {
               em1--;
               EEWriteIndex = em1               ;                //Repeated Error
               em2 = ErrorHistory[em1].ErrorQty ;
               em2++;
               ErrorHistory[em1].ErrorQty = em2 ;
            };

      ErrorHistory[em1].ErrorSec   = sec   ;
      ErrorHistory[em1].ErrorMin   = min   ;
      ErrorHistory[em1].ErrorHour  = hour  ;
      ErrorHistory[em1].ErrorDate  = date  ;
      ErrorHistory[em1].ErrorMonth = month ;
      ErrorHistory[em1].ErrorYear  = year  ;
      }

}

/****************Write on 7Segments********************/
void Write_7Seg(char No,char Chr)
{
    unsigned char temp1=0,temp2=0;
    temp1 = ( 1 << (4+No) ) & 0xF0;


    DDRA=0xFF;       //MAKE PORTA AS OUTPUT
    PORTA = 0xFF;

    if(!No)
      {
       temp1 |= (PINC&0x0F);
       PORTA = ~Chr;
       PORTC = temp1;
       temp1 &= 0x0F;              //Set LE's  0
       PORTC = temp1;
       return;
      }

    PORTC = temp1;    //Set LEi 1  OEi  0

    switch ( Chr )
    {                                //0b1abgfedc
       case 0:
         PORTA = 0b11111111;
         break;
       case '-':
         PORTA = 0b11101111;
         break;
       case '0':
         PORTA = 0b10010000;
         break;
       case '1':
         PORTA = 0b11011110;
         break;
       case '2':
         PORTA = 0b10001001;
         break;
       case '3':
         PORTA = 0b10001100;
         break;
       case '4':
         PORTA = 0b11000110;
         break;
       case '5':
         PORTA = 0b10100100;
         break;
       case '6':
         PORTA = 0b10100000;
         break;
       case '7':
         PORTA = 0b10011110;
         break;
       case '8':
         PORTA = 0b10000000;
         break;
       case '9':
         PORTA = 0b10000100;
         break;
       case 'A':
         PORTA = 0b10000010;
         break;
       case 'G':
         PORTA = 0b10110000;
         break;
       case 'P':
         PORTA = 0b10000011;
         break;
       case 'F':
         PORTA = 0b10100011;
         break;
       case 'E':
         PORTA = 0b10100001;
         break;
       case 'C':
         PORTA = 0b10110001;
         break;
       case 'L':
         PORTA = 0b11110001;
         break;
       case 'N':
         PORTA = 0b11101010;
         break;
       case 'O':
         PORTA = 0b11101000;
         break;
       case 'R':
         PORTA = 0b11101011;
         break;
       case 'B':
         PORTA = 0b11100000;
         break;
       case 'D':
         PORTA = 0b11001000;
         break;
       case 'H':
         PORTA = 0b11000010;
         break;
       case 'J':
         PORTA = 0b11011100;
         break;
       case 'T':
         PORTA = 0b11100001;
         break;
       case 'U':
         PORTA = 0b11010000;
         break;
       case 'V':
         PORTA = 0b11111000;
         break;
       case 'Y':
         PORTA = 0b11000100;
         break;
    }
    temp1 &= 0x0F;              //Set LE's  0
    PORTC = temp1;
}

#include "defines.h"
/*****************Read Inputs************************/
void read_inputs(void)
{
    //First, saving previous status of some flags
    if(FTO){FTOC=1;}
      else  FTOC=0;


    PORTA  = 0xFF                    ;
    DDRA   = 0x00                    ;    //Make PORTA as INPUT

    M_INT1 =  (PIND & 0x01)          ;
    M_INT2 = ((PIND & 0x02)>>1)&0x01 ;
    M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
    M_INT4 = ((PINE & 0x20)>>5)&0x01 ;

    PORTG  = 0x1E                    ;      //IOCS1=0  IOCS2=1  IOCS3=1  IOCS4=1
    delay_ms(1)                      ;
    M_IN1  =  (PINA & 0x01)          ;
    M_IN2  = ((PINA & 0x02)>>1)&0x01 ;
    M_IN3  = ((PINA & 0x04)>>2)&0x01 ;
    M_IN4  = ((PINA & 0x08)>>3)&0x01 ;
    M_IN5  = ((PINA & 0x10)>>4)&0x01 ;
    M_IN6  = ((PINA & 0x20)>>5)&0x01 ;
    M_IN7  = ((PINA & 0x40)>>6)&0x01 ;
    M_IN8  = ((PINA & 0x80)>>7)&0x01 ;

    PORTG  = 0x1D                    ;      //IOCS1=1  IOCS2=0  IOCS3=1  IOCS4=1
    delay_ms(1)                      ;
    M_IN9  =  (PINA & 0x01)          ;
    M_IN10 = ((PINA & 0x02)>>1)&0x01 ;
    M_IN11 = ((PINA & 0x04)>>2)&0x01 ;
    M_IN12 = ((PINA & 0x08)>>3)&0x01 ;
    M_IN13 = ((PINA & 0x10)>>4)&0x01 ;
    M_IN14 = ((PINA & 0x20)>>5)&0x01 ;
    M_IN15 = ((PINA & 0x40)>>6)&0x01 ;
    M_IN16 = ((PINA & 0x80)>>7)&0x01 ;

    PORTG  = 0x1B                    ;      //IOCS1=1  IOCS2=1  IOCS3=0  IOCS4=1
    delay_ms(1)                      ;
    M_IN17 =  (PINA & 0x01)          ;
    M_IN18 = ((PINA & 0x02)>>1)&0x01 ;
    M_IN19 = ((PINA & 0x04)>>2)&0x01 ;
    M_IN20 = ((PINA & 0x08)>>3)&0x01 ;
    M_IN21 = ((PINA & 0x10)>>4)&0x01 ;
    M_IN22 = ((PINA & 0x20)>>5)&0x01 ;
    M_IN23 = ((PINA & 0x40)>>6)&0x01 ;
    M_IN24 = ((PINA & 0x80)>>7)&0x01 ;

    PORTG  = 0x17                    ;      //IOCS1=1  IOCS2=1  IOCS3=1  IOCS4=0
    delay_ms(1)                      ;
    HV_IN1 =  (PINA & 0x01)          ;
    HV_IN2 = ((PINA & 0x02)>>1)&0x01 ;
    HV_IN3 = ((PINA & 0x04)>>2)&0x01 ;
    HV_IN4 = ((PINA & 0x08)>>3)&0x01 ;
    HV_IN5 = ((PINA & 0x10)>>4)&0x01 ;
    HV_IN6 = ((PINA & 0x20)>>5)&0x01 ;
    HV_IN7 = ((PINA & 0X40)>>6)&0x01 ;
    HV_IN8 = ((PINA & 0X80)>>7)&0x01 ;

    PORTG  = 0x1F                    ;

    DDRA   = 0xFF                    ;

#ifdef Extra_Inputs
       if((!OVL)&&(OVL_M)){OVL=1;OVL_MC=1;}

       if((!FUL)&&(FUL_M)){FUL=1;FUL_MC=1;}

       if((!DOO)&&(DO_M)){DOO=1;DOO_MC=1;}

       if((!DOC)&&(DC_M)){DOC=1;DOC_MC=1;}


       if((!OVL_M)&&(OVL_MC)){OVL=0;OVL_MC=0;}
       if((!FUL_M)&&(FUL_MC)){FUL=0;FUL_MC=0;}
       if((!DO_M)&&(DOO_MC)){DOO=0;DOO_MC=0;}
       if((!DC_M)&&(DOC_MC)){DOC=0;DOC_MC=0;}

#endif

}
#include "defines.h"
#include "can.h"
//#define ETHERNET_ENABLE    1
//#define debug 1

void supervisor(void)
{
     unsigned char sv1=0,sv2=0,carout=0;
     unsigned char turnoffhallleds=0;
     unsigned long int ID,sv3=0;
     unsigned int sv4=0;
     struct protocol_id_structure_sep sep_id;

/***********Critical Fatal and Cyclic Errors************/
read_inputs();				//Update Inputs' Flags




if(_OVL_timeout)_OVL_timeout++;
if(_OVL_timeout>200)OVL=0;

if(_DOO_timeout)_DOO_timeout++;
if(_DOO_timeout>200)DOO=0;


if(_OVL_timeout_announuce)
 {
        _OVL_timeout_announuce++;
        if(_OVL_timeout_announuce>250)_OVL_timeout_announuce=0;
 }

if(!POWER24)				//E51
	{errormsg(51);		  	//24v-DC power supply is turn off.
	 floor=0;			      //Get out of inf. loop and wait for RESET.
	// needreset=1;
	 recovery(3);                    //erase last saved floor
	 return;
	}

if(!POWER110)				//E52
	{errormsg(52);			//110v-ac power supply is turn off.
	 floor=0;			      //Get out of inf. loop and wait for RESET.
	// needreset=1;
	 return;
	}

if(!SS63)				      //E63
	{pathfinder(0,32);		//Safety-Serie is disconnected from 63.
	 if(!FLT)errormsg(63);			//63 --> Top of hoist-way.
	 floor=0;			      //Get out of inf. loop.
	 return;
	}

if(!SS64)				      //E64
	{pathfinder(0,32);		//Safety-Serie is disconnected from 64.
	 errormsg(64);			//64 --> Down of hoistway.
	 floor=0;			      //Get out of inf. loop.
	 return;
	 }

if(!SS65)				      //E65
	{pathfinder(0,32);		//Safety-Serie is disconnected from 65.
	 /***/				//turn off all leds
	 errormsg(65);			//65 --> Cabin's safety serie.
	 floor=0;			      //Get out of inf. loop.
	 return;
	}

if(I&&!SS66)				//E66
	{pathfinder(0,32);		//Safety-Serie is disconnected from 66 durnig movement.
	 /***/				//turn off all leds
	 errormsg(66);			//66 --> Hall-doors contacts.
	 floor=0;			      //66 --> connect to 65 in Full-automatic doors.
	 return;			      //Get out of inf. loop.
	}
if(I&&!SS69)				//E69
	{
	 if((ptimer_preopening>0)&&(Preopening_en=='s')&&(CONTF==0))
	   {
	        #asm("NOP")
	   }
	 else
	   {

        	pathfinder(0,32);		//Safety-Serie is disconnected from 69 during movement
        	 /***/				//turn off all leds
        	 errormsg(69);			//69 --> Cabin's door contact
        	 floor=0;			      //Get out of inf. loop
        	 return;
	   }
	}
if(I&&!SS68)				//E68
	{
        	 pathfinder(0,32);		//Safety-Serie is disconnected from 68 during movement
	         /***/				//turn off all leds
        	 errormsg(68);			//68 --> Door's lock in Sematic and Simple doors
        	 floor=0;			      //68 --> Cabins door in full-Automatic
        	 return;			      //Get out of inf. loop
	}

if(CA1&&CAn)				//E71
					      //Turning on the CA1 and CAn simultaneously
	{pathfinder(0,32);		//
         turnoffallleds=1;      	//turn off all leds
	 errormsg(71);			//Get out of inf. loop
	 floor=0;
	 recovery(3);                    //erase last saved floor
	 return;
	 }


if(Fsflag<2)
  {
   if(CF1&&CF3)				//E72
					      // 1CF and CF3 are active simultaneously
	{pathfinder(0,32);		//Get out of inf. loop
	 turnoffallleds=1;	      //turn off all leds
	 errormsg(72);
	 floor=0;
	 recovery(3);                    //erase last saved floor
	 return;
	}
  }


if(traveltimer>Timetravel)
        {
         traveltimer=0;
         pathfinder(0,32);
         turnoffallleds=1;
         floor=0;
         needreset=1;               //***\\
	     recovery(3);                    //erase last saved floor
         errormsg(35);
        }

if((!FTO)&&FTOC)
		{FTOC=0;
		 if(last_display_msg==82)errormsg(old_display_msg);
		 /****/			//Stop FTO announcing
		}
if((!FLT)&&FLTC)
		{FLTC=0;
		 errormsg(old_display_msg);
		 /****/			//Stop FLT announcing
		}
if((!OVL)&&OVLC)
		{
		 OVLC=0;		      //Turning off FTO or OVL
		 _OVL_timeout=0;
		 if(last_display_msg==81)errormsg(old_display_msg);
		 /****/			//Stop OVL announcing
		}



if(OVL&&(!OVLC))
   {
    OVLC=1;			      //Overload is active

     if(!I)
       {
      	 errormsg(81);			//Announce it on LCD and 7seg
         if(!_OVL_timeout_announuce)
          {
            _OVL_timeout_announuce=1;
            set_annunciator=4;
          }
         turnoffallleds=1;
         _set_numerator=1;
         pathfinder(0,32);		//Erase requests lists
         direction=0;
         NNOD=0;
       }

   }


if(FTO&&(!FTOC))			      //P12
	{FTOC=1;			      //FTO is active
	 errormsg(82);			//Announce it on LCD and 7seg
	 if(opmode=='n')
	 {
	  if(I&&floor==AL[0])		//Stop in nearest floor according to direction
	    {pathfinder(0,32);		//Erase the Requests List
	     turnoffallleds=1;	      //turn off all leds
	      AL[0]=floor;
	     _al[0]=1;			//Active during stopping
	     NNOD=0;			//NNOD=0 for Opening the door
	    } else
		    {
		     pathfinder(0,32);
		     turnoffallleds=1;		            //turn of all leds
		     if((direction==1)&&I)
			{AL[0]=floor+1;		//Active during moving
		 	 _al[0]=1;
		 	 NNOD=0;}
	 	     else if ((direction==2)&&I)
			{AL[0]=floor-1;
		 	 _al[0]=1;
		 	 NNOD=0;}
		     else if(!I)
			{direction=0;}
		    }
	  }
          _set_numerator=1;
	}

if(FLT&&(!FLTC)&&(opmode=='n'))			      //E91
	{FLTC=1;			      //FAULT IN LPC OR DROVE is active
	 //errormsg(91);			//Announce it on LCD and 7seg
	 if(opmode=='n')
	  {
	   if(I&&floor==AL[0])		//Stop in nearest floor according to direction
	    {pathfinder(0,32);		//Erase the Requests List
	     turnoffallleds=1;	      //turn off all leds
	      AL[0]=floor;
	     _al[0]=1;			//Active during stopping
	     NNOD=0;			//NNOD=0 for Opening the door
	     } else
		    {
		     pathfinder(0,32);
		     turnoffallleds=1;		            //turn of all leds
		     if((direction==1)&&I)
			{AL[0]=floor+1;		//Active during moving
		 	 _al[0]=1;
		 	 NNOD=0;}
	 	     else if ((direction==2)&&I)
			{AL[0]=floor-1;
		 	 _al[0]=1;
		 	 NNOD=0;}
		     else if(!I)
			{direction=0;}
		    }
	    }
          _set_numerator=1;
	}

if((!FUL)&&FULC)FULC=0;

if(FUL&&(!FULC))			      //P12
	{FULC=1;			      //FTO is active
	 //errormsg(82);			//Announce it on LCD and 7seg
 	 if(opmode=='n')
 	 {
	  if(I&&floor==AL[0])		//Stop in nearest floor according to direction
	    {pathfinder(0,32);		//Erase the Requests List
	     turnoffallleds=1;	      //turn off all leds
	      AL[0]=floor;
	     _al[0]=1;			//Active during stopping
	     NNOD=0;			//NNOD=0 for Opening the door
	    } else
		    {
		     pathfinder(0,32);
		     turnoffallleds=1;		            //turn of all leds
		     if((direction==1)&&I)
			{AL[0]=floor+1;		//Active during moving
		 	 _al[0]=1;
		 	 NNOD=0;}
	 	     else if ((direction==2)&&I)
			{AL[0]=floor-1;
		 	 _al[0]=1;
		 	 NNOD=0;}
		     else if(!I)
			{direction=0;}
		    }
	  }
          _set_numerator=1;
	}
	/*

if(FUL&&(!FULC))
	{
	 FULC=1;
	 if((!I)&&(!OVLC))
		{
		 pathfinder(0,16);	//FUL is active
		 turnoffhallleds=1;      //turn off all of floors leds
	  	}			      //Erase the Request List from Hall requests
	};
*/

/*************Mode Selection***************/

if(REVC&&(opmode!='c'))			//P13
	{errormsg(0);    		//for clearin 7 seg
	 opmode='c';			//
	 pathfinder(0,32);		//Erase the Request List
	 floor=0;			      //Get out of inf. loop
	 return;
	}

if(REVM&&(opmode!='m'))			//P14
	{errormsg(0);    		//for clearing 7 seg
	 opmode='m';			//
	 pathfinder(0,32);		//Erase the Request List
	 floor=0;			      //Get out of inf. loop
	 return;
	}

if((!FIRE)&&firemf)
	{firemf=0;
	 NNOD=0;}


if(FIRE&&(!firemf)&&floor&&(opmode=='n'))			//P15
	{				      //Active Fire switch or detector
	 if(Firefloor)errormsg(85);			//Announce Fire mode by 7seg, LCD & etc.
	   else errormsg(89);                            //Announce the Lifter Mode

	 VIPmf=0;

	 if((floor==AL[0])&&floor)	//Fire durnig stopping
		{pathfinder(0,32);
		 turnoffallleds=1;	//turn off all leds
		 _set_numerator=1;
		 AL[0]   = floor;
		 _al[0] = 1;
		 if(Firefloor)
		  {
		    pathfinder(Firefloor,1);
		    if(floor!=Firefloor)
			{NNOD=1;}
			 else NNOD=0;
	           }
		 firemf=1;
		}
	 else
	 if(!I)				       //Fire when being idle
		{pathfinder(0,32);	      //Erase the Request List
		 turnoffallleds=1;	      //Must turn off leds here...
		 _set_numerator=1;
		 if(Firefloor)pathfinder(Firefloor,1);
		                			//must turn on the led for fire floor in Car
		 firemf=1;                          //??????must be checked later
		}
	 else
	 if(direction==1)		              //Fire during movement UP
		{pathfinder(0,32);	      //Erase the Request List
		 turnoffallleds=1;	     //turn off leds
		 _set_numerator=1;
		 if((Firefloor<=floor)||(!Firefloor))
			{
			 if(floor<Floorsno)AL[0]=floor+1;
			    else AL[0]=floor;
			 _al[0]=1;
			 if(Firefloor==AL[0])
				{NNOD=0;}
				 else if(Firefloor)NNOD=1;
			}
		 if(Firefloor)pathfinder(Firefloor,1);
		 /*  */			     //turn on firefloor led in car
		 firemf=1;		             //place firefloor in request list
		}
	else
	if(direction==2)		            //Fire during movement DOWN
		{pathfinder(0,32);	    //Erase request list
		 turnoffallleds=1;			//turn off leds
		 _set_numerator=1;
		 if((Firefloor>=floor)||(!Firefloor))
			{
			 if(floor==1)AL[0]=1;
			    else AL[0]=floor-1;
			 _al[0]=1;
			 if(Firefloor==AL[0])
						{NNOD=0;}
				else if(Firefloor)NNOD=1;
			}
		 if(Firefloor)pathfinder(Firefloor,1);
		 /***/			//turn on firefloor led in car
		 firemf=1;		      //place firefloor in request list
		}
	}

if((!VIP)&&VIPmf)
	{VIPmf=0;
         NNOD=0;}

if(VIP&&(!VIPmf)&&floor&&(opmode=='n')&&(!firemf))
	{				      //Active Fire switch or detector

	 errormsg(90);			//Announce Fire mode by 7seg, LCD & etc.

	 if((floor==AL[0])&&floor)	//Fire durnig stopping
		{pathfinder(0,32);
		 turnoffallleds=1;	//turn off all leds
		 _set_numerator=1;
		 AL[0]   = floor;
		 _al[0] = 1;
		    pathfinder(VIPfloor,1);
		    if(floor!=VIPfloor)
			{NNOD=1;}
			 else NNOD=0;
		 VIPmf=1;
		}
	 else
	 if(!I)				       //Fire when being idle
		{pathfinder(0,32);	      //Erase the Request List
		 turnoffallleds=1;	      //Must turn off leds here...
		 _set_numerator=1;
		 pathfinder(VIPfloor,1);
		                			//must turn on the led for fire floor in Car
		 VIPmf=1;                          //??????must be checked later
		}
	 else
	 if(direction==1)		              //Fire during movement UP
		{pathfinder(0,32);	      //Erase the Request List
		 turnoffallleds=1;	     //turn off leds
		 _set_numerator=1;
		 if(VIPfloor<=floor)
			{
			 if(floor<Floorsno)AL[0]=floor+1;
			    else AL[0]=floor;
			 _al[0]=1;
			 if(VIPfloor==AL[0])
				{NNOD=0;}
				 else if(VIPfloor)NNOD=1;
			}
		 pathfinder(VIPfloor,1);
		 /*  */			     //turn on firefloor led in car
		 VIPmf=1;		             //place firefloor in request list
		}
	else
	if(direction==2)		            //Fire during movement DOWN
		{pathfinder(0,32);	    //Erase request list
		 turnoffallleds=1;			//turn off leds
		 _set_numerator=1;
		 if(VIPfloor>=floor)
			{
			 if(floor==1)AL[0]=1;
			    else AL[0]=floor-1;
			 _al[0]=1;
			 if(VIPfloor==AL[0])
						{NNOD=0;}
				else if(VIPfloor)NNOD=1;
			}
		 pathfinder(VIPfloor,1);
		 /***/			//turn on firefloor led in car
		 VIPmf=1;		      //place firefloor in request list
		}
	}


   /**********Diagnostic Errors***********/
if(identyle)
        {errormsg(37);
         identyle=0;
         pathfinder(0,32);
         floor=0;
         _set_numerator=1;
         turnoffallleds=1;}



if(hdoorct)				      //Hall door(s) closing
	{errormsg(41);			//time is over
	 //announce and alert for this	//E41
	 pathfinder(0,32);
	 turnoffallleds=1;		//Turn off all leds
	 _set_numerator=1;
	 direction=0;
	 hdoorct=0;
         set_annunciator=41;
	}
if(doorcnc)				      //Cabin door is not closed
	{errormsg(42);			// after N time try
	 pathfinder(0,32);
	 turnoffallleds=1;		//Turn off all leds
	 _set_numerator=1;
	 direction=0;			//E42
	 doorcnc=0;
         set_annunciator=42;
	}
if(doorcno)				      //Cabin door contact is
	{errormsg(43);			//not open after DOORDUR
	 pathfinder(0,32);
	 turnoffallleds=1;            //announce and alert for this
	 _set_numerator=1;
	 direction=0;
	 doorcno=0;                   //E43
	}
if(doorlcc)				      //Door's lock contact is close
	{errormsg(44);			//before to be closed by URA
	                              //announce and alert for this
	 pathfinder(0,32);
	 turnoffallleds=1;		//Turn off all leds
	 _set_numerator=1;
	 direction=0;			//E44
	 doorlcc=0;
	 }
if(carniva)
	{errormsg(45);			//Car is not in valid area
					      //for opening the door
	 /***/                        //announce and alert for this
	 pathfinder(0,32);		//E45
	 turnoffallleds=1;		//Turn off all leds
	 direction=0;
	 _set_numerator=1;
	 //floor=0;
	 //recovery(3);                    //erase last saved floor
	 carniva=0;
	}
if(uraminw)
	{errormsg(46);			//URA magnet doesn't work
	 				      //announce and alert for this
	 uraminw=0;			      //E46
	}
if(doorlcns)				//door-lock contact is not shown
	{errormsg(47);
	 pathfinder(0,32);
 	 turnoffallleds=1;
	 _set_numerator=1;
	 direction=0;
	 doorlcns=0;
	}				      //E47
if(doorlcs)
	{errormsg(48);			//URA did not open the door lock
//	 turnoffallleds=1;                  ///
	 doorlcs=0;
//	 floor=0;			      //E48
//	 recovery(3);                    //erase last saved floor
	}

					      //in following contactor error,
					      // AL is cleared & stop untill reset

if(contce)
	{errormsg(31);			//Conts. are closed before trigging
	 pathfinder(0,32);		//Erase the request List
	 floor=0;			      //E31
	 turnoffallleds=1;      	//Turn off all leds
	 contce=0;
	 needreset=1;			//Must be reset
	 recovery(3);                    //erase last saved floor
	}
if(contcea)
	{errormsg(32);			//Conts. are still closed after
	 pathfinder(0,32);		//E32
	 floor=0;			      //turning off the bobines
	 turnoffallleds=1;      	//turn off all leds
	 contcea=0;
	 needreset=1;
	 recovery(3);                    //erase last saved floor
	}
if(contnca)
	{errormsg(33);			//Conts. aren't be closed till
	 pathfinder(0,32);		//E33
	 floor=0;			      //turning off the bobines
	 turnoffallleds=1;		//turn off all leds
	 contnca=0;
	// needreset=1;			//must be reset
	// recovery(3);                    //erase last saved floor
	}
if(feedb4bse)
	{errormsg(34);			//Break FB is not shown
	 pathfinder(0,32);		//E34
	 floor=0;			      //turning off the bobines
	 turnoffallleds=1;		//turn off all leds
	 feedb4bse=0;
	 needreset=1;
	 recovery(3);                    //erase last saved floor
	}


if(motordi)
	{errormsg(73);			//Motor direction is incorrect
	 turnoffallleds=1;            //cause a fault in phase-sequence
	 floor=0;				//announce and alert for this
	 motordi=0;			      //E73
	 needreset=1;
	 recovery(3);                    //erase last saved floor
	}

if(time1cf)
	{errormsg(74);			//Valid time for sensing 1CF is elapsed
	 time1cf=0;			      //E74
	 pathfinder(0,32);		//Erase the request list
	 turnoffallleds=1;      	//turn off all leds
	 floor=0;
	 recovery(3);                    //erase last saved floor
	}
if(timecf3e)
	{errormsg(75);			//Valid time for sensing 1CF is elapsed
	 pathfinder(0,32);
	 floor=0;			      //turning off the bobines
	 turnoffallleds=1;		//turn off all leds
	 timecf3e=0;
	 recovery(3);                    //erase last saved floor
	}

if(cf3cf1)				      //CF3 & 1CF are sensed
	{errormsg(72);			//simultaneously on motor-start-moment
	 pathfinder(0,32);
	 floor=0;			      //turning off the bobines
	 turnoffallleds=1;		//turn off all leds
	 cf3cf1=0;
	 needreset=1;
	 recovery(3);                    //erase last saved floor
	}

if(canins)
	{errormsg(76);			//cabin in highest stair
	 canins=0;			      //but CAn is not active
	 pathfinder(0,32);		//E76
	 turnoffallleds=1;		//turn off all leds
	 _set_numerator=1;
	 pathfinder(Floorsno,1);
	 /***/				//turn on last floor led in cabin
	}

if(ca1ins)
	{errormsg(77);			//cabin in lowest stair
 	 ca1ins=0;			      //but CA1 is not active
	 pathfinder(0,32);		//E77
	 turnoffallleds=1; 	//turn off all leds
	 _set_numerator=1;
	 pathfinder(1,1);
	}

if(calmto)				      //Max. Time for Calibration
	{errormsg(78);			//is over
	 floor=0;
	 calmto=0;			      //E78
      }

if(mcbycan)
	{errormsg(79);			//Moving Up is canceled by CaN
	 floor=0;			      //E79
	 mcbycan=0;
	 recovery(3);                    //erase last saved floor
	}

if(mcbyca1)
	{errormsg(80);			//Moving Down is canceled by Ca1
	 floor=0;			      //E80
	 mcbyca1=0;
	 recovery(3);                    //erase last saved floor
	}

/***********Calls of Functions******/

if(_pf_park)
	{pathfinder(Parkfloor,1);
	 /***/				//turn on 'parkfloor' led on cabin

	 errormsg(86);
	 _pf_park=0;}

if(_pf_reform)
      {
       pathfinder(0,8);
	 _pf_reform=0;
	}


if(Doorparkunderl=='n')
 {
   if(_delay_park_unload_door)                            //cancel unloading door process
        if((RELAYC==0)||(RELAYO==1)||(standbymf==0)||(SS66==0)||(DOO))_delay_park_unload_door = 0;

   if(_delay_park_unload_door)                         //for unload door after a delay DULD
   {                                                              // in park mode
     _delay_park_unload_door--;
     if(_delay_park_unload_door==0)
           {
             if((RELAYC==1)&&(RELAYO==0))
              {
                     RELAYC=0;SO9=0;
                     RELAYO=0;SO10=0;
                     _set_output_car = 1;
              }
           }
   }
}

if((Doorunderl==68)||(Doorunderl==69))
 {
   if(_delay_unload_door)                                //cancel unloading door process
        if((RELAYC==0)||(RELAYO==1))_delay_unload_door = 0;

   if(_delay_unload_door)                            //for unload door after a delay DULD
    {
     _delay_unload_door--;
     if(_delay_unload_door==0)
          {
            if((RELAYC==1)&&(RELAYO==0))
             {
                     RELAYC=0;SO9=0;
                     RELAYO=0;SO10=0;
                     _set_output_car = 1;
             }           //if((RELAYC
          }              //if(_delay_unload_door==0)
     }                //if(_delay_unload_door)
 }                    //if(Dooruldelay)


if(_ack_can_transmit_error!=0xFF)
    {
     if( (_ack_can_transmit_error>=TxStaMob) && (_ack_can_transmit_error<15) )
        {
              #ifdef debug
                  printf("Resend in mob # %d cause an error \r\n",_ack_can_transmit_error);
              #endif
              delay_ms(5);
              current_message = can_send_history[_ack_can_transmit_error - TxStaMob];
              can_send_message(current_message);
              delay_ms(10);
        }
      _ack_can_transmit_error = 0xFF;
    }


if(turnoffleds)
      {				//turn off leds
        if(Numtype=='n')
	{
        sv1=(turnoffleds%16);                //sv1=> index of dest_address  //floor
        if(!sv1)sv1=16;

        sep_id.source_address = node_address;
        sep_id.dest_address = 0xE0+((turnoffleds-1)/16);
        sep_id.message_num = 0;
        sep_id.priority = 0;
        ID = assemble_id(sep_id);
        current_message.id = ID;
        current_message.datalen=2;
        current_message.data[0]='F';
        current_message.data[1]=sv1;
        can_send_message(current_message);   //turn off led in cabin
        delay_ms(10);

        if(Navistrategy=='s')
         {
           sv2=(turnoffleds%8);
           if(!sv2)sv2=8;
           sv2*=2;
           sep_id.source_address = node_address;
           sep_id.dest_address = 0xF0+((turnoffleds-1)/8);
           sep_id.message_num = 0;
           sep_id.priority = 0;
           ID = assemble_id(sep_id);
           current_message.id = ID;
           current_message.datalen=2;
           current_message.data[0]='F';
           current_message.data[1]=sv2;
           can_send_message(current_message);
           delay_ms(10);
           sv2--;
           current_message.data[1]=sv2;
           can_send_message(current_message);
           delay_ms(10);
	 }
	 else
	 if((Navistrategy=='f')||(Navistrategy=='d'))
	   {
	    sv2=(turnoffleds%16);                //sv1=> index of dest_address    //floor
            if(!sv2)sv2=16;

            sep_id.source_address = node_address;
            sep_id.dest_address = 0xF0+((turnoffleds-1)/16);
            sep_id.message_num = 0;
            sep_id.priority = 0;
            ID = assemble_id(sep_id);
            current_message.id = ID;
            current_message.datalen=2;
            current_message.data[0]='F';
            current_message.data[1]=sv1;
            can_send_message(current_message);
            delay_ms(10);
           }

         }
         else                //else for Numtype=='n'
         if(Numtype=='d')
            {
              sep_id.source_address = node_address;
              sep_id.dest_address = 0x10;
              sep_id.message_num = 0;
              sep_id.priority = 0;
              ID = assemble_id(sep_id);
              current_message.id = ID;
              current_message.datalen=2;
              current_message.data[0]='F';
              current_message.data[1]=turnoffleds;
              can_send_message(current_message);   //turn off led in Num-extension
              delay_ms(10);
              sep_id.dest_address = 0xB0;
              ID = assemble_id(sep_id);       //turn off led on num-floor board
              current_message.id = ID;
              current_message.datalen=2;
              current_message.data[0]='F';
              current_message.data[1]=turnoffleds;
              can_send_message(current_message);
              delay_ms(10);
            }
         else
         if(Numtype=='s')
           {
              serial_current_message.address=0xD3;
              serial_current_message.length=2;
              serial_current_message.data[0]='F';
              serial_current_message.data[1]=turnoffleds;
              RS485_send_message(serial_current_message);
              delay_ms(10);           //************


              sep_id.source_address = node_address;
              sep_id.message_num = 0;
              sep_id.priority = 0;
              sep_id.dest_address = 0xC0;
              ID = assemble_id(sep_id);       //turn off led on num-floor board
              current_message.id = ID;
              current_message.datalen=2;
              current_message.data[0]='F';
              current_message.data[1]=turnoffleds;
              can_send_message(current_message);
              delay_ms(10);
           }
        else
        if(Numtype=='f')
	{
        sv1=(turnoffleds%16);                //sv1=> index of dest_address  //floor
        if(!sv1)sv1=16;

        sep_id.source_address = node_address;
        sep_id.dest_address = 0xC0;
        sep_id.message_num = 0;
        sep_id.priority = 0;
        ID = assemble_id(sep_id);
        current_message.id = ID;
        current_message.datalen=4;
        current_message.data[0]='X';
        current_message.data[1]=0xE0+((turnoffleds-1)/16);
        current_message.data[2]='F';
        current_message.data[3]=sv1;
        can_send_message(current_message);
        delay_ms(10);
               //turn off led in cabin



        if(Navistrategy=='s')
         {
           sv2=(turnoffleds%8);
           if(!sv2)sv2=8;
           sv2*=2;

              serial_current_message.address=0xF0+((turnoffleds-1)/8);
              serial_current_message.length=2;
              serial_current_message.data[0]='F';
              serial_current_message.data[1]=sv2;
              RS485_send_message(serial_current_message);
              delay_ms(10);
              sv2--;
              serial_current_message.data[1]=sv2;
              RS485_send_message(serial_current_message);
              delay_ms(10);

	 }
	 else
	 if((Navistrategy=='f')||(Navistrategy=='d'))
	   {
	    sv2=(turnoffleds%16);                //sv1=> index of dest_address    //floor
            if(!sv2)sv2=16;

              serial_current_message.address=0xF0+((turnoffleds-1)/16);
              serial_current_message.length=2;
              serial_current_message.data[0]='F';
              serial_current_message.data[1]=sv1;
              RS485_send_message(serial_current_message);
              delay_ms(10);


           }

         }


         if(Eth_number)
            {
               if(Monitoring == 'y')_reset_out_request=turnoffleds;        //ethernet_enable
               if(Grouptype!='s')Send_Request_via_Eth(0,'F');

            }

       turnoffleds=0;

       return;
    }


if(_set_output_car)
      {
        carout=0x00;
        if(URA)carout|=0x80;
        if(RELAYSTBY)carout|=0x10;

        if(Tunnel_Door_en=='2')
         {
           if(RELAYC)carout|=0x48 ;        //c1=1 c2=1
           if((!RELAYC)&&(!RELAYO)&&(!I))               //c=0 o=0
             {
                sv1=Tunnel_door_find(floor);
                if(sv1==1)carout|=0x08;                     //only 1st door is 00 2nd is close
                if(sv1==2)carout|=0x40;                     //only 2nd door is 00 1st is close
             }

           if(RELAYO)
           {
                sv1=Tunnel_door_find(floor);
                if(sv1==1)carout|=0x28;                     //only 1st door is open 2nd is close
                if(sv1==2)carout|=0x44;                     //only 2nd door is open 1st is close
                if(sv1==3)carout|=0x24;                     //both of doors are open
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

       _set_output_car=0;

       return;
      }




if(turnoffallleds)
      {
       if(Numtype=='n')
       {
        for(sv1=0;sv1<((Floorsno-1)/16+1);sv1++)        //Reset Extension Board outputs
        {
         sep_id.source_address = node_address;
         sep_id.dest_address = 0xE0+sv1;
         sep_id.message_num = 0;
         sep_id.priority = 0;
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=1;
         current_message.data[0]='R';
         can_send_message(current_message);
         delay_ms(10);
        }
        for(sv1=0;sv1<((Floorsno-1)/8+1);sv1++)       //Reset Floor Boards outputs
        {
         sep_id.source_address = node_address;
         sep_id.dest_address = 0xF0+sv1;
         sep_id.message_num = 0;
         sep_id.priority = 0;
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=1;
         current_message.data[0]='R';
         can_send_message(current_message);
         delay_ms(10);
        }
       }
       else
       if(Numtype=='d')
       {
        sep_id.dest_address = 0xB0;              //Num-Floor Board
        ID = assemble_id(sep_id);
        current_message.id = ID;
        current_message.datalen=1;
        current_message.data[0]='R';
        can_send_message(current_message);   //Turn off cabin relays
        delay_ms(10);

        sep_id.dest_address = 0x10;              //Num-Ext Board
        ID = assemble_id(sep_id);
        current_message.id = ID;
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

         sep_id.source_address = node_address;
         sep_id.message_num = 0;
         sep_id.priority = 0;
         sep_id.dest_address = 0xC0;              //cabin Board
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=1;
         current_message.data[0]='R';
         can_send_message(current_message);   //Turn off cabin relays
         delay_ms(10);
       }
       else
       if(Numtype=='f')
       {
        for(sv1=0;sv1<((Floorsno-1)/16+1);sv1++)        //Reset Extension Board outputs
        {
         sep_id.source_address = node_address;
         sep_id.dest_address = 0xC0;
         sep_id.message_num = 0;
         sep_id.priority = 0;
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=3;
         current_message.data[0]='X';
         current_message.data[1]=0xE0+sv1;
         current_message.data[2]='R';
         can_send_message(current_message);
         delay_ms(10);
        }
        for(sv1=0;sv1<((Floorsno-1)/8+1);sv1++)       //Reset Floor Boards outputs
        {
         serial_current_message.address=0xF0+sv1;
         serial_current_message.length=1;
         serial_current_message.data[0]='R';
         RS485_send_message(serial_current_message);
         delay_ms(10);
        }
       }

       turnoffallleds=0;

       if(Eth_number)
       {
         if(Monitoring == 'y')_reset_out_request = 0xFF;   //ethernet_enable
         if(Grouptype!='s')Send_Request_via_Eth(0,'E');
        }

       return;
      }


if(turnoffhallleds)
      {

      for(sv1=0;sv1<((Floorsno-1)/16+1);sv1++)        //Reset Extension Board outputs
        {                                             //For Blinking outputs on them
         sep_id.source_address = node_address;
         sep_id.dest_address = 0xE0+sv1;
         sep_id.message_num = 0;
         sep_id.priority = 0;
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=2;
         current_message.data[0]='K';
         current_message.data[1]=0x00;
         current_message.data[2]=0x00;
         current_message.data[3]=0x00;
         can_send_message(current_message);
         delay_ms(10);
        }

       for(sv1=0;sv1<((Floorsno-1)/8+1);sv1++)       //Reset Floor Boards outputs
       {
         sep_id.source_address = node_address;
         sep_id.dest_address = 0xF0+sv1;
         sep_id.message_num = 0;
         sep_id.priority = 0;
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=1;
         current_message.data[0]='R';
         current_message.data[1]=0x00;
         current_message.data[2]=0x00;
         current_message.data[3]=0x00;
         can_send_message(current_message);
         delay_ms(10);
       }
        turnoffhallleds=0;
      }

if(_set_numerator)
       {
         sep_id.source_address = node_address;
         sep_id.message_num = 0;
         sep_id.priority = 0;
         current_message.datalen=4;

         if(Numtype=='n')
         {
          current_message.data[0]='S';
          current_message.data[1]=showfloor(floor,1);
          current_message.data[2]=showfloor(floor,2);
          if((direction==1)&&(!stayinstair))current_message.data[3]='U';   //LF1 on LF2 off
          else if((direction==2)&&(!stayinstair))current_message.data[3]='D';   //LF1 off LF2 on
          else if((direction==1)&&(stayinstair))current_message.data[3]='u';    //LF1 blink LF2 off
          else if((direction==2)&&(stayinstair))current_message.data[3]='d';    //LF1 off LF2 blink
          else current_message.data[3]='n';

          sep_id.dest_address = 0xD0;            //numerator
          ID = assemble_id(sep_id);
          current_message.id = ID;
          can_send_message(current_message);
          delay_ms(10);
         }
         else
         if(Numtype=='d')
         {
           sep_id.dest_address = 0xB0;           //Num-Floor
           ID = assemble_id(sep_id);
           current_message.id = ID;
           current_message.data[0]='S';          //'S' for first digit of numerator
           current_message.data[1]=showfloor(floor,1);
           current_message.data[2]=0x00;
           if((direction==1)&&(!stayinstair))current_message.data[3]='U';   //LF1 on LF2 off
           else if((direction==2)&&(!stayinstair))current_message.data[3]='D';   //LF1 off LF2 on
           else if((direction==1)&&(stayinstair))current_message.data[3]='u';    //LF1 blink LF2 off
           else if((direction==2)&&(stayinstair))current_message.data[3]='d';    //LF1 off LF2 blink
           else current_message.data[3]='n';

           can_send_message(current_message);
           delay_ms(10);

           sep_id.dest_address = 0x10;           //Num-Ext
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
           serial_current_message.data[1]=showfloor(floor,1);
           serial_current_message.data[2]=showfloor(floor,2);  //***//
           if((direction==1)&&(!stayinstair))serial_current_message.data[3]='U';   //LF1 on LF2 off
           else if((direction==2)&&(!stayinstair))serial_current_message.data[3]='D';   //LF1 off LF2 on
           else if((direction==1)&&(stayinstair))serial_current_message.data[3]='u';    //LF1 blink LF2 off
           else if((direction==2)&&(stayinstair))serial_current_message.data[3]='d';    //LF1 off LF2 blink
           else serial_current_message.data[3]='n';

           RS485_send_message(serial_current_message);
           delay_ms(10);

           if(_set_numerator!='n')                     //send data to floor via can in movement is cancelled
            {
             sep_id.dest_address = 0xC0;           //Cabin
             ID = assemble_id(sep_id);
             current_message.id = ID;
             current_message.data[0]='S';          //'S' for first digit of numerator
             current_message.data[1]=showfloor(floor,1);
             current_message.data[2]=showfloor(floor,2);        //***//
             if((direction==1)&&(!stayinstair))current_message.data[3]='U';   //LF1 on LF2 off
             else if((direction==2)&&(!stayinstair))current_message.data[3]='D';   //LF1 off LF2 on
             else if((direction==1)&&(stayinstair))current_message.data[3]='u';    //LF1 blink LF2 off
             else if((direction==2)&&(stayinstair))current_message.data[3]='d';    //LF1 off LF2 blink
             else current_message.data[3]='n';

             can_send_message(current_message);
             delay_ms(10);
           }            //         if(_set_numerator!='n')

       }
       else
       if(Numtype=='f')
       {
           serial_current_message.address=0xD0;
           serial_current_message.length=4;
           serial_current_message.data[0]='S';
           serial_current_message.data[1]=showfloor(floor,1);
           serial_current_message.data[2]=showfloor(floor,2); ;
           if((direction==1)&&(!stayinstair))serial_current_message.data[3]='U';   //LF1 on LF2 off
           else if((direction==2)&&(!stayinstair))serial_current_message.data[3]='D';   //LF1 off LF2 on
           else if((direction==1)&&(stayinstair))serial_current_message.data[3]='u';    //LF1 blink LF2 off
           else if((direction==2)&&(stayinstair))serial_current_message.data[3]='d';    //LF1 off LF2 blink
           else serial_current_message.data[3]='n';

           RS485_send_message(serial_current_message);
           delay_ms(10);

           if(_set_numerator!='n')                     //send data to floor via can in movement is cancelled
            {
             sep_id.dest_address = 0xC0;           //Cabin
             ID = assemble_id(sep_id);
             current_message.id = ID;
             current_message.datalen=6;
             current_message.data[0]='X';
             current_message.data[1]=0xD0;
             current_message.data[2]='S';          //'S' for first digit of numerator
             current_message.data[3]=showfloor(floor,1);
             current_message.data[4]=showfloor(floor,2);
             if((direction==1)&&(!stayinstair))current_message.data[5]='U';   //LF1 on LF2 off
             else if((direction==2)&&(!stayinstair))current_message.data[5]='D';   //LF1 off LF2 on
             else if((direction==1)&&(stayinstair))current_message.data[5]='u';    //LF1 blink LF2 off
             else if((direction==2)&&(stayinstair))current_message.data[5]='d';    //LF1 off LF2 blink
             else current_message.data[5]='n';

             can_send_message(current_message);
             delay_ms(10);
           }            //         if(_set_numerator!='n')

       }
   _set_numerator=0;
   return;                                     //************************
     }


if(set_annunciator)
    {
      sep_id.source_address = node_address;
      sep_id.dest_address = 0xD1;           //annunciator
      sep_id.message_num = 0;
      sep_id.priority = 0;
      ID = assemble_id(sep_id);
      current_message.id = ID;
      current_message.datalen = 4;
      switch(set_annunciator)
       {
        case 1:
        current_message.data[0] = 'S';          //'S' for first digit of numerator
        current_message.data[1] = showfloor(floor,1);
        sv1=showfloor(floor,2);
        if(sv1==0)sv1='0';
        current_message.data[2] = sv1;
        if((direction==1)&&(!arrivedstair))current_message.data[3]='U';   //LF1 on LF2 off
          else if((direction==2)&&(!arrivedstair))current_message.data[3]='D';   //LF1 off LF2 on
          else if((direction==1)&&(arrivedstair))current_message.data[3]='u';    //LF1 blink LF2 off
          else if((direction==2)&&(arrivedstair))current_message.data[3]='d';    //LF1 off LF2 blink
          else current_message.data[3]='n';
         // can_send_message(current_message);
         // delay_ms(10);
         break;

        case 2:
          current_message.datalen = 1;
          current_message.data[0] = 'W';          //'W for welcome note
        //  can_send_message(current_message);
         // delay_ms(10);
        break;

        case 3:
          current_message.datalen = 1;
          current_message.data[0] = 'R';          //'R for Reset
       //   can_send_message(current_message);
       //   delay_ms(10);
        break;

        case 4:
          current_message.datalen = 1;
          current_message.data[0] = 'L';          //'L for Overload
       //   can_send_message(current_message);
       //   delay_ms(10);
        break;

        case 41:
          current_message.datalen = 2;
          current_message.data[0] = 'E';          //'E' for Error message
          current_message.data[1] = 41;
        //  can_send_message(current_message);
        //  delay_ms(10);
        break;

        case 42:
          current_message.datalen = 2;
          current_message.data[0] = 'E';          //'E' for Error message
          current_message.data[1] = 42;
        //  can_send_message(current_message);
        //  delay_ms(10);
        break;

        default:
           set_annunciator=0;
        break;
       }

     if(set_annunciator)
      {
        if(IVA_comm=='c')
          {
            can_send_message(current_message);
            delay_ms(10);
          }
        else
        if(IVA_comm=='p')
          {
             serial_current_message.address=0xD1;
             serial_current_message.length=current_message.datalen;
             serial_current_message.data[0]=current_message.data[0];
             serial_current_message.data[1]=current_message.data[1];
             serial_current_message.data[2]=current_message.data[2];
             serial_current_message.data[3]=current_message.data[3];
             RS485_send_message(serial_current_message);
             delay_ms(10);
          }
        else
        if(IVA_comm=='s')
          {
            sep_id.dest_address = 0xC0;           //CABIN
            ID = assemble_id(sep_id);
            current_message.id = ID;
            current_message.datalen = current_message.datalen + 2;
            current_message.data[5] = current_message.data[3];          //send data ti IVA401 via Cabin Board
            current_message.data[4] = current_message.data[2];
            current_message.data[3] = current_message.data[1];
            current_message.data[2] = current_message.data[0];
            current_message.data[1] = 0xD1;
            current_message.data[0] = 'X';
            can_send_message(current_message);
            delay_ms(10);
          }
        set_annunciator=0;

      }
    }


if(_set_calibration_num)
     {
        sep_id.source_address = node_address;
        sep_id.message_num = 0;
        sep_id.priority = 0;
        current_message.datalen=4;
        current_message.data[0]='S';
        current_message.data[1]='C';
        current_message.data[2]=0;
        current_message.data[3]='n';

        if(Numtype=='n')
        {
         sep_id.dest_address = 0xD0;         //Numerator
         ID = assemble_id(sep_id);
         current_message.id = ID;
         can_send_message(current_message);
         delay_ms(10);
        }
        else
        if(Numtype=='d')
        {
         sep_id.dest_address = 0xB0;           //Num-Floor
         ID = assemble_id(sep_id);
         current_message.id = ID;
         can_send_message(current_message);
         delay_ms(10);
         sep_id.dest_address = 0x10;            //Num-Ext
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
         serial_current_message.data[1]='C';
         serial_current_message.data[2]=0;
         serial_current_message.data[3]='n';
         RS485_send_message(serial_current_message);
         delay_ms(10);           //************

         sep_id.dest_address = 0xC0;            //Cabin
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
         serial_current_message.data[1]='C';
         serial_current_message.data[2]=0;
         serial_current_message.data[3]='n';
         RS485_send_message(serial_current_message);
         delay_ms(10);           //************

         sep_id.dest_address = 0xC0;            //Cabin
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=6;
         current_message.data[0]='X';
         current_message.data[1]=0xD0;
         current_message.data[2]='S';          //'S' for first digit of numerator
         current_message.data[3]='C';
         current_message.data[4]= 0 ;
         current_message.data[5]='n';
         can_send_message(current_message);
         delay_ms(10);


       }

        _set_calibration_num=0;
    }


if(_set_blink_num)
   {

     if(_set_blink_num=='O')
      {
        sep_id.source_address = node_address;
        sep_id.message_num = 0;
        sep_id.priority = 0;
        current_message.datalen=2;

        current_message.data[0]='D';
        current_message.data[1]='O';
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
         sep_id.dest_address = 0xB0;           //Num-Floor
         ID = assemble_id(sep_id);
         current_message.id = ID;
         can_send_message(current_message);
         delay_ms(10);
         sep_id.dest_address = 0x10;            //numerator
         ID = assemble_id(sep_id);
         current_message.id = ID;
         can_send_message(current_message);
         delay_ms(10);
        }
       else
       if(Numtype=='s')
       {
         serial_current_message.address=0xD3;
         serial_current_message.length=2;
         serial_current_message.data[0]='D';
         serial_current_message.data[1]='O';
         RS485_send_message(serial_current_message);
         delay_ms(10);           //************

         sep_id.dest_address = 0xC0;            //Cabin
         ID = assemble_id(sep_id);
         current_message.id = ID;
         can_send_message(current_message);
         delay_ms(10);
       }
       else
       if(Numtype=='f')
       {
         serial_current_message.address=0xD0;
         serial_current_message.length=2;
         serial_current_message.data[0]='D';
         serial_current_message.data[1]='O';
         RS485_send_message(serial_current_message);
         delay_ms(10);           //************

         sep_id.dest_address = 0xC0;            //Cabin
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=4;
         current_message.data[0]='X';
         current_message.data[1]=0xD0;
         current_message.data[2]='D';          //'S' for first digit of numerator
         current_message.data[3]='O';
         can_send_message(current_message);
         delay_ms(10);
       }
      }
     else
     if(_set_blink_num=='B')
      {
        sep_id.source_address = node_address;
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
         sep_id.dest_address = 0xB0;           //Num-Floor
         ID = assemble_id(sep_id);
         current_message.id = ID;
         can_send_message(current_message);
         delay_ms(10);
         sep_id.dest_address = 0x10;            //numerator
         ID = assemble_id(sep_id);
         current_message.id = ID;
         can_send_message(current_message);
         delay_ms(10);
        }
       else
       if(Numtype=='s')
       {
         serial_current_message.address=0xD3;
         serial_current_message.length=2;
         serial_current_message.data[0]='D';
         serial_current_message.data[1]='B';
         RS485_send_message(serial_current_message);
         delay_ms(10);           //************

         sep_id.dest_address = 0xC0;            //Cabin
         ID = assemble_id(sep_id);
         current_message.id = ID;
         can_send_message(current_message);
         delay_ms(10);
       }
       else
       if(Numtype=='f')
       {
         serial_current_message.address=0xD0;
         serial_current_message.length=2;
         serial_current_message.data[0]='D';
         serial_current_message.data[1]='B';
         RS485_send_message(serial_current_message);
         delay_ms(10);           //************

         sep_id.dest_address = 0xC0;            //Cabin
         ID = assemble_id(sep_id);
         current_message.id = ID;
         current_message.datalen=4;
         current_message.data[0]='X';
         current_message.data[1]=0xD0;
         current_message.data[2]='D';          //'S' for first digit of numerator
         current_message.data[3]='B';
         can_send_message(current_message);
         delay_ms(10);
       }
      }
     _set_blink_num=0;
  }


////// Ethernet Monitoring Send flags

if(_send_door_status)
        {
          Send_door_stat();
         _send_door_status=0;
        }

if(_send_move_status)
        {
          Send_move_stat();
          _send_move_status=0;
        }

if(_send_requests_status_4)
        {
          Send_requests_stat_4(_send_requests_status_4);
          _send_requests_status_4=0;
        }

if(_send_requests_status_2)
        {
          Send_requests_stat_2(_send_requests_status_2);
          _send_requests_status_2=0;
        }

if(_send_requests_status_1)
        {
          Send_requests_stat_1(_send_requests_status_1);
          _send_requests_status_1=0;
        }

if(_reset_out_request)
        {
          Reset_out_request_stat(_reset_out_request);
          _reset_out_request=0;
        }

/* We can use flags for receiving and etc. here */

}
#include "90can128.h"
#include "can.h"
#include "defines.h"



//#define debug 1
extern unsigned int can_rxfull_mobs;
extern unsigned int request_data,Received_Add,command_type,command_data;
extern unsigned char rx_buffer1[RX_BUFFER_SIZE1],Ready,Address,LPC_error;
extern struct can_message last_rec_message;
void interpreter(void)
{
unsigned int inp1,inp2,inp3;
     unsigned char inp4,inp5,inp6,Eval;
     unsigned char rec_data0,rec_data1,rec_data2,rec_data3,rec_data4,rec_data5;

     unsigned int i,ReqType;
     unsigned long int ID;
     unsigned char IO_number = 0;
     unsigned char  date , month , year;
     struct protocol_id_structure_sep sep_id;
      //For test only Can be removed In Main Usage

      if(Ready)serial_interpreter();

       if(Ready2)
        {
          Ready2=0;
          Ready=1;
          for(inp4=0;inp4<RX_BUFFER_SIZE1;inp4++)rx_buffer1[inp4]=rx_buffer2[inp4];
        }

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
      //can_send_message(current_message);
      //   long_message_send_buff.ide=1;
      //   for(i=0;i<96;i++){
      //      long_message_send_buff.data[i]=0x31+i;
      //   }
      //   long_message_send_buff.datalen=96;
      //   send_long_message(2);

      while(can_rxfull_mobs)
      {
            message_detector();
            //Make ID TO REPLY to destinatin if it needs
            sep_id.source_address = node_address;
            sep_id.dest_address = Received_Add;
            sep_id.message_num = 0;
            sep_id.priority = 0;
            ID = assemble_id(sep_id);
            current_message.id = ID;

            rec_data0 = last_rec_message.data[0];
            rec_data1 = last_rec_message.data[1];
            rec_data2 = last_rec_message.data[2];
            rec_data3 = last_rec_message.data[3];
            rec_data4 = last_rec_message.data[4];
            rec_data5 = last_rec_message.data[5];

            if(Received_Add == 0xCF)                 //TEST BOARD for Setting TIME and DATE
                 {
                   if(rec_data0=='G' && rec_data1=='T' &&  rec_data2=='I' )  //Time
                     {
                        rtc_get_time(&inp4,&inp5,&inp6);
                        can_send_quick(Received_Add,7,0,'R','T','I',0,inp4,inp5,inp6,0);
                        continue;
                     }
                   if(rec_data0=='G' && rec_data1=='D' &&  rec_data2=='A' )  //Date
                     {
                        rtc_get_date( &inp4,&inp5,&inp6);
                        can_send_quick(Received_Add,7,0,'R','D','A',0,inp4,inp5,inp6,0);
                        continue;
                     }
                   if(rec_data0=='S' && rec_data1=='T' &&  rec_data2=='I')//time
                     {
                       if(Expired_date==0xFF)
                             rtc_set_time( rec_data4,
                                                 last_rec_message.data[5],
                                                 last_rec_message.data[6]);
                        continue;
                     }
                   if(rec_data0=='S' && rec_data1=='D' && rec_data2=='A')  //date
                     {
                       if(Expired_date==0xFF)
                            rtc_set_date( rec_data4,
                                                last_rec_message.data[5],
                                                last_rec_message.data[6]);
                        continue;
                     }

                   if(rec_data0=='P' && rec_data1=='S')
                    {
                           can_send_quick(Received_Add,3,5,'P','S',1,0,0,0,0,0);
                           continue;
                    }
                   if(rec_data0=='S' && rec_data1=='b' && rec_data2=='z')
                    {
                       if(rec_data3==1)BUZ=1;
                        else BUZ=0;

                    }
                   if(rec_data0=='S' && rec_data1=='a' && rec_data2=='o')
                    {
                      inp4=rec_data3;
                      if(inp4&0x01)SO1=1; else SO1=0;
                      if(inp4&0x02)SO2=1; else SO2=0;
                      if(inp4&0x04)SO3=1; else SO3=0;
                      if(inp4&0x08)SO4=1; else SO4=0;
                      if(inp4&0x10)SO5=1; else SO5=0;
                      if(inp4&0x20)SO6=1; else SO6=0;
                      if(inp4&0x40)SO7=1; else SO7=0;
                      if(inp4&0x80)SO8=1; else SO8=0;

                      inp4=rec_data4;
                      if(inp4&0x01)SO9=1; else SO9=0;
                      if(inp4&0x02)SO10=1; else SO10=0;


                    }
                   if(rec_data0=='G' && rec_data1=='a'
                       && rec_data2=='i')
                    {

                      current_message.datalen=8;
                      current_message.data[0]='R';
                      current_message.data[1]='a';
                      current_message.data[2]='i';

                      inp4=0x00;
                      if(!M_INT4)inp4|=0x80;
                      if(!M_INT3)inp4|=0x40;
                      if(!M_INT2)inp4|=0x20;
                      if(!M_INT1)inp4|=0x10;
                      if(HV_IN8) inp4|=0x01;
                      current_message.data[3]=inp4;

                      inp4=0x00;
                      if(M_IN1)inp4|=0x80;
                      if(M_IN2)inp4|=0x40;
                      if(M_IN3)inp4|=0x20;
                      if(M_IN4)inp4|=0x10;
                      if(M_IN5)inp4|=0x08;
                      if(M_IN6)inp4|=0x04;
                      if(M_IN7)inp4|=0x02;
                      if(M_IN8)inp4|=0x01;
                      current_message.data[4]=inp4;

                      inp4=0x00;
                      if(M_IN9)inp4|=0x80;
                      if(M_IN10)inp4|=0x40;
                      if(M_IN11)inp4|=0x20;
                      if(M_IN12)inp4|=0x10;
                      if(M_IN13)inp4|=0x08;
                      if(M_IN14)inp4|=0x04;
                      if(M_IN15)inp4|=0x02;
                      if(M_IN16)inp4|=0x01;
                      current_message.data[5]=inp4;

                      inp4=0x00;
                      if(M_IN17)inp4|=0x80;
                      if(M_IN18)inp4|=0x40;
                      if(M_IN19)inp4|=0x20;
                      if(M_IN20)inp4|=0x10;
                      if(M_IN21)inp4|=0x08;
                      if(M_IN22)inp4|=0x04;
                      if(M_IN23)inp4|=0x02;
                      if(M_IN24)inp4|=0x01;
                      current_message.data[6]=inp4;

                      inp4=0x00;
                      if(SS63)inp4|=0x80;
                      if(SS64)inp4|=0x40;
                      if(SS65)inp4|=0x20;
                      if(SS66)inp4|=0x10;
                      if(SS69)inp4|=0x08;
                      if(SS68)inp4|=0x04;
                      if(POWER110)inp4|=0x02;
                      if(POWER24)inp4|=0x01;
                      current_message.data[7]=inp4;

                      can_send_message(current_message);
                      delay_ms(5);
                      continue;

                    }

                 }

            if((Received_Add & 0xF0) == 0xC0)
                 {
                  //Data has been sent by a cabin board
                  //consist of:
                  //1- Inputs changing value
                  if(command_type=='C')
                  {
                    switch(command_data)
                     {
                      case  0 :
                           DOO   = 1 ;
                           _DOO_timeout=1;
                           break;
                      case  1 :
                           DOO   = 0 ;
                           _DOO_timeout=0;
                           break;
                      case  2 :
                           DOC   = 1 ;
                           break;
                      case  3 :
                           DOC   = 0 ;
                           break;
                      case  4 :
                           REVUC = 1 ;
                           #ifdef  REVC_TIMEOUT_BUTTON
                           revc_timeout = REVC_TIMEOUT_BUTTON;
                           #endif
                           break;
                      case  5 :
                           REVUC = 0 ;
                           break;
                      case  6 :
                           REVDC = 1 ;
                           #ifdef  REVC_TIMEOUT_BUTTON
                           revc_timeout = REVC_TIMEOUT_BUTTON;
                           #endif
                           break;
                      case  7 :
                           REVDC = 0 ;
                           break;
                      case  8 :
                           FUL   = 1 ;
                           break;
                      case  9 :
                           FUL   = 0 ;
                           break;
                      case 10 :
                           OVL   = 1 ;
                           _OVL_timeout=1;
                           break;
                      case 11 :
                           OVL   = 0 ;
                           break;
                      }
                   }
                   else
                   if(command_type=='E')
                    {

                    }
                   else
                   if(command_type=='c'&&request_data==0)
                   {
                     if(command_data>Floorsno || command_data==0)continue;
                     if(I)continue;
                     if(Car_Call_Erase_En!='y')continue;
                     IO_number = command_data;
                     inp4=pathfinder(IO_number,64);
                     if(!inp4)continue;
                     if(inp4==2)continue;  //no need to turn off leds
                     turnoffleds = IO_number;
                     continue;
                   }
                   else
                   if(command_type=='o')
                   {
                      IO_number = command_data;
                      if(Numtype!='s')continue;
                      if(IO_number>Floorsno)continue;
                      if((IO_number>9)||(IO_number<1))continue;

                      if((Navistrategy=='d')||(Navistrategy=='f'))
                      if(!pathfinder(IO_number,1))IO_number=0;
                             else  beep_request=1;   //Num-floor

                      if(Navistrategy=='s')IO_number=0;

                      if(IO_number)
                         {
                            if((Grouptype=='s')||(Eth_number==0))
                             {
                               serial_current_message.length=2;
                               serial_current_message.address=0xD3;
                               serial_current_message.data[0]='B';
                               serial_current_message.data[1]=IO_number;
                               RS485_send_message(serial_current_message);
                               delay_ms(5);
                             }
                               sep_id.source_address = node_address;
                               sep_id.dest_address = 0xC0;
                               sep_id.message_num = 0;
                               sep_id.priority = 0;
                               ID = assemble_id(sep_id);
                               current_message.id = ID;
                               current_message.datalen=2;
                               current_message.data[0]='o';
                               current_message.data[1]=IO_number;
                               can_send_message(current_message);
                               delay_ms(10);
                               IO_number=0;
                               continue;
                            }  //IO_number

                  }   //if(command_type=='o'
                  if(command_type=='X')
                  {
                      if(command_data==0xD1)  //data sent by IVA401
                        {
                           if(rec_data2=='V')
                                 {
                                    if(IVA_comm!='s')                //IVA is connected via CABIN
                                         {
                                            IVA_comm='s';
                                            EE_IVA_comm='s';
                                         }
                                    if(_volume_set_rs485==1)             //1 means request volume is sent by LPC
                                         {
                                          _volume_set_rs485=0;

                                           serial_current_message.length=5;
                                           serial_current_message.address=0xD2;              //send to LPC
                                           serial_current_message.data[0]='R';
                                           serial_current_message.data[1]='V';
                                           serial_current_message.data[2]='V';
                                           serial_current_message.data[3]=0x00;
                                           serial_current_message.data[4]=rec_data3;
                                           RS485_send_message(serial_current_message);
                                          }
                                     else
                                     if(_volume_set_rs485==0)             //0 means request volume is sent by LPC
                                         {
                                           sep_id.source_address = node_address;
                                           sep_id.dest_address = 0xA0;                    //send to HHP
                                           sep_id.message_num = 0;
                                           sep_id.priority = 0;
                                           ID = assemble_id(sep_id);
                                           current_message.id = ID;
                                           current_message.ide=1;
                                           current_message.rb=0;
                                           current_message.rtr=0;
                                           current_message.datalen=5;
                                           current_message.data[0]='R';
                                           current_message.data[1]='V';
                                           current_message.data[2]='V';
                                           current_message.data[3]=0;
                                           current_message.data[4]=rec_data3;
                                           can_send_message(current_message);
                                           delay_ms(10);
                                          }
                            }

                           continue;
                        } //if(command_data

                      if((command_data & 0xF0)!=0xE0)continue;
                      if(rec_data2!='O')continue;
                      if(Numtype!='f')continue;
                      IO_number = (command_data & 0x0F)*16 + rec_data3;
                      if(IO_number>Floorsno)continue;
                      if(IO_number<1)continue;

                      if(!pathfinder(IO_number,1))IO_number=0;
                             else  beep_request=1;   //Num-floor



                      if(IO_number)
                         {

                             if(Navistrategy!='s')
                             {
                                inp4 = IO_number%16;
                                if(!inp4)inp4=16;
                                if((Grouptype=='s')||(Eth_number==0))
                                  {
                                     serial_current_message.length=2;
                                     serial_current_message.address=(command_data & 0x0F) | 0xF0;
                                     serial_current_message.data[0]='B';
                                     serial_current_message.data[1]=inp4;
                                     RS485_send_message(serial_current_message);
                                     delay_ms(5);
                                  }
                              }
                               sep_id.source_address = node_address;
                               sep_id.dest_address = 0xC0;
                               sep_id.message_num = 0;
                               sep_id.priority = 0;
                               ID = assemble_id(sep_id);
                               current_message.id = ID;
                               current_message.datalen=4;
                               current_message.data[0]='X';
                               current_message.data[1]=command_data;        //rs485 st. address
                               current_message.data[2]='O';
                               current_message.data[3]=rec_data3;
                               can_send_message(current_message);
                               delay_ms(10);

                               IO_number=0;
                               continue;
                            }  //IO_number

                  }   //if(command_type=='X'
                  else continue;
             }//End of Cabin board
            else
            if(Received_Add == 0xD1)                               //Voice Baord
              {
                if(command_type=='V')
                      {
                        if(IVA_comm!='c')                //IVA is on CAN-BUS
                          {
                            IVA_comm='c';
                            EE_IVA_comm='c';
                          }
                        if(_volume_set_rs485==1)             //1 means request volume is sent by LPC
                          {
                               _volume_set_rs485=0;

                               serial_current_message.length=5;
                               serial_current_message.address=0xD2;              //send to LPC
                               serial_current_message.data[0]='R';
                               serial_current_message.data[1]='V';
                               serial_current_message.data[2]='V';
                               serial_current_message.data[3]=0x00;
                               serial_current_message.data[4]=command_data;
                               RS485_send_message(serial_current_message);
                          }
                        else
                        if(_volume_set_rs485==0)             //0 means request volume is sent by LPC
                        {
                           sep_id.source_address = node_address;
                           sep_id.dest_address = 0xA0;                    //send to HHP
                           sep_id.message_num = 0;
                           sep_id.priority = 0;
                           ID = assemble_id(sep_id);
                           current_message.id = ID;
                           current_message.ide=1;
                           current_message.rb=0;
                           current_message.rtr=0;
                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='V';
                           current_message.data[2]='V';
                           current_message.data[3]=0;
                           current_message.data[4]=command_data;
                           can_send_message(current_message);
                           delay_ms(10);
                        }
                       continue;
                     }
              }//End of Voice Board
            else
            if(Received_Add == 0xA0)            //Hand-held Programmer setting
            {

               if(command_type=='P')     //to announce for being Present
                {
                  if(rec_data1=='O')
                   {
                      if((rec_data2!=last_display_msg)||(rec_data2==102))
                       { current_message.datalen=2;
                         current_message.data[0]= 'P';
                         current_message.data[1]= last_display_msg;
                         if(last_display_msg==102)
                           {
                              if((rec_data3!=showfloor(floor,1))||
                                   (rec_data2!=102))
                                {
                                  current_message.datalen=4;
                                  current_message.data[2]=showfloor(floor,1);
                                  current_message.data[3]=showfloor(floor,2);
                                  can_send_message(current_message);
                                }
                              continue;
                           }
                         can_send_message(current_message);
                         continue;
                       }
                    }
                 }

               if(command_type=='G')     //To read a value
               {
                  if(rec_data1=='S' &&
                      rec_data2=='N' )   //SERIALNUMBER
                     {
                        sprintf(current_message.data,SERIAL_NUMBER);
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='H' &&
                      rec_data2=='V' )   //
                     {
                        sprintf(current_message.data,HARDWARE_VERSION);
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='S' &&
                      rec_data2=='V' )   //SOFTWAREVERSION
                     {
                        sprintf(current_message.data,SOFTWARE_VERSION);
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='P' &&
                      rec_data2=='M' )   //PANELMODEL
                     {
                        sprintf(current_message.data,PANEL_MODEL);
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='P' &&
                      rec_data2=='D' )   //PORODUCTDATE
                     {
                        sprintf(current_message.data,PORODUCT_DATE);
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='G' &&
                      rec_data2=='E' )   //GUARANTEEEXPIREDDATE
                     {
                        sprintf(current_message.data,GUARANTEE_EXPIRED_DATE);
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='M' &&
                      rec_data2=='M' )   //MOTOR MODEL
                     {
                        if(Lifttype=='2')sprintf(current_message.data,"*2 SPEED");
                        if(Lifttype=='v')sprintf(current_message.data,"*  3VF  ");
                        if(Lifttype=='h')sprintf(current_message.data,"*Hydraul");

                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='D' &&
                      rec_data2=='M' )   //DOORSMODEL
                     {
                        sprintf(current_message.data,"*ASR    ");
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='L' &&
                      rec_data2=='Y' )   //Lift Type
                     {
                        can_send_quick(Received_Add,5,0,'R','L','Y',0,Lifttype,0,0,0);
                        continue;
                     }
                  if(rec_data1=='E' &&
                      rec_data2=='N' )   //
                     {
                        inp1=EENumberoferrors;
                        inp2=EEWriteIndex;
                        inp1++;
                        inp2++;
                        if(!inp1||(inp1>50000))inp1=0;
                        if(!inp2||(inp2>200))inp1=0;
                        if(!EEWriteIndexOV)inp2=200;
                        current_message.datalen=4;
                        current_message.data[0]= 'R';
                        current_message.data[1]=inp1/256;
                        current_message.data[2]=inp1%256;
                        current_message.data[3]=inp2;
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='E' && rec_data2=='L' )   //Report Last Error
                     {
                        inp1=EEWriteIndex;
                        inp2=inp1+1;
                        if(!EEWriteIndexOV)inp1=200; else inp1++;      //total of saved errors
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        current_message.data[1]='C';
                        current_message.data[2]=0x00;
                        current_message.data[3]=ErrorHistory[inp2-1].ErrorNum;
                        current_message.data[4]=ErrorHistory[inp2-1].ErrorQty;
                        current_message.data[5]=inp1;
                        current_message.data[6]=inp2;
                        can_send_message(current_message);

                        //delay_ms(5);

                        current_message.data[0]= 'R';
                        current_message.data[1]='T';
                        current_message.data[2]=ErrorHistory[inp2-1].ErrorYear;
                        current_message.data[3]=ErrorHistory[inp2-1].ErrorMonth;
                        current_message.data[4]=ErrorHistory[inp2-1].ErrorDate;
                        current_message.data[5]=ErrorHistory[inp2-1].ErrorHour;
                        current_message.data[6]=ErrorHistory[inp2-1].ErrorMin;
                        current_message.data[7]=ErrorHistory[inp2-1].ErrorSec;
                        can_send_message(current_message);
                        continue;
                     }
                  if(rec_data1=='E' && rec_data2==0x00 )   //Error Read
                     {
                        inp1=EEWriteIndex;
                        if(!EEWriteIndexOV)inp1=200; else inp1++;      //total of saved errors
                        inp2=rec_data3;   //Index of Error
                        if((EEWriteIndexOV)&&(inp2>inp1))continue;
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        current_message.data[1]='C';
                        current_message.data[2]=0x00;
                        current_message.data[3]=ErrorHistory[inp2-1].ErrorNum;
                        current_message.data[4]=ErrorHistory[inp2-1].ErrorQty;
                        current_message.data[5]=inp1;
                        current_message.data[6]=inp2;
                        current_message.data[7]=0x00;
                        can_send_message(current_message);
                        //delay_ms(5);
                        current_message.datalen=8;
                        current_message.data[0]= 'R';
                        current_message.data[1]='T';
                        current_message.data[2]=ErrorHistory[inp2-1].ErrorYear;
                        current_message.data[3]=ErrorHistory[inp2-1].ErrorMonth;
                        current_message.data[4]=ErrorHistory[inp2-1].ErrorDate;
                        current_message.data[5]=ErrorHistory[inp2-1].ErrorHour;
                        current_message.data[6]=ErrorHistory[inp2-1].ErrorMin;
                        current_message.data[7]=ErrorHistory[inp2-1].ErrorSec;
                        can_send_message(current_message);
                        continue;
                     }

                   if(rec_data1=='M' && rec_data2=='F' )   //Maskfloor
                     {
                        inp4=floorvalidity(rec_data3);
                        can_send_quick(Received_Add,5,0,'R','M','F',rec_data3,inp4,0,0,0);
                        continue;
                     }

                   if(rec_data1=='b' && rec_data2=='M' )   //Maskfloor Byte
                     {
                        if(rec_data3=='C')can_send_quick(Received_Add,5,0,'R','b','M','C',EE_Maskfloor_Chksum,0,0,0);
                        if(rec_data3>='0' && rec_data3<='7')
                                can_send_quick(Received_Add,5,0,'R','b','M',rec_data3,EE_Maskfloor[rec_data3-'0'],0,0,0);
                        continue;
                     }


                   if(rec_data1=='H' && rec_data2=='f' )   //Halffloor
                     {
                        inp4=halffloorstatus(rec_data3);
                        can_send_quick(Received_Add,5,0,'R','H','f',rec_data3,inp4,0,0,0);
                        continue;
                     }
                   if(rec_data1=='b' && rec_data2=='H' )   //Halffloor Byte
                     {
                        if(rec_data3=='C')can_send_quick(Received_Add,5,0,'R','b','H','C',EE_HalfFloor_Chksum,0,0,0);
                        if(rec_data3>='0' && rec_data3<='7')
                                can_send_quick(Received_Add,5,0,'R','b','H',rec_data3,EE_HalfFloor[rec_data3-'0'],0,0,0);
                        continue;
                     }

                   if(rec_data1=='C' && rec_data2=='A' && rec_data3=='2' )   //CA1Second_En
                     {
                        can_send_quick(Received_Add,5,0,'R','C','A','2',EE_CA1Second_En,0,0,0);

                        continue;
                     }


                   if(rec_data1=='H' && rec_data2=='F' && rec_data3=='t' )   //HalffloorDelay
                     {
                        can_send_quick(Received_Add,5,0,'R','H','F','t',EE_HalffloorDelay,0,0,0);
                        continue;
                     }


                   if(rec_data1=='H' && rec_data2=='F' && rec_data3=='S' )   //HalffloorSpeed
                     {
                        can_send_quick(Received_Add,5,0,'R','H','F','S',EE_HalffloorSpeed,0,0,0);
                        continue;
                     }


                   if(rec_data1=='O' &&
                      rec_data2=='M' &&
                      rec_data3=='t' )   //OneMovefloorDelay
                     {
                        can_send_quick(Received_Add,5,0,'R','O','M','t',EE_OneMovefloorDelay,0,0,0);
                        continue;
                     }


                   if(rec_data1=='O' && rec_data2=='M' && rec_data3=='S' )   //OneMovefloorSpeed
                     {
                        can_send_quick(Received_Add,5,0,'R','O','M','S',EE_OneMovefloorSpeed,0,0,0);
                        continue;
                     }


                   if(rec_data1=='T' && rec_data2=='u' && rec_data3=='d' )   //tunnel door
                     {
                        inp4=Tunnel_door_find(rec_data4);
                        can_send_quick(Received_Add,6,0,'R','T','u','d',rec_data4,inp4,0,0);
                        continue;
                     }

                   if(rec_data1=='b' && rec_data2=='1' )   //FirstDoor Byte
                     {
                        if(rec_data3=='C')can_send_quick(Received_Add,5,0,'R','b','1','C',EE_FirstDoor_Chksum,0,0,0);
                        if(rec_data3>='0' && rec_data3<='7')
                                can_send_quick(Received_Add,5,0,'R','b','1',rec_data3,EE_FirstDoor[rec_data3-'0'],0,0,0);
                        continue;
                     }
                   if(rec_data1=='b' && rec_data2=='2' )   //SecondDoor Byte
                     {
                        if(rec_data3=='C')can_send_quick(Received_Add,5,0,'R','b','2','C',EE_SecondDoor_Chksum,0,0,0);
                        if(rec_data3>='0' && rec_data3<='7')
                                can_send_quick(Received_Add,5,0,'R','b','2',rec_data3,EE_SecondDoor[rec_data3-'0'],0,0,0);
                        continue;
                     }

                   if(rec_data1=='T' && rec_data2=='u' && rec_data3=='s' )   //tunnel door
                     {
                        inp4=EE_Tunnel_Door_en;
                        if(inp4=='n')inp5=0;
                        if(inp4=='s')inp5=1;
                        if(inp4=='2')inp5=2;
                        can_send_quick(Received_Add,5,0,'R','T','u','s',inp5,0,0,0);
                        continue;
                     }


                  if(rec_data1=='S' && rec_data2=='S' )   //Safety Serie
                     {
                        if(SS68)inp4='C';
                        else if(SS69)inp4=68;
                        else if(SS66)inp4=69;
                        else if(SS65)inp4=66;
                        else if(SS64)inp4=65;
                        else if(SS63)inp4=64;
                        else if(POWER110)inp4=63;
                        else inp4='N';

                        can_send_quick(Received_Add,2,0,'R',inp4,0,0,0,0,0,0);
                        continue;
                     }


                  if(rec_data1=='D' && rec_data2=='S' )   //SPEED & DIRECTION OF MOVEMONET
                     {

                        if(CONTU&&!CONTD)inp4='U';
                        else if(!CONTU&&CONTD)inp4='D';
                        else inp4=0;

                        if(CONTF&&!CONTS&&!CONTM&&!CONTL)inp5='F';
                        else if(!CONTF&&CONTS&&!CONTM&&!CONTL)inp5='S';
                        else if(!CONTF&&!CONTS&&CONTM&&!CONTL)inp5='M';
                        else if(!CONTF&&!CONTS&&!CONTM&&CONTL)inp5='L';
                        else inp5=0;
                        can_send_quick(Received_Add,3,0,'R',inp4,inp5,0,0,0,0,0);
                        continue;
                     }


                  if(rec_data1=='S' && rec_data2=='p' )
                     {
                        inp4=0;
                        inp5=0;
                        if(OVL)inp4|=0x01;
                        if(FUL)inp4|=0x02;
                        if(DOC)inp4|=0x04;
                        if(DOO)inp4|=0x08;
                        if(CAn)inp4|=0x10;
                        if(CA1)inp4|=0x20;
                        if(CF3)inp4|=0x40;
                        if(CF1)inp4|=0x80;
                        if(REVUC)inp5|=0x01;
                        if(REVDC)inp5|=0x02;
                        if(REVM)inp5|=0x04;
                        if(REVC)inp5|=0x08;
                        if(FEEDBS)inp5|=0x10;
                        if(FEEDBF)inp5|=0x20;
                        if(FEEDBD)inp5|=0x40;
                        if(FEEDBU)inp5|=0x80;
                        can_send_quick(Received_Add,5,0,'R','S','p',inp4,inp5,0,0,0);

                        continue;
                     }


                  if(rec_data1=='D' && rec_data2=='X' )   //SPEED & DIRECTION OF MOVEMONET
                     {

                        if(RELAYC&&!RELAYO&&!RELAYCed)inp4='C';
                        else if(RELAYO&&!RELAYC&&!RELAYOed)inp4='O';
                        else if(RELAYOed&&!RELAYCed)inp4='o';
                        else if(!RELAYOed&&RELAYCed)inp4='c';
                        else   inp4=0;
                        can_send_quick(Received_Add,2,0,'R',inp4,0,0,0,0,0,0);
                        continue;
                     }


                  if(rec_data1=='P' && rec_data2=='T' )   //PANEL TEMPERATURE
                     {
                        inp1=lm75_temperature_10(1);
                        can_send_quick(Received_Add,4,0,'R',inp1/100,(inp1/10)%10,inp1%10,0,0,0,0);
                        continue;
                     }


                   if(rec_data1=='N' && rec_data2=='O' && rec_data3=='S')//Cf1mt
                     {
                        inp1 = number_of_start / 256;
                        inp2 = number_of_start % 256;
                        can_send_quick(Received_Add,6,0,'R','N','O','S',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='F' && rec_data2=='n' )   //Floorsno
                     {
                        can_send_quick(Received_Add,5,0,'R','F','n',0,EE_Floorsno,0,0,0);
                        continue;
                     }


                   if(rec_data1=='N' && rec_data2=='S' )   //Navistrategy
                     {
                        can_send_quick(Received_Add,5,0,'R','N','S',0,EE_Navistrategy,0,0,0);
                        continue;
                     }


                   if(rec_data1=='F' && rec_data2=='F' )   //Firefloor
                     {
                        can_send_quick(Received_Add,5,0,'R','F','F',0,EE_Firefloor,0,0,0);
                        continue;
                     }


                   if(rec_data1=='P' &&
                      rec_data2=='F' )   //Parkfloor
                     {
                        can_send_quick(Received_Add,5,0,'R','P','F',0,EE_Parkfloor,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' && rec_data2=='O' && rec_data3=='t')   //Doorsiso(Door Stay in stair Open)      //zamane baz mandane dar
                     {
                        can_send_quick(Received_Add,5,0,'R','D','O','t',EE_Doorsiso,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' && rec_data2=='W' && rec_data3=='C') //Doorwtc     //Zamane baz mandane darbe lolaei
                     {
                        inp1 = EE_Doorwtc / 256;
                        inp2 = EE_Doorwtc % 256;
                        can_send_quick(Received_Add,6,0,'R','D','W','C',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='O' && rec_data2=='C' && rec_data3=='t')//Doordur  //Zamane baz ya baste shodane dar
                     {
                        inp1 = EE_Doordur / 256;
                        inp2 = EE_Doordur % 256;
                        can_send_quick(Received_Add,6,0,'R','O','C','t',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='S' && rec_data2=='d' && rec_data3=='t')//Doordur_open   //Zamane baz shodane jodaganeye dar
                     {
                        inp1 = EE_Doordur_open / 256;
                        inp2 = EE_Doordur_open % 256;
                        can_send_quick(Received_Add,6,0,'R','S','d','t',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='C' && rec_data2=='F' && rec_data3=='S')   //Calfs(Calibration Fast Slow Flag)
                     {
                        can_send_quick(Received_Add,5,0,'R','C','F','S',EE_Calfs,0,0,0);
                        continue;
                     }


                   if(rec_data1=='N' && rec_data2=='T' && rec_data3=='C')   //Needtocalibration
                     {
                        can_send_quick(Received_Add,5,0,'R','N','T','C',EE_Needtocalibration,0,0,0);
                        continue;
                     }


                   if(rec_data1=='F' && rec_data2=='S' && rec_data3=='F')   //Fsflag(First Second Flag)
                     {
                        can_send_quick(Received_Add,5,0,'R','F','S','F',EE_Fsflag,0,0,0);
                        continue;
                     }


                   if(rec_data1=='O' && rec_data2=='F' && rec_data3==0)   //Oneflag(First Second Flag)
                     {
                        can_send_quick(Received_Add,5,0,'R','O','F',0,EE_Oneflag,0,0,0);
                        continue;
                     }


                   if(rec_data1=='C' && rec_data2=='t')  //Calmt
                     {
                        inp1 = EE_Calmt / 256;
                        inp2 = EE_Calmt % 256;
                        can_send_quick(Received_Add,6,0,'R','C','t',0,inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='P' && rec_data2=='t') //Timepark
                     {
                        inp1 = EE_Timepark / 256;
                        inp2 = EE_Timepark % 256;
                        can_send_quick(Received_Add,6,0,'R','P','t',0,inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='S' && rec_data2=='B' &&  rec_data3=='t')//Timestndby
                     {
                        inp1 = EE_Timestndby / 256;
                        inp2 = EE_Timestndby % 256;
                        can_send_quick(Received_Add,6,0,'R','S','B','t',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='2' && rec_data2=='C' && rec_data3=='t')  //Time2cf3
                     {
                        inp1 = EE_Time2cf3 / 256;
                        inp2 = EE_Time2cf3 % 256;
                        can_send_quick(Received_Add,6,0,'R','2','C','t',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='1' && rec_data2=='C' && rec_data3=='t')//Time1cf3
                     {
                        inp1 = EE_Time1cf3 / 256;
                        inp2 = EE_Time1cf3 % 256;
                        can_send_quick(Received_Add,6,0,'R','1','C','t',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='1' &&  rec_data2=='F' && rec_data3=='t')//Cf1mt
                     {
                        inp1 = EE_Cf1mt / 256;
                        inp2 = EE_Cf1mt % 256;
                        can_send_quick(Received_Add,6,0,'R','1','F','t',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='C' && rec_data2=='O' && rec_data3=='t')//Cont_overlap
                     {
                        inp1 = EE_Cont_overlap;
                        can_send_quick(Received_Add,5,0,'R','C','O','t',inp1,0,0,0);
                        continue;
                     }


                   if(rec_data1=='B' && rec_data2=='D' && rec_data3=='t')//Timebreakdelay
                     {
                        inp1 = EE_Timebreakdelay;
                        can_send_quick(Received_Add,5,0,'R','B','D','t',inp1,0,0,0);
                        continue;
                     }


                   if(rec_data1=='T' && rec_data2=='R' && rec_data3=='t')//Timetravel
                     {
                        inp3 = EE_Timetravel ;  //unit of Traveltime is 0.5 sec
                        inp3 /= 10;
                        inp1 = inp3 / 256;
                        inp2 = inp3 % 256;
                        can_send_quick(Received_Add,6,0,'R','T','R','t',inp1,inp2,0,0);
                        continue;
                     }


                   if(rec_data1=='C' && rec_data2=='U' && rec_data3=='t') //Contofftu
                     {
                        can_send_quick(Received_Add,5,0,'R','C','U','t',EE_Contofftu,0,0,0);
                        continue;
                     }


                   if(rec_data1=='C' && rec_data2=='D' && rec_data3=='t')//Contofftd
                     {
                        can_send_quick(Received_Add,5,0,'R','C','D','t',EE_Contofftd,0,0,0);
                        continue;
                     }


                   if(rec_data1=='R' && rec_data2=='M' )  //Response Mode
                     {
                        can_send_quick(Received_Add,5,0,'R','R','M',0,EE_Navistrategy,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' &&rec_data2=='Y')   //doortype
                     {
                        can_send_quick(Received_Add,5,0,'R','D','Y',0,EE_Doortype,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' && rec_data2=='y' && rec_data3=='2' )   //door2type
                     {
                        can_send_quick(Received_Add,5,0,'R','D','y','2',EE_Door2type,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' &&rec_data2=='N' && rec_data3=='R')   //Doornrc
                     {
                        can_send_quick(Received_Add,5,0,'R','D','N','R',EE_Doornrc,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' &&rec_data2=='C' && rec_data3=='P')   //Doorclosepark
                     {
                        can_send_quick(Received_Add,5,0,'R','D','C','P',EE_Doorclosepark,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' &&rec_data2=='U' && rec_data3=='L')   //Doorunderl
                     {
                        can_send_quick(Received_Add,5,0,'R','D','U','L',EE_Doorunderl,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' &&rec_data2=='P' && rec_data3=='U')   //Doorparkunderl
                     {
                        can_send_quick(Received_Add,5,0,'R','D','P','U',EE_Doorparkunderl,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' &&rec_data2=='D' && rec_data3=='U')   //Dooruldelay
                     {
                        can_send_quick(Received_Add,5,0,'R','D','D','U',EE_Dooruldelay,0,0,0);
                        continue;
                     }


                   if(rec_data1=='D' &&rec_data2=='D' && rec_data3=='t')   //Doorcdebouncet
                     {
                        can_send_quick(Received_Add,5,0,'R','D','D','t',EE_Doorcdebouncet,0,0,0);
                        continue;
                     }


                   if(rec_data1=='G' && rec_data2=='X' )  //Groundexist
                     {
                        can_send_quick(Received_Add,5,0,'R','G','X',0,EE_Groundexist,0,0,0);
                        continue;
                     }


                   if(rec_data1=='P' && rec_data2=='S' )  //Parksign
                     {
                        can_send_quick(Received_Add,5,0,'R','P','S',0,EE_Parksign,0,0,0);
                        continue;
                     }


                   if(rec_data1=='U' && rec_data2=='N' )  //Undergroundno
                     {
                        can_send_quick(Received_Add,5,0,'R','U','N',0,EE_Undergroundno,0,0,0);
                        continue;
                     }


                   if(rec_data1=='V' && rec_data2=='V' )  //volume
                     {
                        if(IVA_comm=='c')
                        {
                        can_send_quick(0xD1,2,0,'G','V',0,0,0,0,0,0);
                        }
                       else
                       if(IVA_comm=='p')
                        {
                         serial_current_message.length=2;
                         serial_current_message.address=0xD1;
                         serial_current_message.data[0]='G';
                         serial_current_message.data[1]='V';
                         RS485_send_message(serial_current_message);
                        }
                       else
                        if(IVA_comm=='s')
                        {
                        can_send_quick(0xC0,4,0,'X',0xD1,'G','V',0,0,0,0);
                        }
                       _volume_set_rs485=0;                            //0 means volume request is sent by HHP
                       continue;
                     }
                   if(rec_data1=='p' && rec_data2=='n')  //Phone Nmber
                     {
                       serial_current_message.length=1;
                       serial_current_message.address=0xD4;
                       serial_current_message.data[0]='n';
                       serial_current_message.data[1]=0;
                       serial_current_message.data[2]=0;
                       serial_current_message.data[3]=0;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='E' &&  rec_data3=='I')  //error reading rescue
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='E';
                       serial_current_message.data[1]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='I' &&  rec_data3=='M')  //rescue monitoring
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='I';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='R' &&  rec_data3=='T')  //BLACKOUTTIME RESCUE
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='R';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' &&  rec_data2=='L' &&  rec_data3=='P')  //low current protection rescue
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='P';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' && rec_data2=='B' &&  rec_data3=='Q')  //rescue BOOSTTORQUE
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='Q';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='B' &&  rec_data3=='B')  //RESCUE BOOST TIME
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='B';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='M' &&  rec_data3=='T')  //rescue MOTOR TORQUE
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='M';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='T' &&  rec_data3=='T')  //rescue TIME
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='T';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='L' &&  rec_data3=='S')  //rescue LS OVERLAY
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='L';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='M' &&  rec_data3=='C')  //rescue MOTOR CURRENT
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='C';
                       serial_current_message.data[2]=0;
                       serial_current_message.data[3]=0;
                       serial_current_message.data[4]=0x00;
                       serial_current_message.data[5]=0x00;
                       serial_current_message.data[6]=0x00;
                       serial_current_message.data[7]=0x00;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='D' &&  rec_data3=='K')  //rescue DC brake torque
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='K';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' && rec_data2=='R' &&  rec_data3=='F')  //rescue fault in enable
                     {
                       serial_current_message.length=2;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='F';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='S' && rec_data2=='M' &&  rec_data3=='l')  //SMS_Level
                     {
                        current_message.datalen=5;
                        current_message.data[0]='R';
                        current_message.data[1]='S';
                        current_message.data[2]='M';
                        current_message.data[3]='l';
                        current_message.data[4]=EE_SMS_Level;
                        can_send_message(current_message);
                        continue;
                     }
                   if(rec_data1=='F' && rec_data2=='N' &&  rec_data3=='c')  //FNC
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='F';
                       serial_current_message.data[2]='N';
                       serial_current_message.data[3]='c';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='F' && rec_data2=='s' &&  rec_data3=='c')  //FSC
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='F';
                       serial_current_message.data[2]='s';
                       serial_current_message.data[3]='c';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='S' && rec_data2=='n' &&  rec_data3=='c')  //SNC
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='S';
                       serial_current_message.data[2]='n';
                       serial_current_message.data[3]='c';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='S' &&
                      rec_data2=='s' &&  rec_data3=='c')  //SSC
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='S';
                       serial_current_message.data[2]='s';
                       serial_current_message.data[3]='c';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='D' &&
                      rec_data2=='e' &&  rec_data3=='l')  //DEL
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='D';
                       serial_current_message.data[2]='e';
                       serial_current_message.data[3]='l';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='P' &&
                      rec_data2=='h' &&  rec_data3=='e')  //PHE
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='P';
                       serial_current_message.data[2]='h';
                       serial_current_message.data[3]='e';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='U' &&
                      rec_data2=='c' &&  rec_data3=='p')  //UCP
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='U';
                       serial_current_message.data[2]='c';
                       serial_current_message.data[3]='p';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='L' &&
                      rec_data2=='e' &&  rec_data3=='c')  //LEC
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='G';
                       serial_current_message.data[1]='L';
                       serial_current_message.data[2]='e';
                       serial_current_message.data[3]='c';
                       RS485_send_message(serial_current_message);
                        continue;
                     }


                   if(rec_data1=='E' && rec_data2=='t' && rec_data3=='h' )  //Eth_number
                     {
                        can_send_quick(Received_Add,5,0,'R','E','t','h',EE_Eth_number,0,0,0);
                        continue;
                     }



                   if(rec_data1=='G' && rec_data2=='t' && rec_data3=='y' )  //Group type
                     {
                        can_send_quick(Received_Add,5,0,'R','G','t','y',EE_Grouptype,0,0,0);
                        continue;
                     }


                   if(rec_data1=='V' && rec_data2=='d' && rec_data3=='c' )  //Expired_date
                     {
                       inp4=EE_Expired_date;
                       if((inp4==0xFF)||(inp4==0xFA))
                        {
                          if(inp4==0xFA)inp4=0xFE;              //for interface with HHP
                          can_send_quick(Received_Add,5,0,'R','V','d','c',inp4,0,0,0);
                          continue;
                        }

                        inp5=EE_Expired_month;

                         if(inp5>0)inp1=inp4;
                         if(inp5>1)inp1+=31;
                         if(inp5>2)inp1+=28;
                         if(inp5>3)inp1+=31;
                         if(inp5>4)inp1+=30;
                         if(inp5>5)inp1+=31;
                         if(inp5>6)inp1+=30;
                         if(inp5>7)inp1+=31;
                         if(inp5>8)inp1+=31;
                         if(inp5>9)inp1+=30;
                         if(inp5>10)inp1+=31;
                         if(inp5>11)inp1+=30;

                       rtc_get_date(&date,&month,&year);
                       if((month==0)||(month>12)||(date>31)||(date==0))           //invalid date
                            {
                                can_send_quick(Received_Add,6,0,'R','V','d','c',0,'i',0,0);
                                continue;
                            }

                       if(month>0)inp2=date;
                       if(month>1)inp2+=31;
                       if(month>2)inp2+=28;
                       if(month>3)inp2+=31;
                       if(month>4)inp2+=30;
                       if(month>5)inp2+=31;
                       if(month>6)inp2+=30;
                       if(month>7)inp2+=31;
                       if(month>8)inp2+=31;
                       if(month>9)inp2+=30;
                       if(month>10)inp2+=31;
                       if(month>11)inp2+=30;           //current date in inp2

                       if(inp1>=inp2)inp1=inp1-inp2;
                         else inp1=(365-inp2)+inp1;

                                can_send_quick(Received_Add,5,0,'R','V','d','c',inp1,0,0,0);
                                continue;

                     }
                   if(rec_data1=='M' && rec_data2=='o' && rec_data3=='n' )  //Monitoring
                     {
                        can_send_quick(Received_Add,6,0,'R','M','o','n',EE_Gateway_lsb,EE_Monitoring,0,0);
                        continue;
                     }


                   if(rec_data1=='P' &&
                      rec_data2=='W' )  //Passsword
                     {
                        can_send_quick(Received_Add,8,0,'R','P','W',EE_Password[0],EE_Password[1],EE_Password[2],EE_Password[3],EE_Password[4]);
                        continue;
                     }


                   if(rec_data1=='m' &&
                      rec_data2=='P' )  //Passsword  2
                     {
                        can_send_quick(Received_Add,8,0,'R','m','P','1','3','5','7','9');
                        continue;
                     }


                   if(rec_data1=='T' && rec_data2=='I' )  //Time
                     {
                        rtc_get_time(&inp4,&inp5,&inp6);
                        can_send_quick(Received_Add,7,0,'R','T','I',0,inp4,inp5,inp6,0);
                        continue;
                     }


                   if(rec_data1=='D' &&
                      rec_data2=='A' )  //Date
                     {
                        rtc_get_date(&inp4,&inp5,&inp6);
                        can_send_quick(Received_Add,7,0,'R','D','A',0,inp4,inp5,inp6,0);
                        continue;
                     }


                   if(rec_data1=='h' && rec_data2=='y'  && rec_data3=='s' )  //hydraulic_start_time
                     {
                        can_send_quick(Received_Add,5,0,'R','h','y','s',EE_Hyd_Start_Time,0,0,0);
                        continue;
                     }


                   if(rec_data1=='h' && rec_data2=='y'  && rec_data3=='p' )  //hydraulic_stop_time
                     {
                        can_send_quick(Received_Add,5,0,'R','h','y','p',EE_Hyd_Stop_Time,0,0,0);
                        continue;
                     }


                   if(rec_data1=='h' && rec_data2=='y'  && rec_data3=='d' )  //hydraulic_S@D_time
                     {
                        can_send_quick(Received_Add,5,0,'R','h','y','d',EE_Hyd_S2D_Time,0,0,0);
                        continue;
                     }


                   if(rec_data1=='f' && rec_data2=='A'  && rec_data3=='d' )  //floor_announce_delay
                     {
                        can_send_quick(Received_Add,5,0,'R','f','A','d',EE_floor_announce_delay,0,0,0);
                        continue;
                     }


                   if(rec_data1=='M' && rec_data2=='P'  && rec_data3=='E' )  //Music_Play_En
                     {
                        can_send_quick(Received_Add,5,0,'R','M','P','E',EE_Music_Play_En,0,0,0);
                        continue;
                     }


                   if(rec_data1=='A' && rec_data2=='d'  && rec_data3=='e' )  //Preopening enable
                     {
                        can_send_quick(Received_Add,5,0,'R','A','d','e',EE_Preopening_en,0,0,0);
                        continue;
                     }


                   if(rec_data1=='C' && rec_data2=='C'  && rec_data3=='C' )  //CAR CALL ERASE
                     {
                        can_send_quick(Received_Add,5,0,'R','C','C','C',EE_Car_Call_Erase_En,0,0,0);
                        continue;
                     }

                   if(rec_data1=='M' && rec_data2=='f'  && rec_data3=='N' )  //manual_floor_naming
                     {
                        can_send_quick(Received_Add,5,0,'R','M','f','N',EE_manual_floor_naming,0,0,0);
                        continue;
                     }

                   if(rec_data1=='M' && rec_data2=='f'  && rec_data3=='C' )  //Floor_names_1
                     {
                        can_send_quick(Received_Add,7,0,'R','M','f','C',rec_data4,EE_Floor_names_1[rec_data4],EE_Floor_names_2[rec_data4],0);
                        continue;
                     }


                }

                           //******************************************************
               if(command_type=='S')     //To Set a value
                {
                   if(rec_data1=='C' && rec_data2=='A'  && rec_data3=='2' )  //CA1Second_En enable
                     {
                        EE_CA1Second_En =   rec_data4;
                        send_save_ok_ack(3);
                        continue;

                     }
                   if(rec_data1=='H' && rec_data2=='F'  && rec_data3=='t' )  //HalffloorDelay
                     {
                        EE_HalffloorDelay =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='H' && rec_data2=='F'  && rec_data3=='S' )  //HalffloorSpeed
                     {
                        EE_HalffloorSpeed =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='b' && rec_data2=='H' )   //Halffloor Byte
                     {
                        if(rec_data3=='C')EE_HalfFloor_Chksum=rec_data4;
                        if(rec_data3>='0' && rec_data3<='7'){EE_HalfFloor[rec_data3-'0']=rec_data4;HalfFloor[rec_data3-'0']=rec_data4;}
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='b' && rec_data2=='M' )   //Maskfloor Byte
                     {
                        if(rec_data3=='C')EE_Maskfloor_Chksum=rec_data4;
                        if(rec_data3>='0' && rec_data3<='7'){EE_Maskfloor[rec_data3-'0']=rec_data4;Maskfloor[rec_data3-'0']=rec_data4;}
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='b' && rec_data2=='1' )   //FirstDoor Byte
                     {
                        if(rec_data3=='C')EE_FirstDoor_Chksum=rec_data4;
                        if(rec_data3>='0' && rec_data3<='7'){EE_FirstDoor[rec_data3-'0']=rec_data4;FirstDoor[rec_data3-'0']=rec_data4; }
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='b' && rec_data2=='2' )   //SecondDoor Byte
                     {
                        if(rec_data3=='C')EE_SecondDoor_Chksum=rec_data4;
                        if(rec_data3>='0' && rec_data3<='7'){EE_SecondDoor[rec_data3-'0']=rec_data4;SecondDoor[rec_data3-'0']=rec_data4;}
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='O' &&
                       rec_data2=='M'  &&
                       rec_data3=='t' )  //OneMovefloorDelay
                     {
                        EE_OneMovefloorDelay =   rec_data4;
                        can_send_quick(Received_Add,5,0,'S','O','M','t',1,0,0,0);
                        continue;
                     }

                   if(rec_data1=='O' && rec_data2=='M'  && rec_data3=='S' )  //OneMovefloorSpeed
                     {
                        EE_OneMovefloorSpeed =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='A' && rec_data2=='d'  &&  rec_data3=='e' )  //Preopening enable
                     {
                        EE_Preopening_en =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='f' && rec_data2=='A'  && rec_data3=='d' )  //EE_floor_announce_delay
                     {
                        EE_floor_announce_delay =   rec_data4;
                        floor_announce_delay =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='M' && rec_data2=='P'  && rec_data3=='E' )  //Music_Play_En
                     {
                        EE_Music_Play_En =   rec_data4;
                        Music_Play_En =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='h' &&  rec_data2=='y'  && rec_data3=='d' )  //hydraulic_S@D_time
                     {
                        EE_Hyd_S2D_Time =   rec_data4;
                        Hyd_S2D_Time =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='h' &&  rec_data2=='y'  && rec_data3=='s' )  //hydraulic_Start_time
                     {
                        EE_Hyd_Start_Time =   rec_data4;
                        Hyd_Start_Time =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='h' &&  rec_data2=='y'  &&  rec_data3=='p' )  //hydraulic_Stop_time
                     {
                        EE_Hyd_Stop_Time =   rec_data4;
                        Hyd_Stop_Time =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='F' &&  rec_data2=='n')              //Floorsno
                     {
                        EE_Floorsno = rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='N' &&   rec_data2=='S')                //Navistrategy
                     {
                        EE_Navistrategy = rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='F' && rec_data2=='F')          //Firefloor
                     {
                        EE_Firefloor = rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='P' && rec_data2=='F')             //Parkfloor
                     {
                        EE_Parkfloor = rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='D' && rec_data2=='O' &&  rec_data3=='t')//Doorsiso
                     {
                        EE_Doorsiso = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='D' && rec_data2=='W' && rec_data3=='C')//Doorwtc
                     {

                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        EE_Doorwtc = inp1*256 + inp2;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='O' && rec_data2=='C' && rec_data3=='t') //Doordur
                     {

                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        EE_Doordur = inp1*256 + inp2;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='S' &&  rec_data2=='d' &&   rec_data3=='t') //Doordur_open
                     {

                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        EE_Doordur_open = inp1*256 + inp2;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='C' &&  rec_data2=='t')
                     {
                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];    //Calmt
                        EE_Calmt = inp1*256 + inp2;
                        send_save_ok_ack(2);
                        continue;
                    }

                   if(rec_data1=='O' && rec_data2=='F' &&  rec_data3==0 )
                     {
                        EE_Oneflag = rec_data4; //Oneflag
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='F' && rec_data2=='S' && rec_data3=='F' )
                     {
                        EE_Fsflag = rec_data4; //Fsflag
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='C' &&  rec_data2=='F' &&  rec_data3=='S' )
                     {
                        EE_Calfs = rec_data4; //Calfs
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='N' &&  rec_data2=='T' &&   rec_data3=='C' )
                     {
                        EE_Needtocalibration = rec_data4; //Needtocalibration
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='C' && rec_data2=='C' && rec_data3=='C' )
                     {
                        EE_Car_Call_Erase_En = rec_data4; //Car_Call_Erase_En
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='M' && rec_data2=='f' && rec_data3=='N' )
                     {
                        EE_manual_floor_naming = rec_data4; //manual_floor_naming
                        send_save_ok_ack(3);

                        continue;
                     }

                   if(rec_data1=='M' && rec_data2=='f' && rec_data3=='C' )
                     {
                        if(rec_data4<1 || rec_data4>64)continue;
                        EE_Floor_names_1[rec_data4] = last_rec_message.data[5]; //Floor_names_1
                        EE_Floor_names_2[rec_data4] = last_rec_message.data[6];
                        send_save_ok_ack(3);
                        continue;
                     }



                   if(rec_data1=='P' &&  rec_data2=='t')
                     {

                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        EE_Timepark = inp1*256 + inp2;        //Timepark
                        send_save_ok_ack(2);
                        continue;
                   }

                   if(rec_data1=='D' && rec_data2=='Y') //doortype
                     {
                        EE_Doortype = rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='D' && rec_data2=='y' && rec_data3=='2') //door2type
                     {
                        EE_Door2type = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='D' &&  rec_data2=='N' && rec_data3=='R')  //dooornrc
                     {
                        EE_Doornrc = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='D' && rec_data2=='C' && rec_data3=='P')  //Doorclosepark
                     {
                        EE_Doorclosepark = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='D' && rec_data2=='U' && rec_data3=='L')  //Doorunderl
                     {
                        EE_Doorunderl = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='D' && rec_data2=='P' && rec_data3=='U')  //Doorparkunderl
                     {
                        EE_Doorparkunderl = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='D' && rec_data2=='D' && rec_data3=='U')  //Dooruldelay
                     {
                        EE_Dooruldelay = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='D' && rec_data2=='D' && rec_data3=='t')  //Doorcdebouncet
                     {
                        EE_Doorcdebouncet = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='S' && rec_data2=='B' && rec_data3=='t')
                     {
                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        EE_Timestndby = inp1*256 + inp2;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='1' && rec_data2=='C' && rec_data3=='t')
                     {

                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        EE_Time1cf3 = inp1*256 + inp2;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='2' &&  rec_data2=='C' && rec_data3=='t')
                     {

                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        EE_Time2cf3 = inp1*256 + inp2;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='1' && rec_data2=='F' &&  rec_data3=='t')
                     {

                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        EE_Cf1mt = inp1*256 + inp2;
                        send_save_ok_ack(3);
                        continue;
                     }

                   if(rec_data1=='T' && rec_data2=='R' && rec_data3=='t')
                     {
                        inp1 = rec_data4;
                        inp2 = last_rec_message.data[5];
                        inp3 = inp1*256 + inp2;
                        inp3 *= 10;
                        EE_Timetravel = inp3;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='C' && rec_data2=='O' && rec_data3=='t') //Cont_overlap
                     {
                        EE_Cont_overlap = rec_data4;
                        Cont_overlap = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                  if(rec_data1=='B' && rec_data2=='D' && rec_data3=='t') //Timebreakdelay
                     {
                        EE_Timebreakdelay = rec_data4;
                        Timebreakdelay = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='C' && rec_data2=='U' && rec_data3=='t') //Contofftu
                     {
                        EE_Contofftu = rec_data4;
                        Contofftu = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='C' &&  rec_data2=='D' && rec_data3=='t')  //Contofftd
                     {
                        EE_Contofftd = rec_data4;
                        Contofftd = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='h' &&   rec_data2=='y'  &&  rec_data3=='d' )  //hydraulic_S@D_time
                     {
                        EE_Hyd_S2D_Time =   rec_data4;
                        Hyd_S2D_Time =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='h' &&  rec_data2=='y'  && rec_data3=='s' )  //hydraulic_Start_time
                     {
                        EE_Hyd_Start_Time =   rec_data4;
                        Hyd_Start_Time =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='h' && rec_data2=='y' && rec_data3=='p' )  //hydraulic_Stop_time
                     {
                        EE_Hyd_Stop_Time =   rec_data4;
                        Hyd_Stop_Time =   rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='P' && rec_data2=='W')        //passsword
                     {
                       if(Expired_date==0xFF)
                       {
                         EE_Password[0] = rec_data3;
                         EE_Password[1] = rec_data4;
                         EE_Password[2] = last_rec_message.data[5];
                         EE_Password[3] = last_rec_message.data[6];
                         EE_Password[4] = last_rec_message.data[7];
                         send_save_ok_ack(2);
                       }
                       else  can_send_quick(Received_Add,5,0,'S','P','W',0,2,0,0,0);          //not sent OK


                        continue;
                     }
                   if(rec_data1=='R' &&  rec_data2=='M')
                     {
                        EE_Navistrategy = rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='G' && rec_data2=='X')
                     {
                        EE_Groundexist = rec_data4;
                        Groundexist=rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='P' && rec_data2=='S')
                     {
                        EE_Parksign = rec_data4;
                        Parksign=rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='S' && rec_data2=='M' && rec_data3=='l' )
                     {
                        EE_SMS_Level = rec_data4;
                        SMS_Level =rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }

                    if(rec_data1=='U' && rec_data2=='N')
                     {
                        EE_Undergroundno = rec_data4;
                        Undergroundno=rec_data4;
                        send_save_ok_ack(2);
                        continue;
                     }

                    if(rec_data1=='V' &&  rec_data2=='d'  &&  rec_data3=='c' )
                     {
                        inp4 = rec_data4;
                        if(inp4==0xFF)
                           {EE_Expired_date=0xFF;
                                Expired_date=0xFF;
                            EE_Expired_month=0xFF;
                                Expired_month=0xFF;
                            EE_Start_dayofyear=0xFFFF;
                               Start_dayofyear=0xFFFF;
                            EE_Block_date_en='n';
                                can_send_quick(Received_Add,7,0,'S','V','d','c',1,0xFF,0xFF,0);
                                continue;}
                        else
                        if(inp4>200)continue;
                        rtc_get_date(&date,&month,&year);

                         if((month==0)||(month>12)||(date>31)||(date==0))           //invalid date
                            {
                                can_send_quick(Received_Add,5,0,'S','V','d','c','i',0,0,0);
                                continue;
                            }

                         Start_dayofyear=calcdayofyear(date,month);

              /*           if(month>0)inp1=date;
                         if(month>1)inp1+=31;
                         if(month>2)inp1+=28;
                         if(month>3)inp1+=31;
                         if(month>4)inp1+=30;
                         if(month>5)inp1+=31;
                         if(month>6)inp1+=30;
                         if(month>7)inp1+=31;
                         if(month>8)inp1+=31;
                         if(month>9)inp1+=30;
                         if(month>10)inp1+=31;
                         if(month>11)inp1+=30; */

                         if(Start_dayofyear>365)Start_dayofyear=365;

                         inp1 = Start_dayofyear + inp4;
                         if(inp1>365)
                             {inp1-=365;
                              year++;}   //sent year

                         while(1)
                          {
                             if(inp1<=31)
                               {date= inp1;
                                month=1;          //in jan
                                break;}
                             inp1-=31;
                             if(inp1<=28)
                               {date= inp1;
                                month=2;           //in feb
                                break;}
                             inp1-=28;
                             if(inp1<=31)
                               {date= inp1;            //in mar
                                month=3;
                                break;}
                             inp1-=31;
                             if(inp1<=30)             //in apr
                               {date= inp1;
                                month=4;
                                break;}
                             inp1-=30;
                             if(inp1<=31)
                               {date= inp1;             //in may
                                month=5;
                                break;}
                             inp1-=31;
                             if(inp1<=30)
                               {date= inp1;              //in jun
                                month=6;
                                break;}
                             inp1-=30;
                             if(inp1<=31)               //in jul
                               {date= inp1;
                                month=7;
                                break;}
                             inp1-=31;
                             if(inp1<=31)                //in aug
                               {date= inp1;
                                month=8;
                                break;}
                             inp1-=31;
                             if(inp1<=30)
                               {date= inp1;               //in sep
                                month=9;
                                break;}
                             inp1-=30;
                             if(inp1<=31)
                               {date= inp1;
                                month=10;             //in oct
                                break;}
                             inp1-=31;
                             if(inp1<=30)
                               {date= inp1;
                                month=11;
                                break;}
                             inp1-=30;
                             if(inp1<=31)
                               {date= inp1;
                                month=12;
                                break;}

                             month=12;
                             date=31;
                             break;
                          }
                       EE_Expired_date=date;
                       Expired_date=date;
                       EE_Expired_month=month;
                       Expired_month=month;
                       EE_Start_dayofyear=Start_dayofyear;
                       EE_Block_date_en='y';

                        can_send_quick(Received_Add,8,0,'S','V','d','c',1,date,month,year);
                       continue;
                      }

                    if(rec_data1=='V' &&  rec_data2=='V' )  //volume
                     {
                        EE_Volume = rec_data4;
                        if(IVA_comm=='c')
                        {
                         can_send_quick(0xD1,2,10,'V',rec_data4,0,0,0,0,0,0);
                         can_send_quick(0xD1,1,0,'D',0,0,0,0,0,0,0);                   //ding
                        }
                       else
                       if(IVA_comm=='s')
                        {
                         //send volume and ding in CAN format via CABIN board to IVA401
                         can_send_quick(0xC0,4,10,'X',0xD1,'V',rec_data4,0,0,0,0);
                         can_send_quick(0xC0,3,0,'X',0xD1,'D',0,0,0,0,0);          //ding
                        }
                       else
                       if(IVA_comm=='p')
                        {
                          //send volume and ding in RS485 to IVA401
                         serial_current_message.length=2;
                         serial_current_message.address=0xD1;
                         serial_current_message.data[0]='V';
                         serial_current_message.data[1]=rec_data4;
                         RS485_send_message(serial_current_message);
                         delay_ms(20);
                         serial_current_message.length=1;
                         serial_current_message.address=0xD1;
                         serial_current_message.data[0]='D';
                         RS485_send_message(serial_current_message);
                        }

                       continue;
                     }

                    if(rec_data1=='p' && rec_data2=='n')  //Set PHONE no
                     {
                       serial_current_message.length=6;
                       serial_current_message.address=0xD4;
                       serial_current_message.data[0]='N';
                       serial_current_message.data[1]=rec_data3;
                       serial_current_message.data[2]=rec_data4;
                       serial_current_message.data[3]=last_rec_message.data[5];
                       serial_current_message.data[4]=last_rec_message.data[6];
                       serial_current_message.data[5]=last_rec_message.data[7];
                       RS485_send_message(serial_current_message);
                       delay_ms(10);
                       send_save_ok_ack(2);
                       continue;

                     }
                    if(rec_data1=='F' &&  rec_data2=='N'&&rec_data3=='c' )  //FNC
                     {
                       serial_current_message.length=6;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='F';
                       serial_current_message.data[2]='N';
                       serial_current_message.data[3]='c';
                       serial_current_message.data[4]=rec_data4;
                       serial_current_message.data[5]=last_rec_message.data[5];
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='F' &&  rec_data2=='s'&&rec_data3=='c' )  //FSC
                     {
                       serial_current_message.length=6;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='F';
                       serial_current_message.data[2]='s';
                       serial_current_message.data[3]='c';
                       serial_current_message.data[4]=rec_data4;
                       serial_current_message.data[5]=last_rec_message.data[5];
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='S' && rec_data2=='n'&&rec_data3=='c' )  //SNC
                     {
                       serial_current_message.length=6;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='S';
                       serial_current_message.data[2]='n';
                       serial_current_message.data[3]='c';
                       serial_current_message.data[4]=rec_data4;
                       serial_current_message.data[5]=last_rec_message.data[5];
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='S' && rec_data2=='s'&&rec_data3=='c' )  //SSC
                     {
                       serial_current_message.length=6;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='S';
                       serial_current_message.data[2]='s';
                       serial_current_message.data[3]='c';
                       serial_current_message.data[4]=rec_data4;
                       serial_current_message.data[5]=last_rec_message.data[5];
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='D' &&  rec_data2=='e'&&rec_data3=='l' )  //DEL
                     {
                       serial_current_message.length=5;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='D';
                       serial_current_message.data[2]='e';
                       serial_current_message.data[3]='l';
                       serial_current_message.data[4]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='P' && rec_data2=='h'&&rec_data3=='e' )  //PHE
                     {
                       serial_current_message.length=5;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='P';
                       serial_current_message.data[2]='h';
                       serial_current_message.data[3]='e';
                       serial_current_message.data[4]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='L' && rec_data2=='e'&&rec_data3=='c' )  //LEC
                     {
                       serial_current_message.length=6;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='L';
                       serial_current_message.data[2]='e';
                       serial_current_message.data[3]='c';
                       serial_current_message.data[4]=rec_data4;
                       serial_current_message.data[5]=last_rec_message.data[5];
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='R' &&  rec_data2=='s'&&rec_data3=='t' )  //REST
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='R';
                       serial_current_message.data[2]='s';
                       serial_current_message.data[3]='t';
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' &&  rec_data2=='c'&&rec_data3=='p' )  //UCP
                     {
                       serial_current_message.length=5;
                       serial_current_message.address=0xD2;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='U';
                       serial_current_message.data[2]='c';
                       serial_current_message.data[3]='p';
                       serial_current_message.data[4]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                   if(rec_data1=='E' &&   rec_data2=='t' &&  rec_data3=='h')    //Eth_number
                     {
                       if(rec_data4>64)continue;
                        EE_Eth_number = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='G' &&   rec_data2=='t' && rec_data3=='y')//Grouptype
                     {
                       if( (rec_data4!='s')&&
                             ((rec_data4>'8')||(rec_data4<'2')))continue;
                        EE_Grouptype = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='M' && rec_data2=='o' && rec_data3=='n')   //Monitoring
                     {
                       if((last_rec_message.data[5]!='y')&&(last_rec_message.data[5]!='n'))continue;
                       EE_Monitoring = last_rec_message.data[5];
                       if(rec_data4==255)continue;
                        EE_Gateway_lsb = rec_data4;
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='C' && rec_data2=='R')
                     {
                        if(!pathfinder(rec_data4,1))continue;
                        if(Monitoring == 'y')
                             _send_requests_status_1 = rec_data4;
                        beep_request=1;
                        send_save_ok_ack(2);
                        continue;
                     }
                   if(rec_data1=='H' && rec_data2=='R')
                     {
                        if(!pathfinder(rec_data4,2))continue;
                        if(Monitoring == 'y')
                              _send_requests_status_2 = rec_data4;
                        beep_request=1;
                        send_save_ok_ack(2);
                        continue;
                     }
                   if(rec_data1=='T' && rec_data2=='I')//time
                     {
                       if(Expired_date==0xFF)
                             rtc_set_time( last_rec_message.data[4],last_rec_message.data[5],last_rec_message.data[6]);

                        send_save_ok_ack(2);
                        continue;
                     }
                   if(rec_data1=='D' &&  rec_data2=='A') //date
                     {
                       if(Expired_date==0xFF)
                            rtc_set_date( rec_data4,
                                                last_rec_message.data[5],
                                                last_rec_message.data[6]);

                        send_save_ok_ack(2);
                        continue;
                     }
                   if(rec_data1=='M' && rec_data2=='F') //Set Mask floor
                     {

                        if((!rec_data4)&&(floorvalidity(rec_data3)))
                                     {
                                      inp1=EE_Maskfloor_Chksum;
                                      inp1++;
                                      EE_Maskfloor_Chksum=inp1;
                                     }
                        if((rec_data4)&&(!floorvalidity(rec_data3)))
                                     {
                                      inp1=EE_Maskfloor_Chksum;
                                      inp1--;
                                      EE_Maskfloor_Chksum=inp1;
                                     }

                        setfloormask(rec_data3,rec_data4,0);

                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='H' && rec_data2=='f')  //Set Half floor
                     {

                        if((rec_data4)&&(!halffloorstatus(rec_data3)))
                                     {
                                      inp1=EE_HalfFloor_Chksum;
                                      inp1--;
                                      EE_HalfFloor_Chksum=inp1;
                                     }
                        if((!rec_data4)&&(halffloorstatus(rec_data3)))
                                     {
                                      inp1=EE_HalfFloor_Chksum;
                                      inp1++;
                                      EE_HalfFloor_Chksum=inp1;
                                     }

                        setfloormask(rec_data3,rec_data4,3);   //3 for half floor
                        send_save_ok_ack(2);
                        continue;
                     }

                   if(rec_data1=='T' &&  rec_data2=='u' && rec_data3=='d')//Set tunnel door
                     {

                        inp4=rec_data4;  //floor
                        inp5=Tunnel_door_find(rec_data4);  //stored status of floor

                        if((last_rec_message.data[5]&0x01)&&!(inp5&0x01))
                                     {
                                      inp1=EE_FirstDoor_Chksum;
                                      inp1--;
                                      EE_FirstDoor_Chksum=inp1;
                                      setfloormask(inp4,1,1);
                                     }
                        if(!(last_rec_message.data[5]&0x01)&&(inp5&0x01))
                                     {
                                      inp1=EE_FirstDoor_Chksum;
                                      inp1++;
                                      EE_FirstDoor_Chksum=inp1;
                                      setfloormask(inp4,0,1);
                                     }
                        if((last_rec_message.data[5]&0x02)&&!(inp5&0x02))
                                     {
                                      inp1=EE_SecondDoor_Chksum;
                                      inp1--;
                                      EE_SecondDoor_Chksum=inp1;
                                      setfloormask(inp4,1,2);
                                     }
                        if(!(last_rec_message.data[5]&0x02)&&(inp5&0x02))
                                     {
                                      inp1=EE_SecondDoor_Chksum;
                                      inp1++;
                                      EE_SecondDoor_Chksum=inp1;
                                      setfloormask(inp4,0,2);
                                     }

                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='T' && rec_data2=='u' &&  rec_data3=='s') //tunnel door
                     {
                       inp4=rec_data4;
                       if(inp4==0)
                         {EE_Tunnel_Door_en='n';
                          Tunnel_Door_en='n';}
                       else
                       if(inp4==1)
                         {EE_Tunnel_Door_en='s';
                          Tunnel_Door_en='s';}
                       else
                       if(inp4==2)
                         {EE_Tunnel_Door_en='2';
                          Tunnel_Door_en='2';}
                       else continue;
                        send_save_ok_ack(3);
                        delay_ms(10);
                        continue;
                     }

                   if(rec_data1=='C' && rec_data2=='a' && rec_data3=='E')//clear all errors
                     {
                        inp4= EEWriteIndex;
                        if(inp4!=0xFF)
                        {
                         inp4 = EEWriteIndexOV;
                         if(inp4==0)inp4=199;
                           else if(inp4==0xFF)inp4=EEWriteIndex;
                         EEWriteIndexOV = 0xFF;
                         EEWriteIndex    = 0xFF;
                         for(inp5=0;inp5<=inp4;inp5++)
                           {
                            ErrorHistory[inp5].ErrorNum   = 0xFF;
                            ErrorHistory[inp5].ErrorQty   = 0xFF;
                            ErrorHistory[inp5].ErrorSec    = 0xFF;
                            ErrorHistory[inp5].ErrorMin   = 0xFF;
                            ErrorHistory[inp5].ErrorHour  = 0xFF;
                            ErrorHistory[inp5].ErrorDate  = 0xFF;
                            ErrorHistory[inp5].ErrorMonth = 0xFF;
                            ErrorHistory[inp5].ErrorYear  = 0xFF;
                            WDRS=1;
                           }
                         }  //end if(inp4!=0xff
                        send_save_ok_ack(3);
                        continue;
                     }
                   if(rec_data1=='F' && rec_data2=='a' && rec_data3=='s') //set factory setting
                     {
                        EE_Navistrategy ='d';        EE_VIPfloor=0;
                        EE_Parkfloor=0        ;        EE_Firefloor=1 ;
                        EE_Calfs =0            ;        EE_Needtocalibration = 'n';
                        EE_Contofftd =0       ;        EE_Contofftu =0;
                        EE_Cont_overlap =0   ;        EE_Timebreakdelay = 0;
                        EE_Fsflag =0           ;        EE_Oneflag =0;
                        EE_Calmt =300          ;       EE_Timepark =400;
                        EE_Timestndby= 400  ;        EE_Time1cf3=80;
                        EE_Time2cf3 =80       ;       EE_Cf1mt =80;
                        EE_Doorwtc = 600      ;        EE_Doordur = 50;
                        EE_Doorsiso =40       ;        EE_Doornrc =4;
                        EE_Doorparkunderl = 'y' ;    EE_Dooruldelay =30;
                        EE_Door68t =40;                EE_Doorunderl ='y'    ;
                        EE_Doortype ='s';
                        EE_Doorclosepark ='n';         EE_Doorcdebouncet = 4;
                        EE_Maskfloor_Chksum = 0;     EE_Eth_number = 0;

                        send_save_ok_ack(3);
                        continue;
                     }
                     if(rec_data1=='U' && rec_data2=='M'&& rec_data3=='T' )  //RESCUE MOTOR TORQUE
                     {
                       serial_current_message.length=3;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='M';
                       serial_current_message.data[2]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                     if(rec_data1=='U' && rec_data2=='L'&& rec_data3=='P' )  //RESCUE MOTOR TORQUE
                     {
                       serial_current_message.length=3;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='P';
                       serial_current_message.data[2]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' && rec_data2=='R'&& rec_data3=='T' )  //RESCUE BLACK OUT TIME
                     {
                       serial_current_message.length=3;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='R';
                       serial_current_message.data[2]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' &&  rec_data2=='B'&& rec_data3=='Q' )  //RESCUE BOOST TORQUE
                     {
                       serial_current_message.length=3;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='Q';
                       serial_current_message.data[2]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' && rec_data2=='B'&&rec_data3=='B' )  //RESCUE BOOST TIME
                     {
                       serial_current_message.length=3;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='B';
                       serial_current_message.data[2]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' &&  rec_data2=='T'&&rec_data3=='T' )  //RESCUE TRAVEL TIME
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='T';
                       serial_current_message.data[2]=rec_data4;
                       serial_current_message.data[3]=last_rec_message.data[5];
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' && rec_data2=='L'&&rec_data3=='S' )  //RESCUE LS OVERLAY
                     {
                       serial_current_message.length=3;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='L';
                       serial_current_message.data[2]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' &&
                         rec_data2=='M'&&rec_data3=='C' )  //RESCUE MOTOR CURRENT
                     {
                       serial_current_message.length=4;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='C';
                       serial_current_message.data[2]=rec_data4;
                       serial_current_message.data[3]=last_rec_message.data[5];
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' &&
                         rec_data2=='D'&&rec_data3=='K' )  //RESCUE DC brake torque
                     {
                       serial_current_message.length=3;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='K';
                       serial_current_message.data[2]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
                    if(rec_data1=='U' && rec_data2=='R'&&rec_data3=='F' )  //RESCUE fault in enable
                     {
                       if((rec_data4!='y')&&(rec_data4!='n'))continue;
                       Force_to_Rescue_En=rec_data4;
                       EE_Force_to_Rescue_En=Force_to_Rescue_En;
                       serial_current_message.length=3;
                       serial_current_message.address=0xD5;
                       serial_current_message.data[0]='S';
                       serial_current_message.data[1]='F';
                       serial_current_message.data[2]=rec_data4;
                       RS485_send_message(serial_current_message);
                        continue;
                     }
               }
            }

//********************************************************************
//********************


if(Eth_number)
{
 if(Grouptype!='s')

 {
   if(Received_Add == 0xB0)            //numfloor

    {
              IO_number = request_data;
              if(IO_number>Floorsno)continue;
              if((IO_number>9)||(IO_number<1))continue;

              if((Navistrategy!='d')&&(Navistrategy!='f'))continue;

              if((floor==IO_number)&&(stayinstair==1))
                       {
                                 pathfinder(IO_number,2);  //exit from standby
                                 continue;
                       }

               Eval = Evaluation(IO_number,2);

               inp4 = Waiting_List_Manager(IO_number,2,Eval,'w',0,0);      //place in blank row

               if(inp4!=0xFF)                       //return 0xFF for unsuccessful placement
                  {
                     	   Send_Request_via_Eth(WL[inp4], 'R');
                            continue;
                  }

    }     // if(Received_Add == 0xB0)

    else
    if((Received_Add & 0xF0) == 0xF0)
          {
            //Data has been sent by a floor board
             IO_number  = Received_Add & 0x0F;
             IO_number *= 16;
             IO_number += request_data;       //1 < IO_number < 64 for non-directional collective
                                               //65 < IO_number < 128 for directional collective

             if((Navistrategy!='s')&&((IO_number>64)||(IO_number<1)))  continue;
             if((Navistrategy=='s')&&((IO_number>128)||(IO_number<1))) continue;

             if((Navistrategy == 'f')||(Navistrategy == 'd'))
                  {
                    if((!IO_number)||(IO_number > Floorsno)) continue;  //Don't care of invalid requests

                    if((floor==IO_number)&&(stayinstair==1))
                          {
                                 pathfinder(IO_number,2);  //exit from standby
                                 continue;
                          }

                    Eval = Evaluation(IO_number,2);

                    inp4 = Waiting_List_Manager(IO_number,2,Eval,'w',0,0);      //place in blank row
                    if(inp4!=0xFF)                       //return 0xFF for unsuccessful placement
                      {
                     	   Send_Request_via_Eth(WL[inp4], 'R');
              	           continue;

                      }   //  if(inp4!=0xFF)


                  }
             else
             if(Navistrategy == 's')
                 {
                  if(IO_number > (Floorsno*2)) continue;
                  if(IO_number%2)
                   {                //odd IO_number for down-ward requests
                      IO_number/=2;
                      IO_number++;
                      Eval = Evaluation(IO_number,4);


                    inp4 = Waiting_List_Manager(IO_number,4,Eval,'w',0,0);      //place in blank row
                    if(inp4!=0xFF)                       //return 0xFF for unsuccessful placement
                      {
                     	   Send_Request_via_Eth(WL[inp4], 'R');
              	           continue;

                      }   //  if(inp4!=0xFF)

                   }   // if(IO_number%2)
                  else
                   {
                      IO_number/=2;   //even IO_number for up-ward requests

                      Eval = Evaluation(IO_number,2);


                    inp4 = Waiting_List_Manager(IO_number,2,Eval,'w',0,0);      //place in blank row
                    if(inp4!=0xFF)                       //return 0xFF for unsuccessful placement
                      {
                     	   Send_Request_via_Eth(WL[inp4], 'R');
              	           continue;
                      }

                   }  //END ELSE

                  }      //end if NS =s


          }    //if((Received_Add & 0


 }   //grouptype!='s'

}  //if(Eth_number




//   **************           Data is sent by Num-floor or Num-Cabin Board  *****************

            if((Received_Add == 0xB0)||(Received_Add == 0x10))
             {
              IO_number = request_data;
              if(IO_number>Floorsno)continue;
              if((IO_number>9)||(IO_number<1))continue;

              if((Received_Add==0xB0))
                 {
                  if((Navistrategy=='d')||(Navistrategy=='f'))
                        if(!pathfinder(IO_number,2))request_data=0;
                             else  beep_request=1;   //Num-floor

                 }

              if((Received_Add==0x10))
               {
                if((Navistrategy=='d')||(Navistrategy=='f'))
                        if(!pathfinder(IO_number,1))request_data=0;
                             else   beep_request=1;   //Num-Ext
               }
              if(Navistrategy=='s')request_data=0;

              if(request_data)
                {
                     //Request is accepted
                     current_message.datalen=2;
                     current_message.data[0]='O';
                     current_message.data[1]=request_data;
                     can_send_message(current_message);
                     delay_ms(5);
                 if(Received_Add==0xB0)
                   {                            //Num-Floor
                     sep_id.dest_address = 0xE0;
                     ID = assemble_id(sep_id);
                     current_message.id = ID;
                     current_message.datalen=2;
                     current_message.data[0]='B';
                     current_message.data[1]=request_data;
                     can_send_message(current_message);
                     delay_ms(5);
                    //Destination => Num-Extension board
                     sep_id.dest_address = 0x10;
                     ID = assemble_id(sep_id);
                     current_message.id = ID;
                     can_send_message(current_message);
                     delay_ms(5);

                     if(Monitoring == 'y')
                          _send_requests_status_2=IO_number; //Ethernet monitoring packet
                   }

                 if(Received_Add==0x10)
                   {                           //Num-Ext


                        sep_id.dest_address = 0xF0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=2;
                        current_message.data[0]='B';
                        current_message.data[1]=request_data;
                        can_send_message(current_message);
                        delay_ms(5);
                       //Destination => Num-Floor board
                        sep_id.dest_address = 0xB0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=2;
                        can_send_message(current_message);
                         delay_ms(5);

                     if(Monitoring == 'y')
                         _send_requests_status_1=IO_number; //Ethernet monitoring packet
                   }


                }  //request_data

             }   //received address

             if((Received_Add & 0xF0) == 0xF0)
             {
                //Data has been sent by a floor board
                IO_number  = Received_Add & 0x0F;
                IO_number *= 16;
                IO_number += request_data;       //1 < IO_number < 64 for non-directional collective
                                               //65 < IO_number < 128 for directional collective

                if((Navistrategy!='s')&&((IO_number>64)||(IO_number<1))) continue;
                if((Navistrategy=='s')&&((IO_number>128)||(IO_number<1))) continue;

                if(command_type=='E')
                  {
                   //A short-circuit occured in "IO_number"
                   //and must turn off, alert and run errormsg()
                   //
                   continue;
                  }//end if command type



                if((Navistrategy == 'f')||(Navistrategy == 'd'))
                  {
                       ReqType=2;
                   //Down-Collective & Full-Collective Algorithm

                   if((!IO_number)||(IO_number > Floorsno)) continue;  //Don't care of invalid requests
                   if(pathfinder(IO_number,2))
                      {
                       beep_request=1;
                      } else request_data=0;

                  }//end if NS=f|d
                 if(Navistrategy == 's')
                 {
                  if(IO_number > (Floorsno*2)) continue;
                  if(IO_number%2)
                   {                //odd IO_number for down-ward requests
                    ReqType=4;
                    IO_number/=2;
                    IO_number++;

                    if(!pathfinder(IO_number,4))request_data=0;
                        else  beep_request=1;
                   }
                  else
                   {
                    ReqType=2;
                    IO_number/=2;   //even IO_number for up-ward requests

                    if(!pathfinder(IO_number,2))request_data=0;
                        else  beep_request=1;
                   }
                  } //end if NS =s
                  if(request_data)
                   {

                     //Request is accepted
                     current_message.datalen=2;
                     current_message.data[0]='O';
                     current_message.data[1]=request_data;
                     can_send_message(current_message);

                     if(Navistrategy=='s')
                        {
                         request_data=IO_number%16;
                         if(!request_data)request_data=16;
                         Received_Add&=0x0F;
                         Received_Add/=2;
                        };

                         //Destination => Extension board
                         sep_id.dest_address = ((Received_Add & 0x0F) | 0xE0);
                         ID = assemble_id(sep_id);
                         current_message.id = ID;
                         current_message.datalen=2;
                         current_message.data[0]='B';
                         current_message.data[1]=request_data;
                         can_send_message(current_message);

                         if(Monitoring == 'y')
                          {
                            if(ReqType==2)_send_requests_status_2=IO_number;
                            if(ReqType==4)_send_requests_status_4=IO_number;
                          }
                   }//end of if(request_data)
                 }//end of Floor

               else
               if((Received_Add & 0xF0) == 0xE0)
               {
                 //Data has been sent by a cabin extension board
                 IO_number  = Received_Add & 0x0F;
                 IO_number *= 16;
                 IO_number += request_data;       //0 < IO_number < 65

                 if((IO_number>Floorsno)||(!IO_number)) continue;
                 if(command_type=='E')
                  {
                   //A short-circuit occured in "IO_number"
                   //and must turn off, alert and run errormsg()
                   //
                   continue;
                  }//end if command type
                 if(pathfinder(IO_number,1))
                  {
                    beep_request=1;
                    //Request is accepted
                    current_message.datalen=2;
                    current_message.data[0]='O';
                    current_message.data[1]=request_data;
                    can_send_message(current_message);



                    //Send command for floor board to
                    //blink lamp of same floor in hall

                  if(Navistrategy!='s')
                   {

                      sep_id.dest_address = ((Received_Add & 0x0F) | 0xF0);
                      ID = assemble_id(sep_id);
                      current_message.id = ID;
                      current_message.datalen=2;
                      current_message.data[0]='B';
                      current_message.data[1]=request_data;
                      can_send_message(current_message);
                      delay_ms(5);
                      if(IO_number<8)
                       {
                        sep_id.dest_address = 0xB0;           //For Num_floor Board
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=2;
                        current_message.data[0]='B';
                        current_message.data[1]=request_data;
                        can_send_message(current_message);
                        delay_ms(5);
                       }
                    }


                     if(Monitoring == 'y')
                          _send_requests_status_1=IO_number; //Ethernet monitoring packet
                  }//end if pathfinder

                 }//End of Cabin Extension board




      }//end of while can_rxfull_mobs
}



//*********************   Serial(RS485 or RS422) Received Data Interpreter **********

void serial_interpreter(void)
{
  unsigned int sinp1,sinp2,sinp3;
  unsigned char IO_number=0,Eval,sinp4;

  if(!Ready)return;

  Ready=0;

   if(SerialSource == 0xCF)            //TESTER

    {
      if((rx_buffer1[0]=='P') && (rx_buffer1[1]=='S'))
       {
                     serial_current_message.length=3;
                     serial_current_message.address=0xCF;
                     serial_current_message.data[0]='P';
                     serial_current_message.data[1]='S';
                     serial_current_message.data[2]=1;
                     serial_current_message.data[3]=0x00;
                     serial_current_message.data[4]=0x00;
                     RS485_send_message(serial_current_message);
                     return;
       }
    }

if(Eth_number)
{
 if(Grouptype!='s')

 {
   if(SerialSource == 0xD3)            //numfloor

    {
              IO_number = rx_buffer1[2];
              if(IO_number>Floorsno)return;
              if((IO_number>9)||(IO_number<1))return;

              if((Navistrategy!='d')&&(Navistrategy!='f'))return;

              if((floor==IO_number)&&(stayinstair==1))
                      {
                                 pathfinder(IO_number,2);  //exit from standby
                                 return;
                       }

               Eval = Evaluation(IO_number,2);

               sinp4 = Waiting_List_Manager(IO_number,2,Eval,'w',0,0);      //place in blank row


               if(sinp4!=0xFF)                       //return 0xFF for unsuccessful placement
                  {
                     	   Send_Request_via_Eth(WL[sinp4], 'R');
                            return;
                  }

    }     // if(Received_Add == 0xB0)

    else
    if((SerialSource & 0xF0) == 0xF0)
          {
            //Data has been sent by a floor board
             IO_number  = SerialSource & 0x0F;
             IO_number *= 16;
             IO_number += rx_buffer1[2];       //1 < IO_number < 64 for non-directional collective
                                               //65 < IO_number < 128 for directional collective

             if((Navistrategy!='s')&&((IO_number>64)||(IO_number<1)))  return;
             if((Navistrategy=='s')&&((IO_number>128)||(IO_number<1))) return;

             if((Navistrategy == 'f')||(Navistrategy == 'd'))
                  {
                    if((!IO_number)||(IO_number > Floorsno)) return;  //Don't care of invalid requests

                    if((floor==IO_number)&&(stayinstair==1))
                          {
                                 pathfinder(IO_number,2);  //exit from standby
                                 return;
                          }

                    Eval = Evaluation(IO_number,2);

                    sinp4 = Waiting_List_Manager(IO_number,2,Eval,'w',0,0);      //place in blank row
                    if(sinp4!=0xFF)                       //return 0xFF for unsuccessful placement
                      {
                     	   Send_Request_via_Eth(WL[sinp4], 'R');
              	           return;

                      }   //  if(inp4!=0xFF)


                  }
             else
             if(Navistrategy == 's')
                 {
                  if(IO_number > (Floorsno*2)) return;
                  if(IO_number%2)
                   {                //odd IO_number for down-ward requests
                      IO_number/=2;
                      IO_number++;
                      Eval = Evaluation(IO_number,4);


                    sinp4 = Waiting_List_Manager(IO_number,4,Eval,'w',0,0);      //place in blank row
                    if(sinp4!=0xFF)                       //return 0xFF for unsuccessful placement
                      {
                     	   Send_Request_via_Eth(WL[sinp4], 'R');
              	           return;

                      }   //  if(inp4!=0xFF)

                   }   // if(IO_number%2)
                  else
                   {
                      IO_number/=2;   //even IO_number for up-ward requests

                      Eval = Evaluation(IO_number,2);


                    sinp4 = Waiting_List_Manager(IO_number,2,Eval,'w',0,0);      //place in blank row
                    if(sinp4!=0xFF)                       //return 0xFF for unsuccessful placement
                      {
                     	   Send_Request_via_Eth(WL[sinp4], 'R');
              	           return;
                      }

                   }  //END ELSE

                  }      //end if NS =s


          }    //if((Received_Add & 0


 }   //grouptype!='s'

}  //if(Eth_number


  if(SerialSource==0xD2)
  {
     if(rx_buffer1[0]=='E')
                  {
                     if((rx_buffer1[1]>90)&&(rx_buffer1[1]<100))LPC_error=rx_buffer1[1];
                      else LPC_error=0;

                     return;
                  }
     else
      if((rx_buffer1[0]=='G'))                                 //get data from lpc
     {
         if((rx_buffer1[1]=='F')&&
             (rx_buffer1[2]=='n'))
           {                                  //Floorsno
                     serial_current_message.length=5;
                     serial_current_message.address=0xD2;
                     serial_current_message.data[0]='R';
                     serial_current_message.data[1]='F';
                     serial_current_message.data[2]='n';
                     serial_current_message.data[3]=0x00;
                     serial_current_message.data[4]=EE_Floorsno;
                     RS485_send_message(serial_current_message);
                     return;
           }
         if((rx_buffer1[1]=='T')&&
             (rx_buffer1[2]=='R')&&(rx_buffer1[3]=='t'))
           {                                      //Timetravel
                     sinp3 = EE_Timetravel ;
                     sinp3 /= 10;
                     sinp1 = sinp3 / 256;
                     sinp2 = sinp3 % 256;
                     serial_current_message.length=6;
                     serial_current_message.address=0xD2;
                     serial_current_message.data[0]='R';
                     serial_current_message.data[1]='T';
                     serial_current_message.data[2]='R';
                     serial_current_message.data[3]='t';
                     serial_current_message.data[4]=sinp1;
                     serial_current_message.data[5]=sinp2;
                     RS485_send_message(serial_current_message);
                     return;
           }
         if((rx_buffer1[1]=='O')&&
             (rx_buffer1[2]=='C')&&(rx_buffer1[3]=='t'))
           {                                         //Doordur
                     sinp3 = EE_Doordur ;
                     sinp1 = sinp3 / 256;
                     sinp2 = sinp3 % 256;
                     serial_current_message.length=6;
                     serial_current_message.address=0xD2;
                     serial_current_message.data[0]='R';
                     serial_current_message.data[1]='O';
                     serial_current_message.data[2]='C';
                     serial_current_message.data[3]='t';
                     serial_current_message.data[4]=sinp1;
                     serial_current_message.data[5]=sinp2;
                     RS485_send_message(serial_current_message);
                     return;
           }
         if((rx_buffer1[1]=='U')&&
             (rx_buffer1[2]=='N'))         //Undergroundno
           {
                     serial_current_message.length=5;
                     serial_current_message.address=0xD2;
                     serial_current_message.data[0]='R';
                     serial_current_message.data[1]='U';
                     serial_current_message.data[2]='N';
                     serial_current_message.data[3]=0x00;
                     serial_current_message.data[4]=EE_Undergroundno;
                     RS485_send_message(serial_current_message);
                     return;
           }
         if((rx_buffer1[1]=='P')&&
             (rx_buffer1[2]=='S'))        //Parksign
           {
                     serial_current_message.length=5;
                     serial_current_message.address=0xD2;
                     serial_current_message.data[0]='R';
                     serial_current_message.data[1]='P';
                     serial_current_message.data[2]='S';
                     serial_current_message.data[3]=0x00;
                     serial_current_message.data[4]=EE_Parksign;
                     RS485_send_message(serial_current_message);
                     return;
           }
         if((rx_buffer1[1]=='G')&&
             (rx_buffer1[2]=='X'))    //Groundexist
           {
                     serial_current_message.length=5;
                     serial_current_message.address=0xD2;
                     serial_current_message.data[0]='R';
                     serial_current_message.data[1]='G';
                     serial_current_message.data[2]='X';
                     serial_current_message.data[3]=0x00;
                     serial_current_message.data[4]=EE_Groundexist;
                     RS485_send_message(serial_current_message);
                     return;
           }
         if((rx_buffer1[1]=='D')&&
             (rx_buffer1[2]=='Y'))             //Doortype
           {
                     serial_current_message.length=5;
                     serial_current_message.address=0xD2;
                     serial_current_message.data[0]='R';
                     serial_current_message.data[1]='D';
                     serial_current_message.data[2]='Y';
                     serial_current_message.data[3]=0x00;
                     serial_current_message.data[4]=EE_Doortype;
                     RS485_send_message(serial_current_message);
                     return;
           }
         if((rx_buffer1[1]=='N')&&
             (rx_buffer1[2]=='S'))                    //navistrategy
           {
                     serial_current_message.length=5;
                     serial_current_message.address=0xD2;
                     serial_current_message.data[0]='R';
                     serial_current_message.data[1]='N';
                     serial_current_message.data[2]='S';
                     serial_current_message.data[3]=0x00;
                     serial_current_message.data[4]=EE_Navistrategy;
                     RS485_send_message(serial_current_message);
                     return;
           }
         if((rx_buffer1[1]=='V')&&
             (rx_buffer1[2]=='V'))                    //Volume
           {
                if(IVA_comm=='c')
                   {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xD1;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=2;
                        current_message.data[0]='G';
                        current_message.data[1]='V';
                        can_send_message(current_message);
                   }
                else
                if(IVA_comm=='s')
                   {               //sent G->V to IVA401 via cabin board
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xC0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=4;
                        current_message.data[0]='X';
                        current_message.data[1]=0xD1;
                        current_message.data[2]='G';
                        current_message.data[3]='V';
                        can_send_message(current_message);
                   }
                else
                if(IVA_comm=='p')
                   {              //sent G->V to IVA401 in RS485 format
                        serial_current_message.length=2;
                        serial_current_message.address=0xD1;
                        serial_current_message.data[0]='G';
                        serial_current_message.data[1]='V';
                        RS485_send_message(serial_current_message);
                   }
                _volume_set_rs485=1;            //1 means sent request volume by LPC
                return;
           }

     }   // if((rx_buffer1[0]=='G'))

     else
     if((rx_buffer1[0]=='R'))             //setting delivery meesage from LPC
     {
         if((rx_buffer1[1]=='F')&&
             (rx_buffer1[2]=='N')&&(rx_buffer1[3]=='c')&&(rx_buffer1[4]==1))                    //fnc
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='S';
                        current_message.data[1]='F';
                        current_message.data[2]='N';
                        current_message.data[3]='c';
                        current_message.data[4]=1;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='F')&&
             (rx_buffer1[2]=='s')&&(rx_buffer1[3]=='c')&&(rx_buffer1[4]==1))                    //fsc
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='S';
                        current_message.data[1]='F';
                        current_message.data[2]='s';
                        current_message.data[3]='c';
                        current_message.data[4]=1;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='S')&&
             (rx_buffer1[2]=='s')&&(rx_buffer1[3]=='c')&&(rx_buffer1[4]==1))                    //ssc
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='S';
                        current_message.data[1]='S';
                        current_message.data[2]='s';
                        current_message.data[3]='c';
                        current_message.data[4]=1;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='S')&&
             (rx_buffer1[2]=='n')&&(rx_buffer1[3]=='c')&&(rx_buffer1[4]==1))                    //snc
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='S';
                        current_message.data[1]='S';
                        current_message.data[2]='n';
                        current_message.data[3]='c';
                        current_message.data[4]=1;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='D')&&
             (rx_buffer1[2]=='e')&&(rx_buffer1[3]=='l')&&(rx_buffer1[4]==1))                    //del
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='S';
                        current_message.data[1]='D';
                        current_message.data[2]='e';
                        current_message.data[3]='l';
                        current_message.data[4]=1;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='U')&&
             (rx_buffer1[2]=='c')&&(rx_buffer1[3]=='p')&&(rx_buffer1[4]==1))                    //ucp
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='S';
                        current_message.data[1]='U';
                        current_message.data[2]='c';
                        current_message.data[3]='p';
                        current_message.data[4]=1;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='P')&&
             (rx_buffer1[2]=='h')&&(rx_buffer1[3]=='e')&&(rx_buffer1[4]==1))                    //phe
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='S';
                        current_message.data[1]='P';
                        current_message.data[2]='h';
                        current_message.data[3]='e';
                        current_message.data[4]=1;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='R')&&
             (rx_buffer1[2]=='s')&&(rx_buffer1[3]=='t')&&(rx_buffer1[4]==1))                    //reset
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='S';
                        current_message.data[1]='R';
                        current_message.data[2]='s';
                        current_message.data[3]='t';
                        current_message.data[4]=1;
                        can_send_message(current_message);
                        return;
           }
     }   // if((rx_buffer1[0]=='R'))


     else
     if((rx_buffer1[0]=='S'))
      {
         if((rx_buffer1[1]=='F')&&
             (rx_buffer1[2]=='n'))
           {                                    //Floorsno
                  if((rx_buffer1[4]>64)||(rx_buffer1[4]==0))return;
                  EE_Floorsno=rx_buffer1[4];
                  return;
           }
         if((rx_buffer1[1]=='T')&&
             (rx_buffer1[2]=='R')&&(rx_buffer1[3]=='t'))
           {                          //Timetravel
                  sinp1=rx_buffer1[4];
                  sinp2=rx_buffer1[5];
                  sinp3 = sinp1*256 + sinp2;
                  sinp3 *= 10;
                  EE_Timetravel = sinp3;
                  return;
           }
         if((rx_buffer1[1]=='O')&&
             (rx_buffer1[2]=='C')&&(rx_buffer1[3]=='t'))
           {                                      ///Doordur
                  sinp1=rx_buffer1[4];
                  sinp2=rx_buffer1[5];
                  sinp3 = sinp1*256 + sinp2;
                  EE_Doordur = sinp3;
                  return;
           }
         if((rx_buffer1[1]=='U')&&
             (rx_buffer1[2]=='N'))    //Undergroundno
           {
                  EE_Undergroundno = rx_buffer1[4];
                  Undergroundno = rx_buffer1[4];
                  return;
           }
         if((rx_buffer1[1]=='P')&&
             (rx_buffer1[2]=='S'))         //Parksign
           {
                  EE_Parksign = rx_buffer1[4];
                  Parksign = rx_buffer1[4];
                  return;
           }
         if((rx_buffer1[1]=='G')&&
             (rx_buffer1[2]=='X'))              //Groundexist
           {
                  EE_Groundexist = rx_buffer1[4];
                  Groundexist = rx_buffer1[4];
                  return;
           }
         if((rx_buffer1[1]=='D')&&
             (rx_buffer1[2]=='Y'))     //Doortype
           {
                  EE_Doortype = rx_buffer1[4];
                  return;
           }
         if((rx_buffer1[1]=='N')&&
             (rx_buffer1[2]=='S'))         //Navistrategy
           {
                  EE_Navistrategy = rx_buffer1[4];
                  return;
           }
         if((rx_buffer1[1]=='F')&&
             (rx_buffer1[2]=='N')&&(rx_buffer1[3]=='c'))                    //send data from lpc to pct
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=6;
                        current_message.data[0]='R';
                        current_message.data[1]='F';
                        current_message.data[2]='N';
                        current_message.data[3]='c';
                        current_message.data[4]=rx_buffer1[4];
                        current_message.data[5]=rx_buffer1[5];
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='F')&&
             (rx_buffer1[2]=='s')&&(rx_buffer1[3]=='c'))                    //send data from lpc to pct
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=6;
                        current_message.data[0]='R';
                        current_message.data[1]='F';
                        current_message.data[2]='s';
                        current_message.data[3]='c';
                        current_message.data[4]=rx_buffer1[4];
                        current_message.data[5]=rx_buffer1[5];
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='S')&&
             (rx_buffer1[2]=='s')&&(rx_buffer1[3]=='c'))                    //send data from lpc to pct
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=6;
                        current_message.data[0]='R';
                        current_message.data[1]='S';
                        current_message.data[2]='s';
                        current_message.data[3]='c';
                        current_message.data[4]=rx_buffer1[4];
                        current_message.data[5]=rx_buffer1[5];
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='S')&&
             (rx_buffer1[2]=='n')&&(rx_buffer1[3]=='c'))                    //send data from lpc to pct
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=6;
                        current_message.data[0]='R';
                        current_message.data[1]='S';
                        current_message.data[2]='n';
                        current_message.data[3]='c';
                        current_message.data[4]=rx_buffer1[4];
                        current_message.data[5]=rx_buffer1[5];
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='D')&&
             (rx_buffer1[2]=='e')&&(rx_buffer1[3]=='l'))                    //send data from lpc to pct
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='R';
                        current_message.data[1]='D';
                        current_message.data[2]='e';
                        current_message.data[3]='l';
                        current_message.data[4]=rx_buffer1[4];
                        current_message.data[5]=0x00;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='U')&&
             (rx_buffer1[2]=='c')&&(rx_buffer1[3]=='p'))                    //send data from lpc to pct
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='R';
                        current_message.data[1]='U';
                        current_message.data[2]='c';
                        current_message.data[3]='p';
                        current_message.data[4]=rx_buffer1[4];
                        current_message.data[5]=0x00;
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='P')&&
             (rx_buffer1[2]=='h')&&(rx_buffer1[3]=='e'))                    //send data from lpc to pct
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=5;
                        current_message.data[0]='R';
                        current_message.data[1]='P';
                        current_message.data[2]='h';
                        current_message.data[3]='e';
                        current_message.data[4]=rx_buffer1[4];
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='L')&&
             (rx_buffer1[2]=='e')&&(rx_buffer1[3]=='c'))                    //send data from lpc to pct
           {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=6;
                        current_message.data[0]='R';
                        current_message.data[1]='L';
                        current_message.data[2]='e';
                        current_message.data[3]='c';
                        current_message.data[4]=rx_buffer1[4];
                        current_message.data[5]=rx_buffer1[5];
                        can_send_message(current_message);
                        return;
           }
         if((rx_buffer1[1]=='V')&&
             (rx_buffer1[2]=='V'))                    //send data from lpc to voice
           {
                EE_Volume = rx_buffer1[4];
                if(IVA_comm=='c')
                   {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xD1;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=2;
                        current_message.data[0]='V';
                        current_message.data[1]=rx_buffer1[4];
                        can_send_message(current_message);
                        delay_ms(10);
                        current_message.datalen=1;            //ding
                        current_message.data[0]='D';
                        can_send_message(current_message);
                        delay_ms(10);
                        return;
                   }
                else
                if(IVA_comm=='s')
                   {               //sent volume and ding via CABIN BOARD to IVA401
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xC0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=4;
                        current_message.data[0]='X';
                        current_message.data[1]=0xD1;
                        current_message.data[2]='V';
                        current_message.data[3]=rx_buffer1[4];
                        can_send_message(current_message);
                        delay_ms(10);
                        current_message.datalen=3;            //ding
                        current_message.data[0]='X';
                        current_message.data[1]=0xD1;
                        current_message.data[2]='D';
                        can_send_message(current_message);
                        delay_ms(10);
                        return;
                   }
                else
                if(IVA_comm=='p')
                   {               //sent volume and ding in RS485format to IVA401
                        serial_current_message.length=2;
                        serial_current_message.address=0xD1;
                        serial_current_message.data[0]='V';
                        serial_current_message.data[1]=rx_buffer1[4];
                        RS485_send_message(serial_current_message);
                        delay_ms(10);
                        serial_current_message.length=1;
                        serial_current_message.address=0xD1;
                        serial_current_message.data[0]='D';
                        RS485_send_message(serial_current_message);
                        return;
                   }
            }


      }   // if((rx_buffer1[0]=='s'))


 }          //if(serial_source

 else
 if(SerialSource==0xD3)
   {
      if(Numtype!='s')return;
      IO_number = rx_buffer1[2];
      if(IO_number>Floorsno)return;
      if((IO_number>9)||(IO_number<1))return;

     if((Navistrategy=='d')||(Navistrategy=='f'))
                       if(!pathfinder(IO_number,2))IO_number=0;
                             else  beep_request=1;   //Num-floor

     if(Navistrategy=='s')IO_number=0;

              if(IO_number)
                {
                     serial_current_message.length=2;
                     serial_current_message.address=0xD3;
                     serial_current_message.data[0]='O';
                     serial_current_message.data[1]=IO_number;
                     RS485_send_message(serial_current_message);

                     sep_id.dest_address = 0xC0;            //cabin
                     ID = assemble_id(sep_id);
                     current_message.id = ID;
                     current_message.data[0]='B';
                     current_message.data[1]=IO_number;
                     can_send_message(current_message);
                     delay_ms(10);

                }  //IO_number



   }  //if(SerialSource==0xD3)
   else
    if(SerialSource==0xD4)
   {
            if((rx_buffer1[0]=='n'))
              {
                        sep_id.source_address = node_address;
                        sep_id.dest_address = 0xA0;
                        sep_id.message_num = 0;
                        sep_id.priority = 0;
                        ID = assemble_id(sep_id);
                        current_message.id = ID;
                        current_message.datalen=8;
                        current_message.data[0]='R';
                        current_message.data[1]='p';
                        current_message.data[2]='n';
                        current_message.data[3]=rx_buffer1[1];
                        current_message.data[4]=rx_buffer1[2];
                        current_message.data[5]=rx_buffer1[3];
                        current_message.data[6]=rx_buffer1[4];
                        current_message.data[7]=rx_buffer1[5];
                        can_send_message(current_message);

              }
            if((rx_buffer1[0]=='R')&&(rx_buffer1[1]=='E')&&(rx_buffer1[2]=='S')&&
              (rx_buffer1[3]=='E')&&(rx_buffer1[4]=='T'))
              {
                forcetoreset=1;
              }
            if((rx_buffer1[0]=='L')&&(rx_buffer1[1]=='o')&&(rx_buffer1[2]=='C')&&
              (rx_buffer1[3]=='K')&&(rx_buffer1[4]=='S')&&(rx_buffer1[5]=='m')&&(rx_buffer1[6]=='S'))
              {
               EE_Expired_month=0xFA;
               delay_ms(5);
               Expired_month=0xFA;
               EE_Expired_date=0xFA;
               EE_Start_dayofyear=0xFAFA;
               Start_dayofyear=0xFAFA;
               delay_ms(5);
               Expired_date=0xFA;
               EE_Block_date_en='y';
              }
            if((rx_buffer1[0]=='U')&&(rx_buffer1[1]=='n')&&(rx_buffer1[2]=='L')&&
              (rx_buffer1[3]=='K')&&(rx_buffer1[4]=='S')&&(rx_buffer1[5]=='m')&&(rx_buffer1[6]=='s'))
              {
               EE_Expired_month=0xFF;
               delay_ms(5);
               Expired_month=0xFF;
               EE_Expired_date=0xFF;
               EE_Start_dayofyear=0xFFFF;
               Start_dayofyear=0xFFFF;
               delay_ms(5);
               Expired_date=0xFF;
               EE_Block_date_en='n';
              }



   }
   else
    if((SerialSource&0xF0)==0xF0)
   {
      if(Numtype!='f')return;

     // if((rx_buffer1[2]>16)||(rx_buffer1[2]<1))return;
      if((rx_buffer1[2]>18)||(rx_buffer1[2]<1))return;
      if(Navistrategy!='s')IO_number = (SerialSource&0x0F)*16+ rx_buffer1[2];
        else
         {
           IO_number = (SerialSource&0x0F);
           IO_number = IO_number * 8;
           IO_number += (rx_buffer1[2]/2);
         }

      if(IO_number>Floorsno)return;
      if((IO_number<1))return;

     if((Navistrategy=='d')||(Navistrategy=='f'))
              {
                       if(!pathfinder(IO_number,2))IO_number=0;
                             else  beep_request=1;   //Num-floor
                       if(!IO_number)return;
                       serial_current_message.length=2;
                       serial_current_message.address=SerialSource;
                       serial_current_message.data[0]='O';
                       serial_current_message.data[1]=rx_buffer1[2];
                       RS485_send_message(serial_current_message);

                       sep_id.dest_address = 0xC0;            //Cabin
                       ID = assemble_id(sep_id);
                       current_message.id = ID;
                       current_message.datalen=4;
                       current_message.data[0]='X';
                       current_message.data[1]=(SerialSource&0x0F)|0xE0;
                       current_message.data[2]='B';          //'S' for first digit of numerator
                       current_message.data[3]=IO_number;
                       can_send_message(current_message);
                       delay_ms(10);

              }

     if(Navistrategy=='s')

             {
                       if(rx_buffer1[2]%2)
                         {                       //downward request
                         IO_number++;
                           if(!pathfinder(IO_number,4))IO_number=0;
                             else  beep_request=1;   //Num-floor
                         }
                       else                   //upward request
                         {
                           if(!pathfinder(IO_number,2))IO_number=0;
                             else  beep_request=1;   //Num-floor
                         }
                       if(IO_number)
                         {
                               if(!IO_number)return;
                               serial_current_message.length=2;
                               serial_current_message.address=SerialSource;
                               serial_current_message.data[0]='O';
                               serial_current_message.data[1]=rx_buffer1[2];
                               RS485_send_message(serial_current_message);

                               sep_id.dest_address = 0xC0;            //Cabin
                               ID = assemble_id(sep_id);
                               current_message.id = ID;
                               current_message.datalen=4;
                               current_message.data[0]='X';
                               current_message.data[1]=((IO_number-1)/16)|0xE0;
                               current_message.data[2]='B';          //'S' for first digit of numerator
                               IO_number = IO_number%16;
                               if(!IO_number)IO_number=16;
                               current_message.data[3]=IO_number;
                               can_send_message(current_message);
                               delay_ms(10);
                         }  //IO_number

             }    //if(Navistrategy


   }   //if(serial
   else
    if(SerialSource==0xD1)                  //data from IVA401
   {
     if(rx_buffer1[0]=='V')
                      {
                        if(IVA_comm!='p')                //IVA is on RS485-BUS
                          {
                            IVA_comm='p';
                            EE_IVA_comm='p';
                          }
                        if(_volume_set_rs485==1)             //1 means request volume is sent by LPC
                          {
                                  _volume_set_rs485=0;

                               serial_current_message.length=5;
                               serial_current_message.address=0xD2;              //send to LPC
                               serial_current_message.data[0]='R';
                               serial_current_message.data[1]='V';
                               serial_current_message.data[2]='V';
                               serial_current_message.data[3]=0x00;
                               serial_current_message.data[4]=rx_buffer1[1];
                               RS485_send_message(serial_current_message);
                          }
                        else
                        if(_volume_set_rs485==0)             //0 means request volume is sent by LPC
                        {
                           sep_id.source_address = node_address;
                           sep_id.dest_address = 0xA0;                    //send to HHP
                           sep_id.message_num = 0;
                           sep_id.priority = 0;
                           ID = assemble_id(sep_id);
                           current_message.id = ID;
                           current_message.ide=1;
                           current_message.rb=0;
                           current_message.rtr=0;
                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='V';
                           current_message.data[2]='V';
                           current_message.data[3]=0;
                           current_message.data[4]=rx_buffer1[1];
                           can_send_message(current_message);
                           delay_ms(10);
                        }
                       return;
                     }

   }
   else
    if(SerialSource==0xD5)                          //rescue
   {
        sep_id.source_address = node_address;
        sep_id.dest_address = 0xA0;                    //send to HHP
        sep_id.message_num = 0;
        sep_id.priority = 0;
        ID = assemble_id(sep_id);
        current_message.id = ID;
        current_message.ide=1;
        current_message.rb=0;
        current_message.rtr=0;


     if(rx_buffer1[0]=='E')
       {

                           current_message.datalen=6;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='E';
                           current_message.data[3]='I';
                           current_message.data[4]=rx_buffer1[1];
                           current_message.data[5]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

       }
     else
     if((rx_buffer1[0]=='H')&&(rx_buffer1[7]=='Q'))
       {
         if(firstchk)
            {
              Ready=1;
              rescueviamain();
            }
       }
     else
     if(rx_buffer1[0]=='I')
       {
                           current_message.datalen=6;
                           current_message.data[0]='I';
                           current_message.data[1]=rx_buffer1[1];
                           current_message.data[2]=rx_buffer1[2];
                           current_message.data[3]=rx_buffer1[3];
                           current_message.data[4]=rx_buffer1[4];
                           current_message.data[5]=rx_buffer1[5];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;
        }
     else
     if(rx_buffer1[0]=='S')
        {
         if((rx_buffer1[1]=='K'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='D';
                           current_message.data[3]='K';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if((rx_buffer1[1]=='F'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='R';
                           current_message.data[3]='F';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if((rx_buffer1[1]=='M'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='M';
                           current_message.data[3]='T';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if((rx_buffer1[1]=='P'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='L';
                           current_message.data[3]='P';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if((rx_buffer1[1]=='R'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='R';
                           current_message.data[3]='T';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if((rx_buffer1[1]=='Q'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='B';
                           current_message.data[3]='Q';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if((rx_buffer1[1]=='B'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='B';
                           current_message.data[3]='B';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if((rx_buffer1[1]=='T'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='T';
                           current_message.data[3]='T';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if((rx_buffer1[1]=='L'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='L';
                           current_message.data[3]='S';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;
           }
         else
         if((rx_buffer1[1]=='C'))
           {

                           current_message.datalen=5;
                           current_message.data[0]='S';
                           current_message.data[1]='U';
                           current_message.data[2]='M';
                           current_message.data[3]='C';
                           current_message.data[4]=1;
                           can_send_message(current_message);
                           delay_ms(10);
                           return;
           }
         }
     else
     if(rx_buffer1[0]=='R')
        {
         if(rx_buffer1[1]=='K')
           {

                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='D';
                           current_message.data[3]='K';
                           current_message.data[4]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;
           }
         else
         if(rx_buffer1[1]=='F')
           {

                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='R';
                           current_message.data[3]='F';
                           current_message.data[4]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;
           }
         else
         if(rx_buffer1[1]=='M')
           {

                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='M';
                           current_message.data[3]='T';
                           current_message.data[4]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if(rx_buffer1[1]=='P')
           {

                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='L';
                           current_message.data[3]='P';
                           current_message.data[4]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if(rx_buffer1[1]=='R')
           {

                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='R';
                           current_message.data[3]='T';
                           current_message.data[4]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if(rx_buffer1[1]=='Q')
           {

                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='B';
                           current_message.data[3]='Q';
                           current_message.data[4]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if(rx_buffer1[1]=='B')
           {

                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='B';
                           current_message.data[3]='B';
                           current_message.data[4]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if(rx_buffer1[1]=='L')
           {

                           current_message.datalen=5;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='L';
                           current_message.data[3]='S';
                           current_message.data[4]=rx_buffer1[2];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if(rx_buffer1[1]=='T')
           {

                           current_message.datalen=6;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='T';
                           current_message.data[3]='T';
                           current_message.data[4]=rx_buffer1[2];
                           current_message.data[5]=rx_buffer1[3];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;

           }
         else
         if(rx_buffer1[1]=='C')
           {

                           current_message.datalen=6;
                           current_message.data[0]='R';
                           current_message.data[1]='U';
                           current_message.data[2]='M';
                           current_message.data[3]='C';
                           current_message.data[4]=rx_buffer1[2];
                           current_message.data[5]=rx_buffer1[3];
                           can_send_message(current_message);
                           delay_ms(10);
                           return;
           }
         }
      return;

   }



}


void rescueviamain(void)
{

  unsigned char rvm1=0,rvm2=0,rvm3=0,rvm4=0,Rescue_via_main=0;

ptimer=0;
rvm3=0;
while(1)
{
   WDRS=1;
   delay_ms(10);

   ptimer++;
     /*
   if(ptimer>100)
     {
      ptimer=0;

      sep_id.dest_address = 0xC0;
      sep_id.source_address = node_address;
      sep_id.message_num = 0;
      sep_id.priority = 0;
      ID = assemble_id(sep_id);
      current_message.id = ID;
      current_message.ide=1;
      current_message.rb=0;
      current_message.rtr=0;
      current_message.datalen=3;
      current_message.data[0]='X';
      current_message.data[1]=0xD1;
      current_message.data[2]='Q';
      can_send_message(current_message);
      delay_ms(10);

      sep_id.dest_address = 0xD1;
      sep_id.source_address = node_address;
      sep_id.message_num = 0;
      sep_id.priority = 0;
      ID = assemble_id(sep_id);
      current_message.id = ID;
      current_message.ide=1;
      current_message.rb=0;
      current_message.rtr=0;
      current_message.datalen=1;
      current_message.data[0]='Q';
      can_send_message(current_message);
      delay_ms(10);

      serial_current_message.address=0xD1;
      serial_current_message.length=1;
      serial_current_message.data[0]='Q';
      RS485_send_message(serial_current_message);
      delay_ms(10);

     }
          */
   ++rvm3;
    if(rvm3>50)
      {
       rvm3=0;
       if(rvm2==0)rvm2=1; else rvm2=0;
       if(RELAYC&&(rvm2))rvm1 |= 0x40;
       if(RELAYC&&(rvm2==0))rvm1 &= ~0x40;
       if(RELAYO)rvm1 |= 0x40;
       Write_7Seg(0,rvm1);
      }

  if(can_rxfull_mobs)
      {
        WDRS=1;
        message_detector();

         if(Received_Add == 0xA0)
                 {
                   if(  (command_type=='P') ||
                       ( (command_type=='G') && (last_rec_message.data[1]=='P') && (last_rec_message.data[2]=='W') )  )
                      {
                           sep_id.dest_address = 0xA0;
                           sep_id.source_address = node_address;
                           sep_id.message_num = 0;
                           sep_id.priority = 0;
                           ID = assemble_id(sep_id);
                           current_message.id = ID;
                           current_message.ide=1;
                           current_message.rb=0;
                           current_message.rtr=0;
                           current_message.datalen=3;
                           current_message.data[0]='R';
                           current_message.data[1]='Q';
                           current_message.data[2]='Q';
                           current_message.data[3]=0x00;
                           current_message.data[4]=0x00;
                           current_message.data[5]=0x00;
                           can_send_message(current_message);
                           delay_ms(5);
                      }


                 }

      }

   if(Ready)
   {
    Ready=0;

    if(SerialSource==0xD5)                          //rescue
     {
        if((rx_buffer1[0]=='H')&&(rx_buffer1[7]=='Q'))
            {

                if(Rescue_via_main==0)
                  {
                     Rescue_via_main=1;
                    // ptimer=1;
                    // RELAYC=1;
                    // RELAYO=0;
                    // RELAYC2=1;
                    // RELAYO2=0;
                    // URA=0;
                   //  RELAYSTBY=1;
                     //set_outputs_relay(1);
                     sep_id.source_address = node_address;
                     sep_id.dest_address = 0xC0;              //Cabin Board
                     sep_id.message_num = 0;
                     sep_id.priority = 0;
                     ID = assemble_id(sep_id);
                     current_message.id = ID;
                     current_message.datalen=2;
                     current_message.data[0]='O';
                     current_message.data[1]=0b11011000;
                     can_send_message(current_message);   //turn off led in cabin
                     delay_ms(10);

                     continue;
                  }


                   rvm4=0b00010000;         //STANDBY =1
                   if((rx_buffer1[6]&0xF0)==0x50)
                      {
                        rvm4 &= 0b10110111; //RELAYC=0;
                        rvm4 |= 0b00100100; //RELAYO=1;
                      }
                   if((rx_buffer1[6]&0xF0)==0x00)
                      {
                        rvm4 |= 0b01001000; // RELAYC=1;
                        rvm4 &= 0b11011011; // RELAYO=0;
                      }
                   if((rx_buffer1[6]&0x0F)==0x05)
                      {
                        rvm4 |= 0b10000000; //URA=1;
                      }
                   if((rx_buffer1[6]&0x0F)==0x00)
                      {
                        rvm4 &= 0b01111111; //URA=0;
                      }

                   //set_outputs_relay(1);
                   current_message.ide=1;
                   current_message.rb=0;
                   current_message.rtr=0;
                   sep_id.source_address = node_address;
                   sep_id.dest_address = 0xC0;              //Cabin Board
                   sep_id.message_num = 0;
                   sep_id.priority = 0;
                   ID = assemble_id(sep_id);
                   current_message.id = ID;
                   current_message.datalen=2;
                   current_message.data[0]='O';
                   current_message.data[1]=rvm4;
                   can_send_message(current_message);   //turn off led in cabin
                   delay_ms(10);


                   sep_id.dest_address = 0xA0;
                   sep_id.source_address = node_address;
                   sep_id.message_num = 0;
                   sep_id.priority = 0;
                   ID = assemble_id(sep_id);
                   current_message.id = ID;
                   current_message.datalen=6;
                   current_message.data[0]='I';
                   current_message.data[1]=rx_buffer1[1];
                   current_message.data[2]=rx_buffer1[2];
                   current_message.data[3]=rx_buffer1[3];
                   current_message.data[4]=rx_buffer1[4];
                   current_message.data[5]=rx_buffer1[5];
                   can_send_message(current_message);
                   delay_ms(10);

                   /*
                   sep_id.dest_address = 0xC0;
                   sep_id.source_address = node_address;
                   sep_id.message_num = 0;
                   sep_id.priority = 0;
                   ID = assemble_id(sep_id);
                   current_message.id = ID;
                   current_message.ide=1;
                   current_message.rb=0;
                   current_message.rtr=0;
                   current_message.datalen=3;
                   current_message.data[0]='X';
                   current_message.data[1]=0xD1;
                   current_message.data[2]='Q';
                   can_send_message(current_message);
                   delay_ms(10);

                   sep_id.dest_address = 0xD1;
                   sep_id.source_address = node_address;
                   sep_id.message_num = 0;
                   sep_id.priority = 0;
                   ID = assemble_id(sep_id);
                   current_message.id = ID;
                   current_message.ide=1;
                   current_message.rb=0;
                   current_message.rtr=0;
                   current_message.datalen=1;
                   current_message.data[0]='Q';
                   can_send_message(current_message);
                   delay_ms(10);

      serial_current_message.address=0xD1;
      serial_current_message.length=1;
      serial_current_message.data[0]='Q';
      RS485_send_message(serial_current_message);
      delay_ms(10);
                   */

           }



    }  //ADDRESS==0XD5

  }  //ready




 }


}



#include "defines.h"

unsigned char doorclose(void)
{
unsigned char temp=0,temp2=0,temp3=0,errorinclose=0,temp1,app_doortype;
unsigned int temp4=0,temp5=0;



if(Tunnel_Door_en=='2')
 {
    if(Doortype=='a' && Door2type=='n')app_doortype='s';
    else
    if(Doortype=='n' && Door2type=='a')app_doortype='s';
    else
    if(Doortype=='n' && Door2type=='n')app_doortype='n';
    else
    if(Doortype=='a' && Door2type=='a')app_doortype='a';
    else app_doortype='s';
 }
else app_doortype = Doortype;


RELAYCed = 0;
RELAYOed = 0;

WDRS=1;

if(SS66 && SS69 && SS68)
	{
           RELAYCed = 1;

	   if(app_doortype!='a')
	        {
	          URA=1;
	        }
	   if((Doorunderl==68)||(Doorunderl==69))
		{
                  if(!Dooruldelay)
                         {
                           RELAYO=0;
                           RELAYC=0;
                           set_outputs_relay(1);
                           return 1;
                         }
                   _delay_unload_door = Dooruldelay;   	//Door is not underloaded
		   RELAYC=1;
		   RELAYO=0;
                   set_outputs_relay(1);
	         }
   	           else
	             {
	               RELAYC=1;
	               RELAYO=0;
                       set_outputs_relay(1);
                     }
	   return 1;                          //if door was closed, return 1 and get out
	}

WDRS=1;

if((!SS66)||(DOO))
{_set_blink_num='B';
 while(_set_blink_num&&floor)WDRS=1;
blink_en=1;}                   //For blinking


ptimer=0;
while((!SS66)||DOO)                       //SS66 ==> Halldoor Contact
  {
	WDRS=1;
	if(!floor)break;
	if((ptimer>Doorwtc)&&(Doorwtc!=0))                  //Hall door is not closed after doorwtc
	            {
	              blink_en=0;
	              if(!SS66)
	               {
  	                 hdoorct=1;             //doorwtc is "waiting time for closing hall door"
	        	 while(hdoorct)
			  {
			   if(!floor)break;
			   WDRS=1;    //for Watchdog reset
			  }
	               }
	              else
	               {
  	                 doorcnc=1;             //doorwtc is "waiting time for closing hall door"
	        	 while(doorcnc)
			  {
			   if(!floor)break;
			   WDRS=1;    //for Watchdog reset
			  }
	               }
			 return 0;}

	if(SS66&&(!DOO))
	     {temp3=1;                        //temp3 --->  flag for getting out cause of debouncing
	      temp4=ptimer;                //save a copy of ptimer cause we need 2 timer
	      while(ptimer<temp4+Doorcdebouncet)
	               {
	                if(!SS66)
	                          {temp3=0;
	                           break;}     //Debounce 1 second for ss66
	                if(!floor)break;
	                WDRS=1;
	               }
	      if(temp3&&(!DOO))break;
	     }

  }			                    //wait for closing hall doors(SS66)


blink_en=0;
if(!floor)return 0;

_set_blink_num='O';
while(_set_blink_num&&floor)WDRS=1;


if(app_doortype!='n')                         //This part is for Automatic and semi-automatic door
{
   temp=1;				            //temp for to count number of re-closing

   while((temp<=Doornrc)||(Doornrc==0))                    //Doornrc ==> Number of Re-closing door(for normal door=0)
       {
                                               //DOO must be updated by netwrok interrupts
	WDRS=1;
        errorinclose=0;	                                    //Waiting for DOO(PHC,DO, etc.) to be free
	RELAYC=1;
	RELAYO=0;
        set_outputs_relay(1);

        ptimer=0;
	do
	  {
	    if(!floor)return 0;
	    WDRS=1;    //for Watchdog reset
	  }while((ptimer<5));

	if(!relevelingmf)_hall_button_during_doorclose='a';
	  else  _hall_button_during_doorclose=0;

	while((ptimer<=Doordur)&&floor)      //Doordur : maximum estimated
	  {                                 //time for closing car door
	      WDRS=1;

	      _inp_in_use = 1;
	      interpreter();     ////******************
	      _inp_in_use = 0;

	      if(DOO||(!SS66)||(_hall_button_during_doorclose==1))              //PHC, DO , ... is active during closing door
			{
			 RELAYC=0;
			 RELAYO=1;	             //Re-open door if hall door was be opened
                         set_outputs_relay(1);

			 blink_en=1;
			 _set_blink_num='B';
                         while(_set_blink_num)
                          {
			    if(!floor)return 0;
			    WDRS=1;    //for Watchdog reset
			  }

                         temp5=ptimer+5;
                         ptimer=0;
			 do
			   {
			    if(!floor)return 0;
			     WDRS=1;    //for Watchdog reset
			   }while((ptimer<temp5));

			 RELAYC=0;                                      //wait for complete opening as long as
			 RELAYO=0;                                      // uncomplete closing door time before(temp2)
                         set_outputs_relay(1);

			 ptimer=0;
			 do
			   {
			    if(!floor)return 0;
			    WDRS=1;    //for Watchdog reset
			   }while(ptimer<Doorsiso);

			 ptimer=0;
			 do
			   {
			    WDRS=1;
			    if((ptimer>Doorwtc) && (Doorwtc!=0))	   //-Doorsiso
					      {
    			  		        blink_en=0;
    			  		        if(!SS66)
    			  		         {
    			  		           hdoorct=1;
					           while(hdoorct)
			                            {
		                                  	    if(!floor)return 0;
		                                  	    WDRS=1;    //for Watchdog reset
		                                    }
		                                  }
		                                 else
		                                  {
    			  		           doorcnc=1;
					           while(doorcnc)
			                            {
		                                  	    if(!floor)return 0;
		                                  	    WDRS=1;    //for Watchdog reset
		                                    }
		                                  }
			   		        return 0;}
			   }while((!SS66)&&floor);

			 blink_en=0;
			 _set_blink_num='O';
                          while(_set_blink_num)
			  {
			    if(!floor)return 0;
			    WDRS=1;    //for Watchdog reset
			  }
			 temp++;
			 errorinclose=1;                              //for reclosing the door
			 ptimer=0;
			 break;
			}
	     if(SS69)
	             {
	              temp3 = 1;
	              temp4 = ptimer;
	              while(ptimer<temp4+Doorcdebouncet)
	                {
	                 if(!SS69){temp3 = 0;break;}               //Debounce a few seconds ss66
	                 WDRS=1;
	                }
	              if(temp3)break;
	             }
	   } //end while

	if(!floor)return 0;
	if(errorinclose)continue;

	//read_inputs();
	if(!SS69){
			RELAYO=0;
			RELAYC=0;
                        set_outputs_relay(1);
			ptimer=0;
			do
			  {
			    if(!floor)return 0;
			    WDRS=1;    //for Watchdog reset
			  }while((ptimer<5));

			RELAYO=1;
			RELAYC=0;
                        set_outputs_relay(1);
                        ptimer=0;
			do
			  {
			    if(!floor)return 0;
			    WDRS=1;    //for Watchdog reset
			  }while(ptimer<Doordur);
			RELAYC=0;
			RELAYO=0;
                        set_outputs_relay(1);
                        ptimer=0;
 			do
			  {
			    if(!floor)return 0;
			    WDRS=1;    //for Watchdog reset
			  }while((ptimer<5));
			if(!floor)return 0;
			temp++;
			if(temp<=Doornrc)continue;
			doorcnc=1;
			do
			  {
			    if(!floor)return 0;	//keep door open for 6s
			    WDRS=1;    //for Watchdog reset
			  }while(doorcnc);
			return 0;
		    }


	       if(Doorunderl==69)
	        {
                  if(!Dooruldelay)
                         {
                           RELAYO=0;
                           RELAYC=0;
                           set_outputs_relay(1);
                         }
	           else  _delay_unload_door = Dooruldelay;   //Door is not underloaded
	        }

       	if(!floor)return 0;
	break;
	}	//end while

if((temp>Doornrc)&&(Doornrc))
{
  doorcnc=1;
  do
   {
      if(!floor)return 0;	//keep door open for 6s
      WDRS=1;    //for Watchdog reset
   }while(doorcnc);
  return 0;
 }

}		//end if doortype



if(app_doortype!='a')
  {                                                      //This part is for semi-automatic and normal doors
/*	  if(SS68)
	      {
               RELAYO=0;
	       RELAYC=0;
	       set_outputs_relay(1);
	       doorlcc=1;
	       while(doorlcc);                   //Door lock is closed before closing
	      return 0;}*/

	  ptimer=0;
	  do		//wait for a half second
   	   {
	       if(!floor)return 0;	//keep door open for 6s
	       WDRS=1;    //for Watchdog reset
	     }while(ptimer<5);

	  URA=1;                                      //CAM
          set_outputs_relay(1);

	  if(!floor)return 0;

          ptimer=0;
	  do               //wait 5 seonds for sensing ss68
	      {                                              //and 1 second  to debounce it
	        WDRS=1;    //for Watchdog reset
	        if(!floor)break;
	        if(SS68)
	            {temp3=1;
	              temp4=ptimer;
	              do
	                {
                          if(!floor)break;
   	                  if(!SS68){temp3=0;break;}
   	                  WDRS=1;    //for Watchdog reset
   	                }while(ptimer<temp4+Doorcdebouncet);
                      if(temp3)break;
                     }
	      }while(ptimer<Door68t);

	  if((!SS68)||(!floor))                                             //if ss68 is not shown after a while
	        {
                  RELAYO=0;
		  RELAYC=0;
		  set_outputs_relay(1);
		  if(!floor)return 0;
	          doorlcns=1;
	          do
	   	   {
		       if(!floor)break;
		       WDRS=1;    //for Watchdog reset
		    }while(doorlcns);
		  return 0;
		 }
  }
  else  if(app_doortype=='a')                                  //This part is for automatic door
		    {
		     ptimer=0;
		     do         //wait 5 seonds for sensing ss68
		      {                                           //and 1 second  to debounce it
		        WDRS=1;    //for Watchdog reset
		        if(!floor)break;
	                if(SS68)
	                  {
	                   temp3=1;
	                   temp4=ptimer;
	                   do
	                       {
	                         WDRS=1;    //for Watchdog reset
	                         if(!SS68){temp3=0;break;}
	                         if(!floor)break;
	                       }while(ptimer<temp4+Doorcdebouncet);
	                   if(temp3)break;
	                  }
		        }while(ptimer<Door68t);  //end of while Door68t

		      if((!SS68)||(!floor))
		              {
		                 RELAYO=0;
		                 RELAYC=0;
		                 set_outputs_relay(1);
		                 if(!floor)return 0;
		                 doorcnc=1;
				 do    //door contact is not closed after a while
				   {
				     if(!floor)break;
				     WDRS=1;    //for Watchdog reset
				   }while(doorcnc);
		     		 return 0;
		              }
		     }

if(Doorunderl==68)
       {
                  if(!Dooruldelay)
                         {
                           RELAYO=0;
                           RELAYC=0;
                           set_outputs_relay(1);
                         }
	           else  _delay_unload_door = Dooruldelay;   //Door is not underloaded
       }

if((!SS66)||(!SS68)||(!SS69))return 0;
RELAYCed = 1;
if(Eth_number)_send_door_status=1;            //Ethernet Monitornig packet
return 1;
}



unsigned char doorclosestandby(void)
{
unsigned char temp=0,temp2=0,temp3=0,errorinclose=0,app_doortype;
unsigned int temp4=0;


if(Tunnel_Door_en=='2')
 {
    if(Doortype=='a' && Door2type=='n')app_doortype='s';
    else
    if(Doortype=='n' && Door2type=='a')app_doortype='s';
    else
    if(Doortype=='n' && Door2type=='n')app_doortype='n';
    else
    if(Doortype=='a' && Door2type=='a')app_doortype='a';
    else app_doortype='s';
 }
else app_doortype = Doortype;

WDRS=1;    //for Watchdog reset

RELAYCed = 0;
RELAYOed = 0;

if(SS66 && SS69)
	{
	  RELAYCed = 1;
	  if(Doorparkunderl=='n')
		{
                  if(!Dooruldelay)
                         {
                           RELAYO=0;
                           RELAYC=0;
                           set_outputs_relay(1);
                           return 1;
                         }
		 _delay_park_unload_door = Dooruldelay;
		 RELAYC=1;
		 RELAYO=0;
                 set_outputs_relay(1);
		}
 	        else
 	        {
	             RELAYC=1;
	             RELAYO=0;
                     set_outputs_relay(1);
                 }

	 return 1;                          //if door was closed, return 1 and get out
	}


WDRS=1;    //for Watchdog reset

if((!SS66)||(!standbymf)||DOO)return 0;

if(!floor)return 0;

if(Doortype!='n')                         //This part is for Automatic and semi-automatic door
{
   temp=1;				            //temp for to count number of re-closing
   while((temp<=Doornrc)||(Doornrc==0))                    //Doornrc ==> Number of Re-closing door(for normal door=0)
       {
	                			//DOO must be updated by netwrok interrupts
	WDRS=1;                               //for Watchdog reset
        errorinclose=0;	                //Waiting for DOO(PHC,DO, etc.) to be free
	RELAYC=1;
	RELAYO=0;
        set_outputs_relay(1);

        ptimer=0;
	do      //Doordur : maximum estimated
	  {                                 //time for closing car door
             WDRS=1;    //for Watchdog reset
             if(!floor)break;
             if((!SS66)||(!standbymf)||DOO)return 0;
	     if(SS69)
	             {temp3=1;
	              temp4=ptimer;
	              do
	                 {
	                   if(!floor)break;
	                   if(!SS69){temp3=0;break;}     //Debounce 1 second for ss66
	                   WDRS=1;    //for Watchdog reset
	                 }while(ptimer<temp4+Doorcdebouncet);
	              if(temp3){break;}
	              }
	   }while((ptimer<Doordur)); //end while doordur

        if((!SS69)||(!floor))return 0;

	if(Doorparkunderl=='n')
	   {
                  if(!Dooruldelay)
                         {
                           RELAYO=0;
                           RELAYC=0;
                           set_outputs_relay(1);
                         }
	          else _delay_park_unload_door = Dooruldelay;    //Door is not underloaded
	   }

	break;
	}	//end while
}		//end if doortype


if(!floor)return 0;
if((!SS66)||(!standbymf)||DOO)return 0;

RELAYCed = 1;
if(Eth_number)_send_door_status=1;            //Ethernet Monitornig packet
return 1;
}


/*
   it returns 0  for changing  retry in closing door
                  1 for completing closing process of door  or a critical fault
*/

unsigned char doorcloserevision(void)
{
unsigned char temp=0,temp2=0,temp3=0,errorinclose=0,app_doortype;
unsigned int temp4=0;


if(Tunnel_Door_en=='2')
 {
    if(Doortype=='a' && Door2type=='n')app_doortype='s';
    else
    if(Doortype=='n' && Door2type=='a')app_doortype='s';
    else
    if(Doortype=='n' && Door2type=='n')app_doortype='n';
    else
    if(Doortype=='a' && Door2type=='a')app_doortype='a';
    else app_doortype='s';
 }
else app_doortype = Doortype;


RELAYSTBY=0;

read_inputs();
if(SS66 && SS69 && SS68)
	{
          if(app_doortype!='a')
	        {
	          URA=1;
	        }
	   if((Doorunderl==68)||(Doorunderl==69))
		{
                  if(!Dooruldelay)
                         {
                           RELAYO=0;
                           RELAYC=0;
                           set_outputs_relay(1);
                           return 1;
                         }
                   _delay_revision_unload_door = Dooruldelay;   	//Door is not underloaded
	           RELAYC=1;
	           RELAYO=0;
                   set_outputs_relay(1);
		}
	else {
	             RELAYC=1;
	             RELAYO=0;
                     set_outputs_relay(1);
                 }

	 return 1;                          //if door was closed, return 1 and get out
	}

WDRS=1;

read_inputs();
if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
if((opmode=='c')&&(!REVC))return 0;
if((opmode=='m')&&(!REVM))return 0;

ptimer=0;
while(!SS66)                       //SS66 ==> Halldoor Contact
  {
        WDRS=1;    //for Watchdog reset
        delay_ms(100);
        ptimer++;

        LED_indicator();
        interpreter();
        read_inputs();
        if(Eth_number)netstackService();    	// service the network
	if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
        if((opmode=='c')&&(!REVC))return 0;
        if((opmode=='m')&&(!REVM))return 0;

	if((ptimer>Doorwtc)&&(Doorwtc!=0))return 0;

	if(SS66)
	     {temp3=1;                        //temp3 --->  flag for getting out cause of debouncing
	      temp4=ptimer;                //save a copy of ptimer cause we need 2 timer
	      do
	         {
	                 WDRS=1;    //for Watchdog reset
	                 delay_ms(100);
	                 ptimer++;
	                 read_inputs();
	                 LED_indicator();
	                 if(Eth_number)netstackService();    	// service the network
	                 interpreter();
	                 if(!SS66){temp3=0;break;}     //Debounce 1 second for ss66
	         }while(ptimer<temp4+Doorcdebouncet);
	      if(temp3)break;
	     }

  }

WDRS=1;    //for Watchdog reset

read_inputs();
if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
if((opmode=='c')&&(!REVC))return 0;
if((opmode=='m')&&(!REVM))return 0;

if(app_doortype!='n')                         //This part is for Automatic and semi-automatic door
{
   while(1)                    //Doornrc ==> Number of Re-closing door(for normal door=0)
       {
	WDRS=1;    //for Watchdog reset
	RELAYC=1;
	RELAYO=0;
        set_outputs_relay(1);
        ptimer=0;
	do      //Doordur : maximum estimated
	  {                                 //time for closing car door
            WDRS=1;    //for Watchdog reset
            delay_ms(100);
            ptimer++;
            LED_indicator();
            interpreter();
            read_inputs();
            if(Eth_number)netstackService();    	// service the network
            if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
            if((opmode=='c')&&(!REVC))return 0;
            if((opmode=='m')&&(!REVM))return 0;

	     if(SS69)
	             {
	                temp3=1;
	                temp4=ptimer;
	                do
	                 {
	                   WDRS=1;    //for Watchdog reset
	                   delay_ms(100);
	                   ptimer++;
                           LED_indicator();
                           interpreter();
	                   read_inputs();
	                   if(Eth_number)netstackService();    	// service the network
	                   if(!SS69){temp3=0;break;}     //Debounce 1 second for ss66
	                 }while(ptimer<temp4+Doorcdebouncet);
	                read_inputs();
	                if(temp3)break;
	              }

	  }while(ptimer<Doordur); //end while

        read_inputs();
        if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
        if((opmode=='c')&&(!REVC))return 0;
        if((opmode=='m')&&(!REVM))return 0;

        if(!SS69)
         {
            RELAYO=0;
            RELAYC=0;
            set_outputs_relay(1);
            return 0;
         }

	   if(Doorunderl==69)
	    {
                  if(!Dooruldelay)
                         {
                           RELAYO=0;
                           RELAYC=0;
                           set_outputs_relay(1);
                         }
	         else  _delay_revision_unload_door = Dooruldelay;	//Door is not underloaded
	    }

	break;
	}	//end while
}		//end if doortype

WDRS=1;    //for Watchdog reset

if(app_doortype!='a')
	{                                   //This part is for semi-automatic and normal doors
	  read_inputs();

	  ptimer=0;
	  do
	     {
 	       WDRS=1;    //for Watchdog reset
 	       interpreter();
 	       LED_indicator();
 	       read_inputs();
 	       if(Eth_number)netstackService();    	// service the network
  	       delay_ms(100);
	       ptimer++;
	     }while(ptimer<5);		//wait for a half second

             read_inputs();
             if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
             if((opmode=='c')&&(!REVC))return 0;
             if((opmode=='m')&&(!REVM))return 0;

             URA=1;                            //CAM
             set_outputs_relay(1);
	     ptimer=0;
	     do
	                {
	                  WDRS=1;    //for Watchdog reset
	                  delay_ms(100);
	                  ptimer++;
                          LED_indicator();
                          interpreter();
                          if(Eth_number)netstackService();    	// service the network
                        }while(ptimer<5);		//wait for a half second

            read_inputs();
            if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
            if((opmode=='c')&&(!REVC))return 0;
            if((opmode=='m')&&(!REVM))return 0;
	    ptimer=0;
	    while(!SS68)
			    {
			     WDRS=1;    //for Watchdog reset
			     ptimer++;
			     delay_ms(100);
			     if(ptimer>Door68t)return 0;
                              read_inputs();
                              interpreter();
                              if(Eth_number)netstackService();    	// service the network
                              if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
                              if((opmode=='c')&&(!REVC))return 0;
                              if((opmode=='m')&&(!REVM))return 0;
                             }
}
else if(app_doortype=='a')        //This part is for automatic door
		    {
		     ptimer=0;
		     do                        //5 seconds for sensing ss68
		      {
		        WDRS=1;    //for Watchdog reset
		        delay_ms(100);
		        ptimer++;
                        LED_indicator();
                        interpreter();
                        read_inputs();
                        if(Eth_number)netstackService();    	// service the network
                        if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
                        if((opmode=='c')&&(!REVC))return 0;
                        if((opmode=='m')&&(!REVM))return 0;

	                if(SS68)
	                  {temp3=1;
	                   temp4=ptimer;
	                   do
	                     {
	                       WDRS=1;    //for Watchdog reset
	                       delay_ms(100);
	                       ptimer++;
                               LED_indicator();
                               interpreter();
                               if(Eth_number)netstackService();    	// service the network
	                       if(!SS68){temp3=0;break;}     //Debounce 1 second for ss66
	                     } while(ptimer<temp4+Doorcdebouncet);
	                   if(temp3)break;
	                  } //if ss68

		      }while((ptimer<Door68t));  //while
		       read_inputs();
		       if(!SS68)return 0;
		     }

if(Doorunderl==68)
        {
         if(!Dooruldelay)
           {
             RELAYO=0;
             RELAYC=0;
             set_outputs_relay(1);
           }
           else _delay_revision_unload_door = Dooruldelay;   	//Door is not underloaded
         }

WDRS=1;    //for Watchdog reset
if((!SS66)||(!SS68)||(!SS69))return 0;
else return 1;
}

unsigned char dooropen (void)
{
  char temp1;
  unsigned int doordur_opentemp=0,app_doortype;


if(Tunnel_Door_en=='2')
 {
    temp1=Tunnel_door_find(floor);
    if(temp1==2)app_doortype=Door2type;                     //only 2nd door is 00 1st is close
    else
    if(temp1==3)                                            //both of doors are open
       {
         if(Doortype=='n' && Door2type=='n')app_doortype='n';
         else  app_doortype=Doortype;
       }
    else app_doortype=Doortype;                     //only 1st door is 00 2nd is close
 }
else app_doortype = Doortype;


if(Doordur_open<10)doordur_opentemp=Doordur;
else doordur_opentemp=Doordur_open;

RELAYCed = 0;
RELAYOed = 0;

if(((!CF1)&&(Fsflag!=2))||(!(CF1&&CF3)&&Fsflag==2))
      {carniva=1;
	 while(carniva)
	    {
	      WDRS=1;    //for Watchdog reset
	      if(!floor)break;
	    }
	 return 0;}		                  //car is not in valid area

URA=0;                 //Deactivate URA relay for opening door's lock

if(app_doortype=='n')
	{
	 set_outputs_relay(1);
	}

WDRS=1;    //for Watchdog reset

if(!floor)return 0;

if(app_doortype!='n')
	{
	 temp1=0;                            //1
	 while((temp1<Doornrc)||(Doornrc==0))
	 {
	  if((Preopening_en=='s')&&(app_doortype=='a')&&(ptimer_preopening>0))
	      {
	       ptimer=ptimer_preopening;
	       ptimer_preopening=0;
	      }
           else
            {
   	          WDRS=1;    //for Watchdog reset
        	  RELAYC=0;
        	  RELAYO=1;                         //Turn on Open Relay for opening the door
        	  set_outputs_relay(1);
                  ptimer=0;
            }
	  do
	    {
	      if(!floor)break;
	      WDRS=1;    //for Watchdog re=set
	    }while(ptimer<doordur_opentemp);

	                                    //Doordur ==> Esimated time for opening
          RELAYC=0;                         // or closing car's door
	  RELAYO=0;
	  set_outputs_relay(1);

          if(!floor)return 0;

	  if(SS69)                          //Check door's contact after Doordur
	        {
	         temp1++;
	 	 ptimer=0;
	 	 do
	 	     	    {
                     	      if(!floor)break;
                     	      WDRS=1;    //for Watchdog reset
                     	    }while(ptimer<50);
	         continue;
	        }
	  break;
          } //end while

          if(SS69)
		{doorcno=1;
	 	  do
	                   {
          	                   if(!floor)break;
                        	    WDRS=1;    //for Watchdog reset
                     	    }while(doorcno);
		 return 0;}

	}


ptimer=0;
while(SS68)
   {
        WDRS=1;    //for Watchdog reset
        if(!floor)break;
        if(ptimer>Door68t)
		{doorlcs=1;
		 while(doorlcs)	      //Report error after 2 seconds
	                {
	                   if(!floor)break;
	                   WDRS=1;    //for Watchdog reset
	                 }
  		 return 0;}
   }		            //checking for open the lock


ptimer=0;
do
        {
         WDRS=1;    //for Watchdog reset
         if(!floor)break;
         if(ptimer>Doorsiso)break;
        }while((!DOC));
				//waiting in stair untill canceling time by doc

if(!floor)return 0;

RELAYOed = 1;
if(Eth_number)_send_door_status=1;            //Ethernet Monitornig packet

return 1;
}

char doorfire   ( void )
{return dooropen();}


unsigned char dooropenrevision (void)
{
unsigned char temp1,app_doortype;


if(Tunnel_Door_en=='2')
 {
    temp1=Tunnel_door_find(floor_cal);
    if(temp1==2)app_doortype=Door2type;                     //only 2nd door is 00 1st is close
    else
    if(temp1==3)                                            //both of doors are open
       {
         if(Doortype=='n' && Door2type=='n')app_doortype='n';
         else  app_doortype=Doortype;
       }
    else app_doortype=Doortype;                     //only 1st door is 00 2nd is close
 }
else app_doortype = Doortype;

 /*
read_inputs();
if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
if((opmode=='c')&&(!REVC))return 0;
if((opmode=='m')&&(!REVM))return 0;
if( ((!CF1)&&Fsflag<2)||(!(CF1&&CF3)&&Fsflag==2))return 0;		                  //car is not in valid area
*/
 URA=0;                 //Deactivate URA relay for opening door's lock

if(Doortype=='n')
	{
	  set_outputs_relay(1);
	}

WDRS=1;    //for Watchdog reset

if(Doortype!='n')
	{
	  WDRS=1;    //for Watchdog reset
	  if(revision_open_door_en==1)
	   {
   	     RELAYC=0;
	     RELAYO=1;                         //Turn on Open Relay for opening the door
	   }
	  else {set_outputs_relay(1); return 1;}

	  set_outputs_relay(1);
      ptimer=0;
	  do
	    {
	         read_inputs();
             interpreter();
             LED_indicator();
             if(Eth_number)netstackService();    	// service the network
             if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))return 0;
             if((opmode=='c')&&(!REVC))return 0;
             if((opmode=='m')&&(!REVM))return 0;
             WDRS=1;    //for Watchdog reset
	      delay_ms(100);
	      ptimer++;
	    }while(ptimer<Doordur);

	                                    //Doordur ==> Esimated time for opening
          RELAYC=0;                         // or closing car's door
	  RELAYO=0;
	  set_outputs_relay(1);
	}
return 1;
}



void Releveling(void)
{
unsigned char rl1,rl2;
 if((CF1&&Fsflag!=2)||(CF1&&CF3&&Fsflag==2))return;

 if(Lifttype!='h')return;

 relevelingmf=1;
 while(floor)
   {
     WDRS=1;    //for Watchdog reset
     if(!doorclose())
        {
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
        }
     else break;
   }

  relevelingmf=0;

  if(!floor)return;

  if(!SS68)return;

  I=1;

  while(floor)
    {
/*//*********/

             CONTD=0;
             CONTS=1;
	     set_outputs_relay(0);			      //CONTS for Motor
	     rl1=0;
	     rl2=0;
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

	            if((ptimer>=Hyd_S2D_Time)&&(!rl1))
	              {
	               CONTL=1;
	               set_outputs_relay(0);			      //contl for delta
	               rl1=1;
	               if(rl2)break;
	              }


	            if((ptimer>=Hyd_Start_Time)&&(!rl2))
	              {
	               CONTU=1;
	               set_outputs_relay(0);
	               rl2=1;
	               if(rl1)break;
	              }

                    if(ptimer>200)break;

                  }while(1);


          if(!floor)break;

	   ptimer=0;
           do
		  {
		    WDRS=1;    //for Watchdog reset
		    if(ptimer>20)
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

      break;
    }

  if(!floor)return;


ptimer=0;                                       //Reset timer for seeking 1CF
 // while(!CF1)
 while(1)
   {
        if(Fsflag!=2)
           if(CF1)break;
        if(Fsflag==2)
           if(CF1&&CF3)break;
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
			 break;}                      //Ptimer is not reset when preopening,

   }
  if(!floor)return;

 rl1=0;
 while(rl1<Contofftu)
	               {  WDRS=1;
	                  delay_ms(100);
                         if(!floor)break;
                         rl1++;
                       }

I=0;

CONTF=0;
CONTM=0;
CONTD=0;
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
      if(!floor)return;
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
      if(!floor)return;
      CONTL=0;
      CONTS=0;         //turn the motor off
     }


set_outputs_relay(0);

ptimer=40;
do
  {
 	    if(!floor)break;
	    if(ptimer>50)
	             {contcea=1;
			  while(contcea)
                               {
                                    WDRS=1;
                                    if(!floor)break;
                               }
			  break;}
            WDRS=1;
  }while(FEEDBD || FEEDBS || FEEDBU || FEEDBF);

if(!floor)return;


}

unsigned char Tunnel_door_find(unsigned char doorfloor)
{
        // return 0 ==> return 1
        //return 1  ==> only First Door
        //return 2  ==> only Second Door
        //return 3  ==> Both of Doors
  unsigned char td1=0,td2=0,td3,td4,td5=0;
  td1 = doorfloor % 8  ;
  if(!td1)td1=8;
  td1--;

  td4= (doorfloor - 1) / 8;

  td2 = FirstDoor[td4];
  if((td2>>td1) & 0x01)td5=1;
  td2 = SecondDoor[td4];
  if((td2>>td1) & 0x01)td5+=2;

  if(!td5)td5=1;
  return td5;

}

#include "defines.h"


/***************************************************************************
/***************************************************************************
 *                                                                         *
 *          CABIN  REVISION   MODE                                         *
 *                                                                         *
/***************************************************************************
****************************************************************************/

void revision_c(void)
{
  unsigned int rev_timestby = 0 , rev_timestby_counter;
  unsigned char rc1=10,rc2=0,rct1,rct2,rev_int_en=0;

rev_timestby = Timestndby * 10 ;     //change 0.1 sec to 0.01 sec
rev_timestby_counter = rev_timestby;


_delay_revision_unload_door = 0;

revmove=0;
REVUC=0;
REVDC=0;

#ifdef  REVC_TIMEOUT_BUTTON
revc_timeout = REVC_TIMEOUT_BUTTON ;
#endif

read_inputs();
while(REVC)       //REVC
  {
        WDRS=1;    //for Watchdog reset
        if(needreset)break;
        delay_ms(10);
        LED_indicator();
        read_inputs();

        #ifdef RCM_Enable
        if(!REVD)
            {REVUC=0;
             REVDC=0;}
        #endif

        interpreter();

        if(rev_int_en)  //***\\
         {
           if(Fsflag==2)
             {
               if((!CF1)&&(!CF3))rev_int_en++;
                    else rev_int_en=1;
               if(rev_int_en>50)
                {
                   rev_int_en=0;
                   _int5_en=1;
                   _int4_en=1;
                }
             }
           else
            {
              if(!(CF1))rev_int_en++;
                 else rev_int_en=1;
              if(rev_int_en>50)
                {
                  rev_int_en=0;
                  _int5_en=1;
                }
            }
           if(!revmove)rev_int_en=0;
         }

        if(++rc2>9)                                       //for 100 ms cycle in netstackservice
          {if(Eth_number)netstackService();    	// service the network
           rc2=0;}

        if((!revmove)&&rev_timestby_counter)
           {
            rev_timestby_counter--;
            if(!rev_timestby_counter)
               {
                RELAYSTBY=1;
                set_outputs_relay(1);
               }
           }
        if(revmove)rev_timestby_counter = rev_timestby ;     //change 0.1 sec to 0.01 sec


        #ifdef REVC_TIMEOUT_BUTTON
        if(revmove&&revc_timeout)revc_timeout--;

        if(revmove&&(!revc_timeout))
           {
              REVUC = 0;
              REVDC = 0;
           }
         #endif

        if((Doorunderl==68)||(Doorunderl==69))
         {
           if(_delay_revision_unload_door)                    //cancel unloading the door
               if((RELAYC==0)||(RELAYO==1))_delay_revision_unload_door=0;

            if(_delay_revision_unload_door)                     //unload the door after a few seconds
              {
                if((--rc1)==0)
                 {
                  rc1=10;
                  _delay_revision_unload_door--;
                 if(_delay_revision_unload_door==0)
                   {
                     if((RELAYC==1)&&(RELAYO==0))
                           {
                                 RELAYC=0;
                                 RELAYO=0;
                                 set_outputs_relay(1);
                           }
                   }
                 }
              }
           }

	if(!POWER110)
	        {
	          errormsg(52);
	          break;
	        }
	else
	if(!SS63)
		{
		 errormsg(63);                //Only report the error
		 break;
		}
        else
	if(!SS64)
		{
		 errormsg(64);
		 break;
		 }
        else
	if(!SS65)
		{
		 errormsg(65);
		 break;
		 }
        else
 	if((!SS66)&&revmove)
      	  {
      	     errormsg(66);
	     break;
	  }
        else
 	if((!SS69)&&revmove)
      	  {
      	     errormsg(69);
	     break;
	  }
        else
 	if((!SS68)&&revmove)
      	  {
      	     errormsg(68);
	     break;
	  }

	if(!POWER24)
	        {
	         errormsg(51);
	         break;
	         }
        else
	if(CA1&&CAn)
		{
		 errormsg(71);
		 break;
		}
	else
	if(CF1&&CF3&&(Fsflag<2))
		{
		 errormsg(72);
		 break;
		}

	if(REVUC&&REVDC)                 //For press REVU & REVD simultaneously
		{
                 current_message.data[0]='S';          //'S' for first digit of numerator
                 current_message.data[1]='R';
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

                      sep_id.dest_address = 0xC0;
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
                       current_message.datalen=4;
                     }

		 revmove=0;
		 CONTU = 0;
		 CONTD = 0;
		 CONTF = 0;
		 CONTS = 0;
		 CONTL = 0;
		 CONTM = 0;
		 set_outputs_relay(0);
		 ptimer=30;
		 if(Lifttype=='d')ptimer=0;
		 if(Lifttype=='v')ptimer=0;
                 while(ptimer<40)
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
			{errormsg(32);			//contcea
			 needreset=1;
			 break;}
		 }

        if(FTOC&&(!FTO))
                {errormsg(83);
                 FTOC=0;
                 WDRS=1;    //for Watchdog reset
                 delay_ms(100);
                 continue;}

        if(FLTC&&(!FLT))
                {errormsg(83);
                 FLTC=0;
                 WDRS=1;    //for Watchdog reset
                 delay_ms(100);
                 continue;}

	if(revmove)
		{
		 if((CAn&&REVUC)||(CA1&&REVDC)||((!REVDC)&&(!REVUC))||(FTO)||(FLT)
		 #ifdef  RCM_Enable
		 ||(!REVD)
  		 #endif
		 )
		      {
                     	if(FTO&&(!FTOC))
             		   {errormsg(82);
        		    FTOC=1;}

                     	if(FLT&&(!FLTC))
        	            {//errormsg(91);
        		     FLTC=1;}

		       revmove=0;
		       rev_int_en=0;

                       WDRS=1;    //for Watchdog reset

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

                         sep_id.dest_address = 0xC0;
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
                       current_message.datalen=4;
                     }

		       CONTL=0;
		       CONTM=0;
		       if((Lifttype!='v')||(Dir_delay=='y'))
		         {CONTU=0;
		          CONTD=0;}                      //For break the movement
		       CONTS=0;                     //in invalid conditions such as:
		       CONTF=0;                     //1-Sensing CAn when going up
		       set_outputs_relay(0);    //2-Sensing CA1 when going down
		                                       //3-Sensing FTO when moving
		                                       //4-Cutting Cabin Safety-Serie when moving
		                                       //5-No sensing any Up/Down key

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
		       ptimer=30;
		       if(Lifttype=='d')ptimer=0;
		       if(Lifttype=='v')ptimer=0;
		       while(FEEDBD || FEEDBU || FEEDBS || FEEDBF)
		         {
                           WDRS=1;    //for Watchdog reset
                           delay_ms(100);
                           ptimer++;                         //delay 0.5 seconds
                           read_inputs();
                           LED_indicator();
                           interpreter();
                           if(Eth_number)netstackService();    	// service the network
		            if(ptimer>40)
	           		  {errormsg(32);
				       needreset=1;
				       break;}
		         }
    		   if(Lifttype=='v')
		         {CONTU=0;
		          CONTD=0;
		          set_outputs_relay(0);}
		       if(needreset==1)break;

               if((CF1 && Fsflag!=2) || (CF1&&CF3&&(Fsflag==2)))                           //stop on stair
                    {
        	           if(!dooropenrevision())
                     		  {
                     		       /*RELAYO=0;
                     		       RELAYC=0;*/
                     		       URA=0;
                     		       set_outputs_relay(1);
                                   WDRS=1;    //for Watchdog reset
                                   read_inputs();
                                   if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                                   if(!REVC)break;
		                      }
                    } //end if cf1
                 else
                   {
                       if(Doortype!='a')
                             {
                                   WDRS=1;
                                   URA=0;
                                   set_outputs_relay(1);
                             }
                   }

		       }

 		 continue;
		}

	if(REVUC&&(!CAn)&&(!FTO)&&(!FLT))
	 {                                  //Starting Up-ward cause press Up key
	      while(REVC)
     	      {	    //wait here to complete closing door
                 WDRS=1;    //for Watchdog reset
		 read_inputs();
		 interpreter();
		 if(Eth_number)netstackService();    	// service the network
		 if( (!REVUC) || CAn || (!REVC) || (REVDC)
		 #ifdef  RCM_Enable
		 ||(!REVD)
  		 #endif
		 )break;

              if(!doorcloserevision())
                {
                 WDRS=1;    //for Watchdog reset
                 rev_timestby_counter = rev_timestby;
                 RELAYO=0;
                 RELAYC=0;
                 URA=0;
                 RELAYSTBY=0;
                 set_outputs_relay(1);
                 read_inputs();
                 if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                 if(!REVC)break;

                 errormsg(49);
                 ptimer=0;
                 while(ptimer<50)
                     {
                         WDRS=1;    //for Watchdog reset
                         delay_ms(100);
                         ptimer++;                         //delay 5 seconds in start
                         LED_indicator();
                         interpreter();
                         if(Eth_number)netstackService();    	// service the network
                         read_inputs();
                        if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                        if(!REVC)break;
                      }
                 read_inputs();
                 if(!REVC)break;
                 if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                 errormsg(83);

                 read_inputs();
                 if(!REVC)break;
                 continue;
                }

              //**************

                                      //***\\
              if(Fsflag==2)
              {
               if((CF1&&CF3)||(dir_cal==0))
                {
                   EIMSK=0x00;
                   EIFR=0x30;
                   rev_int_en=1;
                }
               if(dir_cal==0)dir_prev=1;
              }
              else
              {
               if((CF1)||(dir_cal==0))
                   {
                    EIMSK=0x00;
                    EIFR=0x20;
                    rev_int_en=1;
                   }
               if(dir_cal==0)dir_prev=1;
              }
              dir_cal=1;

               revmove=1;
              if( (!REVUC)
		 #ifdef  RCM_Enable
		 ||(!REVD)
  		 #endif
		 )break;

                         current_message.data[0]='S';          //'S' for first digit of numerator
                         current_message.data[1]='R';
                         current_message.data[3]='U';
                         current_message.datalen=4;
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
                            serial_current_message.data[3]='U';
                            RS485_send_message(serial_current_message);

                            sep_id.dest_address = 0xC0;
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
                      serial_current_message.data[3]='U';
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
                       current_message.data[5]='U';
                       can_send_message(current_message);
                       delay_ms(10);
                       current_message.datalen=4;
                     }

                          recovery(3);                 //erase last saved floor

            if(Lifttype=='h')
	         {
    	             CONTS=1;
        	     set_outputs_relay(0);			      //CONTS for Motor
        	     rct1=0;
        	     rct2=0;
                     ptimer=0;
                     while(1)
                         {
                            WDRS=1;    //for Watchdog reset
                            delay_ms(100);
                            ptimer++;                         //delay 0.5 seconds
                            read_inputs();
                            LED_indicator();
                            interpreter();
                            if(Eth_number)netstackService();    	// service the network
                            if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                            if(!REVC)break;
                            if( (!REVUC)
        		       #ifdef  RCM_Enable
            		        ||(!REVD)
                         	 #endif
	        	        )break;
                            if((ptimer>20)&&(!FEEDBS))
                                   {errormsg(33);
   				       needreset=1;
     				       break;}


	                     if((ptimer>=Hyd_S2D_Time)&&(!rct1))
        	              {
        	               CONTL=1;
        	               set_outputs_relay(0);			      //contl for delta
        	               rct1=1;
        	               if(rct2)break;
        	              }


	                     if((ptimer>=Hyd_Start_Time)&&(!rct2))
	                             {
                	               CONTU=1;
                	               set_outputs_relay(0);
                	               rct2=1;
                	               if(rct1)break;
                	              }

                             if(ptimer>200)break;
                          }


	          }

                         if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                         if(!REVC)break;
                         if( (!REVUC)
        		 #ifdef  RCM_Enable
            		        ||(!REVD)
                         #endif
	        	        )break;

			 CONTU=1;
			 CONTD=0;
			 set_outputs_relay(0);
			 WDRS=1;
			 delay_ms(100);
        		 if((Lifttype=='v')||(Lifttype=='d'))
			           {CONTL=0;
			            CONTS=0;
			            CONTM=1;
                     		    CONTF=0;}
                         else
                         if(Lifttype=='h')
			           {CONTM=0;
                     		    CONTF=0;}
                         else
			           {CONTS=1;         //select a speed for
                     		    CONTF=0;}       //3vf or 2speed

			 set_outputs_relay(0);
		         ptimer=0;
                         while(ptimer<2)
                          {
                            WDRS=1;    //for Watchdog reset
                            delay_ms(100);
                            ptimer++;                         //delay 0.5 seconds
                            LED_indicator();
                            interpreter();
                            if(Eth_number)netstackService();    	// service the network
                          }


	                 read_inputs();
			  if((Lifttype=='v')||(Lifttype=='d'))
			    {
        		         ptimer=0;
                                 while(!FEEDBF)
                                     {
                                           WDRS=1;    //for Watchdog reset
                                           delay_ms(100);
                                           ptimer++;                         //delay 0.5 seconds
                                           read_inputs();
                                           LED_indicator();
                                           interpreter();
                                           if(Eth_number)netstackService();    	// service the network
                                  	   if(ptimer>20)
                                             {errormsg(33);
        				       needreset=1;
        				       break;}
                                     }
                                  if(needreset)break;
			     }
			  else
	                    {
	                        if((!(FEEDBU&&FEEDBS))||FEEDBD||FEEDBF)
                                     {errormsg(33);
				       needreset=1;
				       break;}
			    }
			 ptimer=0;
     	                 do
	                  {
                            WDRS=1;    //for Watchdog reset
                            if(!Timebreakdelay)break;
                            delay_ms(100);
                            ptimer++;
                            interpreter();
                            LED_indicator();
                            read_inputs();
                            if(Eth_number)netstackService();    	// service the network
       	                    if(ptimer>Timebreakdelay)break;
	                   }while(!FEEDB4BS);                //Check Break  feedback

                         if((!FEEDB4BS)&&(Timebreakdelay))
                                      {errormsg(34);
				       needreset=1;
				       break;}

                         recovery(4);           //to increase number_of_starts
			 revmove=1;
			 read_inputs();
			 break;
			}
		 }

	if(REVDC&&(!CA1)&&(!FTO)&&(!FLT))
		{
		 while(REVC)                            //Starting Downward cause press Down key
			{
                         WDRS=1;    //for Watchdog reset
			 read_inputs();
			 interpreter();                  //wait here to complete closing door
			 if(Eth_number)netstackService();    	// service the network
			 if( (!REVDC) || CA1 || (!REVC) || (REVUC)
        		 #ifdef  RCM_Enable
        		 ||(!REVD)
          		 #endif
        		 )break;

	 //************************

          if(!doorcloserevision())
            {
               WDRS=1;    //for Watchdog reset
               rev_timestby_counter = rev_timestby;
               RELAYO=0;
               RELAYC=0;
               URA=0;
               set_outputs_relay(1);
               read_inputs();
               if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
               if(!REVC)break;

               errormsg(49);
               ptimer=0;
               while(ptimer<50)
                     {
                         WDRS=1;    //for Watchdog reset
                         delay_ms(100);
                         ptimer++;                         //delay 5 seconds in start
                         LED_indicator();
                         interpreter();
                         if(Eth_number)netstackService();    	// service the network
                         read_inputs();
                        if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                        if(!REVC)break;
                      }
               read_inputs();

               if(!REVC)break;

               if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
               errormsg(83);     //revc

               read_inputs();
               if(!REVC)break;
               continue;
            }

             if(Fsflag==2)                //***\\
              {
               if((CF1&&CF3)||(dir_cal==0))
                {
                   EIMSK=0x00;
                   EIFR=0x30;
                   rev_int_en=1;
                }
               if(dir_cal==0)dir_prev=2;
              }
              else
              {
               if((CF1)||(dir_cal==0))
                   {
                    EIMSK=0x00;
                    EIFR=0x20;
                    rev_int_en=1;
                   }
               if(dir_cal==0)dir_prev=2;
              }
              dir_cal=2;


             revmove=1;
             if( (!REVDC)
   	     #ifdef  RCM_Enable
	      ||(!REVD)
  	     #endif
	     )break;

                        //**************

                         current_message.data[0]='S';          //'S' for first digit of numerator
                         current_message.data[1]='R';
                         current_message.data[3]='D';
                         current_message.datalen=4;
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
                            serial_current_message.data[3]='D';
                            RS485_send_message(serial_current_message);

                            sep_id.dest_address = 0xC0;
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
                      serial_current_message.data[3]='D';
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
                       current_message.data[5]='D';
                       can_send_message(current_message);
                       delay_ms(10);
                       current_message.datalen=4;
                     }

                         recovery(3);			 //erase last saved floor

			 CONTD = 1;
			 CONTU = 0;
			 set_outputs_relay(0);
                         WDRS=1;    //for Watchdog reset
			 delay_ms(100);

			 if((Lifttype=='v')||(Lifttype=='d'))
			           {CONTL=0;
			            CONTS=0;
			            CONTM=1;
                     		    CONTF=0;}
                         else
			 if(Lifttype=='h')
			           {CONTL=0;
			            CONTS=0;
			            CONTM=0;
                     		    CONTF=0;}
                        else
			           {CONTS=1;                //3vf or 2speed
                     		    CONTF=0;}
			 set_outputs_relay(0);
		         ptimer=0;
                         while(ptimer<2)
                          {
                            WDRS=1;    //for Watchdog reset
                            delay_ms(100);
                            ptimer++;                         //delay 0.5 seconds
                            LED_indicator();
                            interpreter();
                            if(Eth_number)netstackService();    	// service the network
                          }
	                 read_inputs();
			 if((Lifttype=='v')||(Lifttype=='d'))
			    {
        		         ptimer=0;
                                 while(!FEEDBF)
                                     {
                                           WDRS=1;    //for Watchdog reset
                                           delay_ms(100);
                                           ptimer++;                         //delay 0.5 seconds
                                           read_inputs();
                                           LED_indicator();
                                           interpreter();
                                           if(Eth_number)netstackService();    	// service the network
                                  	   if(ptimer>20)
                                             {errormsg(33);
        				       needreset=1;
        				       break;}
                                     }
                                  if(needreset)break;

			     }
			  else
			 if(Lifttype=='h')
			    {
			      if((!FEEDBD)||FEEDBS||FEEDBU||FEEDBF)
                                     {errormsg(33);
				       needreset=1;
				       break;}
			    }
			  else
	                    {
			      if((!(FEEDBD&&FEEDBS))||FEEDBU||FEEDBF)
                                     {errormsg(33);
				       needreset=1;
				       break;}
			    }
			 ptimer=0;
     	                 do
	                  {
                            WDRS=1;    //for Watchdog reset
                            if(!Timebreakdelay)break;
                            delay_ms(100);
                            ptimer++;
                            interpreter();
                            LED_indicator();
                            read_inputs();
                            if(Eth_number)netstackService();    	// service the network
       	                    if(ptimer>Timebreakdelay)break;
	                   }while(!FEEDB4BS);                 //Check Break  feedback

                         if((!FEEDB4BS)&&(Timebreakdelay))
                                      {errormsg(34);
				       needreset=1;
				       break;}


                         recovery(4);           //to increase number_of_starts
			 revmove=1;
			 break;
			}
		}
 }

}



/***************************************************************************
/***************************************************************************
 *                                                                                                                      *
 *                                       PANEL     REVISION   MODE                                               *
 *                                                                                                                      *
/***************************************************************************
****************************************************************************/


void revision_m(void)
{
  unsigned int rev_timestby = 0 , rev_timestby_counter;
  unsigned char rp1=10,rp2=0,rev_int_en=0,rmt1,rmt2;

rev_timestby = Timestndby * 10 ;     //change 0.1 sec to 0.01 sec
rev_timestby_counter = rev_timestby;

_delay_revision_unload_door=0;

revmove=0;
while(REVM&&(!REVC))
 {
        WDRS=1;    //for Watchdog reset
        if(needreset)break;
        delay_ms(10);
        LED_indicator();
	read_inputs();
	interpreter();


        if(rev_int_en)
         {

           if(Fsflag==2)
             {
               if((!CF1)&&(!CF3))rev_int_en++;
                 else rev_int_en=1;
               if(rev_int_en>50)
                {rev_int_en=0;
                 _int5_en=1;
                 _int4_en=1;}
             }
           else
            {
              if(!CF1)rev_int_en++;
                 else rev_int_en=1;
              if(rev_int_en>50)
               {rev_int_en=0;
                _int5_en=1;}

            }
          if(!revmove)rev_int_en=0;
         }


	if(++rp2>9)
	   {if(Eth_number)netstackService();    	// service the network
	    rp2=0;}

        if((!revmove)&&rev_timestby_counter)
           {
            rev_timestby_counter--;
            if(!rev_timestby_counter)
               {
                RELAYSTBY=1;
                set_outputs_relay(1);
               }
           }
        if(revmove)rev_timestby_counter = rev_timestby ;     //change 0.1 sec to 0.01 sec


        if((Doorunderl==68)||(Doorunderl==69))
         {
           if(_delay_revision_unload_door)                    //cancel unloading the door
               if((RELAYC==0)||(RELAYO==1))_delay_revision_unload_door=0;

            if(_delay_revision_unload_door)                     //unload the door after a few seconds
              {
                if((--rp1)==0)
                 {
                  rp1=10;
                  _delay_revision_unload_door--;
                 if(_delay_revision_unload_door==0)
                   {
                     if((RELAYC==1)&&(RELAYO==0))
                           {
                                 RELAYC=0;
                                 RELAYO=0;
                                 set_outputs_relay(1);
                           }
                   }
                 }
              }
           }


        if(!POWER110)
           {
             errormsg(52);
              break;
            }
        else
	if(!SS63)
      	 {
      	    errormsg(63);
	    break;
	 }	                  //Only report the error
	else                                   //stay in Revision mode
	if(!SS64)                           //until solving problem
	  {
	    errormsg(64);
	    break;
	  }
        else
 	if(!SS65)
      	  {
      	     errormsg(65);
	     break;
	  }
        else
 	if((!SS66)&&revmove)
      	  {
      	     errormsg(66);
	     break;
	  }
        else
 	if((!SS69)&&revmove)
      	  {
      	     errormsg(69);
	     break;
	  }
        else
 	if((!SS68)&&revmove)
      	  {
      	     errormsg(68);
	     break;
	  }

        if(!POWER24)
                {
                  errormsg(51);
                  break;
                }
        else
	if(CA1&&CAn)
		{errormsg(71);
		 break; 		//if ca1 & can is active simultaneously
		}                 //get out till reset
	else
	if(CF1&&CF3&&(Fsflag<2))
		{
		 errormsg(72);
		 break;
		}


	if(REVU && REVD)
		{
		 revmove=0;
        	 WDRS=1;    //for Watchdog reset

                 current_message.data[0]='S';          //'S' for first digit of numerator
                 current_message.data[1]='R';
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

                            sep_id.dest_address = 0xC0;
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
                       current_message.datalen=4;
                     }

		 CONTU = 0;
		 CONTD = 0;
		 CONTF = 0;
		 CONTS = 0;
		 CONTL = 0;
		 CONTM = 0;
		 set_outputs_relay(0);
		 ptimer=30;
		 if(Lifttype=='d')ptimer=0;
		 if(Lifttype=='v')ptimer=0;
                 while(ptimer<40)
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
			{errormsg(32);
			 needreset=1;		//contcea
			 break;}
		}

        if(FTOC&&(!FTO))
                {
                 errormsg(84);
                 FTOC=0;
                 WDRS=1;    //for Watchdog reset
                 delay_ms(100);
                 continue;}

        if(FLTC&&(!FLT))
                {
                 errormsg(84);
                 FLTC=0;
                 WDRS=1;    //for Watchdog reset
                 delay_ms(100);
                 continue;}

	if(revmove)
		{
		 if((CAn&&REVU)||(CA1&&REVD)||((!REVD)&&(!REVU))||(FTO)||(FLT))
		      {
                     	if(FTO&&(!FTOC))
             		   {errormsg(82);
        		    FTOC=1;}

                     	if(FLT&&(!FLTC))
        	            {//errormsg(91);
        		     FLTC=1;}

		       revmove=0 ;

		       rev_int_en=0;

		       WDRS=1;    //for Watchdog reset

                       current_message.data[0]='S';          //'S' for first digit of numerator
                       current_message.data[1]='R';
                       current_message.data[3]='n';
                         current_message.datalen=4;
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

                            sep_id.dest_address = 0xC0;
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
                           current_message.datalen=4;
                         }

		       CONTL = 0;
		       CONTM = 0;
		       if((Lifttype!='v')||(Dir_delay=='y'))
		         {CONTU=0;
		          CONTD=0;}
		       CONTF = 0 ;                              //For break the movement
		       CONTS = 0 ;                              //in invalid conditions such as:
		       set_outputs_relay(0);                     //
		                                                // 1-Sensing CAn when going up
		                                                 // 2-Sensing CA1 when going down
	                                                       // 3-Sensing FTO when moving
		                                                // 4-Cutting Cabin Safety-Serie when moving
                                                            // 5-No sensing any Up/Down key
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
		       ptimer=30;
		       if(Lifttype=='d')ptimer=0;
		       if(Lifttype=='v')ptimer=0;
		       while(FEEDBD || FEEDBU || FEEDBS || FEEDBF)
		         {
                           WDRS=1;    //for Watchdog reset
                           delay_ms(100);
                           ptimer++;                         //delay 0.5 seconds
                           read_inputs();
                           LED_indicator();
                           interpreter();
                           if(Eth_number)netstackService();    	// service the network
		           if(ptimer>40)
	           		{errormsg(32);
				 needreset=1;
				 break;}
		         }
		       if(Lifttype=='v')
		         {CONTU=0;
		          CONTD=0;
		          set_outputs_relay(0);}
		      if(needreset==1)break;

                      if(( CF1 && (Fsflag!=2) ) || (CF1&&CF3&&(Fsflag==2)))                           //*****
                         {
        	           if(!dooropenrevision())
                     		  {
                                   WDRS=1;    //for Watchdog reset
                     		       /*RELAYO=0;
                     		       RELAYC=0;*/
                     		       URA=0;
                     		       set_outputs_relay(1);
                                       WDRS=1;    //for Watchdog reset
                                       read_inputs();
                                       if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                                       if((REVC)||(!REVM))break;
		                  }

                         } //end if cf1
                       else
                         {
                           if(Doortype!='a')
                                  {
                                       WDRS=1;
                                       URA=0;
                                       set_outputs_relay(1);
                                  }
                         }
		      }

 		 continue;
		}

	if(REVU&&(!CAn)&&(!FTO)&&(!FLT))
		{                              //Starting Upward cause press Up key
		 while(REVM&&(!REVC))
			{
                         WDRS=1;    //for Watchdog reset
                         interpreter();
                         LED_indicator();
			 read_inputs();
			 if(REVC)break;
			 if((!REVU)||CAn)break;

                         if(!doorcloserevision())
                         {
                           WDRS=1;    //for Watchdog reset
                           rev_timestby_counter = rev_timestby;
                           RELAYO=0;
                           RELAYC=0;
                           URA=0;
                           set_outputs_relay(1);
                           read_inputs();
                          if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                          if((REVC)||(!REVM))break;

                            errormsg(49);
                            ptimer=0;
                            while(ptimer<50)
                             {
                                 WDRS=1;    //for Watchdog reset
                                 delay_ms(100);
                                 ptimer++;                         //delay 5 seconds in start
                                 LED_indicator();
                                 interpreter();
                                 if(Eth_number)netstackService();    	// service the network
                                 read_inputs();
                                if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                                if((REVC)||(!REVM))break;
                              }


                          read_inputs();

                          if((REVC)||(!REVM))break;

                          if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                           errormsg(84);     //revc

                           read_inputs();
                          if((REVC)||(!REVM))break;
                          continue;
                       }



                      if(Fsflag==2)                 //***\\
                      {
                       if((CF1&&CF3)||(dir_cal==0))
                        {
                           EIMSK=0x00;
                           EIFR=0x30;
                           rev_int_en=1;
                        }
                       if(dir_cal==0)dir_prev=1;

                      }
                      else
                      {
                       if((CF1)||(dir_cal==0))
                           {
                            EIMSK=0x00;
                            EIFR=0x20;
                            rev_int_en=1;
                           }
                       if(dir_cal==0)dir_prev=1;
                      }
                      dir_cal=1;

                         revmove=1;                //**************
                         read_inputs();
                         if(!REVU)break;

                         recovery(3);	       //erase last saved floor


                   if(Lifttype=='h')
	            {
    	             CONTS=1;
        	     set_outputs_relay(0);			      //CONTS for Motor
        	     rmt1=0;
        	     rmt2=0;
                     ptimer=0;
                     while(1)
                         {
                            WDRS=1;    //for Watchdog reset
                            delay_ms(100);
                            ptimer++;                         //delay 0.5 seconds
                            read_inputs();
                            LED_indicator();
                            interpreter();
                            if(Eth_number)netstackService();    	// service the network
                            if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                            if((REVC)||(!REVM))break;

                            if((ptimer>20)&&(!FEEDBS))
                                   {errormsg(33);
   				       needreset=1;
     				       break;}


	                     if((ptimer>=Hyd_S2D_Time)&&(!rmt1))
        	              {
        	               CONTL=1;
        	               set_outputs_relay(0);			      //contl for delta
        	               rmt1=1;
        	               if(rmt2)break;
        	              }


	                     if((ptimer>=Hyd_Start_Time)&&(!rmt2))
	                             {
                	               CONTU=1;
                	               set_outputs_relay(0);
                	               rmt2=1;
                	               if(rmt1)break;
                	              }

                             if(ptimer>200)break;
                          }


	                } //if(Lifttype=='h'

                         if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                         if((REVC)||(!REVM))break;
                         if(!REVU)break;
			 CONTU = 1;
			 CONTD = 0;
			 set_outputs_relay(0);
			 WDRS=1;    //for Watchdog reset
			 delay_ms(100);
			 if((Lifttype=='v')||(Lifttype=='d'))
			           {CONTL=0;
			            CONTS=0;
			            CONTM=1;
                     		    CONTF=0;}
                         else
                         if(Lifttype=='h')
			           {CONTM=0;
                     		    CONTF=0;}
                         else
			           {CONTS=1;                //3vf or 2speed
                     		    CONTF=0;}
			 set_outputs_relay(0);
			 ptimer=0;
                         while(ptimer<2)
                            {
                               WDRS=1;    //for Watchdog reset
                               delay_ms(100);
                               ptimer++;                         //delay 0.5 seconds
                               LED_indicator();
                               interpreter();
                               if(Eth_number)netstackService();    	// service the network
                             }

	                read_inputs();
			 if((Lifttype=='v')||(Lifttype=='d'))
			    {
        		         ptimer=0;
                                 while(!FEEDBF)
                                     {
                                           WDRS=1;    //for Watchdog reset
                                           delay_ms(100);
                                           ptimer++;                         //delay 0.5 seconds
                                           read_inputs();
                                           LED_indicator();
                                           interpreter();
                                           if(Eth_number)netstackService();    	// service the network
                                  	   if(ptimer>20)
                                             {errormsg(33);
        				       needreset=1;
        				       break;}
                                     }
                                  if(needreset)break;
			    }
			else
	                    {
			      if((!(FEEDBU&&FEEDBS))||FEEDBD||FEEDBF)
                                     {errormsg(33);
				       needreset=1;
				       break;}
			    }
			ptimer=0;
     	                do
	                 {
                           WDRS=1;    //for Watchdog reset
                           if(!Timebreakdelay)break;
                           delay_ms(100);
                           interpreter();
                           LED_indicator();
                           ptimer++;
                           read_inputs();
                           if(Eth_number)netstackService();    	// service the network
       	                   if(ptimer>Timebreakdelay)break;
	                  }while(!FEEDB4BS);                 //Check Break  feedback

                         if((!FEEDB4BS)&&(Timebreakdelay))
                                      {errormsg(34);
				       needreset=1;
				       break;}

                         current_message.data[0]='S';          //'S' for first digit of numerator
                         current_message.data[1]='R';
                         current_message.data[3]='U';
                         current_message.datalen=4;
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
                             serial_current_message.data[3]='U';
                             RS485_send_message(serial_current_message);

                             sep_id.dest_address = 0xC0;
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
                              serial_current_message.data[3]='U';
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
                               current_message.data[5]='U';
                               can_send_message(current_message);
                               delay_ms(10);
                               current_message.datalen=4;
                             }

	                 recovery(4);           //to increase number_of_stat
			 revmove=1;
			 read_inputs();
			 break;
			}
		}

	if(REVD&&(!CA1)&&(!FTO)&&(!FLT))
		{
		 while(REVM&&(!REVC))                //Starting Downward cause press Up key
			{
                         WDRS=1;    //for Watchdog reset
			 read_inputs();
			 if(REVC)break;
			 if((!REVD)||CA1)break;
                         interpreter();
                         LED_indicator();

                       if(!doorcloserevision())
                          {
                            WDRS=1;    //for Watchdog reset
                            rev_timestby_counter = rev_timestby;
                            RELAYO=0;
                            RELAYC=0;
                            URA=0;
                           set_outputs_relay(1);
                           read_inputs();
                           if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                           if((REVC)||(!REVM))break;

                           errormsg(49);
                           ptimer=0;
                           while(ptimer<50)
                                {
                                    WDRS=1;    //for Watchdog reset
                                    delay_ms(100);
                                    ptimer++;                         //delay 5 seconds in start
                                    LED_indicator();
                                    interpreter();
                                    if(Eth_number)netstackService();    	// service the network
                                    read_inputs();
                                    if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;
                                    if((REVC)||(!REVM))break;
                                 }
                          read_inputs();

                         if((REVC)||(!REVM))break;

                         if((!POWER110)||(!SS63)||(!SS64)||(!SS65)||(!POWER24)||(CA1&&CAn)||(CF1&&CF3&&(Fsflag<2)))break;

                         errormsg(84);     //revm

                          read_inputs();
                          if((REVC)||(!REVM))break;
                          continue;
                         }

                            //**************



                      if(Fsflag==2)                 //***\\
                      {
                       if((CF1&&CF3)||(dir_cal==0))
                        {
                           EIMSK=0x00;
                           EIFR=0x30;
                           rev_int_en=1;
                        }
                       if(dir_cal==0)dir_prev=2;

                      }
                      else
                      {
                       if((CF1)||(dir_cal==0))
                           {
                            EIMSK=0x00;
                            EIFR=0x20;
                            rev_int_en=1;
                           }
                       if(dir_cal==0)dir_prev=2;
                      }
                      dir_cal=2;    //***\\

                     revmove=1;
                     read_inputs();
                     if(!REVD)break;

                     recovery(3);          //erase last saved floor

	             CONTD=1;
		     CONTU=0;
		     set_outputs_relay(0);
   		     WDRS=1;    //for Watchdog reset
		     delay_ms(100);
		     if((Lifttype=='v')||(Lifttype=='d'))
			           {CONTL=0;
			            CONTS=0;
			            CONTM=1;
                     		    CONTF=0;}
                         else
			 if(Lifttype=='h')
			           {CONTL=0;
			            CONTS=0;
			            CONTM=0;
                     		    CONTF=0;}
                         else
			           {CONTS=1;                //3vf or 2speed
                     		    CONTF=0;}
		    set_outputs_relay(0);
		    ptimer=0;
                    while(ptimer<2)
                          {
                            WDRS=1;    //for Watchdog reset
                            delay_ms(100);
                             ptimer++;                         //delay 0.5 seconds
                            LED_indicator();
                            interpreter();
                            if(Eth_number)netstackService();    	// service the network
                          }
	             read_inputs();
		     if((Lifttype=='v')||(Lifttype=='d'))
			    {
        		         ptimer=0;
                                 while(!FEEDBF)
                                     {
                                           WDRS=1;    //for Watchdog reset
                                           delay_ms(100);
                                           ptimer++;                         //delay 0.5 seconds
                                           read_inputs();
                                           LED_indicator();
                                           interpreter();
                                           if(Eth_number)netstackService();    	// service the network
                                  	   if(ptimer>20)
                                             {errormsg(33);
        				       needreset=1;
        				       break;}
                                     }
                                  if(needreset)break;
			     }
			  else
			 if(Lifttype=='h')
			    {
			      if((!FEEDBD)||FEEDBS||FEEDBU||FEEDBF)
                                     {errormsg(33);
				       needreset=1;
				       break;}
			    }
		        else
	                     {
			       if((!(FEEDBD&&FEEDBS))||FEEDBU||FEEDBF)
                                     {errormsg(33);
				       needreset=1;
				       break;}
			     }
			ptimer=0;
     	             do
	                 {
                           WDRS=1;    //for Watchdog reset
                           if(!Timebreakdelay)break;
                           delay_ms(100);
                           ptimer++;
                           read_inputs();
                           if(Eth_number)netstackService();    	// service the network
       	                   if(ptimer>Timebreakdelay)break;
	                  }while(!FEEDB4BS);                 //Check Break  feedback

                         if((!FEEDB4BS)&&(Timebreakdelay))
                                      {errormsg(34);
				       needreset=1;
				       break;}

                         current_message.data[0]='S';          //'S' for first digit of numerator
                         current_message.data[1]='R';
                         current_message.data[3]='D';
                         current_message.datalen=4;
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
                             serial_current_message.data[3]='D';
                             RS485_send_message(serial_current_message);

                            sep_id.dest_address = 0xC0;
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
                              serial_current_message.data[3]='D';
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
                               current_message.data[5]='D';
                               can_send_message(current_message);
                               delay_ms(10);
                               current_message.datalen=4;
                             }

                         recovery(4);           //to increase number_of_stat
		         revmove=1;
			 break;
                       }
		}
  }

}

#include "defines.h"

void slow_up_cal(char canshown)
{
unsigned char supc1,supc2;
 while(1)
  {
   I=1;                    //flag for movement   //car is between CAn and 1CF
	   if((Lifttype=='v')||(Lifttype=='d'))
	   {
	     CONTU=1;
             CONTS=1;
	     set_outputs_relay(0);
	     ptimer=0;
	     do
	       {

	        if(ptimer>10)
	            {contnca=1;
		     while(contnca)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
		     break;}
	         if(!floor)break;
                 WDRS=1;    //for Watchdog reset
	        }while(!FEEDBF);                          //Check D feedback
	     if(!floor)break;
	    }	                //lifttype

	   else if(Lifttype=='2')
	   {
	     CONTU=1;
	     set_outputs_relay(0);
	     ptimer=0;
	     do
	       {
	        if(ptimer>10)
	            {contnca=1;
		     while(contnca)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
		     break;}
	         if(!floor)break;
                 WDRS=1;    //for Watchdog reset
	        }while(!FEEDBU);                          //Check D feedback
	    if(!floor)break;


	     CONTS=1;                                         //Go down slowly to show 1CF
	     set_outputs_relay(0);
	     ptimer=0;
	     do
	     {
	      if(ptimer>10)
	                {contnca=1;
			 while(contnca)
                            {
                              WDRS=1;    //for Watchdog reset
                              if(!floor)break;                        //delay 0.5 seconds
                            }
			 break;}
	        if(!floor)break;
                WDRS=1;    //for Watchdog reset
	       }while(!FEEDBS);                          //Check S feedback

	      if(!floor)break;
           } //end else lifttype
           else if(Lifttype=='h')
	   {

	     CONTS=1;
	     set_outputs_relay(0);			      //CONTS for Motor
	     supc1=0;
	     supc2=0;
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

	            if((ptimer>=Hyd_S2D_Time)&&(!supc1))
	              {
	               CONTL=1;
	               set_outputs_relay(0);			      //contl for delta
	               supc1=1;
	               if(supc2)break;
	              }


	            if((ptimer>=Hyd_Start_Time)&&(!supc2))
	              {
	               CONTU=1;
	               set_outputs_relay(0);
	               supc2=1;
	               if(supc1)break;
	              }

                    if(ptimer>200)break;

                  }while(1);


             if(!floor)break;

	     ptimer=0;
	     do
	       {
	        if(ptimer>10)
	            {contnca=1;
		     while(contnca)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
		     break;}
	         if(!floor)break;
                 WDRS=1;    //for Watchdog reset
	        }while(!FEEDBU);                          //Check D feedback
	    if(!floor)break;

	   }



	   ptimer=0;
     	   do
	     {
              if(!floor)break;
              if(!Timebreakdelay)break;
	      if(ptimer>Timebreakdelay)
	            {
	              feedb4bse=1;
		      while(feedb4bse)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
	   	      break;
	   	     }
               WDRS=1;    //for Watchdog reset
	    }while(!FEEDB4BS);                 //Check Break  feedback

           if(!floor)break;

	   ptimer = 0;
//	   while(!(CAn && CF1))
          while(1)
	    {
	       if(canshown==0)
	       {
	        if(Fsflag!=2)
	            if(CAn && CF1)break;

	        if(Fsflag==2)
	            if(CAn && CF1 && CF3)break;
	        }
	        else
	       if(canshown==1)   //no need to check CAn
	       {
	        if(Fsflag!=2)
	            if(CF1)break;

	        if(Fsflag==2)
	            if(CF1 && CF3)break;
	        }


                WDRS=1;    //for Watchdog reset
	        if(!floor)break;
	    /*    if((ptimer>(Cf1mt+10)) && (!Calfs))
	                        {time1cf=1;
				 while(time1cf)
                                    {
                                      WDRS=1;    //for Watchdog reset
                                      if(!floor)break;                        //delay 0.5 seconds
                                    }
				 break;}   */
				                  //in fast calibration mode
				                  //in slow calibration mode
	        if(CA1&&(!CAn))
	            {motordi=1;
		      while(motordi)
                                    {
                                      WDRS=1;    //for Watchdog reset
                                      if(!floor)break;                        //delay 0.5 seconds
                                    }
		      break;}                    //Motor direction Error cause
	   	                                 //CA1 is expected but CAn has been sensed
              if(FTOC||FLTC)break;
              }   //end of while

      	if(!floor)break;
    	if(FTOC||FLTC)break;

	ptimer=0;
	while(ptimer<Contofftu)
                    {
                         WDRS=1;    //for Watchdog reset
                         delay_ms(100);
                         if(!floor)break;                        //delay 0.5 seconds
                    }

        WDRS=1;    //for Watchdog reset
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
	     {CONTU=0;
	      CONTD=0;
              set_outputs_relay(0);}

   break;
  }
}

void slow_down_cal(char ca1shown)
{
    while(1)
        {
           I=1;                    //flag for movement  car is between CA1 and 1CF

	   if((Lifttype=='v')||(Lifttype=='d'))
	   {
	     CONTD=1;
             CONTS=1;
	     set_outputs_relay(0);
	     ptimer=0;
	     do
	       {

	        if(ptimer>10)
	            {contnca=1;
		     while(contnca)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
		     break;}
	         if(!floor)break;
                 WDRS=1;    //for Watchdog reset
	        }while(!FEEDBF);                          //Check D feedback
	     if(!floor)break;
	    }	                //lifttype
	   else if(Lifttype=='h')
	   {
	        CONTD=1;
	        CONTU=0;
	        CONTF=0;
	        CONTL=0;
	        CONTM=0;
	        CONTS=0;
   	       set_outputs_relay(0);
  	       ptimer=0;
	       do
	        {
	         if(ptimer>10)
	            {contnca=1;
		     while(contnca)
                               {
                                 WDRS=1;    //for Watchdog reset
                                 if(!floor)break;                        //delay 0.5 seconds
                               }
		     break;}
	         if(!floor)break;
                 WDRS=1;    //for Watchdog reset
	         }while(!FEEDBD);                          //Check D feedback

	   }
	   else
	   {

	    CONTD=1;
	     set_outputs_relay(0);
	     ptimer=0;
	     do
	      {
	       if(ptimer>10)
	            {contnca=1;
		     while(contnca)
                               {
                                 WDRS=1;    //for Watchdog reset
                                 if(!floor)break;                        //delay 0.5 seconds
                               }
		     break;}
	       if(!floor)break;
               WDRS=1;    //for Watchdog reset
	       }while(!FEEDBD);                          //Check D feedback
	     if(!floor)break;

	     CONTS=1;                                         //Go down slowly to show 1CF
	     set_outputs_relay(0);
	     ptimer=0;
             do
	      {
	       WDRS=1;    //for Watchdog reset
	       if(ptimer>10)
	                {contnca=1;
			 while(contnca)
                               {
                                 WDRS=1;    //for Watchdog reset
                                 if(!floor)break;                        //delay 0.5 seconds
                               }
			 break;}
	       if(!floor)break;

	       }while(!FEEDBS);                          //Check S feedback
	     if(!floor)break;
	  } //else lifttype

	   ptimer=0;
     	   do
	     {
              if(!floor)break;
              if(!Timebreakdelay)break;
	      if(ptimer>Timebreakdelay)
	            {
	              feedb4bse=1;
		      while(feedb4bse)
                               {
                                 WDRS=1;    //for Watchdog reset
                                 if(!floor)break;                        //delay 0.5 seconds
                               }
	   	      break;
	   	     }
               WDRS=1;    //for Watchdog reset
	    }while(!FEEDB4BS);                 //Check Break  feedback

           if(!floor)break;

	   ptimer = 0;
//	   while(!(CA1 && CF1))
           while(1)
	    {

	       if(ca1shown==0)
	       {
	        if(Fsflag!=2)
	          if(CA1 && CF1)break;

	        if(Fsflag==2)
	          {
	             if(CA1Second_En)
	               {
	                 if(CA1 && CF1 && CF3 && CA1S)break;
	               }
	             else
	               {
	                 if(CA1 && CF1 && CF3)break;
	               }
	          }
	        }
	       else
	       if(ca1shown==1)
	       {
	        if(Fsflag!=2)
	          if(CF1)break;

	        if(Fsflag==2)
	          if(CF1 && CF3)break;
	        }


	        WDRS=1;             //FOR WATCHDOG RESET
	        if(!floor)break;
	     /*   if((ptimer>(Cf1mt+10)) && (!Calfs))
	                        {time1cf=1;
				 while(time1cf)
                                       {
                                         WDRS=1;    //for Watchdog reset
                                         if(!floor)break;                        //delay 0.5 seconds
                                       }
				 break;} */
				                  //in fast calibration mode
				                  //in slow calibration mode
	        if(CAn&&(!CA1))
	          {motordi=1;
		   while(motordi)
                              {
                                WDRS=1;    //for Watchdog reset
                                if(!floor)break;                        //delay 0.5 seconds
                              }
		   break;}                    //Motor direction Error cause
	   	                                 //CA1 is expected but CAn has been sensed
              if(FTOC||FLTC)break;
             }   //end of while

      	if(!floor)break;
    	if(FTOC||FLTC)break;

	ptimer=0;
	while(ptimer<Contofftd)
                    {
                         WDRS=1;    //for Watchdog reset
                         delay_ms(100);
                         if(!floor)break;                        //delay 0.5 seconds
                    }

	I=0;
	CONTS=0;
	CONTF=0;
	CONTL=0;
	CONTM=0;
	if(Lifttype!='v')
	     {CONTU=0;
	      CONTD=0;}
	set_outputs_relay(0);

	if(Lifttype=='v')ptimer=0;
	else
	if(Lifttype=='d')ptimer=20;
	  else  ptimer=40;
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
	     {CONTU=0;
	      CONTD=0;
              set_outputs_relay(0);}

       break;

    }
}

void  calibration(void)
{
     EIMSK=0x00;                     //***\\
     _int5_en=0;
     _int4_en=0;


     while(1)             //calibration loop
       {
        WDRS=1;    //for Watchdog reset
        if(!floor)break;
        if(FTOC||FLTC)
         {
          I=0;
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
                           if(!floor)break;                        //delay 0.5 seconds
                         }
         if(!floor)break;

         if(FEEDBU || FEEDBD || FEEDBF || FEEDBS)
                  {contce=1;
		   while(contce)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
                   break;}

         while(FTOC||FLTC)
              {
               WDRS=1;    //for Watchdog reset
               if(!floor)break;
              }
           } //end if(FTO|OVL)

        if(!floor)break;

         ptimer=0;
         while(ptimer<20)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 2 seconds
                         }

         while(OVLC)
              {
               WDRS=1;    //for Watchdog reset
               if(!floor)break;
              }

        if(!floor)break;                //about 2 seconds



       // if(CA1 && CF1) break;                          //retun OK! to Main Program
       // if(CAn && CF1){floor=Floorsno;break;}

       if(Fsflag!=2)
         {
           if(CA1 && CF1)break;
           if(CF1 && CAn)
                        {floor=Floorsno;
                         break;}

         }
       else
         {
             if(CA1Second_En==1)
	        {
                   if(CA1 && CF1 && CF3 && CA1S)break;
                   if(CA1 && CF1 && CF3 && !CA1S){floor=2;break;}
                   if(CAn && CF1 && CF3)
                                {floor=Floorsno;
                                 break;}
	        }
	       else
	        {
                    if(CA1 && CF1 && CF3)break;
                    if(CAn && CF1 && CF3)
                                {floor=Floorsno;
                                 break;}
	        }

         }

	                                               //The car had been in lowest floor
        while(!doorclose())
        {
              if(!floor)break;
              WDRS=1;
              RELAYO=0;
              RELAYC=0;
              URA=0;
              set_outputs_relay(1);

              blink_en=0;

              ptimer=0;
              do
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 5 seconds
                         }while(ptimer<50);
              if(!floor)break;

              errormsg(101);    //CAL

              if(!floor)break;
         }

        if(!floor)break;

        if(FEEDBU || FEEDBD || FEEDBF || FEEDBS)
                  {contce=1;
		    while(contce)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
		    break;}

	if(!floor)break;		                                       //To test contactors for being free
			                                       // after triggering

        ptimer=0;

        while(ptimer<10)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
        if(!floor)break;

        //***\\

        if((!CA1)&&(!CAn))
        {
         if(floor_cal&&dir_prev)
          {

           if(floor_cal>Floorsno)break;

           if(dir_prev==1)
              {
                  if((Fsflag==2)&&(CF1)&&(CF3))floor=floor_cal;
                  else
                  if((Fsflag!=2)&&(CF1))floor=floor_cal;
                  else
                  slow_down_cal(1);   //slowly down till 1cf
                  if(!floor)break;
                  if(FTOC||FLTC)continue;
                  if(Fsflag==2 && CA1Second_En)
                   {
                     if(CA1 && CA1S)floor=1;
                     else if(CA1 && !CA1S)floor=2;
                     else floor=floor_cal;
                   }
                  else
                   {
                      if(CA1)floor=1;
                        else floor=floor_cal;
                   }
                  break;

              }
           if(dir_prev==2)
              {
                  if((Fsflag==2)&&(CF1)&&(CF3))floor=floor_cal;
                  else
                  if((Fsflag!=2)&&(CF1))floor=floor_cal;
                  else
                  slow_up_cal(1);  //slowly up till 1cf
                  if(FTOC||FLTC)continue;
                  if(!floor)break;
                  if(CAn)floor=Floorsno;
                   else floor=floor_cal;
                  break;
              }

           }
          }


       // if( CAn && (!CF1) )      //Slow Calibratoin is selected or
        if( ( (!CF1) && CAn && (Fsflag!=2) ) || ( ((!CF1)||(!CF3)) && CAn && (Fsflag==2) ) )
          {
            slow_up_cal(0);
            if(FTOC||FLTC)continue;
	    if(!floor)break;
     	    floor=Floorsno;                       //CABIN IS NOW IN THE HIGHER FLOOR

            break;
          };    //end of slow-up calibration mode


        if( ( CA1 && (!CF1) && Fsflag!=2 ) || (CA1 && !(CF1&&CF3) && Fsflag==2 )|| Calfs)//Slow Calibratoin is selected or
          {

  	    slow_down_cal(0);
            if(FTOC||FLTC)continue;
            break;
           };    //end of slow-down calibration mode

if((!CA1) && (!Calfs))                    //Fast calibration  is  selected and
   {			       	            //  Car has not been sensed CA1

	I=1;                                     //flag for movement

   if((Lifttype=='v')||(Lifttype=='d'))
	{
	     CONTD=1;
             CONTF=1;
	     set_outputs_relay(0);
	     ptimer=0;
	     do
	       {

	        if(ptimer>10)
	            {contnca=1;
		     while(contnca)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
		     break;}
	         if(!floor)break;
                 WDRS=1;    //for Watchdog reset
	        }while(!FEEDBF);                          //Check F feedback
	     if(!floor)break;
	}	                //lifttype
	else if(Lifttype=='h')
	{
	     CONTU=0;                  //hydraulic
	     CONTD=1;
             CONTF=1;
             CONTL=0;
             CONTM=0;
             CONTS=0;
	     set_outputs_relay(0);
	     ptimer=0;
	     do
	       {
	        if(ptimer>10)
	            {contnca=1;
		     while(contnca)
                         {
                           WDRS=1;    //for Watchdog reset
                           if(!floor)break;                        //delay 0.5 seconds
                         }
		     break;}
	         if(!floor)break;
                 WDRS=1;    //for Watchdog reset
	        }while((!FEEDBF)||(!FEEDBD));                          //Check F feedback
	     if(!floor)break;
	}
      else
	{		     //2 SPEED

	   CONTD=1;
	   set_outputs_relay(0);                //Triggering D cont.
	   ptimer=0;
	   do
	    {
	      WDRS=1;    //for Watchdog reset
	      if(!floor)break;
	      if(ptimer>10)
	            {contnca=1;             //wait 1 second for closing D cont.
			 while(contnca)
                               {
                                    WDRS=1;    //for Watchdog reset
                                    if(!floor)break;                        //delay 0.5 seconds
                               }
			 break;}

	     }while(!FEEDBD);

	     if(!floor)break;

	   CONTF=1;
	   set_outputs_relay(0);                //Triggering F cont.
	   ptimer=0;
	   do
	    {

  	      if(!floor)break;
	      if(ptimer>10)
	            {contnca=1;             //wait 1 second for closing F cont.
			 while(contnca)
                               {
                                    WDRS=1;    //for Watchdog reset
                                    if(!floor)break;                        //delay 0.5 seconds
                               }
	    		 break;}
               WDRS=1;    //for Watchdog reset
	     }while(!FEEDBF);

	    if(!floor)break;
	} //END ELSE LIFTTYPE

	    ptimer=0;
     	    do
	      {
        	     if(!floor)break;
        	     if(!Timebreakdelay)break;
	             if(ptimer>Timebreakdelay)
	               {
	                 feedb4bse=1;
		         while(feedb4bse)
                               {
                                    WDRS=1;    //for Watchdog reset
                                    if(!floor)break;                        //delay 0.5 seconds
                               }
	   	      break;
	   	     }
                  WDRS=1;    //for Watchdog reset
	       }while(!FEEDB4BS);                 //Check Break  feedback            //wait 1 second for opening Break

	if(!floor)break;

	ptimer=0;                           //Now Car goes down Fast
	while(!CA1)
	 {
            WDRS=1;             //FOR WATCHDOG RESET
	    if(FTOC||FLTC)break;                                //Go fast down till sense CA1

	    if(!floor)break;

	    if(CAn&&(!CA1))
		      {motordi=1;
		       while(motordi)
                               {
                                    WDRS=1;    //for Watchdog reset
                                    if(!floor)break;                        //delay 0.5 seconds
                               }
		       break;}             //CA1 is expected but CAn has been sensed
	    if(ptimer>Calmt)
		      {calmto=1;  		//calmt = Calibration Maximum Time
		       while(calmto)
                               {
                                    WDRS=1;    //for Watchdog reset
                                    if(!floor)break;                        //delay 0.5 seconds
                               }
		       break;} 		//calmto = Calibration maximum
	  }                 		      //time is over(ERROR)

        if(!floor)break;
        if(FTOC||FLTC)continue;

	if(Lifttype!='h')
	{
	  CONTS=1;
	 /* set_outputs_relay(0);                //Turn off Fast contactor

	  ptimer=0;
	  while((ptimer<Cont_overlap)&&(ptimer<10))
                    {
                         WDRS=1;    //for Watchdog reset
                         delay_ms(100);
                         if(!floor)break;                        //delay 0.5 seconds
                    }	  */
	 }

	CONTF=0;                            //Turn  on Slow contactor
	set_outputs_relay(0);
	ptimer=0;

	if(Lifttype=='2')
	{
	  do
	    {
              WDRS=1;    //for Watchdog reset
              if(!floor)break;
	      if(ptimer>10)
	            {if(!FEEDBS)contnca=1;
	   		 if(FEEDBF)contcea=1;
			 while(contnca||contcea)
			          {
			           if(!floor)break;
			            WDRS=1;    //for Watchdog reset
			           }
	   		 break;}
	     }while((!FEEDBS)||FEEDBF);        //Check F cont.'s feedback for opening
	 }
        if(!floor)break;                                          //and S cont.'s feedback for closing

        if(Lifttype=='h')
        {
         ptimer=0;                                          //Turn on Slow Contactor
         do{                                                //Make the motor speed slow
        	WDRS=1;    //for Watchdog reset
        	if(ptimer>10){
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
        } //else if(Lifttype=='h'
        if(!floor)break;

	ptimer=0;
	do
	  {
           WDRS=1;    //for Watchdog reset
	   if(!floor)break;
	   if((ptimer>(Cf1mt+5)) && (Cf1mt!=0) )
	                  {time1cf=1;
				 while(time1cf)
			           {
			             if(!floor)break;
			              WDRS=1;    //for Watchdog reset
			            }
	   			 break;}      //Wait to sense 1CF after CA1 (cf1mt + 5)
	   if(FTOC||FLTC)break;

	 //  if( CF1 && CA1 && (Fsflag!=2) )break;
	 //  if( CF1 && CF3 && CA1 && (Fsflag==2))break;

	   if(Fsflag==2)
	     {
        	if(CA1Second_En)
        	 {
        	   if( CF1 && CF3 && CA1 && CA1S)break;
        	 }
        	else
        	 {
        	   if( CF1 && CF3 && CA1)break;
        	 }
             }
           else
             {
                if( CF1 && CA1)break;
             }




	 }while(1);              //Go slowly down till sensing CA1 and
	 //while(!(CA1 && CF1));
	                                      //1CF simultaneously
         if(!floor)break;
         if(FTOC||FLTC)continue;

	ptimer=0;
	while(ptimer<Contofftd)
                    {
                         WDRS=1;    //for Watchdog reset
                         delay_ms(100);
                         if(!floor)break;
                    }

	I=0;
	CONTS=0;
	CONTF=0;
	CONTL=0;
	CONTM=0;
	if(Lifttype!='v')
	     {CONTU=0;
	      CONTD=0;}
	set_outputs_relay(0);

	if(Lifttype=='v')ptimer=0;
	else
	if(Lifttype=='d')ptimer=20;
	  else  ptimer=40;
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
	     {CONTU=0;
	      CONTD=0;
              set_outputs_relay(0);}

	 if(!floor)break;

    } //end if
if(!floor)break;
} //end while(1) for calibration
}
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
