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



