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

