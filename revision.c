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

