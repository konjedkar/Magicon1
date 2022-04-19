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
