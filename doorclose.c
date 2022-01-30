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
                   
