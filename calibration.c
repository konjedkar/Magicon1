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