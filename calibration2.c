#include "defines.h"   

unsigned char calibration(void)
{   
floor=1;
Write_7Seg(3,'L');
Write_7Seg(2,'A'); 
Write_7Seg(1,'C');
Write_7Seg(4, 0 ); 
blink_en=0;

read_inputs();
if(CA1 && CF1)         
      {return 1;}                         //retun OK! to Main Program
	     				            //The car had been in lowest floor
//read_inputs_relay();                        	         
if(FEEDBU || FEEDBD)
                  {contce=1; 
			 while(contce);  
			 return 0;}

if(FEEDBF || FEEDBS)
                  {contce=1;
			 while(contce);   
      		 return 0;}        
					             //To test contactors for being free
			                         // after triggering
if((CA1 && (!CF1))||Calfs)
   {					             //Slow Calibratoin is selected or
	CONTD=1;			             //car is between CA1 and 1CF
	set_outputs_relay();
	ptimer=0;
	do
	   {
	    if(ptimer>10)
	            {contnca=1;             
			 while(contnca);  
			 return 0;} 
	   //read_inputs_relay();	
	   }while(!FEEDBD);                  //Check D feedback
		            
	CONTS=1;                            //Go down slowly to show 1CF
	set_outputs_relay();
	ptimer=0;
	do
	  {
	   if(ptimer>10)
	            {contnca=1; 
			 while(contnca); 
			 return 0;}
	   //read_inputs_relay();
	   }while(!FEEDBS);                  //Check S feedback          	
	  
	ptimer=0;
     /*	
	do
	  {
	   if(ptimer>10)
	            {feedb4bse=1;	
			 while(feedb4bse);
	   		 return 0;}	
	   read_inputs();
	  }while(FEEDB4BS);                 //Check Break  feedback
	*/
	read_inputs();
	ptimer = 0;  
	while(!(CA1 && CF1))
	 {
	   if(opmode!='n')return 0;

	   //read_inputs_relay(); 
	   /*
	   if(!FEEDB4BS)
	            {feedb4bse=1;
			 while(feedb4bse);
	   	       return 0;}
         */
	   if((ptimer>(Cf1mt+5)) && (!Calfs))
	                        {time1cf=1;
					 while(time1cf); 
	   				 return 0;}
				                  //in fast calibration mode
	   				
	   if((ptimer>(Calmt*4+5)) && Calfs)
	                        {time1cf=1;
					 while(time1cf);
	   				 return 0;}
				                  //in slow calibration mode	   				
	   //read_inputs();		  
	   if(CAn)
	      {motordi=1;
		 while(motordi);
	   	 return 0;}                   //Motor direction Error cause
	   	                              //CA1 is expected but CAn has been sensed
				
        }   //end of while 
    	
	delay_ms(Contofftd*100);            //Delay as "Contactor off time delay"	
	
	CONTD=0;
	CONTS=0;
	set_outputs_relay();                //Stop!
	
	ptimer=0;
	do
	  {
	    if(ptimer>10)
	             {contcea=1;    
			  while(contcea); 
	    		  return 0;}            //Wait one second for sense feedbacks
	   // read_inputs();	    
  	  }while(FEEDBD || FEEDBS);

	if(opmode!='n')return 0;	
	floor=1;			
 	return 1;                       
   };    //end of slow calibration mode 
                                   			

if((!CA1) && (!Calfs))                    //Fast calibration  is  selected and 
   {			       	            //  Car has not been sensed CA1
	CONTD=1;                                  
	set_outputs_relay();                //Triggering D cont.
	ptimer=0;
	do
	  {
	   if(ptimer>10)
	            {contnca=1;             //wait 1 second for closing D cont.
			 while(contnca);
	   		 return 0;}
	   //read_inputs_relay();
	  }while(!FEEDBD);

	CONTF=1;
	set_outputs_relay();                //Triggering F cont.
	ptimer=0;
	do
	  {
	   if(ptimer>10)
	            {contnca=1;             //wait 1 second for closing F cont.
			 while(contnca);
	   		 return 0;}
	   //read_inputs_relay();
	  }while(!FEEDBF);

	ptimer=0;
	/*do
	  {
	   if(ptimer>10)
	            {feedb4bse=1;
			 while(feedb4bse);
	   		 return 0;} 
	   //read_inputs_relay();
	  }while(FEEDB4BS);	            //wait 1 second for opening Break
      */
	ptimer=0;                           //Now Car goes down Fast
	read_inputs();                
	while(!CA1)
	 {                                  //Go fast down till sense CA1
	   // read_inputs();
	    //read_inputs_relay();
	    if(opmode!='n')return 0;
	    /*
	    if(!FEEDB4BS)
		      {feedb4bse=1;
		       while(feedb4bse);
	    	       return 0;}
	    */	       
	    if(CAn)
		      {motordi=1;
		       while(motordi);
	    	       return 0;}             //CA1 is expected but CAn has been sensed
	    if(ptimer>Calmt)
		      {calmto=1;  		//calmt = Calibration Maximum Time
		       while(calmto);
	    	       return 0;} 		//calmto = Calibration maximum 
	  }                 		      //time is over(ERROR)

	CONTS=1;
	set_outputs_relay();                //Turn off Fast contactor
	delay_ms(Cont_overlap);             //Delay for overlap 2 conts.
	CONTF=0;                            //Turn  on Slow contactor
	set_outputs_relay();
	ptimer=0;
	/*
	do
	  {
	   //read_inputs_relays();
	   if(ptimer>10)
	            {if(!FEEDBS)contnca=1;
	   		 if(FEEDBF)contcea=1;
			 while(contnca||contcea);
	   		 return 0;}
	  }while((!FEEDBS)||FEEDBF);        //Check F cont.'s feedback for opening
      */                                    //and S cont.'s feedback for closing
	ptimer=0;
	do
	  {
	   if(opmode!='n')return 0;		
	   if(ptimer>(Cf1mt+5))
	                  {time1cf=1;
				 while(time1cf);
				 return 0;}      //Wait to sense 1CF after CA1 (cf1mt + 5)
	   //read_inputs();
	  }while(!(CA1 && CF1));              //Go slowly down till sensing CA1 and 
	                                      //1CF simultaneously

	delay_ms(Contofftd*100);           //Delay as "Contactors off time delay"
	
	CONTD = 0;
	CONTS = 0;
	set_outputs_relay();               //Stop!
	ptimer = 0;
	do{
	   if(ptimer>10)
	            {contcea=1;
			 while(contcea);
	   		 return 0;}
	   //read_inputs_relay();
	  }while(FEEDBD || FEEDBS);

	if(opmode!='n')return 0;

	floor=1;                           //Find first floor
	return 1;                          //Return OK! 
	}
return 0;		
}  
