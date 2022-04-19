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
