void Send_Move_Door_Status(void)
{       
  unsigned char i;
  if (I==0)
  {
     side=0;
     vel=0;
  }        
  else
  {
     if (CONTF==1) vel=1;           //FAST
     if (CONTS==1) vel=2;           //SLOW
     if (CONTU==1) side=1;          //DOWN 
     if (CONTD==1) side=2;          //UP
  }           
  if (RELAYO==1)                    //OPENNING            
     Door_Status=0x00;	            
  if (RELAYC==1)                 
     Door_Status=0x01;              //CLOSING
//if (RELAYO==1)                 
//   Door_Status=0x02;	            //OPENED
//if (RELAYC==1)                    
//   Door_Status=0x03;              //CLOSED            

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
                  
   delay_ms(1);      

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
 
 void Send_Requests_Status(char ReqType, char Req)
 {
   unsigned char Req=0,ReqDir=0,Req1=0,Req2=0,Req3=0 
   unsigned char i;
   
   if (ReqType==2)
   {

	DataBuff[0]=0xFF;
   	DataBuff[1]=0xFF;
   	DataBuff[2]=0x01;     //Status Packet
   	DataBuff[3]=0xF4;     //Outsid cabin Request
   	DataBuff[4]=Req;      //floor to stop
   	DataBuff[5]=1;        //ReqDir;   //wants to go down=0 or up=1
   	DataBuff[6]=0;        
   	DataBuff[7]=IP;       //IP address 
   	for(i=0;i<=7;i++)
      		buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
   	udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		
   }
   if (ReqType==4)
   {

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
   if (ReqType==1)
   {    
   	DataBuff[0]=0xFF;
   	DataBuff[1]=0xFF;
   	DataBuff[2]=0x01;       //Status Packet
   	DataBuff[3]=0xF3;       //Cabin Request
   	DataBuff[4]=Req;        //floor to stop 1
   	DataBuff[5]=0           //floor to stop 2
   	DataBuff[6]=0;          //floor to stop 3
  	DataBuff[7]=IP;         //IP address 
   	for(i=0;i<=7;i++)
      		buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN+i]=DataBuff[i]; 
   	udpSend(GATEWAY, CONTROL_PORT, 8, &buffer[ETH_HEADER_LEN+IP_HEADER_LEN+UDP_HEADER_LEN]);		
   }
 }	
 
 void Reset_Out_Request(void)
 {
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
  