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


