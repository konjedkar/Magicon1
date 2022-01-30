//Specifies Wich MOB(s) is/are full

#define CANSTM (*(unsigned int *) 0xf8)
#define no_mob_index_auto_inc CANPAGE|=0x08
#define mob_index_auto_inc CANPAGE&=~(0x08)
#define NO_MOB 0xff
#define RxStaMob 0
#define RxEndMob 7           //11
#define TxStaMob 8           //12
#define TxEndMob 14

#define Tx_Time 2000 //ms


struct id_structure_sep{ 
   unsigned char rbtag:2;
   unsigned char rtrtag:1;
   unsigned long int id0_28;
};

//General Structure For Recieving and sending Messages
struct can_message {
	unsigned char status;
	unsigned char datalen;
	unsigned char ide;	
	unsigned char rb;
	unsigned char rtr;
	unsigned long int id;
	unsigned int time;
	unsigned char data[8]; 
};

//General Structure For Recieving and sending Multi Messages
/*struct can_long_message {
	unsigned char status;
	unsigned char datalen;
	unsigned char ide;	
	unsigned long int id;	
	unsigned char data[96]; 
} ;   */  



void ClearAllMailbox (void);
void can_handel_interrupts(void);
unsigned char set_mobs_for_receive(unsigned char mob_numbers);
void read_frame_buffer(void);
void set_id_tag(unsigned long int id,unsigned char rtrtag,unsigned char rbtag);
void get_id_tag(struct id_structure_sep *id);
unsigned char can_send_message(struct can_message source);
//void send_long_message(unsigned int destination);
void message_detector(void);    
unsigned char get_mob_free(void);    
void can_send_quick(unsigned char,unsigned char,unsigned char,
                    unsigned char,unsigned char,unsigned char,
                    unsigned char,unsigned char,unsigned char,
                    unsigned char,unsigned char);


//**********PROTOCOL DEFINITIONS AND FUNCTIONS**********
  

//****************CAN FRAME IDENTIFIER SPECIFICATION*****************
//    3bit            11bit                 11bit            4bit
// __________________________________________________________________
// |  |  |  | | | | | | | | | | | | | | | | | | | | | | |  |  |  |  |
// |Priority| Source Address      | Destination Address |MessageNum.|
// |________|_____________________|_____________________|___________|


//General Structure for Protocol Defined ID
struct protocol_id_structure_sep{   
   unsigned char message_num;
   unsigned int dest_address;
   unsigned int source_address;
   unsigned char priority;    
};
     

//****************************************************************
// Parameters: TARGET is the address of can_structure for saving readed message
// Returns: MOB Number if successfull, FF if no MOB has data
// Function: Reads Recieved Message from highest priority Mob (0 has highest priority)
// and ready it for recieve again.
//***************************************************************
unsigned char read_received_message(struct can_message *target);
unsigned long int assemble_id(struct protocol_id_structure_sep sep_id);
void split_id(unsigned long int id,struct protocol_id_structure_sep *sep_id);
