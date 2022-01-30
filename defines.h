              

#define LOOPBACK_PORT	        7	                        // UDP packets sent to this port will be returned to sender
#define CONTROL_PORT	        4950	                        // UDP packets sent to this port will be used for control
#define SERIAL_PORT	        4951	                        // UDP pac kets sent to this port will be printed via serial


#define NETSTACK_BUFFERSIZE	  (576+ETH_HEADER_LEN)

/////////////////////////////// timer defines////////////////////////////

#define TIMER_PRESCALE		  1024
#define TIMER_INTERVAL		  (F_CPU/TIMER_PRESCALE/100)	// 100ms interval

///////////////////////////////////////////////////////////////////////

#define RXB8 1
#define TXB8 0
#define UPE   2
#define OVR  3
#define FE    4
#define UDRE 5
#define RXC  7

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
#define RX_BUFFER_SIZE1 15   // USART1 Receiver buffer
#define TX_BUFFER_SIZE1 30   // USART1 Transmitter buffer

                                                

#define CAN_BAUD_RATE  250                                     //baud rate in  kbps   50kbps or 250 kbps    50 | 250      

#define Eth_Timeout_For_Rec     10                          // in 0.1 sec for receive a response 

#define Ethernet_Disable       1                              //remark for enable  Ethernet
                                                                      
//#define Numtype_Disable        1                           //remark for enable DIPswitch-selection of Num-floor & NUm-e

//#define Grouptype_Disable      1                             //remark for DIPswitch-selection of  Grouptype

//#define BLOCK_DATE_DISABLE      1                            //remark for enable using block date

#define RCM_Enable        1                                    //remark for disable RCM usage

//#define Extra_Inputs     1                                    //remark for disable extra inputs in Main

#define REVC_TIMEOUT_BUTTON   500                        // in   0.01 sec for  button revu and revd in revision cabin

#define PULSE_LEVEL_MARGIN    200



//  Panel Proprties

#define  SERIAL_NUMBER                  "*MV92*15"   
#define  HARDWARE_VERSION               "*Plus   "
#define  SOFTWARE_VERSION               "*Gelaten"
#define  PANEL_MODEL                    "*MAGICON"
#define  PORODUCT_DATE                  "*1393   "
#define  GUARANTEE_EXPIRED_DATE         "*1394   "



//It contents of the definition of the main body and navigation program        
//#define  STANDALONE_TEST                //define STANDALONE_TEST for TEST a standalone MainBoard 
                                        //for NORMAL USE remark the line above

#ifndef  STANDALONE_TEST		
			     
#define SS63       (HV_IN6==0)	        //	Serie-Stop 63	: Up HoistWay  		 : NC
#define SS64       (HV_IN5==0)		//	Serie-Stop 64 	: Down Hoistway 	 : NC
#define SS65       (HV_IN4==0)		//	Serie-Stop 65 	: Car SS    		 : NC
#define SS66       (HV_IN3==0)		//	Serie-Stop 66 	: Hall-Doors	  	 : NC
#define SS69       (HV_IN2==0)		//	Serie-Stop 69 	: Car-Door  		 : NC
#define SS68       (HV_IN1==0)		//	Serie-Stop 68 	: Door-Locks 		 : NC
#define POWER110   (HV_IN7==0)	        //      GND 110v-AC       : 110v Feedback        : NC
#define POWER24     HV_IN8 		//	GND 24v-DC	: 24v Feedback		 : NC  

#define FLT         M_IN3        	//	SW VIP	: VIP Switch		         : NO
#define CA1S       (M_IN8==0) 
#define REVC       (M_IN9==0)    	//	SW RVEC 	: Cab Rev.     		 : NC
#define REVM       (M_IN10==0)	        //	SW REVM 	: Panel Rev.		 : NC	

#else

#define SS63       1
#define SS64       1
#define SS65       1
#define SS66       1
#define SS69       1
#define SS68       1
#define POWER110   1
#define POWER24    1 
 
#define REVC       0
#define REVM       0 
#define FLT        0    
#define CA1S       (M_IN3==0) 
  
#endif



#define CAn      M_INT1 		//	Sensor 1CF  :	    		      : NC
#define CA1      M_INT2		        //	Sensor CF3  :	    		      : NC					      //
#define CF3      M_INT3		        //	Sensor CA1  : Upper Limit    	      : NC
#define CF1	 M_INT4		        //	Sensor CAn  : Down Limit 	      : NC



#define FIRE        M_IN1            	//	SW FIRE	: Fire Sens.		        : NO
#define FTO         M_IN2        	//	Sensor FTO	: Thermal Sens.         : NO

#define DZU        (M_IN4==0) 		//	Sensor DZU	: DoorZone Up           : NC
#define DZD        (M_IN5==0)	      	//	Sensor DZD	: DoorZone Down	        : NC
#define FEEDB4BS    M_IN6               //      Sensor 4BS  : Break Feedback            : NC      
#define RLEVU	   (M_IN7==0)		//	Sensor RLU 	: Releveling Up  	      : NC
#define RLEVD	   (M_IN8==0)		//	Sensor RLD 	: releveling Down	      : NC


#define REVD        M_IN11	      	//	PB REVD	: Down Rev. PB		: NO
#define REVU        M_IN12	      	//	PB REVU	: Up Rev. PB		: NO




#define FEEDBF    M_IN13            //    Up Cont. FB       : NO
#define FEEDBS    M_IN14            //    Down Cont. FB     : NO
#define FEEDBU    M_IN15            //    Fast Cont. FB     : NO
#define FEEDBD    M_IN16            //    Slow Cont. FB     : NO    



#ifdef  Extra_Inputs
#define VIP       M_IN17
#define OVL_M     M_IN18
#define FUL_M     M_IN19
#define DO_M      M_IN20
#define DC_M      M_IN21
#define KT5       M_IN22
#define KT6       MIN23  
#define AUX       MIN24
#endif

#ifndef Extra_Inputs                                                        
#define VIP       0   //put M_IN17 instead of 0 to use VIP
#endif

//Outputs defines
#define SO1     PORTF.0             //SO1 = CONTACTOR UP
#define SO2     PORTF.1             //SO2 = CONTACTOR DOWN
#define SO3     PORTF.2             //SO3 = CONTACTOR FAST
#define SO4     PORTF.3             //SO4 = CONTACTOR SLOW
#define SO5     PORTF.4
#define SO6     PORTF.5
#define SO7     PORTF.6
#define SO8     PORTF.7
#define SO9     PORTB.7
#define SO10    PORTB.6

#define BUZ    PORTB.5


