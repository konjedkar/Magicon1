#include "defines.h"

/*****************Inputs*****************************/
char M_INT1=0 , M_INT2=0 , M_INT3=0 , M_INT4=0 , HV_IN7=0 , HV_IN8=0 ;
char HV_IN1=0 , HV_IN2=0 , HV_IN3=0 , HV_IN4=0 , HV_IN5=0 , HV_IN6=0 ;
char M_IN1 =0 , M_IN2 =0 , M_IN3 =0 , M_IN4 =0 , M_IN5 =0 , M_IN6 =0 ;
char M_IN7 =0 , M_IN8 =0 , M_IN9 =0 , M_IN10=0 , M_IN11=0 , M_IN12=0 ;
char M_IN13=0 , M_IN14=0 , M_IN15=0 , M_IN16=0 , M_IN17=0 , M_IN18=0 ;   
char M_IN19=0 , M_IN20=0 , M_IN21=0 , M_IN22=0 , M_IN23=0 , M_IN24=0 ;   

/*****************Outputs****************************/     

eeprom unsigned char EEWriteIndex,EEWriteIndexOV;
eeprom unsigned int EENumberoferrors;        

eeprom unsigned char nop[10];        

eeprom unsigned char EE_Password[5];
eeprom unsigned char EE_Navistrategy , EE_VIPfloor , EE_Parkfloor   , EE_Firefloor    ;
eeprom unsigned char EE_Floorsno , EE_Calfs   , EE_Needtocalibration;
eeprom unsigned char EE_Contofftd , EE_Contofftu   , EE_Cont_overlap  , EE_Timebreakdelay;  
eeprom unsigned char EE_Fsflag , EE_Oneflag  ; 
eeprom unsigned int  EE_Calmt , EE_Timepark  , EE_Timestndby   ;
eeprom unsigned int  EE_Time1cf3 , EE_Time2cf3  , EE_Cf1mt     , EE_Timetravel  ;
eeprom unsigned int  EE_Doorwtc , EE_Doordur , EE_Doordur_open;
eeprom unsigned char EE_Doorsiso , EE_Doornrc , EE_Doorparkunderl , EE_Dooruldelay , EE_Door68t;
eeprom unsigned char EE_Doorunderl   , EE_Doortype , EE_Doorclosepark , EE_Doorcdebouncet;  
eeprom unsigned char EE_Groundexist , EE_Undergroundno , EE_Parksign;  
eeprom unsigned char EE_Maskfloor[8] , EE_Maskfloor_Chksum , EE_Volume , EE_Eth_number , EE_Enc_enable;  
eeprom unsigned char EE_Grouptype,EE_Monitoring,EE_Expired_date,EE_Expired_month,EE_Gateway_lsb;           
eeprom unsigned char EE_SMS_Level,EE_Hyd_S2D_Time,EE_Hyd_Start_Time,EE_Hyd_Stop_Time,EE_Hyd_Releveling,EE_IVA_comm; 
eeprom unsigned char EE_Force_to_Rescue_En,EE_floor_announce_delay,EE_Preopening_en,EE_Car_Call_Erase_En;
eeprom unsigned char EE_FirstDoor[8],EE_FirstDoor_Chksum,EE_SecondDoor[8],EE_SecondDoor_Chksum,EE_Tunnel_Door_en,EE_Block_date_en;
eeprom unsigned int  EE_Start_dayofyear;    
eeprom unsigned char EE_HalfFloor[8],EE_HalfFloor_Chksum,EE_CA1Second_En;
eeprom unsigned char EE_HalffloorDelay,EE_HalffloorSpeed,EE_OneMovefloorDelay,EE_OneMovefloorSpeed,EE_Door2type,EE_Music_Play_En;     
eeprom unsigned char EE_manual_floor_naming,EE_Floor_names_1[65],EE_Floor_names_2[65];


eeprom struct ErrorOb
      {
       unsigned char ErrorSec;
       unsigned char ErrorMin;                                               
       unsigned char ErrorHour ;
       unsigned char ErrorDate ;
       unsigned char ErrorMonth;
       unsigned char ErrorYear ;
       unsigned char ErrorQty  ;
       unsigned char ErrorNum  ;
      }ErrorHistory[200]; 
        
struct Waiting_List_ob
  {
     unsigned char Req;
     unsigned char ReqType;
     unsigned char ReqEval;
     unsigned int   ReqExpireTime;
     unsigned char ReqUniqueID;     
     unsigned char ReqIP;    
     unsigned char ReqResponse;
  }WL[4];  

//Program Variables Define
unsigned char AL[64] = {0} , _al[64] = {0};
unsigned int ptimer,stimer,traveltimer;
unsigned char opmode=0,revmove;
unsigned char mb1,mb2;

//unsigned floor=0;   
unsigned char direction = 0 ;

unsigned int number_of_start;      
unsigned int _reqexistinq=0;   
unsigned char UniqueID = 0 , Eth_disconnect = 0;
unsigned int Eth_Rec_IGP_Timeout=400,Eth_Send_IGP_Timeout=300;
 
//Definition of program Flags      
#ifdef  REVC_TIMEOUT_BUTTON                                  
unsigned char revc_timeout = REVC_TIMEOUT_BUTTON;
#endif     

#ifdef Extra_Inputs
unsigned char OVL_MC=0,FUL_MC=0,DOO_MC=0,DOC_MC=0;
#endif

unsigned char firemf , parkmf , VIPmf , I=0 , NNOD , stayinstair , arrivedstair;
unsigned char standbymf , last_display_msg , old_display_msg,last_SMS_msg;
unsigned char OE_7seg = 0 , blink_en = 0 , beep_en =0 , beep_request = 0 , beep_request_on = 0,  LED_blink=0;
unsigned char _set_output_car=0 , _set_calibration_num=0 , _set_blink_num=0 , turnoffleds = 0 , set_annunciator=0 ;
unsigned char _send_door_status=0 , _send_move_status=0 , _send_requests_status_4 = 0 , _inp_in_use = 0 , _ack_can_transmit_error=0xFF;
unsigned char _send_requests_status_2 = 0 ,  _send_requests_status_1 = 0 ,_reset_out_request=0;
unsigned char firstchk=1 , needreset=0  , WDRS = 1 ;
unsigned char RELAYOed = 0 , RELAYCed =0 , _delay_unload_door = 0 ;
unsigned char _delay_park_unload_door = 0 , _delay_revision_unload_door = 0;
unsigned char LPC_error=0,_hall_button_during_doorclose=0;                                                                      
unsigned char _sec_saved=0,_min_saved=0,_hour_saved=0 , _set_numerator=0,relevelingmf=0;
unsigned char _volume_set_rs485=0,Rescue_via_main=0;
//unsigned char  turnoffledsout = 0, turnoffallledsout=0
bit _pf_reform=0 , _pf_park=0 , turnoffallleds=0,_set_numerator_CA1CAn=0 ;
unsigned char floor_cal=0,dir_cal=0,dir_prev=0,_int5_en=0,_int4_en=0,forcetoreset=0; 
unsigned char _OVL_timeout=0,_OVL_timeout_announuce=0,_DOO_timeout=0;   


//Definition of error Flags
unsigned char contce , contnca , contcea ,calmto;
unsigned char time1cf , motordi , feedb4bse;
unsigned char hdoorct,doorcnc,doorcno,doorlcc,doorlcns,doorlcs;
unsigned char carniva,uraminw,timecf3e;
unsigned char cf3cf1,canins,ca1ins,mcbycan=0,mcbyca1=0;
unsigned char identyle=0;
unsigned char pulse_counter; 

//Definition of program I/O Flags
unsigned char FTOC = 0, OVLC = 0, FULC = 0, FLTC = 0;
unsigned char OVL = 0 , FUL = 0;
unsigned char RELAYC = 0 , RELAYO = 0 , RELAYSTBY = 0 , DOO = 0 , DOC = 0;
unsigned char REVUC = 0 , REVDC = 0;
unsigned char CONTU = 0 , CONTD = 0 , CONTF = 0 , CONTS = 0 , CONTM = 0 , CONTL = 0 , URA=0;

//Definition of System Parameters and set default values of them       
unsigned char Navistrategy = 'd', Parkfloor = 0 , Firefloor = 1 , VIPfloor = 0;
unsigned char Floorsno = 9      , Calfs = 0 , Needtocalibration = 'n' ; 
unsigned char Groundexist=0 , Undergroundno=1 , Parksign=1;
unsigned char Contofftd = 0  , Contofftu = 0 , Cont_overlap = 0 , Timebreakdelay = 0;  
unsigned char Fsflag = 0 , Oneflag = 0;
unsigned int  Calmt = 300 , Timepark = 400 , Timestndby = 400;          
unsigned int  Time1cf3 = 0 , Time2cf3 = 80 , Cf1mt = 160 , Timetravel=340;
unsigned int  Doorwtc = 600  , Doordur =  50 , Doordur_open =0;
unsigned char Doorsiso = 20 , Doornrc =  4 , Dooruldelay = 30 , Doorcdebouncet = 4 , Door68t = 40;
unsigned char Doorunderl = 'y', Doortype = 's' , Doorclosepark='n' , Doorparkunderl='y';  
unsigned char Lifttype = '2' , Grouptype = 's' , Numtype = 's' , Eth_number = 0 , Enc_enable = 'n'; 
unsigned char Monitoring = 'n' , Expired_date = 0xFF , Expired_month = 0xFF,Gateway_lsb = 10,SMS_Level=0;;
unsigned char Maskfloor[8]={255,255,255,255,255,255,255,255}; 
unsigned char Hyd_S2D_Time=15,Hyd_Start_Time=5,Hyd_Stop_Time=5,Hyd_Releveling='y', IVA_comm='s',Preopening_en='n'; 
unsigned char Force_to_Rescue_En = 'n' , floor_announce_delay = 5 , Dir_delay = 'n' , Car_Call_Erase_En = 'n';
unsigned char FirstDoor[8]={255,255,255,255,255,255,255,255},SecondDoor[8]={255,255,255,255,255,255,255,255},Tunnel_Door_en='n'; 
unsigned int  Start_dayofyear=0xFF;        
unsigned char HalfFloor[8]={255,255,255,255,255,255,255,255},CA1Second_En=0;
unsigned char HalffloorDelay=20,HalffloorSpeed='s',OneMovefloorDelay=50,OneMovefloorSpeed=0,Door2type = 's',Music_Play_En='y';
unsigned char revision_open_door_en=0;
unsigned char manual_floor_naming='n',Floor_names_1[65],Floor_names_2[65];



///Definition of ethernet            
unsigned char IP=0;
unsigned long int IPADDRESS,NETMASK,GATEWAY,MAIN2,BROADCAST;

void calibration( void);
void interpreter( void );
void serial_interpreter(void);
void Write_7Seg ( char , char );      
char pathfinder ( char , char );
void errormsg   ( unsigned char );  
void read_inputs( void );
void recovery   ( unsigned char );
void supervisor ( void );
char doorclose  ( void );
char doorclosestandby(void);  
char doorcloserevision(void);     
char dooropenrevision(void);
char dooropen   ( void ); 
char doorfire   ( void );
void Releveling(void);
void beep       ( void );
void set_default_values(void); 
void LED_indicator ( void );   
char showfloor(unsigned char,unsigned char); 
void  set_outputs_relay(unsigned char);	
void  revision_c(void);
void  revision_m(void);
char Evaluation(unsigned char ,unsigned char);   
void timerInit(void);
void uartInit(void);
void ClearScreen(void);
void initialize(void); 
char floorvalidity(unsigned char);                                     
char block_date(void);
void setfloormask(unsigned char,unsigned char,unsigned char);    
char Tunnel_door_find(unsigned char); 
unsigned int calcdayofyear(unsigned char,unsigned char);
void rescueviamain(void);   
char identifylevels(void);

void Send_move_stat( void );    
void Send_door_stat( void );
void Send_requests_stat_4(unsigned char );  
void Send_requests_stat_2(unsigned char);
void Send_requests_stat_1(unsigned char);
void Reset_out_request_stat( unsigned char );                     
void Send_Request_via_Eth(struct Waiting_List_ob, unsigned char );  
void Send_InGroupPresent_Packet(void);
char Waiting_List_Manager(unsigned char,unsigned char,unsigned char,unsigned char, unsigned char,unsigned char);   
unsigned char halffloorstatus(unsigned char );   

