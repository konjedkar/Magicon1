#include "defines.h"
/*****************Read Inputs************************/
void read_inputs(void)
{   
    //First, saving previous status of some flags 
    if(FTO){FTOC=1;}
      else  FTOC=0;
    
    
    PORTA  = 0xFF                    ;
    DDRA   = 0x00                    ;    //Make PORTA as INPUT

    M_INT1 =  (PIND & 0x01)          ;
    M_INT2 = ((PIND & 0x02)>>1)&0x01 ;
    M_INT3 = ((PINE & 0x10)>>4)&0x01 ;
    M_INT4 = ((PINE & 0x20)>>5)&0x01 ;
    
    PORTG  = 0x1E                    ;      //IOCS1=0  IOCS2=1  IOCS3=1  IOCS4=1 
    delay_ms(1)                      ;           
    M_IN1  =  (PINA & 0x01)          ;   
    M_IN2  = ((PINA & 0x02)>>1)&0x01 ;   
    M_IN3  = ((PINA & 0x04)>>2)&0x01 ;   
    M_IN4  = ((PINA & 0x08)>>3)&0x01 ;   
    M_IN5  = ((PINA & 0x10)>>4)&0x01 ;           
    M_IN6  = ((PINA & 0x20)>>5)&0x01 ;   
    M_IN7  = ((PINA & 0x40)>>6)&0x01 ;       
    M_IN8  = ((PINA & 0x80)>>7)&0x01 ;

    PORTG  = 0x1D                    ;      //IOCS1=1  IOCS2=0  IOCS3=1  IOCS4=1
    delay_ms(1)                      ;            
    M_IN9  =  (PINA & 0x01)          ;   
    M_IN10 = ((PINA & 0x02)>>1)&0x01 ;   
    M_IN11 = ((PINA & 0x04)>>2)&0x01 ;   
    M_IN12 = ((PINA & 0x08)>>3)&0x01 ;   
    M_IN13 = ((PINA & 0x10)>>4)&0x01 ;
    M_IN14 = ((PINA & 0x20)>>5)&0x01 ; 
    M_IN15 = ((PINA & 0x40)>>6)&0x01 ;
    M_IN16 = ((PINA & 0x80)>>7)&0x01 ; 

    PORTG  = 0x1B                    ;      //IOCS1=1  IOCS2=1  IOCS3=0  IOCS4=1
    delay_ms(1)                      ;            
    M_IN17 =  (PINA & 0x01)          ;   
    M_IN18 = ((PINA & 0x02)>>1)&0x01 ;   
    M_IN19 = ((PINA & 0x04)>>2)&0x01 ;   
    M_IN20 = ((PINA & 0x08)>>3)&0x01 ;   
    M_IN21 = ((PINA & 0x10)>>4)&0x01 ;
    M_IN22 = ((PINA & 0x20)>>5)&0x01 ; 
    M_IN23 = ((PINA & 0x40)>>6)&0x01 ;
    M_IN24 = ((PINA & 0x80)>>7)&0x01 ;    

    PORTG  = 0x17                    ;      //IOCS1=1  IOCS2=1  IOCS3=1  IOCS4=0
    delay_ms(1)                      ;            
    HV_IN1 =  (PINA & 0x01)          ;   
    HV_IN2 = ((PINA & 0x02)>>1)&0x01 ;   
    HV_IN3 = ((PINA & 0x04)>>2)&0x01 ;   
    HV_IN4 = ((PINA & 0x08)>>3)&0x01 ;   
    HV_IN5 = ((PINA & 0x10)>>4)&0x01 ;
    HV_IN6 = ((PINA & 0x20)>>5)&0x01 ;
    HV_IN7 = ((PINA & 0X40)>>6)&0x01 ; 
    HV_IN8 = ((PINA & 0X80)>>7)&0x01 ;     
    
    PORTG  = 0x1F                    ;

    DDRA   = 0xFF                    ;        
    
#ifdef Extra_Inputs        
       if((!OVL)&&(OVL_M)){OVL=1;OVL_MC=1;}

       if((!FUL)&&(FUL_M)){FUL=1;FUL_MC=1;}

       if((!DOO)&&(DO_M)){DOO=1;DOO_MC=1;}

       if((!DOC)&&(DC_M)){DOC=1;DOC_MC=1;}


       if((!OVL_M)&&(OVL_MC)){OVL=0;OVL_MC=0;}
       if((!FUL_M)&&(FUL_MC)){FUL=0;FUL_MC=0;}
       if((!DO_M)&&(DOO_MC)){DOO=0;DOO_MC=0;}
       if((!DC_M)&&(DOC_MC)){DOC=0;DOC_MC=0;}
         
#endif

} 