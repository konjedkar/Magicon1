/*! \file arp.c \brief ARP Protocol Library. */
//*****************************************************************************
//
// File Name	: 'arp.c'
// Title		: ARP Protocol Library
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
//*****************************************************************************

#include "global.h"
#include "net.h"
#include "nic.h"
#include "arp.h"  

//#define ARP_DEBUG

/// Single ARP table entry/record     
struct ArpEntry
{
	uint32_t ipaddr;			///< remote-note IP address
	uint8_t time;				///< time to live (in ARP table); this is decremented by arpTimer()
	struct netEthAddr ethaddr;	///< remote-node ethernet (hardware/mac) address	
};

struct ArpEntry ArpMyAddr;		///< my local interface information (IP and MAC address)
struct ArpEntry ArpTable[ARP_TABLE_SIZE];	///< ARP table of matched IP<->MAC associations


void arpInit(void)
{
	u08 i;
	// initialize all ArpTable elements to unused
	for(i=0; i<ARP_TABLE_SIZE; i++)
	{
		ArpTable[i].ipaddr = 0;
		ArpTable[i].time = 0;
	}
}

void arpSetAddress(struct netEthAddr* myeth, uint32_t myip)
{
	// set local address record
	//ArpMyAddr.ethaddr = *myeth;\
	ArpMyAddr.ethaddr.addr[0] = myeth->addr[0];
	ArpMyAddr.ethaddr.addr[1] = myeth->addr[1];
	ArpMyAddr.ethaddr.addr[2] = myeth->addr[2];
	ArpMyAddr.ethaddr.addr[3] = myeth->addr[3];
	ArpMyAddr.ethaddr.addr[4] = myeth->addr[4];
	ArpMyAddr.ethaddr.addr[5] = myeth->addr[5];
	ArpMyAddr.ipaddr = myip;
}

void arpArpIn(unsigned int len, struct netEthArpHeader* packet)
{
	#ifdef ARP_DEBUG
	printf("Received ARP Request\r\n");
//	arpPrintHeader( &packet->arp );
	#endif

	// for now, we just reply to requests
	// need to add ARP cache
	if(   //	(packet->arp.dipaddr == HTONL(ArpMyAddr.ipaddr)) &&
	      	(packet->arp.opcode == htons(ARP_OPCODE_REQUEST)) )
	{        

		// in ARP header
		// copy sender's address info to dest. fields
		packet->arp.dhwaddr = packet->arp.shwaddr;
		packet->arp.dipaddr = packet->arp.sipaddr;
		// fill in our information
	       	packet->arp.shwaddr.addr[0] = ArpMyAddr.ethaddr.addr[0];
	       	packet->arp.shwaddr.addr[1] = ArpMyAddr.ethaddr.addr[1];
	       	packet->arp.shwaddr.addr[2] = ArpMyAddr.ethaddr.addr[2];
	       	packet->arp.shwaddr.addr[3] = ArpMyAddr.ethaddr.addr[3];
	       	packet->arp.shwaddr.addr[4] = ArpMyAddr.ethaddr.addr[4];
	       	packet->arp.shwaddr.addr[5] = ArpMyAddr.ethaddr.addr[5];       
	       		       		       		       		       		       	
		packet->arp.sipaddr = HTONL(ArpMyAddr.ipaddr);
		// change op to reply
		packet->arp.opcode = htons(ARP_OPCODE_REPLY);
		
		// in ethernet header
		packet->eth.dest = packet->eth.src;
		//packet->eth.src  = ArpMyAddr.ethaddr;
		packet->eth.src.addr[0] = ArpMyAddr.ethaddr.addr[0];
		packet->eth.src.addr[1] = ArpMyAddr.ethaddr.addr[1];
		packet->eth.src.addr[2] = ArpMyAddr.ethaddr.addr[2];
		packet->eth.src.addr[3] = ArpMyAddr.ethaddr.addr[3];
		packet->eth.src.addr[4] = ArpMyAddr.ethaddr.addr[4];
		packet->eth.src.addr[5] = ArpMyAddr.ethaddr.addr[5];

		// send reply!
		nicSend(len, (unsigned char*)packet);
		#ifdef ARP_DEBUG
		printf("Sending ARP Reply\r\n");
//		arpPrintHeader( &packet->arp );
		#endif
		
	}
}

void arpIpIn(struct netEthIpHeader* packet)
{
	int8_t index;

	// check if sender is already present in arp table
	index = arpMatchIp(HTONL(packet->ip.srcipaddr));
	if(index != -1)
	{
		// sender's IP address found, update ARP entry
		ArpTable[index].ethaddr = packet->eth.src;
		// and we're done
		return;
	}

	// sender was not present in table,
	// must add in empty/expired slot
	for(index=0; index<ARP_TABLE_SIZE; index++)
	{
		if(!ArpTable[index].time)
		{
			// write entry
			ArpTable[index].ethaddr = packet->eth.src;
			ArpTable[index].ipaddr = HTONL(packet->ip.srcipaddr);
			ArpTable[index].time = ARP_CACHE_TIME_TO_LIVE;
			// and we're done
			return;
		}
	}

	// no space in table, we give up
}

void arpIpOut(struct netEthIpHeader* packet, uint32_t phyDstIp)
{
	int index;
	// check if destination is already present in arp table
	// use the physical dstIp if it's provided, otherwise the dstIp in packet
	if(phyDstIp)
		index = arpMatchIp(phyDstIp);
	else
		index = arpMatchIp(HTONL(packet->ip.destipaddr));
	// fill in ethernet info
	if(index != -1)
	{
		// ARP entry present, fill eth address(es)
		packet->eth.src.addr[0] = ArpMyAddr.ethaddr.addr[0];
		packet->eth.src.addr[1] = ArpMyAddr.ethaddr.addr[1];
		packet->eth.src.addr[2] = ArpMyAddr.ethaddr.addr[2];
		packet->eth.src.addr[3] = ArpMyAddr.ethaddr.addr[3];
		packet->eth.src.addr[4] = ArpMyAddr.ethaddr.addr[4];
		packet->eth.src.addr[5] = ArpMyAddr.ethaddr.addr[5];
		
		packet->eth.dest = ArpTable[index].ethaddr;
		packet->eth.type = HTONS(ETHTYPE_IP);
	}
	else
	{
		// not in table, must send ARP request
		packet->eth.src.addr[0] = ArpMyAddr.ethaddr.addr[0];
		packet->eth.src.addr[1] = ArpMyAddr.ethaddr.addr[1];
		packet->eth.src.addr[2] = ArpMyAddr.ethaddr.addr[2];
		packet->eth.src.addr[3] = ArpMyAddr.ethaddr.addr[3];
		packet->eth.src.addr[4] = ArpMyAddr.ethaddr.addr[4];
		packet->eth.src.addr[5] = ArpMyAddr.ethaddr.addr[5];
		// MUST CHANGE, but for now, send this one broadcast
		packet->eth.dest.addr[0] = 0xFF;
		packet->eth.dest.addr[1] = 0xFF;
		packet->eth.dest.addr[2] = 0xFF;
		packet->eth.dest.addr[3] = 0xFF;
		packet->eth.dest.addr[4] = 0xFF;
		packet->eth.dest.addr[5] = 0xFF;
		packet->eth.type = HTONS(ETHTYPE_IP);
	}
}

void arpTimer(void)
{
	int index;
	// this function meant to be called on a regular time interval

	// decrement time-to-live for all entries
	for(index=0; index<ARP_TABLE_SIZE; index++)
	{
		if(ArpTable[index].time)
			ArpTable[index].time--;
	}
}

int arpMatchIp(uint32_t ipaddr)
{
	uint8_t i;

	// check if IP address is present in arp table
	for(i=0; i<ARP_TABLE_SIZE; i++)
	{
		if(ArpTable[i].ipaddr == ipaddr)
		{
			// IP address found
			return i;
		}
	}

	// no match
	return -1;
}
/*
#ifdef ARP_DEBUG_PRINT
void arpPrintHeader(struct netArpHeader* packet)
{
	printfProgStrM("ARP Packet:\r\n");
	//debugPrintHexTable(60, (unsigned char*)&packet);
	// print operation type
	printfProgStrM("Operation   : ");
	if(packet->opcode == htons(ARP_OPCODE_REQUEST))
		printfProgStrM("REQUEST");
	else if(packet->opcode == htons(ARP_OPCODE_REPLY))
		printfProgStrM("REPLY");
	else
		printfProgStrM("UNKNOWN");
	printfCRLF();
	// print source hardware address
	printfProgStrM("SrcHwAddr   : ");	netPrintEthAddr(&packet->shwaddr);	printfCRLF();
	// print source protocol address
	printfProgStrM("SrcProtoAddr: ");	netPrintIPAddr(HTONL(packet->sipaddr));	printfCRLF();
	// print target hardware address
	printfProgStrM("DstHwAddr   : ");	netPrintEthAddr(&packet->dhwaddr);	printfCRLF();
	// print target protocol address
	printfProgStrM("DstProtoAddr: ");	netPrintIPAddr(HTONL(packet->dipaddr));	printfCRLF();
}


void arpPrintTable(void)
{
	uint8_t i;

	// print ARP table
	printfProgStrM("Time    Eth Address    IP Address\r\n");
	printfProgStrM("---------------------------------------\r\n");
	for(i=0; i<ARP_TABLE_SIZE; i++)
	{
		printfu08(ArpTable[i].time);
		printfProgStrM("   ");
		netPrintEthAddr(&ArpTable[i].ethaddr);
		printfProgStrM("  ");
		netPrintIPAddr(ArpTable[i].ipaddr);
		printfCRLF();
	}
}
#endif
*/