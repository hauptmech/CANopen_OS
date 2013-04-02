/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#if defined(WIN32) && !defined(__CYGWIN__)
	#include <windows.h>
	#define CLEARSCREEN "cls"
	#define SLEEP(time) Sleep(time * 1000)
#else
	#include <unistd.h>
	#include <stdio.h>
	#include <string.h>
	#include <stdlib.h>
	#define CLEARSCREEN "clear"
	#define SLEEP(time) sleep(time)

	#include <readline/readline.h>
	#include <readline/history.h>
#include <semaphore.h>
#include <time.h>
#include <errno.h>
#endif

//****************************************************************************
// INCLUDES
#include "canfestival.h"
#include "CANOpenShell.h"
#include "CANOpenShellMasterOD.h"
#include "CANOpenShellSlaveOD.h"

//****************************************************************************
// DEFINES
#define MAX_NODES 127
#define cst_str4(c1, c2, c3, c4) ((((unsigned int)0 | \
                                    (char)c4 << 8) | \
                                   (char)c3) << 8 | \
                                  (char)c2) << 8 | \
                                 (char)c1

#define WAIT_NS 500000000l
#define INIT_ERR 2
#define QUIT 1
#define handle_error(msg) \
    do { perror(msg); exit(EXIT_FAILURE); } while (0)
//****************************************************************************
// GLOBALS
char BoardBusName[31];
char BoardBaudRate[5];
s_BOARD Board = {BoardBusName, BoardBaudRate};
CO_Data* CANOpenShellOD_Data;
char LibraryPath[512];

sem_t Write_sem;
struct timespec Write_sem_ts;
sem_t Read_sem;
struct timespec Read_sem_ts;
int CurrentNode=0;

/* Sleep for n seconds */
void SleepFunction(int second)
{
#ifdef USE_RTAI
	sleep(second);
#else
	SLEEP(second);
#endif
}

/* Ask a slave node to go in operational mode */
void StartNode(UNS8 nodeid)
{
	masterSendNMTstateChange(CANOpenShellOD_Data, nodeid, NMT_Start_Node);
}

/* Ask a slave node to go in pre-operational mode */
void StopNode(UNS8 nodeid)
{
	masterSendNMTstateChange(CANOpenShellOD_Data, nodeid, NMT_Stop_Node);
}

/* Ask a slave node to reset */
void ResetNode(UNS8 nodeid)
{
	masterSendNMTstateChange(CANOpenShellOD_Data, nodeid, NMT_Reset_Node);
}

/* Reset all nodes on the network and print message when boot-up*/
void DiscoverNodes()
{
	printf("Wait for Slave nodes bootup...\n\n");
	ResetNode(0x00);
}

int get_info_step = 0;
/* Callback function that check the read SDO demand */
void CheckReadInfoSDO(CO_Data* d, UNS8 nodeid)
{
	UNS32 abortCode;
	UNS32 data=0;
	UNS32 size=64;

	if(getReadResultNetworkDict(CANOpenShellOD_Data, nodeid, &data, &size, &abortCode) != SDO_FINISHED)
		printf("Master : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
	else
	{
		/* Display data received */
		switch(get_info_step)
		{
			case 1:
					printf("Device type     : %x\n", data);
					break;
			case 2:
					printf("Vendor ID       : %x\n", data);
					break;
			case 3:
					printf("Product Code    : %x\n", data);
					break;
			case 4:
					printf("Revision Number : %x\n", data);
					break;
		}
	}
	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(CANOpenShellOD_Data, nodeid, SDO_CLIENT);

	GetSlaveNodeInfo(nodeid);
}

/* Retrieve node informations located at index 0x1000 (Device Type) and 0x1018 (Identity) */
void GetSlaveNodeInfo(UNS8 nodeid)
{
		switch(++get_info_step)
		{
			case 1: /* Get device type */
				printf("##################################\n");
				printf("#### Informations for node %x ####\n", nodeid);
				printf("##################################\n");
				readNetworkDictCallback(CANOpenShellOD_Data, nodeid, 0x1000, 0x00, 0, CheckReadInfoSDO, 0);
				break;

			case 2: /* Get Vendor ID */
				readNetworkDictCallback(CANOpenShellOD_Data, nodeid, 0x1018, 0x01, 0, CheckReadInfoSDO, 0);
				break;

			case 3: /* Get Product Code */
				readNetworkDictCallback(CANOpenShellOD_Data, nodeid, 0x1018, 0x02, 0, CheckReadInfoSDO, 0);
				break;

			case 4: /* Get Revision Number */
				readNetworkDictCallback(CANOpenShellOD_Data, nodeid, 0x1018, 0x03, 0, CheckReadInfoSDO, 0);
				break;

			case 5: /* Print node info */
				get_info_step = 0;
		}
}
/* Callback function that check the read SDO demand */
void CheckReadSDO(CO_Data* d, UNS8 nodeid)
{
	UNS32 abortCode;
	UNS32 data=0;
	UNS32 size=64;

	if(getReadResultNetworkDict(CANOpenShellOD_Data, nodeid, &data, &size, &abortCode) != SDO_FINISHED)
		printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
	else
		printf("\n= 0x%x (%d)\n", data,data);

	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(CANOpenShellOD_Data, nodeid, SDO_CLIENT);
}
/* Read a slave node object dictionary entry */
void ReadDeviceEntry(char* sdo)
{
	int ret=0;
	int nodeid;
	int index;
	int subindex;
	int datatype = 0;

	ret = sscanf(sdo, "rsdo#%2x,%4x,%2x", &nodeid, &index, &subindex);
	if (ret == 3)
	{

		printf("##################################\n");
		printf("#### Read SDO                 ####\n");
		printf("##################################\n");
		printf("NodeId   : %2.2x\n", nodeid);
		printf("Index    : %4.4x\n", index);
		printf("SubIndex : %2.2x\n", subindex);

		readNetworkDictCallback(CANOpenShellOD_Data, (UNS8)nodeid, (UNS16)index, (UNS8)subindex, (UNS8)datatype, CheckReadSDO, 0);
	}
	else
		printf("Wrong command  : %s\n", sdo);
}

/* Read a slave node object dictionary entry */
void ReadSDOEntry(int nodeid, int index, int subindex)
{
	int ret=0;
	int datatype = 0;

		readNetworkDictCallback(CANOpenShellOD_Data, (UNS8)nodeid, (UNS16)index, (UNS8)subindex, (UNS8)datatype, CheckReadSDO, 0);
}


UNS8 SDO_read_callback_result;
UNS32 SDO_read_callback_abortCode;
char SDO_read_data[256];
UNS32 SDO_read_data_size;
/* Callback function that check the read SDO demand */
void SDO_read_callback(CO_Data* d, UNS8 nodeid)
{

	SDO_read_data_size = 255;
	SDO_read_callback_result = getReadResultNetworkDict(CANOpenShellOD_Data, nodeid, &SDO_read_data, &SDO_read_data_size, &SDO_read_callback_abortCode);

	SDO_read_data[SDO_read_data_size] = 0;
	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(CANOpenShellOD_Data, nodeid, SDO_CLIENT);
	sem_post(&Read_sem);
}

/* Read a slave node object dictionary entry */
UNS8 SDO_read(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8 dataType, UNS8 useBlockMode)
{
	
	int ret=0;
	
	UNS8 readResult;
	int s;

	SDO_read_callback_result = 44; //set to a random number



	readResult = readNetworkDictCallback(CANOpenShellOD_Data, (UNS8)nodeId, (UNS16)index, (UNS8)subIndex, (UNS8)dataType, SDO_read_callback, useBlockMode);
	LeaveMutex();
	
	if (clock_gettime(CLOCK_REALTIME, &Read_sem_ts) == -1)
		handle_error("clock_gettime");
	
	Read_sem_ts.tv_nsec += WAIT_NS; // 100ms
			
	if (Read_sem_ts.tv_nsec >= 1000000000l){
		Read_sem_ts.tv_sec++;
		Read_sem_ts.tv_nsec -= 1000000000l;
	}

	while ((s = sem_timedwait(&Read_sem, &Read_sem_ts)) == -1 && errno == EINTR)
		continue;       /* Restart if interrupted by handler */

	/* Check what happened */

	if (s == -1) {
		if (errno == ETIMEDOUT)
			printf("sem_timedwait() timed out\n");
		else
			perror("sem_timedwait");
	} else
		if(SDO_read_callback_result != SDO_FINISHED)
			printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeId, SDO_read_callback_abortCode);
		/*else
			printf("Result: %s\n",SDO_read_data); */

	return readResult;
}

UNS8 SDO_write_callback_result;
UNS32 SDO_write_callback_abortCode;
/* Callback function that check the write SDO demand */
void SDO_write_callback(CO_Data* d, UNS8 nodeid)
{
	UNS32 abortCode;
	
	SDO_write_callback_result = getWriteResultNetworkDict(CANOpenShellOD_Data, nodeid, &SDO_write_callback_abortCode);
	
	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(CANOpenShellOD_Data, nodeid, SDO_CLIENT);
	sem_post(&Write_sem);
}
/* Write a slave node object dictionnary entry */
UNS8 SDO_write(CO_Data* d, UNS8 nodeId, UNS16 index,
		       UNS8 subIndex, UNS32 count, UNS8 dataType, void *data, UNS8 useBlockMode)
{
	int ret=0;
	
	UNS8 writeResult;
	int s;

	SDO_write_callback_result = 44; //set to a random number
	
	//-- Write command --//
	writeResult = writeNetworkDictCallBack(d, nodeId,index, subIndex, count, dataType, data, SDO_write_callback, useBlockMode);
	LeaveMutex();
	
	if (clock_gettime(CLOCK_REALTIME, &Write_sem_ts) == -1)
		handle_error("clock_gettime");
	
	Write_sem_ts.tv_nsec += WAIT_NS; // 100ms
			
	if (Write_sem_ts.tv_nsec >= 1000000000l){
		Write_sem_ts.tv_sec++;
		Write_sem_ts.tv_nsec -= 1000000000l;
	}

	while ((s = sem_timedwait(&Write_sem, &Write_sem_ts)) == -1 && errno == EINTR)
		continue;       /* Restart if interrupted by handler */

	/* Check what happened */

	if (s == -1) {
		if (errno == ETIMEDOUT)
			printf("sem_timedwait() timed out\n");
		else
			perror("sem_timedwait");
	} else
		if(SDO_write_callback_result != SDO_FINISHED)
			printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeId, SDO_write_callback_abortCode);

	return writeResult;

}


/* Callback function that check the write SDO demand */
void CheckWriteSDO(CO_Data* d, UNS8 nodeid)
{
	UNS32 abortCode;

	if(getWriteResultNetworkDict(CANOpenShellOD_Data, nodeid, &abortCode) != SDO_FINISHED)
		printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
	else
		printf("\nSend data OK\n");

	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(CANOpenShellOD_Data, nodeid, SDO_CLIENT);
}
/* Write a slave node object dictionnary entry */
void WriteDeviceEntry(char* sdo)
{
	int ret=0;
	int nodeid;
	int index;
	int subindex;
	int size;
	int data;

	ret = sscanf(sdo, "wsdo#%2x,%4x,%2x,%2x,%x", &nodeid , &index, &subindex, &size, &data);
	if (ret == 5)
	{
		printf("##################################\n");
		printf("#### Write SDO                ####\n");
		printf("##################################\n");
		printf("NodeId   : %2.2x\n", nodeid);
		printf("Index    : %4.4x\n", index);
		printf("SubIndex : %2.2x\n", subindex);
		printf("Size     : %2.2x\n", size);
		printf("Data     : %x\n", data);

		writeNetworkDictCallBack(CANOpenShellOD_Data, nodeid, index, subindex, size, 0, &data, CheckWriteSDO, 0);
	}
	else
		printf("Wrong command  : %s\n", sdo);
}


/* Write a slave node object dictionnary entry */
void WriteSDOEntry(int nodeid, int index, int subindex, int size, UNS32  data)
{
	int ret=0;

	writeNetworkDictCallBack(CANOpenShellOD_Data, nodeid, index, subindex, size, 0, &data, CheckWriteSDO, 0);
}

void CANOpenShellOD_post_SlaveBootup(CO_Data* d, UNS8 nodeid)
{
	printf("Slave %x boot up\n", nodeid);
}

/***************************  CALLBACK FUNCTIONS  *****************************************/
void CANOpenShellOD_initialisation(CO_Data* d)
{
	printf("Node_initialisation\n");
}

void CANOpenShellOD_preOperational(CO_Data* d)
{
	printf("Node_preOperational\n");
}

void CANOpenShellOD_operational(CO_Data* d)
{
	printf("Node_operational\n");
}

void CANOpenShellOD_stopped(CO_Data* d)
{
	printf("Node_stopped\n");
}

void CANOpenShellOD_post_sync(CO_Data* d)
{
	//printf("Master_post_sync\n");
}

void CANOpenShellOD_post_TPDO(CO_Data* d)
{
	//printf("Master_post_TPDO\n");
}

/***************************  INITIALISATION  **********************************/

UNS32 OnStatus3Update(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
    printf("Status3: %x\n",Status3);
    return 0;
}
void Init(CO_Data* d, UNS32 id)
{
	if(Board.baudrate)
	{
		/* Init node state*/
		setState(CANOpenShellOD_Data, Initialisation);
	}
}

/***************************  CLEANUP  *****************************************/
void Exit(CO_Data* d, UNS32 nodeid)
{
	if(strcmp(Board.baudrate, "none"))
	{
		/* Reset all nodes on the network */
		masterSendNMTstateChange(CANOpenShellOD_Data, 0 , NMT_Reset_Node);

		/* Stop master */
		setState(CANOpenShellOD_Data, Stopped);
	}
}

int NodeInit(int NodeID, int NodeType)
{
	if(NodeType)
		CANOpenShellOD_Data = &CANOpenShellMasterOD_Data;
	else
		CANOpenShellOD_Data = &CANOpenShellSlaveOD_Data;

	/* Load can library */
	LoadCanDriver(LibraryPath);

	/* Define callback functions */
	CANOpenShellOD_Data->initialisation = CANOpenShellOD_initialisation;
	CANOpenShellOD_Data->preOperational = CANOpenShellOD_preOperational;
	CANOpenShellOD_Data->operational = CANOpenShellOD_operational;
	CANOpenShellOD_Data->stopped = CANOpenShellOD_stopped;
	CANOpenShellOD_Data->post_sync = CANOpenShellOD_post_sync;
	CANOpenShellOD_Data->post_TPDO = CANOpenShellOD_post_TPDO;
	CANOpenShellOD_Data->post_SlaveBootup=CANOpenShellOD_post_SlaveBootup;

	/* Open the Peak CANOpen device */
	if(!canOpen(&Board,CANOpenShellOD_Data)) return INIT_ERR;

	/* Defining the node Id */
	setNodeId(CANOpenShellOD_Data, NodeID);
	/* Start Timer thread */
	StartTimerLoop(&Init);
	return 0;
}

void help_menu(void)
{
	printf("Non-prefixed commands are passed via SDO OS interface on the bus.\n");
	printf("\n");
	printf(".node <nodeid> : Set the node to which unprefixed commands are sent.\n");
	printf("   Setup COMMAND (must be on the process invocation):\n");
	printf("     load#CanLibraryPath,channel,baudrate,nodeid,type (0:slave, 1:master)\n");
	printf("\n");
	printf("   NETWORK: (if nodeid=0x00 : broadcast)\n");
	printf("     .ssta#nodeid : Start a node\n");
	printf("     .ssto#nodeid : Stop a node\n");
	printf("     .srst#nodeid : Reset a node\n");
	printf("     .scan : Reset all nodes and print message when bootup\n");
	printf("     .wait#seconds : Sleep for n seconds\n");
	printf("\n");
	printf("   SDO: (size in bytes)\n");
	printf("     .info#nodeid\n");
	printf("     .rsdo#nodeid,index,subindex : read sdo\n");
	printf("        ex : .rsdo#42,1018,01\n");
	printf("     .wsdo#nodeid,index,subindex,size,data : write sdo\n");
	printf("        ex : .wsdo#42,6200,01,01,FF\n");
	printf("\n");
	printf("   Note: All numbers are hex\n");
	printf("\n");
	printf("     .clear: Clear the display\n");
	printf("     .help : Display this menu\n");
	printf("     .quit : Quit application\n");
	printf("\n");
	printf("\n");
}

int ExtractNodeId(char *command) {
	int nodeid;
	sscanf(command, "%2x", &nodeid);
	return nodeid;
}

int ProcessCommand(char* command)
{
	int ret = 0;
	int sec = 0;
	int NodeID;
	int NodeType;
	UNS32 data = 0;
	char buf[50];

	EnterMutex();
	switch(cst_str4(command[0], command[1], command[2], command[3]))
	{
		case cst_str4('h', 'e', 'l', 'p') : /* Display Help*/
					help_menu();
					break;
		case cst_str4('c', 'l', 'e', 'a') : /* Display Help*/
					system(CLEARSCREEN);
					break;					
		case cst_str4('s', 's', 't', 'a') : /* Slave Start*/
					StartNode(ExtractNodeId(command + 5));
					break;
		case cst_str4('s', 's', 't', 'o') : /* Slave Stop */
					StopNode(ExtractNodeId(command + 5));
					break;
		case cst_str4('s', 'r', 's', 't') : /* Slave Reset */
					ResetNode(ExtractNodeId(command + 5));
					break;
		case cst_str4('i', 'n', 'f', 'o') : /* Retrieve node informations */
					GetSlaveNodeInfo(ExtractNodeId(command + 5));
					break;
		case cst_str4('r', 's', 'd', 'o') : /* Read device entry */
					ReadDeviceEntry(command);
					break;
		case cst_str4('w', 's', 'd', 'o') : /* Write device entry */
					WriteDeviceEntry(command);
					break;
		case cst_str4('n', 'o', 'd', 'e') : /* Write device entry */
					ret = sscanf(command, "node %2x", &NodeID);
					data = 0;
					SDO_write(CANOpenShellOD_Data,NodeID,0x1024,0x00,1, 0, &data, 0);
					CurrentNode = NodeID;
					break;
		case cst_str4('c', 'm', 'd', ' ') : /* Write device entry */

					ret = sscanf(command, "cmd %2x,%s", &NodeID, buf );
					SDO_write(CANOpenShellOD_Data,NodeID,0x1023,0x01,strlen(buf),visible_string, buf, 0);
					SDO_read(CANOpenShellOD_Data,NodeID,0x1023,0x03,visible_string,0);
					
					return 0;
					break;

		case cst_str4('s', 'y', 'n', '0') : /* Display master node state */
                    stopSYNC(CANOpenShellOD_Data);
                    break;
		case cst_str4('s', 'y', 'n', '1') : /* Display master node state */
                    startSYNC(CANOpenShellOD_Data);
                    break;
		case cst_str4('s', 't', 'a', 't') : /* Display master node state */
                    printf("Status3: %x\n",Status3);
                    Status3 = 0;
                    break;
		case cst_str4('s', 'c', 'a', 'n') : /* Display master node state */
					DiscoverNodes();
					break;
		case cst_str4('w', 'a', 'i', 't') : /* Display master node state */
					ret = sscanf(command, "wait#%d", &sec);
					if(ret == 1) {
						LeaveMutex();
						SleepFunction(sec);
						return 0;
					}
					break;
		case cst_str4('g', 'o', 'o', 'o') : /* Quit application */
            setState(CANOpenShellOD_Data, Operational);
            break;
		case cst_str4('q', 'u', 'i', 't') : /* Quit application */
					LeaveMutex();
					return QUIT;
		case cst_str4('l', 'o', 'a', 'd') : /* Library Interface*/
					ret = sscanf(command, "load#%100[^,],%30[^,],%4[^,],%d,%d",
							LibraryPath,
							BoardBusName,
							BoardBaudRate,
							&NodeID,
							&NodeType);

					if(ret == 5)
					{
						LeaveMutex();
						ret = NodeInit(NodeID, NodeType);
						return ret;
					}
					else
					{
						printf("Invalid load parameters\n");
					}
					break;
		default :
					help_menu();
	}
	LeaveMutex();
	return 0;
}

int ProcessFocusedCommand(char* command)
{
	int ret = 0;
	int sec = 0;
	int NodeID;
	int NodeType;
	UNS32 data = 0;
	char buf[50];
    int size;
	int index;
	int subindex = 0;
	int datatype = 0;

	EnterMutex();
	switch(command[0])
	{

		case 's' : /* Slave Start*/
					StartNode(CurrentNode);
					break;
        case 't' : /* slave stop */
					StopNode(CurrentNode);
					break;
        case 'x' : /* slave reset */
					ResetNode(CurrentNode);
					break;
		case 'r' : /* Read device entry */
	                ret = sscanf(command, "r%4x,%2x", &index, &subindex);
                    if (ret)
                        ReadSDOEntry(CurrentNode,index,subindex);
					break;
		case 'w' : /* Write device entry */
	                ret = sscanf(command, "w%4x,%2x,%2x,%x", &index, &subindex, &size, &data);
                    if (ret)
                        WriteSDOEntry(CurrentNode,index,subindex,size,data);
					break;
		case '?' : /* Read device entry */
                    ReadSDOEntry(CurrentNode,0x6041,0);
					break;
		case 'c' : /* Write device entry */
	                ret = sscanf(command, "w%x", &data);
                    if (ret)
                        WriteSDOEntry(CurrentNode,0x6040,0,0x2,data);
					break;
		
		default :
					help_menu();
	}
	LeaveMutex();
	return 0;
}

/* A static variable for holding the line. */
static char *line_read = (char *)NULL;

/* Read a string, and return a pointer to it.
   Returns NULL on EOF. */
char *
rl_gets ()
{
  /* If the buffer has already been allocated,
     return the memory to the free pool. */
  if (line_read)
    {
      free (line_read);
      line_read = (char *)NULL;
    }

  /* Get a line from the user. */
  line_read = readline (">");

  /* If the line has any text in it,
     save it on the history. */
  if (line_read && *line_read)

    add_history (line_read);

  return (line_read);
}


/****************************************************************************/
/***************************  MAIN  *****************************************/
/****************************************************************************/

int main(int argc, char** argv)
{
	extern char *optarg;
	char command[200];
	char* res;
	int ret=0;
	int sysret=0;
	int i=0;

	if (sem_init(&Write_sem, 0, 0) == -1)
        handle_error("Writesem_init");
	if (sem_init(&Read_sem, 0, 0) == -1)
        handle_error("Readsem_init");	
	/* Defaults */
	strcpy(LibraryPath,"/usr/lib/libcanfestival_can_peak_linux.so");
	strcpy(BoardBusName,"0");
	strcpy(BoardBaudRate,"1M");

	/* Init stack timer */
	TimerInit();
        
	if (argc > 1){
        printf("ok\n");
		/* Strip command-line*/
		for(i=1 ; i<argc ; i++)
		{
			if(ProcessCommand(argv[i]) == INIT_ERR) goto init_fail;
		}
	}
    NodeInit(0,1);

    RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2003, 0, &OnStatus3Update);


	help_menu();
	CurrentNode = 3;
    sleep(1);
    //setState(CANOpenShellOD_Data, Operational);     // Put the master in operational mode
    stopSYNC(CANOpenShellOD_Data);

	/* Enter in a loop to read stdin command until "quit" is called */
	while(ret != QUIT)
	{
		// wait on stdin for string command
		rl_on_new_line ();
		res = rl_gets();
		//sysret = system(CLEARSCREEN);
		if(res[0]=='.'){
		ret = ProcessCommand(res+1);
		}
		else if(res[0]==','){
		ret = ProcessFocusedCommand(res+1);
		}
        else if (res[0]=='\n'){

        }
		else {
			
			EnterMutex();
			SDO_write(CANOpenShellOD_Data,CurrentNode,0x1023,0x01,strlen(res),visible_string, res, 0);

			EnterMutex();
			SDO_read(CANOpenShellOD_Data,CurrentNode,0x1023,0x03,visible_string,0);
			
			printf("%s\n",SDO_read_data);
		}
		fflush(stdout);
        usleep(500000);
	}

	printf("Finishing.\n");

	// Stop timer thread
	StopTimerLoop(&Exit);

	/* Close CAN board */
	canClose(CANOpenShellOD_Data);

init_fail:
	TimerCleanup();
	return 0;
}

