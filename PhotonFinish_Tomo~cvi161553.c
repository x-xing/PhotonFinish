#include <analysis.h>
#include "toolbox.h"
#include <gpib.h>
#include "PhotonFinish_tomo.h"
#include <formatio.h>
#include <utility.h>
#include <tcpsupp.h>
#include <ansi_c.h>
#include <cvirte.h>		
#include <userint.h>

#define PEAK_LEFT			0.41
#define FILTER_WIDTH    	2.16
#define RABBIT_IP			"192.168.1.123"
#define RABBIT_PORT			37829
#define DATA_BUFFER_SIZE    5000000

int CVICALLBACK tcpPhotonFinishClientCB (unsigned handle, int xType, int errCode, void *callbackData);
int CVICALLBACK DataPcsThreadFunction (void *functionData); 
int PlotInitialize (void);
int gpibMotorOn (int gpibAddr,int DeviceAxis);
int gpibSearchForHome (int gpibAddr,int DeviceAxis);
double gpibReadActualPosition (int gpibAddr,int DeviceAxis);
int gpibWaitForMotionStop (int gpibAddr,int DeviceAxis,int DelayTime);
int gpibMoveRelativeAngle (int gpibAddr,int DeviceAxis,float Increment);
int gpibMoveAbsoluteAngle (int gpibAddr,int DeviceAxis, float Angle);
int gpibIsMotionDone (int gpibAddr,int DeviceAxis);  
int tomoSetMotorForBasis (int TomoBasisIndex);
int gpibIsPicoDone (int gpibAddr);


static int panelHandle;
int GuiRunning, tcpConnected;							//global flags. unprotected here.
int TcpConnected = 0;
char RawDataBuf[DATA_BUFFER_SIZE];				  //raw data buffer, for writing and reading threads.
unsigned int TcpHandle;
int TestFileHandle;													 //File handle to save test data.
char Msg[100]={0}; 													  //String for output formatting.

double CfgSetup[2][8] = {7,1,7,2,1.1,2.2,3.3,4.4,4,1,4,2,5.5,6.6,7.7,8.8};	   //Configuration setup, 
																  //including 2 channels, and 7 data.
	/*	CfgSetup[0][0] ChA GPIB Addr.
		CfgSetup[0][1] Axis number for HWP
		CfgSetup[0][2] ChA GPIB Addr.
		CfgSetup[0][3] Axis number for QWP
		CfgSetup[0][4] Fast Axis Angle Calibration of HWP
		CfgSetup[0][5] Slow Axis Angle Calibration of HWP
		CfgSetup[0][6] Fast Axis Angle Calibration of QWP
		CfgSetup[0][7] Slow Axis Angle Calibration of QWP
		CfgSetup[1][0] ChB GPIB Addr.
		CfgSetup[1][1] Axis number for HWP
		CfgSetup[1][2] ChB GPIB Addr.
		CfgSetup[1][3] Axis number for HWP
		CfgSetup[1][4] Fast Axis Angle Calibration of HWP
		CfgSetup[1][5] Slow Axis Angle Calibration of HWP
		CfgSetup[1][6] Fast Axis Angle Calibration of QWP
		CfgSetup[1][7] Slow Axis Angle Calibration of QWP*/


double StartTime=0; 
int CountingTime=0;
int CountingFlag=0,homeflag=1;
int FidSaveFile=-1;

char PathNameCali[1024] = {0};
char PathNameTomo[1024] = {0};    
char PathNameHOM[1024] 	= {0};

int Cal_Value[4]={0};
long nFileLength=0;
unsigned int nCoinCounts=0;
char pathName[MAX_PATHNAME_LEN]={0};

int TimeAxis[20000]={0};
unsigned int CoinCountsData[20000]={0};

int TotalCounts;
int CoinFrmPckt = 0;		 //Counted from time stamped packet. Idealy should be the same as CoinAB.



unsigned int tcpDataIndex = 0;

int gpibAddrA = 0;
int gpibAddrB = 0;
int DeviceAxisA =0;
int DeviceAxisB =0;
//int DelayAfterMotion = 0;

float WAngInc = 0;
float WActPosA = 0;
float WActPosB = 0;

double WCaliData[2048][4] = {0};							  //the WP position and the Singles Data.
double TransPWCaliData[4][2048] = {0};
int cWCountingTime = 1;										 //Time for each datapoint. (in Seconds.)

int WCaliDataIndex = 0;														   	  //Data Point Index.


int WCaliFlag = 0;
int W2CountFlag = 0;							   //To make sure the data is taken after the motion.
int WScanIndex = 0;

int TomoFlag = 0;
int cTCountingTime = 1;								  //Time for each waveplate settings, in Seconds.
int TMotionDone = 0; 
int T2CountFlag = 0;							   //To make sure the data is taken after the motion.  
int TomoData[5] = {0};						   //rows: 36 basis; columns: basis, singles A, B, Coins.
int TomoBasisIndex = 0;														 //Basis index from 0-35;
int TomoCompleteBasis[16] = {0, 1, 7, 6, 24, 25, 13, 12, 16, 14, 26, 2, 8, 11, 5, 29}; 
																	//One complete Set of Tomography.
int FilterFlag = 0;
int nFilteredCounts = 0;
int OneCount[5]={0};
double TimeReg[3] = {0};
int ind = 0;
int numTomoBasis = 0;
int LastLEDA = PANEL_LEDAH;
int LastLEDB = PANEL_LEDBH;
double TActPos = 0;


//HOM Data handling
int HOMFlag = 0;
int nPicoPulseCount = 0;
int HEachPosTime = 0;
int HOMFileHandle = -1;
int TomoFileHandle = -1;
int RawDataFileHandle = -1;
int CaliFileHandle = -1;
int HOM2CountFlag = 0;
int HOMData[5]	= {0};
char CharBuffer[64] = {0};
int PicoAddr = 0;														 //gpib address of PicoMotor.
int PicoPos = 0;
int PicoDir = 0;											   //direction of the picomotor movement.
double PicoScanRange = 0;									//the range over which HOM data is taken.


/*************************************** Main function *********************************************/
int main (int argc, char *argv[])
{
	int DataWrThrFuncID, DataRdThrFuncID;
	
	if (InitCVIRTE (0, argv, 0) == 0)
		return -1;	/* out of memory */
	if ((panelHandle = LoadPanel (0, "PhotonFinish_tomo.uir", PANEL)) < 0)
		return -1;
	
	GuiRunning = 1;
	DisableBreakOnLibraryErrors ();								  
	
	CmtScheduleThreadPoolFunction (DEFAULT_THREAD_POOL_HANDLE, \
		DataPcsThreadFunction, NULL, &DataAcqThreadFunctionID);
	
	DisplayPanel (panelHandle);
	RunUserInterface ();
	
	GuiRunning = 0;
	CmtWaitForThreadPoolFunctionCompletion(DEFAULT_THREAD_POOL_HANDLE, \
		DataAcqThreadFunctionID, OPT_TP_PROCESS_EVENTS_WHILE_WAITING);
	CmtReleaseThreadPoolFunctionID(DEFAULT_THREAD_POOL_HANDLE, DataWrThrFuncID);
	CmtDiscardTSQ(RawDataQueue);
	
	
	DiscardPanel (panelHandle);
	return 0;
}

int CVICALLBACK panelCB (int panel, int event, void *callbackData,
		int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_GOT_FOCUS:

			break;
		case EVENT_LOST_FOCUS:

			break;
		case EVENT_CLOSE:
			QuitUserInterface(0);
			break;
	}
	return 0;
}

int CVICALLBACK CalibrateValue (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	unsigned char DataBuffer[4]={0};
	
	switch (event)
	{
		case EVENT_COMMIT:
			Msg[0]='C';
			Msg[1]=0x03;									 //Calibration on high level.
			ClientTCPWrite (TcpHandle, Msg, 2, 1000);
			Msg[0]='3'; 
			ClientTCPWrite (TcpHandle, Msg, 1, 1000);
			Msg[0]='4'; 
			ClientTCPWrite (TcpHandle, Msg, 1, 1000);
			Msg[0]='C';
			Msg[1]=0x01;						   			  //Calibration on low level.
			ClientTCPWrite (TcpHandle, Msg, 2, 1000);
			Msg[0]='3';
			ClientTCPWrite (TcpHandle, Msg, 1, 1000);
			Msg[0]='4'; 
			ClientTCPWrite (TcpHandle, Msg, 1, 1000);
			
						
			ClientTCPRead (TcpHandle, DataBuffer, 3, 2000);
			if (DataBuffer[0]=='3')
			{
				Cal_Value[0] = DataBuffer[1]*256+DataBuffer[2]; 
				sprintf(Msg,"%d",Cal_Value[0]);
				SetCtrlVal(panelHandle,PANEL_A_High,Msg);  
			}
			ClientTCPRead (TcpHandle, DataBuffer, 3, 2000);
			if (DataBuffer[0]=='4')
			{
				Cal_Value[2]=DataBuffer[1]*256+DataBuffer[2];
				sprintf(Msg,"%d",Cal_Value[2]);
				SetCtrlVal(panelHandle,PANEL_B_High,Msg);  
			}
		
			ClientTCPRead (TcpHandle, DataBuffer, 3, 2000);
			if (DataBuffer[0]=='3')
			{
				Cal_Value[1]=DataBuffer[1]*256+DataBuffer[2];
				sprintf(Msg,"%d",Cal_Value[1]);
				SetCtrlVal(panelHandle,PANEL_A_Low,Msg);  
			}
			ClientTCPRead (TcpHandle, DataBuffer, 3, 2000);
			if (DataBuffer[0]=='4')
			{
				Cal_Value[3]=DataBuffer[1]*256+DataBuffer[2];
				sprintf(Msg,"%d",Cal_Value[3]);
				SetCtrlVal(panelHandle,PANEL_B_Low,Msg);  
				SetCtrlVal(panelHandle,PANEL_StatusMessage,"Calibration Completed"); 
			}
			
			break;
	}
	return 0;
}   

/*
int CVICALLBACK ConnectToServer (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			
			if(ConnectToTCPServer (&TcpHandle, RABBIT_PORT, \
							RABBIT_IP, tcpPhotonFinishClientCB, NULL, 5000)<0)
				SetCtrlVal(panelHandle,PANEL_StatusMessage,"Error in connecting to server!");
			else
				SetCtrlVal(panelHandle,PANEL_StatusMessage,"Connected");
				SetCtrlAttribute(panelHandle,PANEL_CalibrateBUTTON,ATTR_DIMMED,0);
				SetCtrlAttribute(panelHandle,PANEL_GetCoinButton,ATTR_DIMMED,0);
				SetCtrlAttribute(panelHandle,PANEL_WCaliButton,ATTR_DIMMED,0);
				SetCtrlAttribute(panelHandle,PANEL_TomoButton,ATTR_DIMMED,0);
				SetCtrlAttribute(panelHandle,PANEL_HOMButton,ATTR_DIMMED,0);
				SetCtrlAttribute(panelHandle,PANEL_ChkBox_TimeStamp,ATTR_DIMMED,0);
				SetCtrlVal(panelHandle,PANEL_ChkBox_TimeStamp,0);
				CountingFlag = 0;

			break;
	}
	return 0;
}
*/

/********************** TCP Callback Function, old file *******************************/
/*int CVICALLBACK tcpPhotonFinishClientCB(unsigned handle, \
											int xType, int errCode, void *callbackData)
{
	unsigned int nCoinCountsPack=0;
	unsigned char CommandToRabbit[4]={0}, SinglesBuffer[15]={0};	
	int		TimedCoinAB, WaitTime;
	
	if((Timer()-StartTime)>CountingTime && CountingFlag)					 //Counting Time ellapse.
	{
		CommandToRabbit[0]='R';
		CommandToRabbit[1]=0x00;											 //Send a 'Stop' Command.
		ClientTCPWrite (TcpHandle, CommandToRabbit, 2, 1000);	
		CountingFlag=0;
		SetCtrlVal(panelHandle, PANEL_CountingLED,0);	   //Set CountingLED to Dark: Counting Stops.
		CloseFile(TestFileHandle);
		if(FidSaveFile !=-1)
			CloseFile(FidSaveFile);
		SetCtrlVal(panelHandle,PANEL_StatusMessage,"Coincidence Data Aquired.");
		
	}
		
	switch(xType)
	{
		case TCP_DISCONNECT:
			SetCtrlVal(panelHandle,PANEL_LedConnected, 0);					 //showing no connection.
			break;
		
		case TCP_DATAREADY:
			ClientTCPRead (TcpHandle, tcpDataBuffer, 1, 1000);
			switch(tcpDataBuffer[0])
			{
				case 'H':
					SetCtrlVal(panelHandle,PANEL_HeartBeat,LED_flag);				
					SetCtrlAttribute(panelHandle,PANEL_PlotButton,ATTR_DIMMED,0);	
																			  //Re-Enable the Button.
					break;															
					
				case 'P':
					ClientTCPRead (TcpHandle, &nCoinCountsPack, 1, 1000);
					ClientTCPRead (TcpHandle, tcpDataBuffer, nCoinCountsPack*7, 1000);
					CoinFrmPckt += nCoinCountsPack;
					//if(FidSaveFile != -1)
						//WriteFile(FidSaveFile, tcpDataBuffer, nCoinCountsPack*7);
					break;
					
				case 'R':
					tcpDataBuffer[0]='R';
					ClientTCPRead (TcpHandle, tcpDataBuffer+1, 6, 1000);
					tcpDataBuffer[0]='R';     
					//if(FidSaveFile != -1)
						//WriteFile(FidSaveFile, tcpDataBuffer, 7);
					break;
					
				case 'S':
					ClientTCPRead (TcpHandle, SinglesBuffer, 15, 1000);
					SinglesA=SinglesBuffer[0]*65536+SinglesBuffer[1]*256+SinglesBuffer[2];
					SinglesB=SinglesBuffer[3]*65536+SinglesBuffer[4]*256+SinglesBuffer[5];
					CoinAB = SinglesBuffer[6]*65536+SinglesBuffer[7]*256+SinglesBuffer[8];
					TimedCoinAB = SinglesBuffer[9]*65536+SinglesBuffer[10]*256+SinglesBuffer[11]; 
					WaitTime = SinglesBuffer[12]*65536+SinglesBuffer[13]*256+SinglesBuffer[14]; 
					
					sprintf(Msg,"%d",SinglesA);
					SetCtrlVal(panelHandle,PANEL_TXTSINGLESA,Msg);  
					sprintf(Msg,"%d",SinglesB);
					SetCtrlVal(panelHandle,PANEL_TXTSINGLESB,Msg);
					sprintf(Msg,"%d",CoinAB);
					SetCtrlVal(panelHandle,PANEL_nCoinRate,Msg);
					sprintf(Msg,"%d",TimedCoinAB);    
					SetCtrlVal(panelHandle,PANEL_TXTTimedRate,Msg);
					sprintf(Msg,"%d",CoinFrmPckt);
					SetCtrlVal(panelHandle,PANEL_TXTCoinFrmPckt,Msg);
					
					
					sprintf(Msg, "%d	%d	%d	%d	%d	%d\n", SinglesA, SinglesB, \
						CoinAB, TimedCoinAB, CoinFrmPckt, WaitTime);
					WriteFile(TestFileHandle, Msg, strlen(Msg));
					
					
					//To calculate the raw data file here?
					//Perform Digital filtering?
					
				  //######### The digital filtering is not working quite well. We will use calibrated 
					background subtraction as our filtering.
					
					if(FilterFlag)
					{
						for(ind=0;ind<tcpDataIndex;ind+=7)
 						{
							//while (cpDataBuffer[i]!=82)
							//{
							//	if(i<nFileLength-7)
							//		i++;
							//	else
							//		break;
							//}  

  							OneCount[0]=tcpDataBuffer[ind+1]*256+tcpDataBuffer[ind+2];
    						OneCount[1]=tcpDataBuffer[ind+3]*256+tcpDataBuffer[ind+4];
    						OneCount[2]=tcpDataBuffer[ind+5];
    						OneCount[3] =(tcpDataBuffer[ind+6]&0x04)/4*2-1;
    						OneCount[4]=(tcpDataBuffer[ind+6]&0x01)||(tcpDataBuffer[ind+6]&0x02);
    					
							TimeReg[0]=1-1.0*(OneCount[0]-Cal_Value[1])/(Cal_Value[0]-Cal_Value[1]);		//Here is for error catching in log computation.
							TimeReg[1]=1-1.0*(OneCount[1]-Cal_Value[3])/(Cal_Value[2]-Cal_Value[3]);		//should be: -log(t_A)*9.47, and -log(t_B)*9.49
							
							
							if(OneCount[4]==0 && TimeReg[0]>0 && TimeReg[1]>0) 
							{
								TimeReg[2]=log(TimeReg[1])*9.49-log(TimeReg[0])*9.47+\
														10*(OneCount[2])*(OneCount[3]);
								sprintf(CharBuffer,"%f\n",TimeReg[2]);
								WriteFile(RawDataFileHandle,CharBuffer, strlen(CharBuffer));
								tempTime = (100*(int)(TimeReg[2]-PEAK_LEFT+FILTER_WIDTH*300))\
																	% (int)(FILTER_WIDTH*100);
									
								if ( tempTime > (50*FILTER_WIDTH) )
									nFilteredCounts +=2;  	/  /this is the signal left in the peaks.
							}else
								nFilteredCounts ++;
						
						}
						
					}#########//

					
					
					
					if(WCaliFlag == 1 && TomoFlag ==0)
					{
						if(gpibIsMotionDone(gpibAddrA,DeviceAxisA) && \
									gpibIsMotionDone(gpibAddrB,DeviceAxisB))
						{
							if(W2CountFlag > 0)   						  //count the second singles, 
																//to make sure this is a full second;
							{
								// Store the data to memory
								WActPosA = gpibReadActualPosition(gpibAddrA,DeviceAxisA);
								WActPosB = gpibReadActualPosition(gpibAddrB,DeviceAxisB);
								sprintf(Msg,"%3.3f",WActPosA);
								SetCtrlVal(panelHandle,PANEL_ActPosRead_ChA,Msg);
								sprintf(Msg,"%3.3f",WActPosB);
								SetCtrlVal(panelHandle,PANEL_ActPosRead_ChB,Msg);
								
								WCaliData[WCaliDataIndex][0] = WActPosA;
								WCaliData[WCaliDataIndex][1] += SinglesA;
								WCaliData[WCaliDataIndex][2] = WActPosB;
								WCaliData[WCaliDataIndex][3] += SinglesB;
								
								if(W2CountFlag == cWCountingTime)
								{
									sprintf(CharBuffer,"%3.2f\t%5.0f\t%3.2f\t%5.0f\n",\
										WActPosA,WCaliData[WCaliDataIndex][1],WActPosB,\
															WCaliData[WCaliDataIndex][3]);
									WriteFile(CaliFileHandle,CharBuffer,strlen(CharBuffer));
									
									WCaliDataIndex++;					 //Stop accumulating singles. 
																			   //For next data point.
									if(WScanIndex < (180/WAngInc))   				   //A full Scan?                                     
									{											  //move to next inc.
										gpibMoveRelativeAngle(gpibAddrA,DeviceAxisA, WAngInc);    
										gpibMoveRelativeAngle(gpibAddrB,DeviceAxisB, WAngInc);
										WScanIndex++;
									}
									else
									{
										//write to file
										//popup window to ask for file name (and path);
										
										Transpose(WCaliData,2048,4,TransPWCaliData);
										//plot on the screen
										SetCtrlAttribute(panelHandle,PANEL_GRAPH,ATTR_XNAME,\
																		"Waveplate Positions");
										SetCtrlAttribute(panelHandle,PANEL_GRAPH,ATTR_YNAME,\
																		 "Singles Count Rate");
										SetCtrlAttribute(panelHandle,PANEL_GRAPH,ATTR_LABEL_TEXT,\
																		 "Waveplate Calibration");
									
										PlotXY(panelHandle,PANEL_GRAPH,TransPWCaliData,\
											&(TransPWCaliData[1][0]),WCaliDataIndex,\
											VAL_DOUBLE,VAL_DOUBLE,VAL_THIN_LINE,\
											VAL_NO_POINT,VAL_SOLID,1,VAL_RED); 
										PlotXY(panelHandle,PANEL_GRAPH,&(TransPWCaliData[2][0]),\
											&(TransPWCaliData[3][0]),WCaliDataIndex,VAL_DOUBLE,\
											VAL_DOUBLE,VAL_THIN_LINE,VAL_NO_POINT,VAL_SOLID,1,\
											VAL_YELLOW); 
										
										WCaliFlag =0;  			//finish Calibration
										WScanIndex = 0;			//Prepare for next Calibration
										WCaliDataIndex = 0;
									
										CloseFile(CaliFileHandle);
										CommandToRabbit[0]='R';
										CommandToRabbit[1]=0x00;												//Send a 'Stop' Command
										ClientTCPWrite (TcpHandle, \
											CommandToRabbit, 2, 1000);	
										SetCtrlVal(panelHandle, PANEL_CountingLED,0);
														   //Set CountingLED to Dark: Counting Stops.
										SetCtrlVal(panelHandle,PANEL_StatusMessage,\
														"Calibration Data Aquired.");
									}
									W2CountFlag = 0;
								}else W2CountFlag++;
							}else W2CountFlag++;
						}
					}
					
					if(WCaliFlag ==0 && TomoFlag ==1)
					{
						TMotionDone = gpibIsMotionDone(CfgSetup[0][0],CfgSetup[0][1]);
						TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[0][2],CfgSetup[0][3]);
						TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][0],CfgSetup[1][1]);
						TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][2],CfgSetup[1][3]);
						if(TMotionDone)
						{
							SetCtrlVal(panelHandle,PANEL_MotorRunning,0);
							if(T2CountFlag > 0)
							{
								//store the data to memory: accumulate the instant coin rate.
								TomoData[0]  = TomoBasisIndex;	
								TomoData[1] += SinglesA;
								TomoData[2] += SinglesB;
								TomoData[3] += CoinAB;
								TomoData[4] += nFilteredCounts;
								
								
								if(T2CountFlag == 1)				  //only read the motor position.
																			//once at every settings.
								{
									TActPos = gpibReadActualPosition(CfgSetup[0][0],CfgSetup[0][1]);
									sprintf(Msg,"%3.2f",TActPos);
									SetCtrlVal(panelHandle,PANEL_ActPosHWPA,Msg);
									TActPos = gpibReadActualPosition(CfgSetup[0][2],CfgSetup[0][3]);
									sprintf(Msg,"%3.2f",TActPos);
									SetCtrlVal(panelHandle,PANEL_ActPosQWPA,Msg);	
									TActPos = gpibReadActualPosition(CfgSetup[1][0],CfgSetup[1][1]);
									sprintf(Msg,"%3.2f",TActPos);
									SetCtrlVal(panelHandle,PANEL_ActPosHWPB,Msg);
									TActPos = gpibReadActualPosition(CfgSetup[1][2],CfgSetup[1][3]);
									sprintf(Msg,"%3.2f",TActPos);
									SetCtrlVal(panelHandle,PANEL_ActPosQWPB,Msg);
								}
								
								
								if(T2CountFlag == cTCountingTime)
								{									
									TomoData[4] = TomoData[3]-TomoData[1]*TomoData[2]\
															*240e-9*4/cTCountingTime;								
									if (TomoData[4]<0)
										TomoData[4]=0;
									//write to file the current data	
									if (numTomoBasis == 35)
										sprintf(CharBuffer,"%d\t%d\t%d\t%d\t%d\n",\
											TomoBasisIndex,TomoData[1],TomoData[2],\
															TomoData[3],TomoData[4]);
									else
										sprintf(CharBuffer,"%d\t%d\t%d\t%d\t%d\n",\
											TomoCompleteBasis[TomoBasisIndex],\
											TomoData[1],TomoData[2],TomoData[3],TomoData[4]);
									WriteFile(TomoFileHandle,CharBuffer,strlen(CharBuffer));
									
									TomoData[1] = 0;
									TomoData[2] = 0;
									TomoData[3] = 0;
									TomoData[4] = 0; 
							
									
									if(TomoBasisIndex < numTomoBasis)
									{
										//move to next setting.
										if (numTomoBasis == 35)
											tomoSetMotorForBasis(++TomoBasisIndex);	
																		   //Basis setting from 0-35.
										else
											tomoSetMotorForBasis(\
												TomoCompleteBasis[++TomoBasisIndex]);  
																	   //Basis setting pre-sequenced.
											
										
									}else
									{
										TomoFlag = 0;								   //finish Tomo.
										TomoBasisIndex = 0;					 //Prepare for next tomo.
										CloseFile(TomoFileHandle);
										if (RawDataFileHandle != -1) CloseFile(RawDataFileHandle);
										
										CommandToRabbit[0]='R';
										CommandToRabbit[1]=0x00;			 //Send a 'Stop' Command.
										ClientTCPWrite (TcpHandle, \
														CommandToRabbit, 2, 1000);	
										SetCtrlVal(panelHandle, PANEL_CountingLED,0);	   
														   //Set CountingLED to Dark: Counting Stops.
										SetCtrlVal(panelHandle,PANEL_StatusMessage,\
														"Tomography Data Aquired.");
									}
									
									T2CountFlag = 0;
							
								}else T2CountFlag++;	
							}else T2CountFlag++;
						}else SetCtrlVal(panelHandle,PANEL_MotorRunning,1);
					}
					
					if(HOMFlag == 1)
					{
						if(gpibIsPicoDone(PicoAddr))
						{
							if(HOM2CountFlag > 0)   
									  //count the second singles, to make sure this is a full second;
							{
								if(HOM2CountFlag == HEachPosTime)
								{
									//write to file
									sprintf(CharBuffer,"%d\t%d\t%d\t%d\t%d\n",\
										HOMData[0],HOMData[1],HOMData[2],HOMData[3],\
																HOMData[3]-HOMData[4]);
									WriteFile(HOMFileHandle,CharBuffer,strlen(CharBuffer));
									HOMData[0] = PicoPos;
									HOMData[1] = 0;
									HOMData[2] = 0;
									HOMData[3] = 0;
									HOMData[4] = 0;  
									if(PicoPos < PicoScanRange*100000)
									{
																				  //move to next inc.
										PicoPos += nPicoPulseCount;
										sprintf(CharBuffer,":SOUR:PULS:COUN %d",nPicoPulseCount);
										Send (0,PicoAddr,CharBuffer,strlen(CharBuffer),2); 
									}
									else
									{
										PicoPos = 0;
										HOMFlag =0;  			   				  //finish HOM Scan.
										CloseFile(HOMFileHandle); //close the file for HOM scanning.
										if (RawDataFileHandle != -1) CloseFile(RawDataFileHandle);
										
										CommandToRabbit[0]='R';
										CommandToRabbit[1]=0x00;			//Send a 'Stop' Command.
										ClientTCPWrite (TcpHandle, \
														CommandToRabbit, 2, 1000);	
										SetCtrlVal(panelHandle, PANEL_CountingLED,0);	   		
														   //Set CountingLED to Dark: Counting Stops.
										SetCtrlVal(panelHandle,PANEL_StatusMessage,\
															"HOM Scan Data Aquired.");
									}
									HOM2CountFlag = 0;
								}
								else
								{
									// Store the data to memory
									HOMData[0] = PicoPos;
									HOMData[1] += SinglesA;
									HOMData[2] += SinglesB;
									HOMData[3] += CoinAB;
									HOMData[4] += nFilteredCounts; 
									HOM2CountFlag++;
								}
							}else HOM2CountFlag++;
						}
					}
					
					
					
					//if(CountingFlag && FidSaveFile != -1)
					//	WriteFile(FidSaveFile, tcpDataBuffer, tcpDataIndex+1);	
													  //Save to file only when counting for spectrum.
					//tcpDataIndex = 0;
					//
					CoinAB = 0;
					CoinFrmPckt = 0;
					nFilteredCounts = 0;
					
					break;
			}
			break;
	}
	LED_flag=1-LED_flag; 
	return 0;
}*/

int CVICALLBACK GetCoincidence (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	unsigned char CommandToRabbit[4]={0};
	unsigned char CharBuffer[256]={0};
	unsigned char FullPath[256]={0};
	int TmStpEn;
	int hour, min, sec, mon, day, year;
	StartTime=Timer();

	switch (event)
	{
		case EVENT_COMMIT:
			
			CountingFlag=1-CountingFlag; 									 //Set the counting Flag.
													  //Set the path of the file for saving the Data.
			GetCtrlVal(panelHandle,PANEL_MY_FOLDER,CharBuffer);				
			strncpy(FullPath,CharBuffer,256);
			
			GetSystemTime(&hour,&min,&sec);
			GetSystemDate(&mon,&day,&year);
			sprintf(Msg, "CoinTest %d;%d;%d, %d-%d-%d.txt",hour,min,sec,mon,day,year);
			strncat(CharBuffer, Msg, 256);
			CloseFile(TestFileHandle);
			if(CountingFlag)
			{
				TestFileHandle = OpenFile(CharBuffer, VAL_WRITE_ONLY, VAL_TRUNCATE, VAL_ASCII);
				sprintf(Msg, "A	B	CoinAB	CoinTimed	Received	WaitTime\n");
				WriteFile(TestFileHandle, Msg, strlen(Msg));
																	 //Open File to Save Timing Data.
				FidSaveFile=OpenFile(FullPath, VAL_WRITE_ONLY, VAL_TRUNCATE, VAL_BINARY); 		
			}
			
			GetCtrlVal(panelHandle,PANEL_MY_FILE,CharBuffer);
			strncat(FullPath,CharBuffer,256);
	
			GetCtrlVal(panelHandle,PANEL_ChkBox_TimeStamp,&TmStpEn);
			
			
			CommandToRabbit[0]='T';
			CommandToRabbit[1]=0x01 & TmStpEn;						   //Send a 'Time Stamp' Command.
			ClientTCPWrite (TcpHandle, CommandToRabbit, 2, 1000);
			
			
			CommandToRabbit[0]='R';
			CommandToRabbit[1]=0x01 & CountingFlag;						 //Send a 'Run/Stop' Command.
			ClientTCPWrite (TcpHandle, CommandToRabbit, 2, 1000);
			SetCtrlVal(panelHandle,PANEL_CountingLED,1);	    //Set LED to Bright: Counting Starts.
																		 //Get the Counting Duration.
			GetCtrlVal(panelHandle,PANEL_CountingTimeNumeric,&CountingTime);
			
			SetCtrlVal(panelHandle,PANEL_StatusMessage,"Counting Now...");
												   //Disable the PlotIt Button when getting the data.
			SetCtrlAttribute(panelHandle,PANEL_PlotButton,ATTR_DIMMED,1);
			
			break;
	}
	return 0;
}

int CVICALLBACK PlotTheSpectrum (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	unsigned char *DataSocket;
	unsigned char *ADC_overflow;
	unsigned int *ADC_A;
	unsigned int *ADC_B;
	unsigned char *Offset_Counts;
	int *A_arrive_first;
	unsigned char CharBuffer[256]={0};
	int i=0;
	int j=0;
	double t_A=0;
	double t_B=0;
	double t_delay=0;
	int nInvalidCounts=0;
	
	switch (event)
	{
		case EVENT_COMMIT:
			//Get the Calibration Values
			//fflush the data to the file
			//reads all the file data to interpret
			//plot
			for(i=0;i<20000;i++)
				CoinCountsData[i]=0;							  	//Clear the Previous Data if any.
			TotalCounts = 0;										//Clear the TotalCounts Register.
			
			if(!PlotInitialize())
				break;
           
			DataSocket=calloc(nFileLength,sizeof(char));
			ADC_overflow=calloc(nCoinCounts,sizeof(char));
			ADC_A=calloc(nCoinCounts,sizeof(int));
			ADC_B=calloc(nCoinCounts,sizeof(int));
			Offset_Counts=calloc(nCoinCounts,sizeof(char));
			A_arrive_first=calloc(nCoinCounts,sizeof(int));

			SetFilePtr(FidSaveFile, 0 , 0);
			ReadFile(FidSaveFile, DataSocket, nFileLength);
			FidSaveFile	= -1;
			
			if (DataSocket==NULL)
				break;
			
			if (!(Cal_Value[0]&&Cal_Value[1]&&Cal_Value[2]&&Cal_Value[3]))
  			{
				MessagePopup ("Warning!", "No Calibration data found. Used Default Value!");  
				Cal_Value[0]=3602;
				Cal_Value[1]=499;
				Cal_Value[2]=3589;
				Cal_Value[3]=513;
			}

			for(i=0;i<(nFileLength-7);i+=7)
 			{
				while (DataSocket[i]!=82)
				{
					if(i<nFileLength-7)
						i++;
					else
						break;
				}

  				ADC_A[j]=DataSocket[i+1]*256+DataSocket[i+2];
    			ADC_B[j]=DataSocket[i+3]*256+DataSocket[i+4];
    			Offset_Counts[j]=DataSocket[i+5];
    			A_arrive_first[j] =(DataSocket[i+6]&0x04)/4*2-1;
    			ADC_overflow[j]=(DataSocket[i+6]&0x01)||(DataSocket[i+6]&0x02);
    			j++;
			}

			for(i=0;i<j;i++)
			{
													 //Here is for error catching in log computation.
				t_A=1-1.0*(ADC_A[i]-Cal_Value[1])/(Cal_Value[0]-Cal_Value[1]);
													 			//-log(t_A)*9.47, and -log(t_B)*9.49.
				t_B=1-1.0*(ADC_B[i]-Cal_Value[3])/(Cal_Value[2]-Cal_Value[3]);		
				if(t_A>0 && t_B>0) 
				{
					t_delay=log(t_B)*9.49-log(t_A)*9.47+10*(Offset_Counts[i])*(A_arrive_first[i]);
					if(t_delay<1000 && t_delay>-1000)
						CoinCountsData[10000+(int)(t_delay*10)]++;
					else
						nInvalidCounts++;
				}
				else
					nInvalidCounts++;
			}
			for(i=0;i<20000;i++)
			{
				TimeAxis[i]=i-10000;
				TotalCounts+=CoinCountsData[i];
			}
			//IS THAT A WAY TO CLEAR THE GRAPH????? 
			SetCtrlAttribute(panelHandle,PANEL_GRAPH,ATTR_XNAME,"Arrival Time Difference(100ps)");
			SetCtrlAttribute(panelHandle,PANEL_GRAPH,ATTR_YNAME,"Coincidence Counts");
			SetCtrlAttribute(panelHandle,PANEL_GRAPH,ATTR_LABEL_TEXT,"Two Photon Time Spectrum");
			PlotXY(panelHandle,PANEL_GRAPH,TimeAxis,CoinCountsData,20000,\
					VAL_INTEGER,VAL_INTEGER,VAL_THIN_LINE,VAL_NO_POINT,VAL_SOLID,1,VAL_DK_GREEN);

			
			//Report Here???
			sprintf(CharBuffer,"%d",TotalCounts);
			SetCtrlVal(panelHandle,PANEL_TotalCountsDisp,CharBuffer); 
			SetCtrlAttribute(panelHandle,PANEL_TotalCountsDisp,ATTR_VISIBLE,1);
			GetCtrlVal(panelHandle,PANEL_CountingTimeNumeric,&CountingTime);
			sprintf(CharBuffer,"%d",TotalCounts/CountingTime); 
			SetCtrlVal(panelHandle,PANEL_CoinRateDisp,CharBuffer); 
			SetCtrlAttribute(panelHandle,PANEL_CoinRateDisp,ATTR_VISIBLE,1); 

			//Total Data, Total Counts, nInvalidCounts, Maximum
			
			free(DataSocket);
			free(ADC_overflow);
			free(ADC_A);
			free(ADC_B);
			free(Offset_Counts);
			free(A_arrive_first);
			break; 
	}
	return 0;
}

int PlotInitialize(void)
{
	//SelectFilePopup
	if(FidSaveFile== -1)
	{
		FileSelectPopup ("", "*.*", "*.*", "Select a Data File to Plot", \
			VAL_OK_BUTTON,0,0,1,0,pathName);
		FidSaveFile = OpenFile(pathName, VAL_READ_ONLY, VAL_OPEN_AS_IS, VAL_BINARY);
		if(FidSaveFile == -1)
			return 0;
		nFileLength=GetFileInfo(pathName, &nFileLength);
	}
	
	SetFilePtr(FidSaveFile, 0, 0);
	//Count the filesize 
	nCoinCounts=nFileLength/7;
	return 1;
}

int CVICALLBACK WCalibrate (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	char CommandToRabbit[16] = {0};
	int initIndexi = 0;
	int initIndexj = 0;
//	char CharBuffer[256] = {0};	 //use only if one want to time-stamp the WP calibration coincidence.
//	unsigned char FullPath[256]={0};
	
	switch (event)
	{
		case EVENT_COMMIT:
			//clear GPIB errors and set gpib0 as the Controller-In-Charge.
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			GetCtrlVal(panelHandle,PANEL_GPIBADDR_CHA,&gpibAddrA);
			GetCtrlVal(panelHandle,PANEL_GPIBAXIS_CHA,&DeviceAxisA);
			GetCtrlVal(panelHandle,PANEL_GPIBADDR_CHB,&gpibAddrB);
			GetCtrlVal(panelHandle,PANEL_GPIBAXIS_CHB,&DeviceAxisB);
			GetCtrlVal(panelHandle,PANEL_INCANG,&WAngInc);
			GetCtrlVal(panelHandle,PANEL_WEachPosTime,&cWCountingTime);
			//GetCtrlVal(panelHandle,PANEL_DELAYM,&DelayAfterMotion);
			
			WCaliFlag = 1;
			//initiate WCaliData
			for (initIndexi=0;initIndexi<2048;initIndexi++)			
				for (initIndexj=0;initIndexj<4;initIndexj++)			
					WCaliData[initIndexi][initIndexj] = 0;
			
			if(gpibMotorOn(gpibAddrA,DeviceAxisA) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor of Ch. A!");
				break;
			}
			if(gpibMotorOn(gpibAddrB,DeviceAxisB) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor of Ch. B!");
				break;
			}
			
			if(abs(WAngInc)<0.18)
			{
				MessagePopup ("Error", "Angle Increment Too Small");
				break;
			}
			
			FileSelectPopup ("", "", "", "Save the Calibration Data...", \
								VAL_SAVE_BUTTON, 0, 0, 0, 1, PathNameCali);
			CaliFileHandle = OpenFile(PathNameCali,VAL_WRITE_ONLY,VAL_TRUNCATE,VAL_ASCII);
			
			gpibSearchForHome(gpibAddrA,DeviceAxisA);							
			gpibSearchForHome(gpibAddrB,DeviceAxisB);
			
			while(1)
			{
				TMotionDone = gpibIsMotionDone(gpibAddrA,DeviceAxisA);
				TMotionDone = TMotionDone && gpibIsMotionDone(gpibAddrB,DeviceAxisB);

				if (!TMotionDone)
				{
					Delay(0.5);
					SetCtrlVal(panelHandle,PANEL_MotorRunning,1);
				}else break;
			}
			SetCtrlVal(panelHandle,PANEL_MotorRunning,0); 
			
			/*	//not time-stamping while doing the calibration.
			if (FidSaveFile == -1)
			{
				GetCtrlVal(panelHandle,PANEL_MY_FOLDER,CharBuffer);				
				strncpy(FullPath,CharBuffer,256);
				GetCtrlVal(panelHandle,PANEL_MY_FILE,CharBuffer);
				strncat(FullPath,CharBuffer,256);
				FidSaveFile=fopen(FullPath,"ab+");
			}*/
			
			CommandToRabbit[0]='R';
			CommandToRabbit[1]=0x01;										  //Send a 'Run' Command.
			ClientTCPWrite (TcpHandle, CommandToRabbit, 2, 1000);
			SetCtrlVal(panelHandle,PANEL_CountingLED,1);
			
			break;
	}
	return 0;
}

int gpibMotorOn(gpibAddr,DeviceAxis)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dMO?",DeviceAxis);
	Send (0,gpibAddr,gpibCommand,4,2);
	sprintf(gpibCommand,"0123");
	Receive (0,gpibAddr,gpibCommand,1,256);
	if(gpibCommand[0] == '1')
		return 1;
	else 	
	{
		sprintf(gpibCommand,"%dMO",DeviceAxis);
		Send (0,gpibAddr,gpibCommand,3,2);
		sprintf(gpibCommand,"%dMO?",DeviceAxis);
		Send (0,gpibAddr,gpibCommand,4,2);
		sprintf(gpibCommand,"0123");
		Receive (0,gpibAddr,gpibCommand,1,256);
		return gpibCommand[0];
	}
}

int gpibSearchForHome(int gpibAddr,int DeviceAxis)
{
	char gpibCommand[16];
					 //search only for switch postion, or include index? see manual file for details.
	sprintf(gpibCommand,"%dOR2",DeviceAxis); 
	Send (0,gpibAddr,gpibCommand,4,2); 
	return 1;
}

double gpibReadActualPosition(int gpibAddr,int DeviceAxis)  
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dTP",DeviceAxis);
	Send (0,gpibAddr,gpibCommand,3,2); 
	Receive (0,gpibAddr,gpibCommand,16,256);
	return atof(gpibCommand);  
}

int gpibWaitForMotionStop(int gpibAddr,int DeviceAxis, int DelayTime)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dWS%d",DeviceAxis,DelayTime);
	Send (0,gpibAddr,gpibCommand,7,2); 
	return 1;
}
	
int gpibMoveRelativeAngle(int gpibAddr,int DeviceAxis, float Increment)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dPR%f",DeviceAxis,Increment);
	Send (0,gpibAddr,gpibCommand,7,2); 
	return 1;
}

int gpibIsMotionDone(int gpibAddr,int DeviceAxis)  
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dMD?",DeviceAxis);
	Send (0,gpibAddr,gpibCommand,4,2); 
	sprintf(gpibCommand,"test");
	Receive (0,gpibAddr,gpibCommand,1,256);
	return atoi(gpibCommand);
}	   

int gpibMoveAbsoluteAngle(int gpibAddr,int DeviceAxis, float Angle)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dPA%4.2f",DeviceAxis,Angle);
	Send (0,gpibAddr,gpibCommand,strlen(gpibCommand),2); 
	return 1;
}

int gpibPicoReset(int gpibAddr)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"*RST");
	Send (0,gpibAddr,gpibCommand,strlen(gpibCommand),2); 
	Receive (0,gpibAddr,gpibCommand,2,256);
	if(gpibCommand[0] == 'O')		//first letter of "OK"
		return 1;				  
	else
		return 0;
}
int gpibIsPicoDone(int gpibAddr)
{	
	char gpibCommand[16];
	sprintf(gpibCommand,"*OPC?");
	Send (0,gpibAddr,gpibCommand,strlen(gpibCommand),2); 
	Receive (0,gpibAddr,gpibCommand,1,256);
	if(gpibCommand[0] == '1')
		return 1;				  
	else
		return 0;
}

int CVICALLBACK BrowseForPath (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			DirSelectPopup ("","Choose a dirctory to save coincidence data", 1, 1, pathName);
			SetCtrlVal(panelHandle,PANEL_MY_FOLDER,pathName); 
			break;
	}
	return 0;
}

int CVICALLBACK LoadCfg (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	char PathNameCfg[1024] = {0};
	int CfgFileHandle = 0;
	int FileOpenSucceed = 0;
	switch (event)
	{
		case EVENT_COMMIT:
			FileOpenSucceed = FileSelectPopup ("C:\\xingxing\\Data\\Tomo", \
				"*.cfg", "*.cfg", "Load Waveplate Configurations...", \
							VAL_LOAD_BUTTON, 0, 1, 1, 0, PathNameCfg);
			CfgFileHandle = OpenFile(PathNameCfg,VAL_READ_ONLY,VAL_OPEN_AS_IS,VAL_ASCII );
			if(FileOpenSucceed == 0)
			{
				MessagePopup ("Warning", \
					"No configuration file is loaded! Reload or I'll use the dummy data");
			}
			else
			{
				ScanFile (CfgFileHandle, "%s>%s[dt#]%16f[x]", &CfgSetup); 
				CloseFile(CfgFileHandle);
			}							 
			break;
	}
	return 0;
}

int CVICALLBACK TomoRun (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	char CommandToRabbit[16] = {0};
	int nFlag = 0;
	switch (event)
	{
		case EVENT_COMMIT:
			
			//Doing the over complete set of Tomography or not?
			GetCtrlVal(panelHandle,PANEL_ChkBox_OverCom,&nFlag);  
			if (nFlag)
				numTomoBasis = 35;
			else
				numTomoBasis = 15;
														
			//Doing Digital fitering or not?
			GetCtrlVal(panelHandle,PANEL_ChkBox_Filter,&FilterFlag); 
			

			
			if (!(Cal_Value[0]&&Cal_Value[1]&&Cal_Value[2]&&Cal_Value[3]) && FilterFlag)
  			{
				MessagePopup ("Warning!", "No Calibration data found. Used Default Value!"); 
				Cal_Value[0]=3602;
				Cal_Value[1]=499;
				Cal_Value[2]=3589;
				Cal_Value[3]=513;
			}
			
				
			
			
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			
			if(CfgSetup[0][3] == 1.1)
			{
				if(!ConfirmPopup ("Warning", "No configuration file is loaded! Use dummy Data?")) 
					break;
			}
			
				
			if(gpibMotorOn(CfgSetup[0][0],CfgSetup[0][1]) == 0)
			{
															//Turn all motors on. Check if succeed???
				MessagePopup ("Error", "Error in turning on the motor 1 of Ch. A!");
				break;
			}
			if(gpibMotorOn(CfgSetup[0][2],CfgSetup[0][3]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 2 of Ch. A!");
				break;
			}
			if(gpibMotorOn(CfgSetup[1][0],CfgSetup[1][1]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 1 of Ch. B!");
				break;
			}
			if(gpibMotorOn(CfgSetup[1][2],CfgSetup[1][3]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 2 of Ch. B!");
				break;
			}
			
			if (!FileSelectPopup ("", "", "", "Save the Tomography Data...", \
									VAL_SAVE_BUTTON, 0, 0, 1, 1, PathNameTomo)) 
				break;
			TomoFileHandle = OpenFile(PathNameTomo,VAL_WRITE_ONLY,VAL_TRUNCATE,VAL_ASCII);
			
													//If doing filtering, also take down the raw data.
			if (FilterFlag)
			{
				strncat(PathNameTomo,".raw.txt",10);
				RawDataFileHandle = OpenFile(PathNameTomo,VAL_WRITE_ONLY,VAL_TRUNCATE,VAL_ASCII);
			}
				
				
			
			//Set all motors home
			gpibSearchForHome(CfgSetup[0][0],CfgSetup[0][1]);							
			gpibSearchForHome(CfgSetup[0][2],CfgSetup[0][3]);
			gpibSearchForHome(CfgSetup[1][0],CfgSetup[1][1]);
			gpibSearchForHome(CfgSetup[1][2],CfgSetup[1][3]);
			
			while(1)
			{
				TMotionDone = gpibIsMotionDone(CfgSetup[0][0],CfgSetup[0][1]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[0][2],CfgSetup[0][3]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][0],CfgSetup[1][1]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][2],CfgSetup[1][3]);
				
				if (!TMotionDone)
				{
					Delay(0.5);
					SetCtrlVal(panelHandle,PANEL_MotorHoming,1);
				}else break;
			}
			SetCtrlVal(panelHandle,PANEL_MotorHoming,0);
			TMotionDone = 0;
																   //read each Setting counting time.
			GetCtrlVal(panelHandle,PANEL_TEachSettingTime,&cTCountingTime);	
			
			//setup the loop to cycle through every settings
			
			TomoFlag = 1;														  //set the tomoflag.
			tomoSetMotorForBasis(0);									//move to the first position.
			
			CommandToRabbit[0]='R';
			CommandToRabbit[1]=0x01;										  //Send a 'Run' Command.
			ClientTCPWrite (TcpHandle, CommandToRabbit, 2, 1000);
			SetCtrlVal(panelHandle,PANEL_CountingLED,1);

			
			break;
	}
	return 0;
}

int tomoSetMotorForBasis(int TomoBasisIndex)
{
	//move the motor to the right settings. think about a way to encode the basis
	//H: HWP Fast @0,		QWP Fast@0
	//V: HWP Fast @45,		QWP Fast@0
	//D: HWP Fast @22.5,	QWP Fast@45
	//A: HWP Fast @67.5,	QWP Fast@45
	//R: HWP Fast @67.5,	QWP Fast@0
	//L: HWP Fast @22.5,	QWP Fast@0
	
	switch (TomoBasisIndex/6)
	{
		case 0:
			//move the motor.
													//HWP @0. See definition of CfgSetup for details.
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]);   
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAH,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAH;
			break;
		case 1:
			//move the motor. +45 or -45? Or calibrated value?
												   //HWP @45. See definition of CfgSetup for details.
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]+CfgSetup[0][5]);
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]);			//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0);									 	 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAV,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAV;
			break;
		case 2:
			//move the motor.
																						 //HWP @22.5. 
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]+CfgSetup[0][5]/2);
																						   //QWP @45.
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]+CfgSetup[0][7]); 
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0);									  	 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAD,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAD;
			break;
		case 3:
			//move the motor.
			   																			 //HWP @67.5. 
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]+CfgSetup[0][5]*1.5);
																						   //QWP @45.
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]+CfgSetup[0][7]);   
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAA,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAA;
			break;
		case 5:
			//move the motor.L.				   
																						 //HWP @67.5. 
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]+CfgSetup[0][5]*1.5);
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0); 									 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAL,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAL;
			break;
		case 4:
			//move the motor.R.					
			   																			 //HWP @22.5.
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]+CfgSetup[0][5]/2);
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAR,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAR;
			break;
	}
	
	switch (TomoBasisIndex%6)
	{
		case 0:
			//move the motor.
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]);  			//HWP @0.
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]);			//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBH,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBH;
			break;
		case 1:
			//move the motor. +45 or -45? Or calibrated value?
												   //HWP @45. See definition of CfgSetup for details.
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]+CfgSetup[1][5]);
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]);			//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBV,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBV;
			break;
		case 2:
			//move the motor.
																						 //HWP @22.5.
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]+CfgSetup[1][5]/2);
			   																			   //QWP @45.
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]+CfgSetup[1][7]);
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBD,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBD;
			break;
		case 3:
			//move the motor.
			   																			 //HWP @67.5.
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]+CfgSetup[1][5]*1.5);
																						   //QWP @45.
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]+CfgSetup[1][7]);   	
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBA,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBA;
			break;
		case 5:
			//move the motor.	   L.
			 	 																		 //HWP @67.5.
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]+CfgSetup[1][5]*1.5); 
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBL,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBL;
			break;
		case 4:
			//move the motor.		 R.
																						 //HWP @22.5. 
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]+CfgSetup[1][5]/2);
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBR,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBR;
			break;
	}
	SetCtrlVal(panelHandle,PANEL_MotorRunning,1);
		
	return 1;
}

int CVICALLBACK SetBasis (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{	
	int BasisIndex = 0;
	switch (event)
	{
		case EVENT_COMMIT:
			if(CfgSetup[0][4] == 1.1)
			{
				if(!ConfirmPopup ("Warning", "No configuration file is loaded! Use dummy Data?")) 
					break;
			}
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			
			GetCtrlVal(panelHandle,PANEL_BasisIndex,&BasisIndex);	

			if(gpibMotorOn(CfgSetup[0][0],CfgSetup[0][1]) == 0)
			{
															  //Turn all motors on. Check if succeed?
				MessagePopup ("Error", "Error in turning on the motor 1 of Ch. A!");	
				break;
			}
			if(gpibMotorOn(CfgSetup[0][2],CfgSetup[0][3]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 2 of Ch. A!");
				break;
			}
			if(gpibMotorOn(CfgSetup[1][0],CfgSetup[1][1]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 1 of Ch. B!");
				break;
			}
			if(gpibMotorOn(CfgSetup[1][2],CfgSetup[1][3]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 2 of Ch. B!");
				break;
			}
			
			if (homeflag == 1)
			{
				
			gpibSearchForHome(CfgSetup[0][0],CfgSetup[0][1]);							
			gpibSearchForHome(CfgSetup[0][2],CfgSetup[0][3]);
			gpibSearchForHome(CfgSetup[1][0],CfgSetup[1][1]);
			gpibSearchForHome(CfgSetup[1][2],CfgSetup[1][3]);
			
			while(1)
			{
				TMotionDone = gpibIsMotionDone(CfgSetup[0][0],CfgSetup[0][1]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[0][2],CfgSetup[0][3]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][0],CfgSetup[1][1]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][2],CfgSetup[1][3]);
				
				if (!TMotionDone)
				{
					Delay(0.5);
					SetCtrlVal(panelHandle,PANEL_MotorHoming,1);
				}else break;
			}
			SetCtrlVal(panelHandle,PANEL_MotorHoming,0);
			homeflag = 0;
			}
			
			tomoSetMotorForBasis(BasisIndex); 

			while(1)
			{
				TMotionDone = gpibIsMotionDone(CfgSetup[0][0],CfgSetup[0][1]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[0][2],CfgSetup[0][3]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][0],CfgSetup[1][1]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][2],CfgSetup[1][3]);
				
				if (!TMotionDone)
				{
					Delay(0.5);
					SetCtrlVal(panelHandle,PANEL_MotorRunning,1);
				}else break;
			}
			
			SetCtrlVal(panelHandle,PANEL_MotorRunning,0);

			
			TMotionDone = 0;
			
			//read the actual value of the motor postion.
			TActPos = gpibReadActualPosition(CfgSetup[0][0],CfgSetup[0][1]);
			sprintf(Msg,"%3.2f",TActPos);
			SetCtrlVal(panelHandle,PANEL_ActPosHWPA,Msg);
			TActPos = gpibReadActualPosition(CfgSetup[0][2],CfgSetup[0][3]);
			sprintf(Msg,"%3.2f",TActPos);
			SetCtrlVal(panelHandle,PANEL_ActPosQWPA,Msg);	
			TActPos = gpibReadActualPosition(CfgSetup[1][0],CfgSetup[1][1]);
			sprintf(Msg,"%3.2f",TActPos);
			SetCtrlVal(panelHandle,PANEL_ActPosHWPB,Msg);
			TActPos = gpibReadActualPosition(CfgSetup[1][2],CfgSetup[1][3]);
			sprintf(Msg,"%3.2f",TActPos);
			SetCtrlVal(panelHandle,PANEL_ActPosQWPB,Msg);
			
			break;
	}
	return 0;
}

int CVICALLBACK HOMRun (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	char gpibCommand[16] = {0};
	char CommandToRabbit[2] = {0};
	switch (event)
	{
		case EVENT_COMMIT:
						//clear GPIB errors and set gpib0 as the Controller-In-Charge.
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			
			GetCtrlVal(panelHandle,PANEL_PicoPulseCount,&nPicoPulseCount);
			GetCtrlVal(panelHandle,PANEL_HEachPosTime,&HEachPosTime);
			GetCtrlVal(panelHandle,PANEL_GPIBADDR_PICO,&PicoAddr);
			GetCtrlVal(panelHandle,PANEL_PicoDir,&PicoDir);
			GetCtrlVal(panelHandle,PANEL_PicoTotalTravel,&PicoScanRange);
			
			GetCtrlVal(panelHandle,PANEL_ChkBox_Filter_HOM,&FilterFlag);
			
			if(!gpibPicoReset(PicoAddr))
			{
				MessagePopup ("Error", "Error in Resetting PicoMotor Driver!");
				break;	
			}
			
			if (!FileSelectPopup ("", "", "", "Save the HOM Scan Data...", \
										VAL_OK_BUTTON, 0, 0, 1, 1, PathNameHOM)) 
				break ;
			HOMFileHandle = OpenFile(PathNameHOM,VAL_WRITE_ONLY,VAL_TRUNCATE,VAL_ASCII);
			
						//If doing filtering, also take down the raw data.
			if (FilterFlag)
			{
				strncat(PathNameHOM,".raw.txt",10);
				RawDataFileHandle = OpenFile(PathNameHOM,VAL_WRITE_ONLY,VAL_TRUNCATE,VAL_ASCII);
			}
			
			if(PicoDir)										  //set the Picomotor movement direction.
				sprintf(gpibCommand,":SOUR:DIR OUT");
			else
				sprintf(gpibCommand,":SOUR:DIR IN");
			Send (0,PicoAddr,gpibCommand,strlen(gpibCommand),2); 
			

			CommandToRabbit[0]='R';
			CommandToRabbit[1]=0x01;										  //Send a 'Run' Command.
			ClientTCPWrite (TcpHandle, CommandToRabbit, 2, 1000);
			SetCtrlVal(panelHandle,PANEL_CountingLED,1);
			HOMFlag = 1; 

			break;
	}
	return 0;
}

int CVICALLBACK HomeAll (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			if(CfgSetup[0][4] == 1.1)
			{
				if(!ConfirmPopup ("Warning", "No configuration file is loaded! Use dummy Data?")) 
					break;
			}
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			

			if(gpibMotorOn(CfgSetup[0][0],CfgSetup[0][1]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 1 of Ch. A!");
				break;
			}
			if(gpibMotorOn(CfgSetup[0][2],CfgSetup[0][3]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 2 of Ch. A!");
				break;
			}
			if(gpibMotorOn(CfgSetup[1][0],CfgSetup[1][1]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 1 of Ch. B!");
				break;
			}
			if(gpibMotorOn(CfgSetup[1][2],CfgSetup[1][3]) == 0)
			{
				MessagePopup ("Error", "Error in turning on the motor 2 of Ch. B!");
				break;
			}
			
			gpibSearchForHome(CfgSetup[0][0],CfgSetup[0][1]);							
			gpibSearchForHome(CfgSetup[0][2],CfgSetup[0][3]);
			gpibSearchForHome(CfgSetup[1][0],CfgSetup[1][1]);
			gpibSearchForHome(CfgSetup[1][2],CfgSetup[1][3]);
			
			while(1)
			{
				TMotionDone = gpibIsMotionDone(CfgSetup[0][0],CfgSetup[0][1]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[0][2],CfgSetup[0][3]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][0],CfgSetup[1][1]);
				TMotionDone = TMotionDone && gpibIsMotionDone(CfgSetup[1][2],CfgSetup[1][3]);
				
				if (!TMotionDone)
				{
					Delay(0.5);
					SetCtrlVal(panelHandle,PANEL_MotorHoming,1);
				}else break;
			}
			SetCtrlVal(panelHandle,PANEL_MotorHoming,0);
			

			break;
	}
	return 0;
}

void CVICALLBACK QueueReadCallback (int queueHandle, unsigned int event, \
											int value, void *callbackData)
{
	char DataBuf[20], cmtMsg[256];
	int  HeartBeat, NumDataRead, qSize, nItem;
	int  SinglesA, SinglesB, CoinAB, TimedCoinAB, WaitTime;
	switch (event)
	{
		case EVENT_TSQ_QUEUE_SIZE:
			break;
		case EVENT_TSQ_ITEMS_IN_QUEUE:			 //@@remember to check how much data is in the queue.
											   //read 1 byte asap to DataBuf, and remove it from TSQ.
			
			CmtGetTSQAttribute(RawDataQueue,  ATTR_TSQ_QUEUE_SIZE, &qSize);
			CmtGetTSQAttribute(RawDataQueue,  ATTR_TSQ_ITEMS_IN_QUEUE , &nItem);

			//here already assumes to start with a full packet.
			NumDataRead = CmtReadTSQData (RawDataQueue, DataBuf, 1, 0, 0);
			if (NumDataRead < 1) 
			{
				if (NumDataRead < 0)
				{
					CmtGetErrorMessage (NumDataRead, cmtMsg);
					MessagePopup("TSQ Error!", cmtMsg);
				}
				break;
			}
			switch(DataBuf[0])
			{
				case 'H':
					GetCtrlVal(panelHandle,PANEL_HeartBeat,&HeartBeat);				
					SetCtrlVal(panelHandle,PANEL_HeartBeat,!HeartBeat);				
					break;															
					
				case 'P':
					CmtReadTSQData (RawDataQueue, DataBuf, 1, 0, 0);					
					if (NumDataRead == 1) CoinFrmPckt += DataBuf[0];
					break;
					
				case 'R':
					CmtReadTSQData (RawDataQueue, DataBuf, 6, 0, 0);					
					if (NumDataRead == 6) {;}
					//process to get coindata
					//save to display
					//save to file
					break;
					
				case 'S':
					NumDataRead = CmtReadTSQData (RawDataQueue, DataBuf, 15, 0, 0);
					if (NumDataRead == 15)
					{
					 	SinglesA=DataBuf[0]*65536+DataBuf[1]*256+DataBuf[2];
						SinglesB=DataBuf[3]*65536+DataBuf[4]*256+DataBuf[5];
						CoinAB = DataBuf[6]*65536+DataBuf[7]*256+DataBuf[8];
						TimedCoinAB = DataBuf[9]*65536+DataBuf[10]*256+DataBuf[11]; 
						WaitTime = DataBuf[12]*65536+DataBuf[13]*256+DataBuf[14]; 
					
						sprintf(Msg,"%d",SinglesA);
						SetCtrlVal(panelHandle,PANEL_TXTSINGLESA,Msg);  
						sprintf(Msg,"%d",SinglesB);
						SetCtrlVal(panelHandle,PANEL_TXTSINGLESB,Msg);
						sprintf(Msg,"%d",CoinAB);
						SetCtrlVal(panelHandle,PANEL_nCoinRate,Msg);
						sprintf(Msg,"%d",TimedCoinAB);    
						SetCtrlVal(panelHandle,PANEL_TXTTimedRate,Msg);
						sprintf(Msg,"%d",CoinFrmPckt);
						SetCtrlVal(panelHandle,PANEL_TXTCoinFrmPckt,Msg);
		
						sprintf(Msg, "%d	%d	%d	%d	%d	%d\n", SinglesA, SinglesB, \
							CoinAB, TimedCoinAB, CoinFrmPckt, WaitTime);
						WriteFile(TestFileHandle, Msg, strlen(Msg));
					}
					break;
				default: 
					break;
			}	
			break;
		case EVENT_TSQ_QUEUE_SPACE_FREE:
			break;
	}
}

/***************************** Thread Function for Data Acquisition *******************************/
/*IMPORTANT: to avoid access confliction, only write to protected variables that may be shared.
  TcpHandle is not protected here, since the ConnectionTOTCPServer is the only function that writes
  to it. Report errors if the main thread trying to access to a un-initialized TcpHandle.*/

int CVICALLBACK DataPcsThreadFunction (void *functionData)
{
	int TcpConnected = 0;
	while(GuiRunning)							//keep the threading going as long as Gui is running.
	{
																//keep trying reconnecting to Rabbit.
		if (!TcpConnected)
			while(ConnectToTCPServer (&TcpHandle, RABBIT_PORT, \
								RABBIT_IP, tcpPhotonFinishClientCB, NULL, 3000)<0) 
			{
				SetCtrlVal(panelHandle,PANEL_LedConnected, 0);				 //showing no connection.
				Delay(1); 
				TcpConnected = 0;
			}
		SetCtrlVal(panelHandle,PANEL_LedConnected, 1);	 						//showing connection.
		TcpConnected = 1;
		ProcessTCPEvents();						 //enable the callback function to handle tcp events.
	}
	return 0;
}

/********************** TCP Callback Function, in Data Acq Threads *******************************/
/*TCP client to read packets from Rabbit. write data to TSQ, which will then be processed in the 
main thread.*/
int CVICALLBACK tcpPhotonFinishClientCB(unsigned handle, int xType, int errCode, void *callbackData)
{
	char tcpData[1000];
	int NumDataRead;													  //number of data been read.
		
	switch(xType)
	{
		case TCP_DISCONNECT:
			SetCtrlVal(panelHandle,PANEL_LedConnected, 0);					 //showing no connection.
			TcpConnected = 0;
			break;
		
		case TCP_DATAREADY:
			NumDataRead = ClientTCPRead (TcpHandle, tcpData, 1000, 1000);
			CmtWriteTSQData (RawDataQueue, tcpData, NumDataRead, 50, NULL);    //Write to TSQ buffer.
			break;
	}
	return 0;
}

