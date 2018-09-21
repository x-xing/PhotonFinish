#include "asynctmr.h"
#include <udpsupp.h>
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

#define RABBIT_IP			"192.168.1.123"
#define RABBIT_PORT			37829
#define DATA_BUFFER_SIZE    1000000
#define MAX_COIN_IN_PKT		200
#define MAX_PKT_SIZE		1460								 //equals to MAX_COIN_IN_PKT * 7 + 2.
#define WR_FILE_BUFF_SIZE	20000
#define TEMP_FILE_PATH_RAW "C:\\photofinish temporary data.raw"
#define TEMP_FILE_PATH_TXT "C:\\photofinish temporary data.txt"
#define tau_a	9490											//Calibrated charging constant in ps.
#define tau_b	9470
#define MAX_ADC_TIME	50000												//SPCM pulse width in ps.
#define GUIVERSION		"2.1.2"
#define CmFpgaVERSION	"1.2.5"											   //compatible FPGA version.
#define CmRbVERSION		"1.2.4"											 //compatible Rabbit version.
#define chkGpibStatus	if (ibsta & 0x8000) {sprintf (Msg, \
		"Error in sending command to GPIB, error code: %d. Retry for basis (if tomo)?", iberr);\
		if (ConfirmPopup ("Error in GPIB", Msg)) {TomoBasisIndex--;return 1;} else return 0; }

#define PLOT_REFRESH 	5
#define TimeAxisInPs	3000
		
/************************************ Function Prototypes *******************************************/

int CVICALLBACK udpPhotoFinishCB (unsigned handle, int xType, int errCode, void *callbackData);
int CVICALLBACK DataWrThrFunc (void *functionData); 
int DataRdThrFunc (void); 
int Initialize(void);
int PcsPPacket (int* RvdCoinAB, int* ArvlTmBuf, int* ArvlTmBufPtrPtr);
int ProcessArrivalTime (unsigned char * databuf, int *ArrivalTime);
int gpibMotorOn (int gpibAddr,int DeviceAxis);
int gpibSearchForHome (int gpibAddr,int DeviceAxis);
double gpibReadActualPosition (int gpibAddr,int DeviceAxis);
int gpibWaitForMotionStop (int gpibAddr,int DeviceAxis,int DelayTime);
int gpibMoveRelativeAngle (int gpibAddr,int DeviceAxis,float Increment);
int gpibMoveAbsoluteAngle (int gpibAddr,int DeviceAxis, float Angle);
int gpibIsMotionDone (int gpibAddr,int DeviceAxis);  
int tomoSetMotorForBasis (int TomoBasisIndex);
int gpibIsPicoDone (int gpibAddr);
void HomeAllMotors (void);
int  IsTomoMotorDone (void);

/************************************** Global Variables *******************************************/
/*Important: this program is multi-thread, and most of the global variables are not protected. But 
the data aquisition thread only changes a few varibles that are not changed by the main thread. */

static int panelHandle, HomPopHandle, WpCalPopHandle, GraphHandle;
int WpCaliFlag, CaliFlag, HOMFlag, PlotFlag, PlotRefresh;				 			  //global flags.
int TomoFlag, HomedOnce, WaitTomoGpib, WaitGpib, HomingFlag;
int CountingFlag, ThisRunDone, SaveDataFlag;				  			             
int udpConnected, udpRetry, udpTimeOut, CountingTime;
int LastLEDA, LastLEDB;
unsigned char RawBuf[DATA_BUFFER_SIZE];		  	  //raw data buffer, for writing and reading threads.
int WrPtr, RdPtr, RollOver, EndOfRawBufPtr;					 //write and read pointer for RawdataBuf.
int WrPtrLock, RdPtrLock, RollOverLock;   	 //protection from writting confliction by multi threads.
unsigned int udpChannel;
int TestFileHandle, FidSaveFile, FuncFileHandle;					 	  //File handle to save data.
char Msg[1024], RawPath[1024], FilePath[1024];						  //String for output formatting.
int Cal_Value[4]={3602, 499, 3589, 513}; 	//ADC calibration values.0-1: A high-low,2-3: B high-low.
int hour, min, sec, mon, day, year; 
int ArTmA[4000], ArTmB[4000];
unsigned int CoinSpectrum[TimeAxisInPs] = {0}; 
int FuncData[4] = {0};					  	   //Accumulating the counts of singles A, B and CoinsAB.
int PicoAddr, PicoPos, PicoDir, nPicoPulseCount;							  //PicoMotor parameters.
double PicoScanRange;										//the range over which HOM data is taken.

int gpibAddrA, gpibAddrB, DeviceAxisA, DeviceAxisB, WScanIndex;			 //For waveplate Calibration.
float WAngInc, WActPosA, WActPosB;

int TomoBasisIndex, numTomoBasis;			  //Basis index from 0-35 or 0-16; num of basis 36 or 16.
int TomoCompleteBasis[16] = {0, 1, 7, 6, 24, 25, 13, 12, 16, 14, 26, 2, 8, 11, 5, 29}; 
char BasisString[36][3];
 
double CfgSetup[2][8] = {7, 1, 7, 2, 9.999, 9.999, 9.999, 9.999,\
						 4, 1, 4, 2, 9.999, 9.999, 9.999, 9.999};	   //Configuration setup, 
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
 
int TotalCounts;


/*************************************** Main function *********************************************/
int main (int argc, char *argv[])
{
	int DataWrThrFuncID;
	
	if (InitCVIRTE (0, argv, 0) == 0)
		return -1;	/* out of memory */
	if ((panelHandle = LoadPanel (0, "PhotonFinish_tomo.uir", PANEL)) < 0)
		return -1;
	if ((HomPopHandle = LoadPanel(panelHandle, "PhotonFinish_tomo.uir", pnlHomPop)) < 0)
		 return -1;
	if ((WpCalPopHandle = LoadPanel(panelHandle, "PhotonFinish_tomo.uir", pnlWpCal)) < 0)
		 return -1;
	if ((GraphHandle  = LoadPanel(panelHandle, "PhotonFinish_tomo.uir", pnlGraph)) < 0)
		 return -1;
	
	Initialize();								 //Initialize the write and read pointers and flags.
	
	CmtNewLock( NULL, 0, &RollOverLock);				     //Create Locks for the global variables
	CmtNewLock( NULL, 0, &RdPtrLock);					  //that will be changed with multi threads.
	CmtScheduleThreadPoolFunction (DEFAULT_THREAD_POOL_HANDLE, \
		DataWrThrFunc, NULL, &DataWrThrFuncID);			   //thread function to write to data buffer.
	//NewAsyncTimer(1, -1, 1, RdTimerTick, 0);

	DisplayPanel (panelHandle);
	RunUserInterface ();
	CmtWaitForThreadPoolFunctionCompletion(DEFAULT_THREAD_POOL_HANDLE, \
		DataWrThrFuncID, OPT_TP_PROCESS_EVENTS_WHILE_WAITING);		 //Wait for data to be processed.
	CmtReleaseThreadPoolFunctionID(DEFAULT_THREAD_POOL_HANDLE, DataWrThrFuncID);
	CmtDiscardLock (RollOverLock);										 //Release the lock resource.
	CmtDiscardLock (RdPtrLock);
	//DiscardAsyncTimer(-1);
	
	if (TestFileHandle != -1) {
		CloseFile(TestFileHandle);
		CloseFile(FidSaveFile);
	}
	
	DiscardPanel (panelHandle);
	DisposeUDPChannel (udpChannel);
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
	switch (event)
	{
		case EVENT_COMMIT:
			CaliFlag = 1;
			sprintf(Msg, "%s", "C 34C 34C");
			Msg[1] = 0x03;												 //Calibration on high level.
			Msg[5] = 0x01;												  //Calibration on low level.
			Msg[9] = 0x00;														   //Calibration off.
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg, 	 2);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+2, 1);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+3, 1);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+4, 2);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+6, 1);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+7, 1);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+8, 2);
			break;
	}
	return 0;
}   


int CVICALLBACK GetCoincidence (int panel, int control, int event, \
		void *callbackData, int eventData1, int eventData2)
{
	unsigned char CommandToRabbit[5];
	int AdcA, AdcB, temp, TmStpEn;
	
	switch (event)
	{
		case EVENT_COMMIT:
			
			GetCtrlVal(panelHandle,PANEL_ChkBox_TimeStamp,&TmStpEn);
			if(!CountingFlag)						  //Set the path of the file for saving the Data.
			{
				CopyString (Msg, 0, TEMP_FILE_PATH_RAW, 0, -1);
				FidSaveFile = OpenFile(Msg, VAL_WRITE_ONLY, VAL_TRUNCATE, VAL_ASCII);
				CopyString (Msg, 0, TEMP_FILE_PATH_TXT, 0, -1);
				TestFileHandle = OpenFile(Msg, VAL_WRITE_ONLY, VAL_TRUNCATE, VAL_ASCII);
				sprintf(Msg, "A	B	CoinAB	CoinTimed	Received	WaitTime\n");
				WriteFile(TestFileHandle, Msg, strlen(Msg));
				
				CommandToRabbit[1]=0x01;									 	  //Send a 'Run' Command.
				SetCtrlVal(panelHandle,PANEL_CountingLED,1);  							//Set LED bright.
				SetCtrlVal(panelHandle,PANEL_StatusMessage,"Counting started...");
				SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_LABEL_TEXT ,"Stop Counting");
				SetCtrlAttribute(panelHandle,PANEL_btnCali,  ATTR_DIMMED,1);
				SetCtrlAttribute(panelHandle,PANEL_btnWpCali,ATTR_DIMMED,1);
				SetCtrlAttribute(panelHandle,PANEL_btnTomo,  ATTR_DIMMED,1);
				SetCtrlAttribute(panelHandle,PANEL_btnHOM,   ATTR_DIMMED,1);
				for (temp = 0; temp< TimeAxisInPs; temp++)
					CoinSpectrum[temp] = 0;
				
				if (TmStpEn){									//create a lookup table for arrival time. 
					for (AdcA = 0; AdcA < 4000; AdcA ++) {			    //initialize. for error catching. 
						ArTmA[AdcA] = 99999;                                                              
						ArTmB[AdcA] = 99999;                                                              
					}                                                                                     
					for (AdcA = Cal_Value[1]; AdcA < Cal_Value[0]; AdcA ++) {                             
						temp = (int) (- tau_a*log(1.0*(Cal_Value[0]-AdcA)/(Cal_Value[0]-Cal_Value[1])));  
						if (temp < MAX_ADC_TIME)	ArTmA[AdcA] = temp;
						}                                                                                     
					for (AdcB = Cal_Value[3]; AdcB < Cal_Value[2]; AdcB ++) {                             
						temp = (int) (- tau_b*log(1.0*(Cal_Value[2]-AdcB)/(Cal_Value[2]-Cal_Value[3])));  
						if (temp < MAX_ADC_TIME) 	ArTmB[AdcB] = temp;
					}                                                                                     
				}
			}
			else {
				SaveDataFlag = (GenericMessagePopup ("Save Data?", \
					"Save aquired data and to files: NAME + hh;mm;ss, mm-dd-yyyy.raw  (.txt)", "OK",\
					   "Cancel", 0, RawPath, 256, 0, VAL_GENERIC_POPUP_INPUT_STRING, 1, 2))==1; 
				if (SaveDataFlag){
					
					//Save the data to the temporary file first!@@
					GetSystemTime(&hour,&min,&sec);
					GetSystemDate(&mon,&day,&year);
					GetCtrlVal(panelHandle,PANEL_MY_FOLDER,Msg);	   //path of folder to save to files.
					CopyString (Msg, StringLength (Msg), "\\", 0, -1);
					CopyString (Msg, StringLength (Msg), RawPath, 0, -1);
					sprintf(RawPath, " %d;%d;%d, %d-%d-%d",hour,min,sec,mon,day,year);
					CopyString (Msg, StringLength (Msg), RawPath, 0, -1);
					CopyString (Msg, StringLength (Msg), ".raw", 0, -1);
					CopyFile (TEMP_FILE_PATH_RAW, Msg);
					CopyString (Msg, StringLength (Msg)-4, ".txt", 0, -1);
					CopyFile (TEMP_FILE_PATH_TXT, Msg);
				}
				CommandToRabbit[1]=0x00;									 	 //Send a 'Stop' Command.
				SetCtrlVal(panelHandle,PANEL_CountingLED,0);  							  //Set LED Dark.
				SetCtrlVal(panelHandle,PANEL_StatusMessage,"Counting stopped.");
				SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_LABEL_TEXT ,"Start Counting");
				SetCtrlAttribute(panelHandle,PANEL_btnCali,  ATTR_DIMMED,0);
				SetCtrlAttribute(panelHandle,PANEL_btnWpCali,ATTR_DIMMED,0);
				SetCtrlAttribute(panelHandle,PANEL_btnTomo,  ATTR_DIMMED,0);
				SetCtrlAttribute(panelHandle,PANEL_btnHOM,   ATTR_DIMMED,0);
			}
		
			GetCtrlVal(panelHandle,PANEL_numCountingTime,&CountingTime);
			
			CommandToRabbit[0]='R';
			CommandToRabbit[2]= CountingTime / 256;					 	 	 //Set the counting time.
			CommandToRabbit[3]= CountingTime % 256;					 	 
			CommandToRabbit[4]=0x01 & TmStpEn;						   	  //Arrival time measurement?
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, CommandToRabbit, 5);
			CountingFlag = !CountingFlag;
			
			break;
	}
	return 0;
}

int CVICALLBACK PlotTheSpectrum (int panel, int control, int event,\
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			InstallPopup(GraphHandle);
			PlotFlag = 1;
			
			//IS THAT A WAY TO CLEAR THE GRAPH????? 
			SetCtrlAttribute(GraphHandle,pnlGraph_GRAPH,ATTR_XNAME,"Arrival Time Difference(0.1ns)");
			SetCtrlAttribute(GraphHandle,pnlGraph_GRAPH,ATTR_YNAME,"Coincidence Counts");
			SetCtrlAttribute(GraphHandle,pnlGraph_GRAPH,ATTR_LABEL_TEXT,"Two Photon Time Spectrum");
			SetAxisScalingMode (GraphHandle, pnlGraph_GRAPH, VAL_LEFT_YAXIS , VAL_AUTOSCALE, 0, 100);
			DeleteGraphPlot(GraphHandle, pnlGraph_GRAPH, -1, 1);
			
			break; 
	}
	return 0;
}


int CVICALLBACK WCalibrate (int panel, int control, int event, \
		void *callbackData, int eventData1, int eventData2)
{
	switch (event){
		case EVENT_COMMIT:
			InstallPopup(WpCalPopHandle);
		break;
	}
	return 0;
}

int gpibMotorOn(gpibAddr,DeviceAxis)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dMO",DeviceAxis);
	Send (0,gpibAddr,gpibCommand,3,2);
	chkGpibStatus;
	return 1;
}

int gpibSearchForHome(int gpibAddr,int DeviceAxis)
{
	char gpibCommand[16];
					 //search only for switch postion, or include index? see manual file for details.
	sprintf(gpibCommand,"%dOR2",DeviceAxis); 
	Send (0,gpibAddr,gpibCommand,4,2);
	chkGpibStatus;
	return 1;
}

double gpibReadActualPosition(int gpibAddr,int DeviceAxis)  
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dTP",DeviceAxis);
	Send (0,gpibAddr,gpibCommand,3,2);
	Receive (0,gpibAddr,gpibCommand,16,256);
	chkGpibStatus;
	return atof(gpibCommand);  
}

int gpibWaitForMotionStop(int gpibAddr,int DeviceAxis, int DelayTime)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dWS%d",DeviceAxis,DelayTime);
	Send (0,gpibAddr,gpibCommand,7,2); 
	chkGpibStatus;
	return 1;
}
	
int gpibMoveRelativeAngle(int gpibAddr,int DeviceAxis, float Increment)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dPR%f",DeviceAxis,Increment);
	Send (0,gpibAddr,gpibCommand,7,2); 
	chkGpibStatus;
	return 1;
}

int gpibIsMotionDone(int gpibAddr,int DeviceAxis)  
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dMD?",DeviceAxis);
	Send (0,gpibAddr,gpibCommand,4,2); 
	sprintf(gpibCommand,"test");
	Receive (0,gpibAddr,gpibCommand,1,256);
	chkGpibStatus;
	return atoi(gpibCommand);
}	   

int gpibMoveAbsoluteAngle(int gpibAddr,int DeviceAxis, float Angle)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"%dPA%4.2f",DeviceAxis,Angle);
	Send (0,gpibAddr,gpibCommand,strlen(gpibCommand),2);
	chkGpibStatus;
	return 1;
}

int gpibPicoReset(int gpibAddr)
{
	char gpibCommand[16];
	sprintf(gpibCommand,"*RST");
	Send (0,gpibAddr,gpibCommand,strlen(gpibCommand),2); 
	Receive (0,gpibAddr,gpibCommand,2,256);
	chkGpibStatus;
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
	chkGpibStatus;
	if(gpibCommand[0] == '1')
		return 1;				  
	else
		return 0;
}

int CVICALLBACK BrowseForPath (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event) {
		case EVENT_COMMIT:
			DirSelectPopup ("","Choose a dirctory to save coincidence data", 1, 1, Msg);
			SetCtrlVal(panelHandle,PANEL_MY_FOLDER, Msg); 
			break;
	}
	return 0;
}

int CVICALLBACK LoadCfg (int panel, int control, int event,\
		  void *callbackData, int eventData1, int eventData2)
{
	int CfgFileHandle;
	int FileOpenSucceed;
	switch (event)
	{
		case EVENT_COMMIT:
			FileOpenSucceed = FileSelectPopup ("C:\\xingxing\\Data\\Tomo", \
				"*.cfg", "*.cfg", "Load Waveplate Configurations...", \
							VAL_LOAD_BUTTON, 0, 1, 1, 0, Msg);
			CfgFileHandle = OpenFile(Msg, VAL_READ_ONLY,VAL_OPEN_AS_IS,VAL_ASCII );
			if(FileOpenSucceed == 1) {
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
	int nFlag = 0;
	switch (event) {
		case EVENT_COMMIT:
			//Doing the over complete set of Tomography or not?
			GetCtrlVal(panelHandle,PANEL_ChkBox_OverCom,&nFlag);  
			if (nFlag)
				numTomoBasis = 35;
			else
				numTomoBasis = 15;
														
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			
			if(CfgSetup[0][4] == 9.999) {
				MessagePopup ("Error", "No configuration file is loaded!"); 
				break;
			}
			
			if(!gpibMotorOn(CfgSetup[0][0],CfgSetup[0][1])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[0][0],CfgSetup[0][1]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[0][2],CfgSetup[0][3])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[0][2],CfgSetup[0][3]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[1][0],CfgSetup[1][1])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[1][0],CfgSetup[1][1]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[1][2],CfgSetup[1][3])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[1][2],CfgSetup[1][3]);
				MessagePopup ("Error", Msg);
				break;
			}
			
			if (!FileSelectPopup ("", "", "", "Save the Tomography Data...", \
									VAL_SAVE_BUTTON, 0, 0, 1, 1, FilePath)) 
				break;
			FuncFileHandle = OpenFile(FilePath,VAL_WRITE_ONLY,VAL_TRUNCATE,VAL_ASCII);
																   //read each Setting counting time.
			GetCtrlVal(panelHandle,PANEL_numTmPerSetting,&CountingTime);	
			
			SetCtrlVal(panelHandle,PANEL_StatusMessage,"Tomography started...");
			SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnWpCali,  ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnHOM,     ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnCali,     ATTR_DIMMED,1);

			HomeAllMotors();
			HomingFlag   = 1;
			HomedOnce	 = 1;
			WaitTomoGpib = 1;
			TomoFlag 	 = 1;												  //set the tomoflag.
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
	//L: HWP Fast @67.5,	QWP Fast@0
	//R: HWP Fast @22.5,	QWP Fast@0
	
	switch (TomoBasisIndex/6)
	{
		case 0:
			//move the motor.
													//HWP @0. See definition of CfgSetup for details.
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]);  
			Delay (0.2);
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
			Delay (0.2);
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
			Delay (0.2);																   //QWP @45.
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
			Delay (0.2);																   //QWP @45.
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]+CfgSetup[0][7]);   
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAA,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAA;
			break;
		case 4:
			//move the motor.R.				   
																						 //HWP @22.5. 
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]+CfgSetup[0][5]*0.5);
			Delay (0.2);
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0); 									 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAR,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAR;
			break;
		case 5:
			//move the motor.L.					
			   																			 //HWP @67.5.
			gpibMoveAbsoluteAngle(CfgSetup[0][0],CfgSetup[0][1], CfgSetup[0][4]+CfgSetup[0][5]*1.5);
			Delay (0.2);
			gpibMoveAbsoluteAngle(CfgSetup[0][2],CfgSetup[0][3], CfgSetup[0][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDA,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDAL,1);								   //Lightup the one.
			LastLEDA = PANEL_LEDAL;
			break;
	}
	Delay (0.2);
	switch (TomoBasisIndex%6)
	{
		case 0:
			//move the motor.
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]);  			//HWP @0.
			Delay (0.2);
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
			Delay (0.2);
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
			Delay (0.2);																   //QWP @45.
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
			Delay (0.2);																   //QWP @45.
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]+CfgSetup[1][7]);   	
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBA,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBA;
			break;
		case 4:
			//move the motor.	   R.
			 	 																		 //HWP @22.5.
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]+CfgSetup[1][5]*0.5); 
			Delay (0.2);
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBR,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBR;
			break;
		case 5:
			//move the motor.		 L.
																						 //HWP @67.5. 
			gpibMoveAbsoluteAngle(CfgSetup[1][0],CfgSetup[1][1], CfgSetup[1][4]+CfgSetup[1][5]*1.5);
			Delay (0.2);
			gpibMoveAbsoluteAngle(CfgSetup[1][2],CfgSetup[1][3], CfgSetup[1][6]);   		//QWP @0.
			
			//set the display LED to indicate the procedure.		
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,PANEL_LEDBL,1);								   //Lightup the one.
			LastLEDB = PANEL_LEDBL;
			break;
	}
	SetCtrlVal(panelHandle,PANEL_MotorRunning,1);
	return 1;
}

int CVICALLBACK SetBasis (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{	
	int BasisIndex;
	
	switch (event)
	{
		case EVENT_COMMIT:
			if(CfgSetup[0][4] == 9.999) {
				MessagePopup ("Error", "No configuration file is loaded!"); 
				break;
			}
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			
			GetCtrlVal(panelHandle,PANEL_BasisIndex,&BasisIndex);	

			if(!gpibMotorOn(CfgSetup[0][0],CfgSetup[0][1])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[0][0],CfgSetup[0][1]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[0][2],CfgSetup[0][3])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[0][2],CfgSetup[0][3]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[1][0],CfgSetup[1][1])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[1][0],CfgSetup[1][1]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[1][2],CfgSetup[1][3])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[1][2],CfgSetup[1][3]);
				MessagePopup ("Error", Msg);
				break;
			}

			if (!HomedOnce) {
				MessagePopup ("Error", "Motors have to be homed before set to basis");
				break;
			}
			tomoSetMotorForBasis(BasisIndex); 
			WaitGpib	= 1;
			break;
	}
	return 0;
}

int CVICALLBACK HOMRun (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			InstallPopup(HomPopHandle);

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
			if(CfgSetup[0][4] == 9.999) {
				MessagePopup ("Error", "No configuration file is loaded!");
				break;
			}
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			
			if(!gpibMotorOn(CfgSetup[0][0],CfgSetup[0][1])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[0][0],CfgSetup[0][1]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[0][2],CfgSetup[0][3])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[0][2],CfgSetup[0][3]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[1][0],CfgSetup[1][1])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[1][0],CfgSetup[1][1]);
				MessagePopup ("Error", Msg);
				break;
			}
			if(!gpibMotorOn(CfgSetup[1][2],CfgSetup[1][3])) {
				sprintf(Msg, "Error turning motor on! GPIB: %d Axis %d.",CfgSetup[1][2],CfgSetup[1][3]);
				MessagePopup ("Error", Msg);
				break;
			}
			
			HomeAllMotors();
			//@@Turn off all the leds.
			SetCtrlVal(panelHandle,LastLEDA,0);										 //Dark last LED.
			SetCtrlVal(panelHandle,LastLEDB,0);										 //Dark last LED.
			HomedOnce	= 1;
			WaitGpib	= 1;
			
			break;
	}
	return 0;
}


/***************************** Thread Function for Data Acquisition *******************************/
/*IMPORTANT: to avoid access confliction, only write to protected variables that may be shared.   */

int CVICALLBACK DataWrThrFunc (void *functionData)
{
	CmtSetCurrentThreadPriority (THREAD_PRIORITY_NORMAL);	 		  //set the thread priority to 2.
	
	CreateUDPChannelConfig (RABBIT_PORT, UDP_ANY_ADDRESS, 1, udpPhotoFinishCB, NULL, &udpChannel);
	udpTimeOut = Timer();
	
	return 0;
}

/*********************** UDP Callback Function, in Data Acq Threads *******************************/
/*UDP client to read packets from Rabbit. write data to TSQ, which will then be processed in the 
main thread.*/
int CVICALLBACK udpPhotoFinishCB(unsigned handle, int xType, int errCode, void *callbackData)
{
	int NumDataRead;													  //number of data been read.
	
	udpTimeOut = Timer();
	switch(xType) { 
		case UDP_DATAREADY:										   		  //Read 1000 byte to RawBuf.
			if (!RollOver) {			  //Normal case, write and read pointer are on the same page.
				if (WrPtr + MAX_PKT_SIZE < DATA_BUFFER_SIZE) { 			//Not near the end of RawBuf.
					NumDataRead = UDPRead (udpChannel, RawBuf + WrPtr, MAX_PKT_SIZE, 100, 0, 0);
					WrPtr += NumDataRead;
				}
				else if (RdPtr > MAX_PKT_SIZE)  {    					  //almost the end, rollover.
					NumDataRead = UDPRead (udpChannel, RawBuf, MAX_PKT_SIZE, 100, 0, 0);
					EndOfRawBufPtr = WrPtr -MAX_PKT_SIZE;   //record the end of RawBuf, for Read out. 
					WrPtr = NumDataRead;					//start to write the beginning of RawBuf.
					RollOver = 1;
				} 
				else {
					MessagePopup ("Warning!", "About to over write data that is not read yet.");
					WrPtr = 0;				 //discard all the data from the beginning of RawDataBuf.
				}
			}
			else if (WrPtr < (RdPtr - MAX_PKT_SIZE)) {			  //Not to write beyond read pointer.
				NumDataRead = UDPRead (udpChannel, RawBuf + WrPtr, MAX_PKT_SIZE, 100, 0, 0);
				WrPtr += NumDataRead;
			}
			else {
				MessagePopup ("Warning!", "About to over write data that is not read yet.");
				WrPtr = 0;					 //discard all the data from the beginning of RawDataBuf.
			}
			break;
	}
	return 0;
}


/****************************** Thread Function for Data Process***********************************/
/*Thread function to process the data in the buffer. After the process, display it on GUI screen. 
It uses flags from the main threads to complete functions like HOM scan, Tomo, Calibration etc. It
also gives flags to the main threads to notify a completion of the current run.*/

//int CVICALLBACK DataRdThrFunc (void *functionData)
int DataRdThrFunc (void)
{
	int  SinglesA, SinglesB, CoinAB, TimedCoinAB, WaitTime;	   			 //variables set by S packet.
	
	static int  HeartBeat = 0;  
	static int  RvdCoinAB = 0;			//variables set by P packet. Numbers of recoverd coincidence.
	static int  ArvlTmBuf[WR_FILE_BUFF_SIZE];
	static int  ArvlTmBufPtr   = 0;
	static int  ARVL_PTR_MAX = WR_FILE_BUFF_SIZE - MAX_COIN_IN_PKT*sizeof(int);
	static int  CaliHigh = 1;
	
	
	while (!RollOver && (RdPtr < WrPtr) || RollOver && (RdPtr < EndOfRawBufPtr)) {
		//do something with the processed data.
		switch (RawBuf[RdPtr++]) {										  		  //process the byte.
			case 'H':
				HeartBeat = !HeartBeat;
				SetCtrlVal(panelHandle,PANEL_HeartBeat,HeartBeat);
				udpConnected = 1;					 //indicate connection once received a heartbeat.
				break;															
			case 'P':
				if (ArvlTmBufPtr < ARVL_PTR_MAX)		//check if near the end of ArvlTmBuf.
					PcsPPacket (&RvdCoinAB, ArvlTmBuf, &ArvlTmBufPtr);   //RdPtr increment done here.
																	   //ArvlTmBufPtr also increases.
				else {
					ArrayToFile (TEMP_FILE_PATH_RAW, ArvlTmBuf, VAL_INTEGER, \
							ArvlTmBufPtr+1, 1, 0, VAL_GROUPS_AS_COLUMNS,  \
									VAL_SEP_BY_TAB, 0, VAL_ASCII , VAL_APPEND);
					ArvlTmBufPtr = 0;
					PcsPPacket (&RvdCoinAB, ArvlTmBuf, &ArvlTmBufPtr);
				}
				break;
			case 'R':
				if (ProcessArrivalTime (RawBuf+RdPtr, ArvlTmBuf+ArvlTmBufPtr))
					ArvlTmBufPtr++;
				RdPtr += 6;
				break;
			case '3':
				if (CaliFlag) {
					if (CaliHigh) {
						Cal_Value[0] = *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++); 
						sprintf(Msg,"%d",Cal_Value[0]);
						SetCtrlVal(panelHandle,PANEL_A_High,Msg);
					}
					else {
						Cal_Value[1] = *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++); 
						sprintf(Msg,"%d",Cal_Value[1]);
						SetCtrlVal(panelHandle,PANEL_A_Low,Msg);  
					}
				}
				break;
			case '4':
				if (CaliFlag) {
					if (CaliHigh) {
						Cal_Value[2] = *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++); 
						sprintf(Msg,"%d",Cal_Value[2]);
						SetCtrlVal(panelHandle,PANEL_B_High,Msg);
						CaliHigh = 0;
					}
					else {
						Cal_Value[3] = *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++); 
						sprintf(Msg,"%d",Cal_Value[3]);
						SetCtrlVal(panelHandle,PANEL_B_Low,Msg);  
						CaliHigh = 1;
						CaliFlag = 0;		 //take out of Calibration mode.  Arrival time sensitive.
					}
				}
				break;
			case 'S':
				SinglesA 	= *(RawBuf+RdPtr++)*65536 + *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++);
				SinglesB 	= *(RawBuf+RdPtr++)*65536 + *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++);
				CoinAB   	= *(RawBuf+RdPtr++)*65536 + *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++);
				TimedCoinAB	= *(RawBuf+RdPtr++)*65536 + *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++); 
				WaitTime   	= *(RawBuf+RdPtr++)*65536 + *(RawBuf+RdPtr++)*256 + *(RawBuf+RdPtr++); 
				
				FuncData[1] += SinglesA;
				FuncData[2] += SinglesB;
				FuncData[3] += CoinAB;
				
				sprintf(Msg,"%d",SinglesA);
				SetCtrlVal(panelHandle,PANEL_TXTSINGLESA,Msg);  
				sprintf(Msg,"%d",SinglesB);
				SetCtrlVal(panelHandle,PANEL_TXTSINGLESB,Msg);
				sprintf(Msg,"%d",CoinAB);
				SetCtrlVal(panelHandle,PANEL_nCoinRate,Msg);
				sprintf(Msg,"%d",TimedCoinAB);    
				SetCtrlVal(panelHandle,PANEL_TXTTimedRate,Msg);
				sprintf(Msg,"%d",RvdCoinAB);
				SetCtrlVal(panelHandle,PANEL_TXTCoinFrmPckt,Msg);
				sprintf(Msg, "%d	%d	%d	%d	%d	%d\n", SinglesA, SinglesB, \
									CoinAB, TimedCoinAB, RvdCoinAB, WaitTime);
				if (TestFileHandle != -1)
					WriteFile(TestFileHandle, Msg, strlen(Msg));
				if (PlotFlag && PlotRefresh) {						//Plot the spectrum in real time.
					PlotY(GraphHandle, pnlGraph_GRAPH, CoinSpectrum, TimeAxisInPs, VAL_INTEGER,\
						VAL_CONNECTED_POINTS, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_DK_RED);
					PlotRefresh = 0;
				}
					
				
				RvdCoinAB 	= 0;									 //RdPtr increment is done above.
				break;																   
			case 'V':
				Fmt (Msg, "%d.%d.%d", *(RawBuf+RdPtr), \
					(*(RawBuf+RdPtr+1)&0xF0)/16, *(RawBuf+RdPtr+1)&0x0F);
				SetCtrlVal(panelHandle,PANEL_txtFpgaVersion,Msg);
				CopyString (Msg, 0, RawBuf+RdPtr+2, 0, -1);
				SetCtrlVal(panelHandle,PANEL_txtRbVersion,Msg);
				RdPtr += 15;
				break;
			case 'D':						
				//##process 'D'
				//Rabbit has finish the current run. What to do? HOM? Tomo?
				CountingFlag = 0;
				SetCtrlVal(panelHandle,PANEL_CountingLED, 0);
				ThisRunDone  = 1;
				RdPtr = WrPtr;
				break;
			default: 
				break;				 //usually shouldn't happen, read 1 byte to find the next header.
				
		}
		if (!CountingFlag) {
			RollOver = 0;
			if (ArvlTmBufPtr)	{			//if there is unsaved arrival time data, save it to file.
				ArrayToFile (TEMP_FILE_PATH_RAW, ArvlTmBuf, VAL_INTEGER, ArvlTmBufPtr+1, 1, 0,\
					        VAL_GROUPS_AS_COLUMNS, VAL_SEP_BY_TAB, 0, VAL_ASCII , VAL_APPEND);
				ArvlTmBufPtr = 0;
			}
		}
	}
	if (RollOver){
		RollOver = 0;
		RdPtr = 0;
	}
	//process the rest data. write the rest of WrFileBuf to file.
	return 0;
}


/********************************** Initialize Function ********************************************/
/* Intitialize funtion. Set read pointer and write pointer to the start of the buffer. Clear all the 
function flags for HOM, Tomo etc.*/
int Initialize(void) {
	int i;
	char basis[6] = {'H','V','D','A','R','L'};
	RdPtr = 0;
	WrPtr = 0;
	RollOver = 0;										  //RollOver Flag for read and write pointer.
	TestFileHandle 	= -1;
	FidSaveFile 	= -1;
	CountingFlag 	= 0;
	udpConnected	= 0;
	udpRetry		= 1;
	SaveDataFlag	= 0;
	WpCaliFlag		= 0;
	CaliFlag		= 0;
	CountingTime	= 0;
	HomedOnce		= 0;
	ThisRunDone		= 0;
	PicoPos 		= 0;
	PlotFlag 		= 0;
	PlotRefresh		= 0;
	WaitTomoGpib	= 0;
	WaitGpib		= 0;
	HomingFlag		= 0;
	LastLEDA		= PANEL_LEDAH;
	LastLEDB		= PANEL_LEDBH;
	EndOfRawBufPtr  = DATA_BUFFER_SIZE;
	SetCtrlVal( panelHandle, PANEL_txtVersion, GUIVERSION);
	SetCtrlVal( panelHandle, PANEL_txtCmRabbit, CmRbVERSION);
	SetCtrlVal( panelHandle, PANEL_txtCmFpga, CmFpgaVERSION);
	for (i=0;i<36;i++){
		BasisString[i][0] = basis[i/6];
		BasisString[i][1] = basis[i%6];
		BasisString[i][2] = 0;
		
	}
		
	
	//clear all the flags.
	return 0;
}

/******************************* Process the 'P' Packet Function ***********************************/
/* Process the 'P' packet. This function directly addresses the global variables, RawBuf and
RdPtr, and return instant coincidence count and arrival time to the function which calls it.
This function already assumes the RdPtr is immediately after 'P' in RawBuf.

Input:  address of RvdCoinAB, address of ArvlTmBuf, address of ArvlTmBufPtr, (explicit)
		RawBuf, RdPtr.												 (inexplicit)
OutPut: RdPtr, *RvdCoinAB, *(ArvlTmBuf+*ArvlTmBufPtrPtr), *ArvlTmBufPtrPtr.			   	   		   */

int PcsPPacket (int* RvdCoinAB, int* ArvlTmBuf, int* ArvlTmBufPtrPtr) {
	int count, i;
	
	count = RawBuf[RdPtr++];
	*RvdCoinAB += count;
	
	for (i=0; i<count; i++){
		if (ProcessArrivalTime (RawBuf+RdPtr+i*7+1, ArvlTmBuf + *ArvlTmBufPtrPtr))
			(*ArvlTmBufPtrPtr)++;			      //Note this is a pointer to an address ArvlTmBufPtr.
	}
	RdPtr += count * 7;
	return 0;
}

/******************************* Process the 'R' Packet Function ***********************************/
/* Process the 'R' packet. This function calculates the arrival time from a single 'R' packet.
This function already assumes the databuf pointer is immediately after 'R'.						   */

int ProcessArrivalTime (unsigned char *databuf, int *ArrivalTime) {
	int AdcA, AdcB, OffsetCounts, AFirst, AdcOverFlow;
	
  	AdcA	=	databuf[0] * 256 + databuf[1];
    AdcB	=	databuf[2] * 256 + databuf[3];
    OffsetCounts	=	10000 * databuf[4];											  //offset in ps.
    AFirst 	=	(databuf[5] & 0x04) / 2 - 1;
    AdcOverFlow	=	(databuf[5] & 0x01) || (databuf[5] & 0x02);

	//if (AdcA >= 4000 || AdcB >= 4000) return 0;
	if (ArTmA[AdcA]!=99999 && ArTmB[AdcB]!=99999 && !AdcOverFlow) {
		*ArrivalTime = ArTmA[AdcA] - ArTmB[AdcB] + OffsetCounts * AFirst;
		CoinSpectrum[(TimeAxisInPs/2*100+*ArrivalTime)/100]++;			  //offset to CoinSpectrum[TimeAxisInPs/2] -> 0 ns.
	}
	else 
		return 0;
	return 1;
}

int CVICALLBACK RdTimerTick (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_TIMER_TICK:
			DataRdThrFunc();
			break;
	}
	return 0;
}

int CVICALLBACK HomOk (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
									//clear GPIB errors and set gpib0 as the Controller-In-Charge.
			ibrsc(0,1);			//set gpib0 as the CIC
			ibsic(0);			//clear all previous errors
			
			GetCtrlVal(HomPopHandle,pnlHomPop_PicoPulseCount,&nPicoPulseCount);
			GetCtrlVal(HomPopHandle,pnlHomPop_HEachPosTime,&CountingTime);
			GetCtrlVal(HomPopHandle,pnlHomPop_GPIBADDR_PICO,&PicoAddr);
			GetCtrlVal(HomPopHandle,pnlHomPop_PicoDir,&PicoDir);
			GetCtrlVal(HomPopHandle,pnlHomPop_PicoTotalTravel,&PicoScanRange);
			
			if(!gpibPicoReset(PicoAddr)) break;	
			if (!FileSelectPopup ("", "", "", "Save the HOM Scan Data...", \
										VAL_OK_BUTTON, 0, 0, 1, 1, FilePath)) 
				break ;
			FuncFileHandle = OpenFile(FilePath,VAL_WRITE_ONLY,VAL_TRUNCATE,VAL_ASCII);
			
			if(PicoDir)										  //set the Picomotor movement direction.
				sprintf(Msg,":SOUR:DIR OUT");
			else
				sprintf(Msg,":SOUR:DIR IN");
			Send (0, PicoAddr, Msg, strlen(Msg), 2); 
			
			Msg[0] = 'R';
			Msg[1] = 0x01;										  //Send a 'Run' Command.
			Msg[2] = CountingTime /256;
			Msg[3] = CountingTime %256; 
			Msg[4] = 0x00; 			
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg, 5);
			
			SetCtrlVal(panelHandle,PANEL_CountingLED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnWpCali,  ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnTomo,    ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnCali,    ATTR_DIMMED,1);
			HOMFlag = 1;
			break;
	}
	return 0;
}

int CVICALLBACK HomCnl (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			RemovePopup(HomPopHandle);
			break;
	}
	return 0;
}

int CVICALLBACK WpCaliOkCB (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{   int MotionDone;
	switch (event)
	{
		case EVENT_COMMIT:
			ibrsc(0,1);				   //clear GPIB errors and set gpib0 as the Controller-In-Charge.
			ibsic(0);			
			GetCtrlVal(WpCalPopHandle,pnlWpCal_GPIBADDR_CHA,&gpibAddrA);
			GetCtrlVal(WpCalPopHandle,pnlWpCal_GPIBAXIS_CHA,&DeviceAxisA);
			GetCtrlVal(WpCalPopHandle,pnlWpCal_GPIBADDR_CHB,&gpibAddrB);
			GetCtrlVal(WpCalPopHandle,pnlWpCal_GPIBAXIS_CHB,&DeviceAxisB);
			GetCtrlVal(WpCalPopHandle,pnlWpCal_INCANG,&WAngInc);
			GetCtrlVal(WpCalPopHandle,pnlWpCal_WEachPosTime,&CountingTime);
			
			SetCtrlAttribute(GraphHandle,pnlGraph_GRAPH,ATTR_XNAME,\
														"Waveplate Positions");
			SetCtrlAttribute(GraphHandle,pnlGraph_GRAPH,ATTR_YNAME,\
														 "Singles Count Rate");
			SetCtrlAttribute(GraphHandle,pnlGraph_GRAPH,ATTR_LABEL_TEXT,\
													 "Waveplate Calibration");
			WScanIndex = 0;
			
			if(!gpibMotorOn(gpibAddrA,DeviceAxisA)) break;
			if(!gpibMotorOn(gpibAddrB,DeviceAxisB)) break;
			
			if(abs(WAngInc)<0.18) {
				MessagePopup ("Error", "Angle Increment Too Small");
				break;
			}
			
			FileSelectPopup ("", "", "", "Save the Calibration Data...", \
								VAL_SAVE_BUTTON, 0, 0, 0, 1, FilePath);
			FuncFileHandle = OpenFile(FilePath,VAL_WRITE_ONLY,VAL_TRUNCATE,VAL_ASCII);
			
			gpibSearchForHome(gpibAddrA,DeviceAxisA);							
			gpibSearchForHome(gpibAddrB,DeviceAxisB);
			
			while(1) {
				MotionDone = gpibIsMotionDone(gpibAddrA,DeviceAxisA);
				MotionDone = MotionDone && gpibIsMotionDone(gpibAddrB,DeviceAxisB);
				if (!MotionDone)
				{
					Delay(0.5);
					SetCtrlVal(panelHandle,PANEL_MotorRunning,1);
				}else break;
			}
			SetCtrlVal(panelHandle,PANEL_MotorRunning,0); 
			
			Msg[0] = 'R';
			Msg[1] = 0x01;										  //Send a 'Run' Command.
			Msg[2] = CountingTime /256;
			Msg[3] = CountingTime %256; 
			Msg[4] = 0x00; 			
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg, 5);
			SetCtrlVal(panelHandle,PANEL_CountingLED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnHOM,     ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnTomo,    ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnCali,    ATTR_DIMMED,1);
			WpCaliFlag = 1;
			break;
	}
	return 0;
}

int CVICALLBACK WpCaliCnlCB (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			RemovePopup(WpCalPopHandle); 
			break;
	}
	return 0;
}

int CVICALLBACK pnlHomPopCB (int panel, int event, void *callbackData,
		int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_GOT_FOCUS:
			break;
		case EVENT_LOST_FOCUS:
			break;
		case EVENT_CLOSE:
			RemovePopup(HomPopHandle);
			break;
	}
	return 0;
}


int CVICALLBACK pnlWpCalCB (int panel, int event, void *callbackData,
		int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_GOT_FOCUS:
			break;
		case EVENT_LOST_FOCUS:
			break;
		case EVENT_CLOSE:
			RemovePopup(WpCalPopHandle);
			break;
	}
	return 0;
}

	
int CVICALLBACK pnlGraphCB (int panel, int event, void *callbackData,
		int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_GOT_FOCUS:
			break;
		case EVENT_LOST_FOCUS:
			break;
		case EVENT_CLOSE:
			RemovePopup(GraphHandle);
			PlotFlag = 0;
			break;
	}
	return 0;
}


void HomeAllMotors (void)
{
	gpibSearchForHome(CfgSetup[0][0],CfgSetup[0][1]);							
	gpibSearchForHome(CfgSetup[0][2],CfgSetup[0][3]);
	gpibSearchForHome(CfgSetup[1][0],CfgSetup[1][1]);
	gpibSearchForHome(CfgSetup[1][2],CfgSetup[1][3]);
}


int IsTomoMotorDone(void)
{
	int MotionDone;
	
	MotionDone = gpibIsMotionDone(CfgSetup[0][0],CfgSetup[0][1]);
	MotionDone = MotionDone && gpibIsMotionDone(CfgSetup[0][2],CfgSetup[0][3]);
	MotionDone = MotionDone && gpibIsMotionDone(CfgSetup[1][0],CfgSetup[1][1]);
	MotionDone = MotionDone && gpibIsMotionDone(CfgSetup[1][2],CfgSetup[1][3]);
		
	if (!MotionDone) {
		SetCtrlVal(panelHandle,PANEL_MotorRunning,1);
		return 0;
	}
	else {
		SetCtrlVal(panelHandle,PANEL_MotorRunning,0);
		return 1;
	}
}

/**************************** Entry point for different functionalities ****************************/
int CVICALLBACK FuncTimerCB (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{   int MotionDone;
	float MotorActPos;
	static int time = 0;
	
	switch (event)
	{
		case EVENT_TIMER_TICK:
			if (time < PLOT_REFRESH) time ++;
			else {
				time = 0;
				PlotRefresh = 1;
			}
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, "H", 1);						//Heart beat.
			if (WaitTomoGpib) {
				if (! IsTomoMotorDone()) break;						//wait until GPIB motor are done.
				else if (HomingFlag) {
					tomoSetMotorForBasis(0);
					HomingFlag = 0;
				}
				else {
					FuncData[1] = 0;
					FuncData[2] = 0;
					FuncData[3] = 0;
					
					Msg[0] = 'R';
					Msg[1] = 0x01;										  //Send a 'Run' Command.
					Msg[2] = CountingTime /256;
					Msg[3] = CountingTime %256; 
					Msg[4] = 0x00; 			
					UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg, 5);
					SetCtrlVal(panelHandle,PANEL_CountingLED,1);
					WaitTomoGpib = 0;
					MotorActPos = gpibReadActualPosition(CfgSetup[0][0],CfgSetup[0][1]);
					sprintf(Msg,"%3.2f",MotorActPos);
					SetCtrlVal(panelHandle,PANEL_ActPosHWPA,Msg);
					MotorActPos = gpibReadActualPosition(CfgSetup[0][2],CfgSetup[0][3]);
					sprintf(Msg,"%3.2f",MotorActPos);
					SetCtrlVal(panelHandle,PANEL_ActPosQWPA,Msg);	
					MotorActPos = gpibReadActualPosition(CfgSetup[1][0],CfgSetup[1][1]);
					sprintf(Msg,"%3.2f",MotorActPos);
					SetCtrlVal(panelHandle,PANEL_ActPosHWPB,Msg);
					MotorActPos = gpibReadActualPosition(CfgSetup[1][2],CfgSetup[1][3]);
					sprintf(Msg,"%3.2f",MotorActPos);
					SetCtrlVal(panelHandle,PANEL_ActPosQWPB,Msg);
				}
			}
			else if (ThisRunDone){
				if (TomoFlag) {															//TOMOGRAPHY.
					if (numTomoBasis == 35)
						FuncData[0] = TomoBasisIndex;				  //store the previous basis.
					else
						FuncData[0] = TomoCompleteBasis[TomoBasisIndex];
					
					sprintf(Msg,"%d\t%d\t%d\t%d\n", FuncData[0], FuncData[1],\
												FuncData[2], FuncData[3]);
					WriteFile(FuncFileHandle, Msg, strlen(Msg));

					if(TomoBasisIndex < numTomoBasis){						  //move to next setting.
						if (numTomoBasis == 35)						   	   //Basis setting from 0-35. 	  
							tomoSetMotorForBasis(++TomoBasisIndex);		   
						else 									   	   //Basis setting pre-sequenced.
							tomoSetMotorForBasis(TomoCompleteBasis[++TomoBasisIndex]);  
						WaitTomoGpib 	= 1;
					}
					else {
							TomoFlag = 0;							   				   //finish Tomo.
							TomoBasisIndex = 0;					 			 //Prepare for next tomo.
							CloseFile(FuncFileHandle);
							SetCtrlVal(panelHandle,PANEL_StatusMessage, "Tomography Data Aquired.");
							SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_DIMMED,0);
							SetCtrlAttribute(panelHandle,PANEL_btnWpCali,  ATTR_DIMMED,0);
							SetCtrlAttribute(panelHandle,PANEL_btnHOM,     ATTR_DIMMED,0);
							SetCtrlAttribute(panelHandle,PANEL_btnCali,    ATTR_DIMMED,0);
					}
				}
				
				else if (WpCaliFlag){					  //WAVEPLATE CALIBRATION BY SINGLE PHOTONS.
					WActPosA = gpibReadActualPosition(gpibAddrA,DeviceAxisA);
					WActPosB = gpibReadActualPosition(gpibAddrB,DeviceAxisB);
					sprintf(Msg,"%3.3f",WActPosA);
					SetCtrlVal(panelHandle,PANEL_ActPosRead_ChA,Msg);
					sprintf(Msg,"%3.3f",WActPosB);
					SetCtrlVal(panelHandle,PANEL_ActPosRead_ChB,Msg);
					
					sprintf(Msg,"%3.2f\t%d\t%3.2f\t%d\n", WActPosA, FuncData[1],\
														  WActPosB, FuncData[2]);
					WriteFile(FuncFileHandle,Msg,strlen(Msg));
					
					//@@PlotXY(panelHandle,PANEL_GRAPH,TransPWCaliData, &(TransPWCaliData[1][0]),
					//		WCaliDataIndex,VAL_DOUBLE,VAL_DOUBLE,VAL_THIN_LINE,
					//			VAL_NO_POINT,VAL_SOLID,1,VAL_RED); 
					//PlotXY(panelHandle,PANEL_GRAPH,&(TransPWCaliData[2][0]),
					//		&(TransPWCaliData[3][0]),WCaliDataIndex,VAL_DOUBLE,
					//			VAL_DOUBLE,VAL_THIN_LINE,VAL_NO_POINT,VAL_SOLID,1,
					//			VAL_YELLOW);
								
					if(WScanIndex < (180/WAngInc)) {   			 //Not a full Scan, move to next inc.
						gpibMoveRelativeAngle(gpibAddrA,DeviceAxisA, WAngInc);    
						gpibMoveRelativeAngle(gpibAddrB,DeviceAxisB, WAngInc);
						while(1) {
							MotionDone = gpibIsMotionDone(gpibAddrA,DeviceAxisA);
							MotionDone = MotionDone && gpibIsMotionDone(gpibAddrB,DeviceAxisB);
							if (!MotionDone) {
								Delay(0.5);
								SetCtrlVal(panelHandle,PANEL_MotorRunning,1);
							}else break;
						}
						SetCtrlVal(panelHandle,PANEL_MotorRunning,0);
						WScanIndex++;
						Msg[0] = 'R';
						Msg[1] = 0x01;										  //Send a 'Run' Command.
						Msg[2] = CountingTime /256;
						Msg[3] = CountingTime %256; 
						Msg[4] = 0x00; 			
						UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg, 5);
						SetCtrlVal(panelHandle,PANEL_CountingLED,1);
					}
					else  {
						WpCaliFlag =0;  											//finish Calibration.
						WScanIndex = 0;								  //Prepare for next Calibration.
						CloseFile(FuncFileHandle);
						SetCtrlVal(panelHandle,PANEL_StatusMessage, "Calibration Data Aquired.");
						SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_DIMMED,0);
						SetCtrlAttribute(panelHandle,PANEL_btnTomo,    ATTR_DIMMED,0);
						SetCtrlAttribute(panelHandle,PANEL_btnHOM,     ATTR_DIMMED,0); 
						SetCtrlAttribute(panelHandle,PANEL_btnCali,    ATTR_DIMMED,0);
					}
				}
				
				else if (HOMFlag) {   									   //HONG-OU-MANDEL SCANNING.
					sprintf(Msg,"%3.2f\t%d\t%d\t%d\n",  PicoPos,FuncData[1],\
													FuncData[2],FuncData[3]);
					WriteFile(FuncFileHandle,Msg,strlen(Msg));
									
					FuncData[1] = 0;
					FuncData[2] = 0;
					FuncData[3] = 0;
					if(PicoPos < PicoScanRange*100000) {						  //move to next inc.
						PicoPos += nPicoPulseCount;
						sprintf(Msg,":SOUR:PULS:COUN %d",nPicoPulseCount);
						Send (0,PicoAddr,Msg,strlen(Msg),2); 
						while(1) {												   //wait motor stop.
							if (gpibIsPicoDone(PicoAddr)) 
								break;
							else Delay(0.5);
						}
						Msg[0] = 'R';
						Msg[1] = 0x01;										  //Send a 'Run' Command.
						Msg[2] = CountingTime /256;
						Msg[3] = CountingTime %256; 
						Msg[4] = 0x00; 			
						UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg, 5);
						SetCtrlVal(panelHandle,PANEL_CountingLED,1);
					}
					else {
						PicoPos = 0;
						HOMFlag =0;  			   				  //finish HOM Scan.
						CloseFile(FuncFileHandle); //close the file for HOM scanning.
						SetCtrlVal(panelHandle,PANEL_StatusMessage, "HOM Scan Data Aquired.");
						SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_DIMMED,0);
						SetCtrlAttribute(panelHandle,PANEL_btnWpCali,  ATTR_DIMMED,0);
						SetCtrlAttribute(panelHandle,PANEL_btnTomo,    ATTR_DIMMED,0);
						SetCtrlAttribute(panelHandle,PANEL_btnCali,    ATTR_DIMMED,0);
					}  
				}
				
				else {														   //G2 MEASUREMENT DATA.
					SetCtrlVal(panelHandle,PANEL_StatusMessage,"Coincidence Data Aquired."); 
					SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_LABEL_TEXT ,"Start Counting");
					SetCtrlAttribute(panelHandle,PANEL_btnCali,  ATTR_DIMMED,0);
					SetCtrlAttribute(panelHandle,PANEL_btnWpCali,ATTR_DIMMED,0);
					SetCtrlAttribute(panelHandle,PANEL_btnTomo,  ATTR_DIMMED,0);
					SetCtrlAttribute(panelHandle,PANEL_btnHOM,   ATTR_DIMMED,0);
					SaveDataFlag = (GenericMessagePopup ("Save Data?", \
						"Save aquired data and to files: NAME + hh;mm;ss, mm-dd-yyyy.raw  (.txt)", "OK",\
					   			"Cancel", 0, RawPath, 256, 0, VAL_GENERIC_POPUP_INPUT_STRING, 1, 2))==1; 
					if (SaveDataFlag){
						GetSystemTime(&hour,&min,&sec);
						GetSystemDate(&mon,&day,&year);
						GetCtrlVal(panelHandle,PANEL_MY_FOLDER,Msg);	   	//path  to save to files.
						CopyString (Msg, StringLength (Msg), "\\", 0, -1);
						CopyString (Msg, StringLength (Msg), RawPath, 0, -1);
						sprintf(RawPath, " %d;%d;%d, %d-%d-%d",hour,min,sec,mon,day,year);
						CopyString (Msg, StringLength (Msg), RawPath, 0, -1);
						CopyString (Msg, StringLength (Msg), ".raw", 0, -1);
						CopyFile (TEMP_FILE_PATH_RAW, Msg);
						CopyString (Msg, StringLength (Msg)-4, ".txt", 0, -1);
						CopyFile (TEMP_FILE_PATH_TXT, Msg);
					}
				}
				ThisRunDone = 0;
			}
			if (WaitGpib) {
				if ( ! IsTomoMotorDone())
					SetCtrlAttribute(panelHandle,PANEL_btnTomo, ATTR_DIMMED,1);
				else {
					SetCtrlAttribute(panelHandle,PANEL_btnTomo, ATTR_DIMMED,0);
					WaitGpib = 0;
					//read the actual value of the motor postion.
					MotorActPos = gpibReadActualPosition(CfgSetup[0][0],CfgSetup[0][1]);
					sprintf(Msg,"%3.2f",MotorActPos);
					SetCtrlVal(panelHandle,PANEL_ActPosHWPA,Msg);
					MotorActPos = gpibReadActualPosition(CfgSetup[0][2],CfgSetup[0][3]);
					sprintf(Msg,"%3.2f",MotorActPos);
					SetCtrlVal(panelHandle,PANEL_ActPosQWPA,Msg);	
					MotorActPos = gpibReadActualPosition(CfgSetup[1][0],CfgSetup[1][1]);
					sprintf(Msg,"%3.2f",MotorActPos);
					SetCtrlVal(panelHandle,PANEL_ActPosHWPB,Msg);
					MotorActPos = gpibReadActualPosition(CfgSetup[1][2],CfgSetup[1][3]);
					sprintf(Msg,"%3.2f",MotorActPos);
					SetCtrlVal(panelHandle,PANEL_ActPosQWPB,Msg);
				}
			}
			break;
	}
	return 0;
}

/*********************************** UDP Timer Callback *******************************************/
// Tasks: 1. periodically check if connection is off.
//		  2. if the connection is first established after a connection lost, do a calibraion, and 
//           version read out. 
int CVICALLBACK udpTimerCB (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_TIMER_TICK:
		if (Timer() - udpTimeOut > 5) {						  //Detect the disconnection from Rabbit.
			udpRetry = 1;
			udpConnected = 0;
			SetCtrlVal(panelHandle,PANEL_StatusMessage,"Connection lost!");
			SetCtrlVal(panelHandle,PANEL_LedConnected, 0);					 //showing no connection.
			SetCtrlAttribute(panelHandle,PANEL_btnStartCnt, ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnWpCali,   ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnTomo,     ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnHOM,      ATTR_DIMMED,1);
			SetCtrlAttribute(panelHandle,PANEL_btnCali,     ATTR_DIMMED,1);
		}
		if (udpConnected && udpRetry)	{	 //Do this only once after the connection is established.
			udpRetry = 0;
			udpTimeOut = Timer();
			SetCtrlVal(panelHandle,PANEL_StatusMessage,"Connected");
			SetCtrlVal(panelHandle,PANEL_LedConnected, 1);	 					//showing connection.
			SetCtrlAttribute(panelHandle,PANEL_btnStartCnt, ATTR_DIMMED,0);	//enable the buttons.
			SetCtrlAttribute(panelHandle,PANEL_btnWpCali,   ATTR_DIMMED,0);
			SetCtrlAttribute(panelHandle,PANEL_btnTomo,     ATTR_DIMMED,0);
			SetCtrlAttribute(panelHandle,PANEL_btnHOM,      ATTR_DIMMED,0);
			SetCtrlAttribute(panelHandle,PANEL_btnCali,     ATTR_DIMMED,0);
			CountingFlag = 0;
			SetCtrlVal(panelHandle,PANEL_CountingLED,0);  							  //Set LED Dark.
			SetCtrlAttribute(panelHandle,PANEL_btnStartCnt,ATTR_LABEL_TEXT ,"Start Counting");
			
			CaliFlag = 1;
			sprintf(Msg, "%s", "C 34C 34C");
			Msg[1] = 0x03;											 //Calibration on high level.
			Msg[5] = 0x01;											  //Calibration on low level.
			Msg[9] = 0x00;													   //Calibration off.
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg, 	 2);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+2, 1);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+3, 1);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+4, 2);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+6, 1);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+7, 1);
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, Msg+8, 2);
			
			UDPWrite (udpChannel, RABBIT_PORT, RABBIT_IP, "V", 	 1);	  //read version numbers.
		}
		break;
	}
	return 0;
}
