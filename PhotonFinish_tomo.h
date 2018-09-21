/**************************************************************************/
/* LabWindows/CVI User Interface Resource (UIR) Include File              */
/* Copyright (c) National Instruments 2010. All Rights Reserved.          */
/*                                                                        */
/* WARNING: Do not add to, delete from, or otherwise modify the contents  */
/*          of this include file.                                         */
/**************************************************************************/

#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

     /* Panels and Controls: */

#define  PANEL                            1       /* callback function: panelCB */
#define  PANEL_btnWpCali                  2       /* callback function: WCalibrate */
#define  PANEL_btnCali                    3       /* callback function: CalibrateValue */
#define  PANEL_btnStartCnt                4       /* callback function: GetCoincidence */
#define  PANEL_PlotButton                 5       /* callback function: PlotTheSpectrum */
#define  PANEL_MY_FOLDER                  6
#define  PANEL_LedConnected               7
#define  PANEL_CountingLED                8
#define  PANEL_HeartBeat                  9
#define  PANEL_StatusMessage              10
#define  PANEL_DECORATION                 11
#define  PANEL_A_Low                      12
#define  PANEL_B_Low                      13
#define  PANEL_A_High                     14
#define  PANEL_B_High                     15
#define  PANEL_TEXTMSG_2                  16
#define  PANEL_TEXTMSG_24                 17
#define  PANEL_TEXTMSG_23                 18
#define  PANEL_TEXTMSG_13                 19
#define  PANEL_TEXTMSG_3                  20
#define  PANEL_TEXTMSG_15                 21
#define  PANEL_TEXTMSG_18                 22
#define  PANEL_TEXTMSG_4                  23
#define  PANEL_TEXTMSG_16                 24
#define  PANEL_TEXTMSG_5                  25
#define  PANEL_TEXTMSG_6                  26
#define  PANEL_DECORATION_6               27
#define  PANEL_DECORATION_9               28
#define  PANEL_numTmPerSetting            29
#define  PANEL_TEXTMSG_25                 30
#define  PANEL_TEXTMSG                    31
#define  PANEL_DECORATION_13              32
#define  PANEL_DECORATION_8               33
#define  PANEL_ActPosQWPB                 34
#define  PANEL_ActPosQWPA                 35
#define  PANEL_TEXTMSG_28                 36
#define  PANEL_TEXTMSG_29                 37
#define  PANEL_TEXTMSG_27                 38
#define  PANEL_TEXTMSG_26                 39
#define  PANEL_TEXTMSG_30                 40
#define  PANEL_ActPosHWPB                 41
#define  PANEL_ActPosHWPA                 42
#define  PANEL_ActPosRead_ChA             43
#define  PANEL_nCoinRate                  44
#define  PANEL_TXTSINGLESB                45
#define  PANEL_ActPosRead_ChB             46
#define  PANEL_TXTTimedRate               47
#define  PANEL_TXTCoinFrmPckt             48
#define  PANEL_TXTSINGLESA                49
#define  PANEL_TEXTMSG_14                 50
#define  PANEL_txtFpgaVersion             51
#define  PANEL_txtRbVersion               52
#define  PANEL_txtCmFpga                  53
#define  PANEL_txtCmRabbit                54
#define  PANEL_txtVersion                 55
#define  PANEL_TEXTMSG_8                  56
#define  PANEL_TEXTMSG_7                  57
#define  PANEL_COMMANDBUTTON              58      /* callback function: BrowseForPath */
#define  PANEL_LoadCfgButton              59      /* callback function: LoadCfg */
#define  PANEL_btnHOM                     60      /* callback function: HOMRun */
#define  PANEL_btnTomo                    61      /* callback function: TomoRun */
#define  PANEL_numCountingTime            62
#define  PANEL_LEDBL                      63
#define  PANEL_LEDBR                      64
#define  PANEL_LEDBA                      65
#define  PANEL_LEDBD                      66
#define  PANEL_LEDBV                      67
#define  PANEL_LEDBH                      68
#define  PANEL_LEDAL                      69
#define  PANEL_LEDAR                      70
#define  PANEL_LEDAA                      71
#define  PANEL_LEDAD                      72
#define  PANEL_LEDAV                      73
#define  PANEL_LEDAH                      74
#define  PANEL_MotorRunning               75
#define  PANEL_TEXTMSG_22                 76
#define  PANEL_TEXTMSG_21                 77
#define  PANEL_TEXTMSG_20                 78
#define  PANEL_TEXTMSG_19                 79
#define  PANEL_TEXTMSG_17                 80
#define  PANEL_SetBasisButton             81      /* callback function: SetBasis */
#define  PANEL_BasisIndex                 82
#define  PANEL_HomeAllButton              83      /* callback function: HomeAll */
#define  PANEL_ChkBox_TimeStamp           84
#define  PANEL_ChkBox_OverCom             85
#define  PANEL_DECORATION_11              86
#define  PANEL_DECORATION_2               87
#define  PANEL_DECORATION_12              88
#define  PANEL_DECORATION_10              89
#define  PANEL_FuncTimer                  90      /* callback function: FuncTimerCB */
#define  PANEL_udpTimer                   91      /* callback function: udpTimerCB */
#define  PANEL_RdTimer                    92      /* callback function: RdTimerTick */

#define  pnlGraph                         2       /* callback function: pnlGraphCB */
#define  pnlGraph_GRAPH                   2

#define  pnlHomPop                        3       /* callback function: pnlHomPopCB */
#define  pnlHomPop_HEachPosTime           2
#define  pnlHomPop_PicoTotalTravel        3
#define  pnlHomPop_PicoPulseCount         4
#define  pnlHomPop_GPIBADDR_PICO          5
#define  pnlHomPop_btnHomCnl              6       /* callback function: HomCnl */
#define  pnlHomPop_btnHomOk               7       /* callback function: HomOk */
#define  pnlHomPop_PicoDir                8

#define  pnlWpCal                         4       /* callback function: pnlWpCalCB */
#define  pnlWpCal_TEXTMSG_12              2
#define  pnlWpCal_TEXTMSG_11              3
#define  pnlWpCal_GPIBAXIS_CHA            4
#define  pnlWpCal_GPIBAXIS_CHB            5
#define  pnlWpCal_GPIBADDR_CHA            6
#define  pnlWpCal_GPIBADDR_CHB            7
#define  pnlWpCal_WEachPosTime            8
#define  pnlWpCal_INCANG                  9
#define  pnlWpCal_btnWpCaliCnl            10      /* callback function: WpCaliCnlCB */
#define  pnlWpCal_btnWpCaliOk             11      /* callback function: WpCaliOkCB */
#define  pnlWpCal_TEXTMSG_9               12
#define  pnlWpCal_TEXTMSG_10              13


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK BrowseForPath(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK CalibrateValue(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK FuncTimerCB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK GetCoincidence(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK HomCnl(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK HomeAll(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK HomOk(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK HOMRun(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK LoadCfg(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK panelCB(int panel, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK PlotTheSpectrum(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK pnlGraphCB(int panel, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK pnlHomPopCB(int panel, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK pnlWpCalCB(int panel, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK RdTimerTick(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SetBasis(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK TomoRun(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK udpTimerCB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK WCalibrate(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK WpCaliCnlCB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK WpCaliOkCB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
