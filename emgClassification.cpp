/*********************************************************************
*
* emgClassification.cpp
*
* Written June 24, 2009
* By Ralph Lin
* University of Washington
* Neurobotics Lab
*
* Description:
*	This program is used to run the single ACT finger (6 DOF)
*	by recognizing EMG signals from a human for the RSS 2009
*	demo. It performs this by several steps:
*		1. Reading in EMG data from EMG electrodes placed on three
*		   muscles on the index finger using installed NiDAQ boards
*		2. Classifying index finger motion as flex/extension,
*		   adduction/abduction, clockwise circle, counterclockwise
*		   circle, half curl, or full curl
*		3. Output movement and period of motion to robot using UDP
*		   protocol
*		4. Output EMG signals to on screen oscilloscope displays
*
* Notes:
*	This program was designed to run with the corresponding 
*	main_RSSdemo.cpp behavior on the ACTHand control machine as well
*	as glsample.cpp to provide a on-screen avatar for finger motion
*
*********************************************************************/

#include <windows.h>

#include <conio.h>
#include "NIDAQmx.h"

#include <fstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <algorithm>
#include <math.h>
#include <deque>
#include <string.h>
#include "winsock.h"
#include "EMG classification.h"
#pragma comment(lib,"winmm.lib")
// Helper macro for displaying errors
#define PRINTERROR(s)   \
               fprintf(stderr,"\n%: %d\n", s, WSAGetLastError())

// FREQUENCY is the communications frequency in Hz
#define FREQUENCY 1000

//stuff for our packets.
#define MAX_COMM_PACKET sizeof(packet)

// window size for first phase of EMG calculation
#define WINDOW_SIZE 25

#define PORT 35
#define SERVER_ADDRESS "128.208.4.65"

// butterworth filter stuff
#define GAIN   2.762663641e+08
#define WINDOW  10

static CRITICAL_SECTION cs;
void setoutput(int behav);
int behavior=0;

//Winplot Includes...
#include "OSC_dll.h"
extern  char *pEmpty = "";
extern  char *osc1Params = "Scope_Desk_Force.ini";
extern  char *osc2Params = "Scope_Desk_EMG1.ini";
extern  char *osc3Params = "Scope_Desk_EMG2.ini";
extern  char *osc4Params = "Scope_Desk_EMG3.ini";

//#include <tchar.h>
using namespace std;

class butterworth_filter {
public:
	butterworth_filter();
	double filter(double* xv);
	int filled_size;
	bool filled_window;
private:
	std::deque<double> xv;
	std::deque<double> yv;
};

butterworth_filter::butterworth_filter(){
	xv = std::deque<double> (11,0);
	yv = std::deque<double> (11,0);
	filled_window = false;
	filled_size = 0;
}

double butterworth_filter::filter(double* num)
{
	this->xv.pop_front();
	this->xv.push_back(*num/GAIN);
	
	if(!filled_window)
		++filled_size;

	/*if(filled_size < 11) {
		return 0;
	} else {*/
		filled_window = true;
		yv.pop_front();
		double temp = 1.0000000042095*xv[0] + 10.0000000420952*xv[1] 
					 + 45.0000001894285*xv[2] + 120.0000005051426*xv[3]
					 + 210.0000008839996*xv[4] + 252.0000010607995*xv[5]
					 + 210.0000008839996*xv[6] + 120.0000005051426*xv[7]
					 + 45.0000001894285*xv[8] + 10.0000000420952*xv[9]
					 + 1.0000000042095*xv[10]
					 + ( -0.132768084192920 * yv[0]) + (  1.594239767690200 * yv[1])
                     + ( -8.645682137526434 * yv[2]) + ( 27.890299172493208 * yv[3])
                     + (-59.280951574099042 * yv[4]) + ( 86.767068040561213 * yv[5])
                     + (-88.587663251263734 * yv[6]) + ( 62.315352281547177 * yv[7])
                     + (-28.912194584176557 * yv[8]) + ( 7.992296662399126 * yv[9]);
		yv.push_back(temp);
		return temp;
	//}
}

/* classification parameters found from off-line machine learning algo */

double flexExtParam[30]={0.170146559156527,0.0157801249790987,-0.114828958290261,
-0.0634086624359277,0.00956343207314041,-0.174042237055961,-0.0179230532823764,
1.87245189778292,0.190938247426429,-0.035846360509161,-1.20388690873209,0.554644329186938,
1.11389749361242,0.8716250528689,-1.83144296289431,-0.842378687893125,-0.203958791726913,
1.07276755819452,-1.45016622345778,-1.47537988609523,-1.41702889672243,-0.613075837244772,
-1.13169703112531,-1.95170088285908,-1.3310644000526,1.23806209901104,1.13606959641412,
2.38895023987682,2.71813309668445,0.993396155082262};

double adAbParam[30]={0.410830999506563,0.360472769068434,0.149502083222677,
-0.0201459995471041,0.0231409201465198,-0.378835630781073,0.0366553912556151,
-0.0264014208577248,0.0857079419998244,-0.418314814713429,-1.02408245069492,
-0.559860718682096,0.572319670159799,2.03173034964075,-4.67531624221262,
-0.872706690183694,5.16921728271158,1.06046500567271,-1.55388273183721,
-1.36432917516963,-1.49587569194387,0.849770555778097,1.27756207885958,
2.69353311279633,-0.181638032607709,0.624044517317172,0.0645244258970303,
-1.56480231797083,-1.44868481524132,-1.09461642447942};

double halfCurlParam[30]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

double fullCurlParam[30]= {-0.00177708737056369,-0.152106841063485,-0.279959469819932,
    -0.244915852527718,-0.0469559318194196,-0.268571954855573,
    -0.211083532601022,-0.844438589809564,0.233403372652309,
    0.276274815247572,2.15316084837944,4.15087760797863,
    2.19558604253565,1.2658823047694,0.544695810244649,
    0.63706429109681,2.85579939275779,-0.00668096330560086,
    -1.06357196772542,0.774091009379639,2.18691257429436,
    -0.458769781729415,0.483241491749431,-0.17127247777634,
    2.53512377907832,-0.63928779502934,-0.26100820351531,
    -1.67191924953166,0.37660409055949,0.681942293528043};

double cwParam[30]={-0.179300266356991,-0.0530829278947829,0.0853417435283217,
    0.0761529489617064,0.0601817181702171,0.672996606442763,
    0.810868111677437,-0.293022815379,0.338384363264691,
    -0.0125184588625037,0.0754974019873139,-3.92494159491237,
    -0.945485690990163,1.55106996744017,1.90318304381503,
    -0.169989029198011,-3.36378195244722,0.661985899069023,
    3.83651655623667,1.98394926316757,-1.77474742968043,
    -1.28582139226806,0.286171137212688,1.57546126182025,
    -0.401173963978455,0.9318415481155,1.15716255502346,
    1.77435768257203,-0.912556439184006,-0.878265528448426};

double ccwParam[30]={-0.17925283447727,-0.0725270271350888,0.250387147316579,
    0.402340030771682,-0.0245691723697643,0.113571590476048,
    -0.622334897786988,-0.250943264896144,-0.464483941259737,
    0.313988507396889,0.476681021361187,0.780275466658321,
    -1.06014513865307,-3.82538708994432,3.99441612039136,
    2.15395462816244,-2.12254634625634,-1.7691184031754,
    0.33767544406799,0.691190714559821,3.58395368096907,
    1.28872041029646,-0.739617800079891,-1.49568788923496,
    0.314913633406889,-1.07427613074564,-1.2692972774472,
    -0.0350871809201075,0.536604893244105,1.47634612001107};

#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

/* calculateStdDev function calculates standard deviation from 
   array[start] to array[end] automatically wraps around buffer 
   if end < start											*/

double calculateStdDev(deque<double> &data, int start, int end) {
	
	double Average=0;
	double sum=0;

	// if end > start, then calculate normally
	if (end > start) {
		for (int i=0;i<(end-start+1);i++) {
			try {
				Average+=data.at(start+i);
				//printf("%0.16f\n",Average);
			}
			catch(out_of_range e)
			{
				//cerr << e.what() << endl; // prints "invalid string position"
				//printf("CalcStdev\n");
			}
		}
		Average=Average/(end-start+1);
		// return (Average/(end-start+1));

		for (int i=0;i<(end-start+1);i++) {
			try {
				sum+=pow(data.at(start+i)-Average,2);
			}
			catch(out_of_range e)
			{
				//cerr << e.what() << endl; // prints "invalid string position"
			}
		}
		return sqrt(sum/(end-start));
	}
	else { 
		// if start > end, then need to wrap around buffer
		if (start > end) {
			for (int i=0;i<(5000-start+end+1);i++) {
				try {
					Average+=data.at((start+i)%5000);
				}
				catch(out_of_range e)
				{
					cerr << e.what() << endl; // prints "invalid string position"
				}
			}
			Average=Average/(5000-start+end+1);
			// return(Average/(5000-start+end+1));
			
			for (int i=0;i<(5000-start+end+1);i++) {
				try {
					sum+=pow(data.at((start+i)%5000)-Average,2);
				}
				catch(out_of_range e)
				{
					cerr << e.what() << endl; // prints "invalid string position"
				}
			}
			return sqrt(sum/(5000-start+end));
		}
	}
	return (-1);
}


/* classify takes the 10 standard deviations within a cycle as input and classifies
   the cycle as (1) flex/ex, (2) ad/ab, (3) half curl, (4) full curl, (5) cw, (6)ccw */

int classify(deque<double> &FDISD, deque<double> &ExtensorSD, deque<double> &FlexorSD) {
	double Classifier[6]={0,0,0,0,0,0};

	int i;

	// build classifier for flex extend
	for (i=0;i<10;i++) {
		Classifier[0]+=FDISD.at(i)*flexExtParam[i];
	}
	for (i=0;i<10;i++) {
		Classifier[0]=FlexorSD.at(i)*flexExtParam[i+10]+Classifier[0];
	}
	for (i=0;i<10;i++) {
		Classifier[0]+=ExtensorSD.at(i)*flexExtParam[i+20];
	}

	// build classifier for adduction/abduction
	for (i=0;i<10;i++) {
		Classifier[1]+=FDISD.at(i)*adAbParam[i];
	}
	for (i=0;i<10;i++) {
		Classifier[1]+=FlexorSD.at(i)*adAbParam[i+10];
	}
	for (i=0;i<10;i++) {
		Classifier[1]+=ExtensorSD.at(i)*adAbParam[i+20];
	}

	// build classifier for half curl
	for (i=0;i<10;i++) {
		Classifier[2]+=FDISD.at(i)*halfCurlParam[i];
	}
	for (i=0;i<10;i++) {
		Classifier[2]+=FlexorSD.at(i)*halfCurlParam[i+10];
	}
	for (i=0;i<10;i++) {
		Classifier[2]+=ExtensorSD.at(i)*halfCurlParam[i+20];
	}

	// build classifier for full curl
	for (i=0;i<10;i++) {
		Classifier[3]+=FDISD.at(i)*fullCurlParam[i];
	}
	for (i=0;i<10;i++) {
		Classifier[3]+=FlexorSD.at(i)*fullCurlParam[i+10];
	}
	for (i=0;i<10;i++) {
		Classifier[3]+=ExtensorSD.at(i)*fullCurlParam[i+20];
	}

	// build classifier for counterclockwise circular motion
	for (i=0;i<10;i++) {
		Classifier[4]+=FDISD.at(i)*ccwParam[i];
	}
	for (i=0;i<10;i++) {
		Classifier[4]+=FlexorSD.at(i)*ccwParam[i+10];
	}
	for (i=0;i<10;i++) {
		Classifier[4]+=ExtensorSD.at(i)*ccwParam[i+20];
	}

	// build classifier for clockwise circular motion
	for (i=0;i<10;i++) {
		Classifier[5]+=FDISD.at(i)*cwParam[i];
	}
	for (i=0;i<10;i++) {
		Classifier[5]+=FlexorSD.at(i)*cwParam[i+10];
	}
	for (i=0;i<10;i++) {
		Classifier[5]+=ExtensorSD.at(i)*cwParam[i+20];
	}

	// find winner of classification (max value)
	double max=Classifier[0];
	int maxPos=1;

	for (i=1;i<6;i++) {
		if (Classifier[i] > max) {
			max=Classifier[i];
			maxPos=i+1;
		}
	}

	return maxPos;
}

// calculates tick offset for sending next tick to robot
int tickOffsetCalc(int behaviorType, int endOfPeak, int peakWidth) {
	switch (behaviorType) {
		case 1:
			// flex/ex middle of 5th of 5 stdev
			return (5*(int)floor((double)peakWidth/5)+(int)floor((double)0.5*peakWidth/5));
			break;
		case 2:
			// ad/ab start of 5th of 5 stdev
			return (4*(int)floor((double)peakWidth/5));
			break;
		case 3:
			// half curl middle of 1st of 5 stdev
			return ((int)floor((double)peakWidth/5));
			break;
		case 4:
			// full curl middle of 3rd of 5 stdev
			return (3*(int)floor((double)peakWidth/5)+(int)floor((double)0.5*peakWidth/5));
			break;
		case 5:
			// clockwise 5 of 5 stdev
			return (5*(int)floor((double)peakWidth/5));
			break;
		case 6:
			// counterclockwise 3 of 5 stdev
			return (3*(int)floor((double)peakWidth/5));
			break;
		default:
			return (-1);
			break;
	}
}

void printType(int i) {
	switch (i) {
		case 1:
			printf("flex/ext\n");
			break;
		case 2:
			printf("ad/ab\n");
			break;
		case 3:
			printf("half curl\n");
			break;
		case 4:
			printf("full curl\n");
			break;
		case 5:
			printf("clockwise\n");
			break;
		case 6:
			printf("counterclockwise\n");
			break;
	}
}

void main(int argc, char *argv[] ) 
{
	int32       error=0;
	TaskHandle  taskHandle=0;
	int32       totalRead=0;
	float64     data[6];
	char        errBuff[2048]={'\0'};
	char		data_string[100];

	InitializeCriticalSection(&cs);

	butterworth_filter* bfFDI = new butterworth_filter();
	butterworth_filter* bfExtensor = new butterworth_filter();
	butterworth_filter* bfFlexor = new butterworth_filter();

	LARGE_INTEGER counterfreq;
    LARGE_INTEGER ctr1;
    LARGE_INTEGER ctr2;	

	// get the high resolution counter's accuracy
    QueryPerformanceFrequency(&counterfreq);

	WORD wVersionRequested = MAKEWORD(1,1);
	WSADATA wsaData;
	int nRet;
	short nPort=PORT;

	// Initialize WinSock and check the version
	nRet = WSAStartup(wVersionRequested, &wsaData);
	if (wsaData.wVersion != wVersionRequested)
	{
		fprintf(stderr,"\n Wrong version\n");
        return;
	}

	char *szServer=SERVER_ADDRESS;

	// Find the server
	LPHOSTENT lpHostEntry;
	lpHostEntry = gethostbyname(szServer);
	if (lpHostEntry == NULL)
	{
		PRINTERROR("gethostbyname()");
		return;
	}

	// Create a UDP/IP datagram socket
	SOCKET  theSocket;

	theSocket = socket(AF_INET,                     // Address family
                        SOCK_DGRAM,          // Socket type
                        IPPROTO_UDP);        // Protocol
	if (theSocket == INVALID_SOCKET)
	{
		PRINTERROR("socket()");
        return;
	}

	// Fill in the address structure for the server
	SOCKADDR_IN saServer;
	saServer.sin_family = AF_INET;
	saServer.sin_addr = *((LPIN_ADDR)*lpHostEntry->h_addr_list);   // ^ Server's address
	saServer.sin_port = htons(nPort);       // Port number from command line

	char outbuff[150];

	// three data streams that are buffered
	deque<double> FDI;
	deque<double> Extensor;
	deque<double> Flexor;
	deque<double> Time;

	// ofstream Output;
	ifstream Input;
	Input.open("c:/documents and settings/neurobotics/desktop/powera.1.txt");
	//Output.open("c:/documents and settings/neurobotics/desktop/EMG index finger positions 4/tester.txt");
	//Output.open("c:/documents and settings/ralph/desktop/Output.txt");
	
	// step is the basic tick, each time data is read in, step increments
	int step=0;
	// start and end contain ticks where a cycle starts and stops
	// period is time between end and start
	// window is size of deci-windows within a cycle for stdev calc
	int start,end,period,window;
	// startAlreadyFound is bool that is used to determine if
	// we are looking for a start of a cycle or if a start has
	// already been found, in which case, we look for the end of the
	// cycle
	int startAlreadyFound=0;
	int firstCycleFound=0;
	// stdDev is a temporary variable holding the standard deviation
	// of the current window of WINDOW_SIZE
	double tempStdDev;
	// arrays to hold the standard deviations of the deci-windows
	// within a cycle
	deque<double> FDIStd;
	deque<double> ExtensorStd;
	deque<double> FlexorStd; 
	deque<double> StdDev; // stores past 100 calculated standard dev
	deque<double> ExtensorStdDev;
	deque<int> Behavior; // stores past 10 classified behaviors
	double tempRead;
	// prevWindow contains tick of previous window, WINDOW_SIZE for
	// calculation of standard deviations
	int prevWindow=0;
	int t=1;
	int offset=0; // offset indicates "true" index of first item in buffer
	int offsetStdDev=0; // offset indicates "true" index of StdDev buffer
	int endOfPeak; // stores index for end of peak
	int tickOffset;
	double timePeriod;
	double a1=0;
	int needToSendTick=0;
	int stopped=0;
	int modeDetect=1;
	int behaviorsSinceStop=0;

	/*// initialize butterworth filter with 1's
	for (int i=0;i<10;i++) {
		bfFDI->filter(&a1);
		bfFlexor->filter(&a1);
		bfExtensor->filter(&a1);
	}*/

	/* OScope setup */
	HINSTANCE DllInst;
	int Scopes[7];
	int32 readData;
	int (__cdecl  * ScopeAfterOpenLib)(int Prm);
	int (__cdecl  *     ScopeCreate)   (int Prm ,char  *P_IniName, char *P_IniSuffix);
	int (__cdecl  *     ScopeDestroy)  (int Prm);
	int (__cdecl  *     ScopeShow)      (int Prm);
	int (__cdecl  *     ScopeHide)       (int Prm);
	int (__cdecl  *     ScopeShowNext)    (int Prm, struct  tagTArrDbl *PrmD );
	int (__cdecl  *     ScopeQuickUpDate)  (int Prm);
	int (__cdecl * ScopeSetFormPos) (int ScopeHandle, int FormLeft, int FormTop);
	int (__cdecl * ScopeSetFormSize) (int ScopeHandle, int FormWidth, int FormHeight);
	DllInst=LoadLibrary("Osc_DLL.dll");
	ScopeAfterOpenLib  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "AtOpenLib");
	ScopeAfterOpenLib(0);

	ScopeShowNext  = (int (__cdecl  * )(int, struct tagTArrDbl *))GetProcAddress(DllInst, "ShowNext");
	ScopeCreate  = (int (__cdecl  * )(int, char *, char *))GetProcAddress(DllInst, "ScopeCreate");
	ScopeDestroy  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeDestroy");
	ScopeShow  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeShow");
	ScopeHide  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "ScopeHide");
	ScopeQuickUpDate  = (int (__cdecl  * )(int))GetProcAddress(DllInst, "QuickUpDate");

	ScopeSetFormPos = (int (__cdecl  * )(int, int, int))GetProcAddress(DllInst, "ScopeSetFormPos");

	ScopeSetFormSize = (int (__cdecl  * )(int, int, int))GetProcAddress(DllInst, "ScopeSetFormSize");


	Scopes[1] = ScopeCreate(0, osc1Params, pEmpty);
	Scopes[2] = ScopeCreate(0, osc2Params, pEmpty);
	Scopes[3] = ScopeCreate(0, osc3Params, pEmpty);
	Scopes[4] = ScopeCreate(0, osc4Params, pEmpty);

	int scrHt = 1024;
	int scrWd = 1280;

    ScopeShow(Scopes[1]);
    ScopeShow(Scopes[2]);
    ScopeShow(Scopes[3]);
    ScopeShow(Scopes[4]);

	ScopeSetFormPos(Scopes[1],0,scrHt/3); 
	ScopeSetFormPos(Scopes[2],0,2*scrHt/3); 
	ScopeSetFormPos(Scopes[3],scrWd/2,0); 
	ScopeSetFormPos(Scopes[4],scrWd/2,scrHt/3);

	ScopeSetFormSize(Scopes[1],scrWd/2, 300);
	ScopeSetFormSize(Scopes[2],scrWd/2, 300);
	ScopeSetFormSize(Scopes[3],scrWd/2, 300);
	ScopeSetFormSize(Scopes[4],scrWd/2, 600);

	tagTArrDbl scopeIn;
	tagTArrDbl scopeInEMG1;
	tagTArrDbl scopeInEMG2;
	tagTArrDbl scopeInEMG3;
	tagTArrDbl scopeInEMG4;

	double tempRead1;

	/* Init and start NiDAQ tasks */
	(DAQmxCreateTask("",&taskHandle));
	(DAQmxCreateAIVoltageChan(taskHandle,"Dev2/ai0,Dev2/ai2,Dev2/ai3,Dev2/ai4,Dev2/ai1,Dev2/ai7","",DAQmx_Val_Cfg_Default,-10.0,10.0,DAQmx_Val_Volts,NULL));
	(DAQmxStartTask(taskHandle));

	printf("Acquiring samples continuously.  Press any key to interrupt\n");
	ofstream mydata;
	mydata.open("c:/documents and settings/neurobotics/desktop/EMG index finger positions 4/tester.txt", ios::trunc);
	
	QueryPerformanceCounter(&ctr1);
	LARGE_INTEGER startTime=ctr1;

	while( !_kbhit() ) {
	
		QueryPerformanceCounter(&ctr2);
		
		if ((float)(ctr2.QuadPart-ctr1.QuadPart)/(counterfreq.QuadPart) > 0.001) {
			ctr1=ctr2;
			if (step > 4999) offset++;

			if (Time.size() < 5000) 
				Time.push_back((float)(ctr2.QuadPart-startTime.QuadPart)/(counterfreq.QuadPart));
			else {
				Time.pop_front();
				Time.push_back((float)(ctr2.QuadPart-startTime.QuadPart)/(counterfreq.QuadPart));
			}
			
			//Input >> tempRead1; // col 1

			/*if (Time.size() < 5000) 
				Time.push_back(tempRead1);
			else {
				Time.pop_front();
				Time.push_back(tempRead1);
			}*/

			//Input >> tempRead1; // col 2
			//Input >> tempRead1; // col 3
	
			// NiDaq read data
			DAQmxReadAnalogF64(taskHandle,-1,-1,DAQmx_Val_GroupByChannel, data, 6, &readData, NULL);
			
			// fill buffer with current data point
			//Input >> tempRead1; // col 4
			tempRead1=data[2];
			tempRead=bfFDI->filter(&tempRead1);
			tempRead=abs(tempRead);
			if (FDI.size() < 5000) 
				FDI.push_back(tempRead);
			else {
				FDI.pop_front();
				FDI.push_back(tempRead);
			}

			//printf("%0.16f\n",FDI.back());

			//Input >> tempRead1; // col 5
			
			tempRead1=data[3];
			tempRead=bfFlexor->filter(&tempRead1);
			tempRead=abs(tempRead);
			if (Flexor.size() < 5000) 
				Flexor.push_back(tempRead);
			else {
				Flexor.pop_front();
				Flexor.push_back(tempRead);
			}
			
			//Input >> tempRead1; // col 6
			
			tempRead1=data[4];
			tempRead=bfExtensor->filter(&tempRead1);
			tempRead=abs(tempRead);
			if (Extensor.size() < 5000) 
				Extensor.push_back(tempRead);
			else {
				Extensor.pop_front();
				Extensor.push_back(tempRead);
			}
			
			//Input >> tempRead1; // col 7
			
			// if needToSendTick flagged and tickOffset has passed since start
			// of cycle, send info to Robot!

			if ((needToSendTick) && ((step-tickOffset) > start)) {
				needToSendTick=0;
				// ** output classification and period to robot here
				printf("%d %f\n",Behavior.back(),timePeriod);

				if (modeDetect && (Behavior.size() > 7) && (behaviorsSinceStop>=7)) {
					// find mode and return to OpenGL code
					int BehaviorCount[]= {0,0,0,0,0,0};
					modeDetect=0;

					int tempIndex=Behavior.size();

					for (int i=tempIndex-7;i<tempIndex;i++) {
						try {
							if (Behavior.at(i) == 1) BehaviorCount[0]++;
							if (Behavior.at(i) == 2) BehaviorCount[1]++;
							if (Behavior.at(i) == 3) BehaviorCount[2]++;
							if (Behavior.at(i) == 4) BehaviorCount[3]++;
							if (Behavior.at(i) == 5) BehaviorCount[4]++;
							if (Behavior.at(i) == 6) BehaviorCount[5]++;
						}
						catch(out_of_range e)
						{
							cerr << e.what() << endl; // prints "invalid string position"
							printf("mode\n");
						}
					}

					/*for (int i=0;i<6;i++) printf("%d ",BehaviorCount[i]);
					printf("\n");*/

					// find winner of classification (max value)
					double max=BehaviorCount[0];
					int maxPos=1;
					for (int i=0;i<6;i++) {
						if (BehaviorCount[i] > max) {
							max=BehaviorCount[i];
							maxPos=i+1;
						}
					}

					// return maxPos to OpenGL code
					printf("				Mode: %d\n",maxPos);
					setoutput(maxPos);					
				}

				sprintf_s(outbuff,"%d %f",Behavior.back(),timePeriod);
				// for (int i=0;i<5;i++) Output << outbuff << "\n";
				nRet = sendto(theSocket,                                // Socket
							 outbuff,                                      // Data buffer
							 strlen(outbuff),                      // Length of data
							 0,                                            // Flags
							 (LPSOCKADDR)&saServer,        // Server address
							 sizeof(struct sockaddr)); // Length of address
				if (nRet == SOCKET_ERROR)
				{
					PRINTERROR("sendto()");
					closesocket(theSocket);
					return;
				}
				sprintf_s(outbuff,"\0");
			} else { 
				// output zero
				sprintf_s(outbuff,"0 0");
				// Output << outbuff;
				//printf("%s\n",outbuff);
				/*nRet = sendto(theSocket,                                // Socket
									 outbuff,                                      // Data buffer
									 strlen(outbuff),                      // Length of data
									 0,                                            // Flags
									 (LPSOCKADDR)&saServer,        // Server address
									 sizeof(struct sockaddr)); // Length of address
				if (nRet == SOCKET_ERROR)
				{
				   PRINTERROR("sendto()");
				   closesocket(theSocket);
				   return;
				}*/
				sprintf_s(outbuff,"\0");
			}

			// if we have elapsed WINDOW_SIZE since last standard deviation calc
			//calculate standard deviation here for previous window of WINDOW_SIZE
			if ((step - prevWindow) >= WINDOW_SIZE) {
				prevWindow=step;
				// calculate standard dev for FDI
				//printf("%d %d\n",step-WINDOW_SIZE-offset,step-1-offset);
				tempStdDev=calculateStdDev(FDI,(step-WINDOW_SIZE-offset),(step-1-offset));
				if (step > 4999) offsetStdDev++;
				if (StdDev.size() < 100) 
					StdDev.push_back(tempStdDev);
				else {
					StdDev.pop_front();
					StdDev.push_back(tempStdDev);
				}
				//printf("%0.10f\n",StdDev.back());

				// calculate standard dev for Extensor
				tempStdDev=calculateStdDev(Extensor,(step-WINDOW_SIZE-offset),(step-1-offset));
				if (ExtensorStdDev.size() < 100) 
					ExtensorStdDev.push_back(tempStdDev);
				else {
					ExtensorStdDev.pop_front();
					ExtensorStdDev.push_back(tempStdDev);
				}
				
				// check for stop condition, if stopped, -1 to robot!
				if (StdDev.size() > 25) {
					int tempIndex=(int)StdDev.size();
					stopped=1;
					for (int i=tempIndex-25;i<(tempIndex-1);i++) {
						if ((StdDev.at(i)>0.15) || (ExtensorStdDev.at(i)>0.15))
							stopped=0;
					}
					if (stopped) {
						printf("stopped\n");
						setoutput(0);
						behaviorsSinceStop=0;
						modeDetect=1; // need to look for mode once stuff starts up again
						// tell robot to stop
						sprintf_s(outbuff,"-1 -1");
						//for (int i=0;i<5;i++) Output << outbuff << "\n";

						//printf("%s\n",outbuff);
						nRet = sendto(theSocket,                                // Socket
											 outbuff,                                      // Data buffer
											 strlen(outbuff),                      // Length of data
											 0,                                            // Flags
											 (LPSOCKADDR)&saServer,        // Server address
											 sizeof(struct sockaddr)); // Length of address
						if (nRet == SOCKET_ERROR)
						{
						   PRINTERROR("sendto()");
						   closesocket(theSocket);
						   return;
						}
					}
				}

				if ((StdDev.back() > 0.15)&&(!firstCycleFound)) { // we found a beginning of cycle, note time
					start=(step-25);
					startAlreadyFound=1;
					firstCycleFound=1;
					//printf("start of cycle %d\n",start);
				}

				// look for end of peak and find time to send
				if ((StdDev.back() < 0.15)&&(startAlreadyFound)) {
					endOfPeak=step-25;
					//printf("end of peak %d\n",endOfPeak);
					startAlreadyFound=0;
				}

				if ((StdDev.back() > 0.15)&&(firstCycleFound)) { // we found potential end of cycle
					if ((step - start + 1) > 750)  { // jump ahead must be 50 greater than actual jump
						//if (t) {
						//	end=(step-25);
						//	t=0;
						//} else {
						//	end=step;
						//	t=1;
						//}
						end=step-25;
						//printf("end of cycle %d\n",end);
						// calculate standard deviations for deci-windows and classify
						period = end - start + 1; // in index, not time
						

						try {
							timePeriod = Time.at(end-offset-1)-Time.at(start-offset-1);
						}
						catch(out_of_range e)
						{
							cerr << e.what() << endl; // prints "invalid string position"
							printf("%d %d\n",end-offset-1,start-offset-1);
							printf("timePeriod\n");
						}

						//printf("%d %d\n",start-offset, end-offset);
						window = (int) floor(period/10.0);
						
						
						for (int i=0;i<10;i++) {
							//printf("%d %d\n",start+i*window,start+(i+1)*window-1);
							tempRead=calculateStdDev(FDI, start+i*window-offset, start+(i+1)*window-1-offset);
							if (FDIStd.size() < 10) 
								FDIStd.push_back(tempRead);
							else {
								FDIStd.pop_front();
								FDIStd.push_back(tempRead);
							}
							tempRead=calculateStdDev(Flexor, start+i*window-offset, start+(i+1)*window-1-offset);
							if (FlexorStd.size() < 10) 
								FlexorStd.push_back(tempRead);
							else {
								FlexorStd.pop_front();
								FlexorStd.push_back(tempRead);
							}
							tempRead=calculateStdDev(Extensor, start+i*window-offset, start+(i+1)*window-1-offset);
							if (ExtensorStd.size() < 10) 
								ExtensorStd.push_back(tempRead);
							else {
								ExtensorStd.pop_front();
								ExtensorStd.push_back(tempRead);
							}
						}
						//printf("\n");
						// perform EMG classification; 
						int temp=classify(FDIStd, ExtensorStd, FlexorStd);
						if (Behavior.size() < 10) 
							Behavior.push_back(temp);
						else {
							Behavior.pop_front();
							Behavior.push_back(temp);
						}
						behaviorsSinceStop++;
						tickOffset=tickOffsetCalc(temp,endOfPeak,endOfPeak-start);
						needToSendTick=1;
						//printType(temp);
						
						start=end;
						//printf("start of cycle %d\n",start);
						// since new start of cycle = end of old cycle, we set
						// startAlreadyFound to 1
						startAlreadyFound=1;
					}
					// loop for potential end of cycle
				} 
				// loop to calculate running standard deviation if passed 50 ms
			} 
			step++;
			// printf("%d\n",step);
			sprintf(data_string,"%f \t %f \t %f \t %f \t %f \t %f \t %f \n", Time.back(), data[0], data[1], data[2], data[3], data[4], data[5]);
			mydata<<data_string;
			
			scopeIn.s1= data[0];
			scopeInEMG1.s1= data[1];
			scopeInEMG2.s1= data[2];
			scopeInEMG3.s1= data[3];
			scopeInEMG3.s2= data[4];

			ScopeShowNext(Scopes[1],&scopeIn);
			ScopeShowNext(Scopes[2],&scopeInEMG1);
			ScopeShowNext(Scopes[3],&scopeInEMG2);
			ScopeShowNext(Scopes[4],&scopeInEMG3);
		} // out of millisecond elapsed loop
	} // out of while loop

	mydata.close();
	_getch();

	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);
	printf("End of program, press Enter key to quit\n");
	getchar();
	// kill oscilloscope windows
	ScopeDestroy(Scopes[1]);
	ScopeDestroy(Scopes[2]);
	ScopeDestroy(Scopes[3]);
	ScopeDestroy(Scopes[4]);

	FreeLibrary(DllInst);  
	ScopeAfterOpenLib = NULL;
	ScopeShowNext     = NULL;
	ScopeCreate       = NULL;
	ScopeQuickUpDate  = NULL;
	ScopeHide         = NULL;
	ScopeShow         = NULL;
	ScopeDestroy      = NULL;

	// tell server to shut down
	sprintf_s(outbuff,"-11 -11");
	// for (int i=0;i<5;i++) Output << outbuff << "\n";

	// printf("%s\n",outbuff);
	nRet = sendto(theSocket,                                // Socket
                         outbuff,                                      // Data buffer
                         strlen(outbuff),                      // Length of data
                         0,                                            // Flags
                         (LPSOCKADDR)&saServer,        // Server address
                         sizeof(struct sockaddr)); // Length of address
    if (nRet == SOCKET_ERROR)
    {
       PRINTERROR("sendto()");
       closesocket(theSocket);
       return;
    }

	//Output.close();
	Input.close();

	// Release WinSock
	WSACleanup();

	return;
}


void setoutput(int behav) {
    EnterCriticalSection(&cs);

	behavior = behav;
	reset_position();
	    LeaveCriticalSection(&cs);
}