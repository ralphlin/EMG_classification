/*********************************************************************
*
* EMG Classification.cpp
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
*
* Notes:
*	This program was designed to run with the corresponding 
*	main_RSSdemo.cpp behavior on the ACTHand control machine as well
*	as glsample.cpp to provide a on-screen avatar for finger motion
*
*********************************************************************/



#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <math.h>
#include <deque>
#include <string.h>
#include "winsock.h"
#include <windows.h>

// Helper macro for displaying errors
#define PRINTERROR(s)   \
               fprintf(stderr,"\n%: %d\n", s, WSAGetLastError())

// FREQUENCY is the communications frequency in Hz
#define FREQUENCY 1000

// required for UDP socket
#define MAX_COMM_PACKET sizeof(packet)
#define PORT 35
#define SERVER_ADDRESS "192.168.123.173"

// window size for first phase of EMG calculation
#define WINDOW_SIZE 25

// butterworth filter constants
#define GAIN   2.762663641e+08
#define WINDOW  10

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

	if(filled_size < 11) {
		return 0;
	} else {
		filled_window = true;
		yv.pop_front();
		double temp = (xv[0] + xv[10]) + 10 * (xv[1] + xv[9]) + 45 * (xv[2] + xv[8])
                     + 120 * (xv[3] + xv[7]) + 210 * (xv[4] + xv[6]) + 252 * xv[5]
                     + ( -0.1327680842 * yv[0]) + (  1.5942397677 * yv[1])
                     + ( -8.6456821375 * yv[2]) + ( 27.8902991720 * yv[3])
                     + (-59.2809515740 * yv[4]) + ( 86.7670680410 * yv[5])
                     + (-88.5876632510 * yv[6]) + ( 62.3153522820 * yv[7])
                     + (-28.9121945840 * yv[8]) + (  7.9922966624 * yv[9]);
		yv.push_back(temp);
		return temp;
	}
}

/* ---------------------global variables ------------------ */

double flexExtParam[30]={-0.0704185539111908,-0.0509644133209178,
	0.155989577511852,0.0565344845018955,-0.0806830907682781,
	-0.256936062246139,0.131349819636918,0.04013852955827,
	-0.158440500777023,0.175475521144678,0.155565935885208,
	2.39117769394643,-0.288413911444426,-0.253285153627377,
	-0.534768787650682,0.0375714120202933,-0.421026595381913,
	0.344082634814105,-0.2820921321241,0.174051577718529,
	0.131244925234907,-0.661787446516588,0.369687124059408,
	-0.92774826124962,-0.654711627231479,0.410737734006936,
	0.112825420270354,0.778565537162082,0.689821782878359,
	-0.20428422049875};

double adAbParam[30]= {0.448384293839565,0.150770022578572,
	0.122458466944805,0.0355226182878514,-0.0601070030865047,
	-0.344193825536536,0.00347420077023943,0.282571028733013,
	-0.158125480581555,-0.107197228219211,0.0824594650077776,
	-0.646545216911303,-0.785177985873371,1.24809913578239,
	0.174727237027558,-0.724074284035272,-0.237368974462815,
	0.525254408864586,-0.126303068600678,-0.0520946983148471,
	-0.137214733214741,0.148587613070319,0.519697734715248,
	-0.0691068811925909,-0.331227766297592,0.175670540904215,
	0.273818963455834,-0.211942996562835,-0.188745471635689,
	-0.19231800776133};

double halfCurlParam[30]= {0.251937113689973,-0.00389636909242982,
	-0.0471582416731027,-0.142099469604645,-0.0248959237314641,
	-0.120574979700506,0.261710794823815,0.145320788559491,
	0.0834696964038139,-0.0208138704215914,0.349040286472841,
	-0.379521110093216,0.439118485859242,-0.167662982580212,
	1.42127635425993,2.41175770881157,2.17194025465127,
	0.705610177909601,0.0282527658058393,-0.805521870242696,
	-0.50811749985878,0.201351035715312,-0.203466298612285,
	-0.0984289452570868,0.0394930176803006,-0.457701141367486,
	0.152694830042969,-0.126565028761296,0.176165493278056,
	-0.452101757479861};

double fullCurlParam[30]= {-0.0501034673944974,0.0254428161000497,
	-0.158383357632884,-0.111072563302928,0.0939817187219553,
	0.0375633748981856,0.176141851462788,-0.644089202568428,
	0.487427239396993,0.193530788259946,0.394755276720406,
	0.190316241608299,1.7297426196748,0.670647710548357,
	-0.621051099992364,-0.406942090894192,0.00784111647061799,
	-0.552310917426334,-0.49802011732161,1.43369129099849,
	0.410721127112978,0.0455797292233791,-0.826088442584968,
	1.05924302776121,0.601274848055661,0.0928167654798143,
	-0.388031735533321,-0.0931595689481049,-0.23058322304689,
	0.305306623373766};

double cwParam[30]={-0.0924370248753236,0.0179877742300016,
	0.0495877861744192,0.0243150036740003,-0.0946184673836505,
	0.236273742481731,0.221769870681246,-0.175690200948104,
	0.00993540700879069,-0.267505641351568,-0.139382311687104,
	-1.2342468830045,-0.000176812531068541,-0.221143565102234,
	0.149833438516451,-0.920213011436675,-0.68953756344448,
	0.120173000769221,1.35039313040995,0.791010013789936,
	-0.405359234447415,-0.330987616456016,0.82354285436647,
	0.387606266846799,1.1148780901571,0.242818314325177,
	0.578656986239237,-0.102447619627935,-0.188342034290102,
	-0.00392389777709217};

double ccwParam[30]={-0.0159705323689506,-0.0284842178737315,
	0.0391722837612064,0.253674925642582,0.267175987074843,
	0.373727756473034,-0.620692081883824,0.520675250142073,
	-0.402503273087574,0.210313741900515,-0.388930692049727,
	-0.124069489725007,-0.977809254068903,-0.765429938513861,
	0.570175034047645,0.104435661375974,-0.501110877328229,
	-0.277850924962991,0.0424112821718353,-0.743263824504383,
	0.595560848589707,0.81241626359576,0.181863568077012,
	-0.33883185798531,-0.0320689170918146,-0.337877346793402,
	-0.247581435409055,0.158029979381685,0.141776271022238,
	0.736419681125928};

// Function prototype
void DatagramClient(char *szServer, short nPort);

/* calculateStdDev function calculates standard deviation from 
   array[start] to array[end] automatically wraps around buffer 
   if end < start											*/

double calculateStdDev(deque<double> &data, int start, int end) {
	
	double Average=0;
	double sum=0;

	// if end > start, then calculate normally
	if (end > start) {
		for (int i=0;i<(end-start+1);i++) {
			Average+=data.at(start+i);
		}
		//Average=Average/(end-start+1);
		return (Average/(end-start+1));

		/*for (int i=0;i<(end-start+1);i++) {
			sum+=pow(data.at(start+i)-Average,2);
		}
		return sqrt(sum/(end-start)); */
	}
	else { 
		// if start > end, then need to wrap around buffer
		if (start > end) {
			for (int i=0;i<(5000-start+end+1);i++) {
				Average+=data.at((start+i)%5000);
			}
			return(Average/(5000-start+end+1));
			/* for (int i=0;i<(5000-start+end+1);i++) {
				sum+=pow(data.at((start+i)%5000)-Average,2);
			}
			return sqrt(sum/(5000-start+end));*/
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

	// build classifier for clockwise circular motion
	for (i=0;i<10;i++) {
		Classifier[4]+=FDISD.at(i)*cwParam[i];
	}
	for (i=0;i<10;i++) {
		Classifier[4]+=FlexorSD.at(i)*cwParam[i+10];
	}
	for (i=0;i<10;i++) {
		Classifier[4]+=ExtensorSD.at(i)*cwParam[i+20];
	}

	// build classifier for counterclockwise circular motion
	for (i=0;i<10;i++) {
		Classifier[5]+=FDISD.at(i)*ccwParam[i];
	}
	for (i=0;i<10;i++) {
		Classifier[5]+=FlexorSD.at(i)*ccwParam[i+10];
	}
	for (i=0;i<10;i++) {
		Classifier[5]+=ExtensorSD.at(i)*ccwParam[i+20];
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
			// flex/ex middle of 4th of 5 stdev
			return (4*(int)floor((double)peakWidth/5)+(int)floor((double)0.5*peakWidth/5));
			break;
		case 2:
			// ad/ab middle of 5th of 5 stdev
			return (5*(int)floor((double)peakWidth/5)+(int)floor((double)0.5*peakWidth/5));
			break;
		case 3:
			// half curl middle of 1st of 5 stdev
			return ((int)floor((double)peakWidth/5)+(int)floor((double)0.5*peakWidth/5));
			break;
		case 4:
			// full curl middle of 3rd of 5 stdev
			return (3*(int)floor((double)peakWidth/5)+(int)floor((double)0.5*peakWidth/5));
			break;
		case 5:
			// clockwise 4/5 of 5 stdev
			return (4*(int)floor((double)peakWidth/5));
			break;
		case 6:
			// counterclockwise 2/3 of 5 stdev
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

void _tmain(int argc, _TCHAR* argv[])
{
	butterworth_filter* bfFDI = new butterworth_filter();
	butterworth_filter* bfExtensor = new butterworth_filter();
	butterworth_filter* bfFlexor = new butterworth_filter();

	
	WORD wVersionRequested = MAKEWORD(1,1);
	WSADATA wsaData;
	int nRet;
	short nPort=PORT;

	LARGE_INTEGER freq;
    LARGE_INTEGER ctr1;
    LARGE_INTEGER ctr2;
	
	// get the high resolution counter's accuracy
    QueryPerformanceFrequency(&freq);


	//
	// Initialize WinSock and check the version
	//
	nRet = WSAStartup(wVersionRequested, &wsaData);
	if (wsaData.wVersion != wVersionRequested)
	{
		fprintf(stderr,"\n Wrong version\n");
        return;
	}

	char *szServer=SERVER_ADDRESS;

	//
	// Find the server
	//
	LPHOSTENT lpHostEntry;
	lpHostEntry = gethostbyname(szServer);
	if (lpHostEntry == NULL)
	{
		PRINTERROR("gethostbyname()");
		return;
	}

	//
	// Create a UDP/IP datagram socket
	//
	SOCKET  theSocket;

	theSocket = socket(AF_INET,                     // Address family
                                      SOCK_DGRAM,          // Socket type
                                      IPPROTO_UDP);        // Protocol
	if (theSocket == INVALID_SOCKET)
	{
		PRINTERROR("socket()");
        return;
	}

	//
	// Fill in the address structure for the server
	//
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

	ifstream FDI_file;
	ifstream Extensor_file;
	ifstream Flexor_file;
	ifstream Time_file;
	ofstream Output;

	FDI_file.open("rawFDI.txt", ios::in);
	Extensor_file.open("rawExtensor.txt", ios::in);
	Flexor_file.open("rawFlexor.txt", ios::in);
	Time_file.open("Time.txt",ios::in);
	Output.open("Output.txt", ios::out);
	
	// step is the basic tick, each time data is read in, step increments
	//int step=0;
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

	// initialize butterworth filter with 1's
	for (int i=0;i<10;i++) {
		bfFDI->filter(&a1);
		bfFlexor->filter(&a1);
		bfExtensor->filter(&a1);
	}
	
	QueryPerformanceCounter(&ctr1);
	//while (!FDI_file.eof()) {
	for (int step=0;step<5600;step++) {
		QueryPerformanceCounter(&ctr2);
		if (step > 4999) offset++;
		
		
		// fill buffer with current data point
		FDI_file >> tempRead;
		tempRead=bfFDI->filter(&tempRead);
		tempRead=abs(tempRead);
		if (FDI.size() < 5000) 
			FDI.push_back(tempRead);
		else {
			FDI.pop_front();
			FDI.push_back(tempRead);
		}
		Extensor_file >> tempRead;
		tempRead=bfExtensor->filter(&tempRead);
		tempRead=abs(tempRead);
		if (Extensor.size() < 5000) 
			Extensor.push_back(tempRead);
		else {
			Extensor.pop_front();
			Extensor.push_back(tempRead);
		}
		Flexor_file >> tempRead;
		tempRead=bfFlexor->filter(&tempRead);
		tempRead=abs(tempRead);
		if (Flexor.size() < 5000) 
			Flexor.push_back(tempRead);
		else {
			Flexor.pop_front();
			Flexor.push_back(tempRead);
		}
		Time_file >> tempRead;
		if (Time.size() < 5000) 
			Time.push_back(tempRead);
		else {
			Time.pop_front();
			Time.push_back(tempRead);
		}

		// if needToSendTick flagged and tickOffset has passed since start
		// of cycle, send info to Robot!

		if ((needToSendTick) && ((step-tickOffset) > start)) {
			needToSendTick=0;
			// ** output classification and period to robot here
			sprintf_s(outbuff,"%d %f\0",Behavior.back(),timePeriod);
			//for (int i=0;i<5;i++) Output << outbuff << "\n";
			printf("%s\n",outbuff);
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
			sprintf_s(outbuff,"0 0\n");
			// Output << outbuff;
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
			sprintf_s(outbuff,"\0");
		}

		// if we have elapsed WINDOW_SIZE since last standard deviation calc
		//calculate standard deviation here for previous window of WINDOW_SIZE
		if ((step - prevWindow) >= WINDOW_SIZE) {
			prevWindow=step;
			// calculate standard dev for FDI
			tempStdDev=calculateStdDev(FDI,(step-WINDOW_SIZE-offset),(step-1-offset));
			if (step > 4999) offsetStdDev++;
			if (StdDev.size() < 100) 
				StdDev.push_back(tempStdDev);
			else {
				StdDev.pop_front();
				StdDev.push_back(tempStdDev);
			}

			// calculate standard dev for Extensor
			tempStdDev=calculateStdDev(Extensor,(step-WINDOW_SIZE-offset),(step-1-offset));
			if (step > 4999) offsetStdDev++;
			if (ExtensorStdDev.size() < 100) 
				ExtensorStdDev.push_back(tempStdDev);
			else {
				ExtensorStdDev.pop_front();
				ExtensorStdDev.push_back(tempStdDev);
			}
			
			// check for stop condition, if stopped, -1 to robot!
			if (StdDev.size() > 15) {
				int tempIndex=(int)StdDev.size();
				stopped=1;
				for (int i=tempIndex-16;i<(tempIndex-1);i++) {
					if ((StdDev.at(i)>0.1) && (ExtensorStdDev.at(i)>0.1))
						stopped=0;
				}
				if (stopped) {
					// tell robot to stop
					sprintf_s(outbuff,"-1");
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

			if ((StdDev.back() > 0.1)&&(!firstCycleFound)) { // we found a beginning of cycle, note time
				start=(step-25);
				startAlreadyFound=1;
				firstCycleFound=1;
				printf("start of peak %d\n",start);
			}

			// look for end of peak and find time to send
			if ((StdDev.back() < 0.1)&&(startAlreadyFound)) {
				endOfPeak=step-25;
				printf("end of peak %d\n",endOfPeak);
				startAlreadyFound=0;
			}

			if ((StdDev.back() > 0.1)&&(firstCycleFound)) { // we found potential end of cycle
				if ((step - start + 1) > 450)  { // jump ahead must be 50 greater than actual jump
					/*if (t) {
						end=(step-25);
						t=0;
					} else {
						end=step;
						t=1;
					}*/
					end=step-25;
					printf("end of cycle %d\n",end);
					// calculate standard deviations for deci-windows and classify
					period = end - start + 1; // in index, not time
					timePeriod = Time.at(end-offset)-Time.at(start-offset);
					printf("%d %d\n",start-offset, end-offset);
					window = (int) floor(period/10.0);
					for (int i=0;i<10;i++) {
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
					// perform EMG classification; 
					int temp=classify(FDIStd, ExtensorStd, FlexorStd);
					if (Behavior.size() < 10) 
						Behavior.push_back(temp);
					else {
						Behavior.pop_front();
						Behavior.push_back(temp);
					}
					tickOffset=tickOffsetCalc(temp,endOfPeak,endOfPeak-start);
					needToSendTick=1;
					//printType(temp);
					
					start=end;
					printf("start of peak %d\n",start);
					// since new start of cycle = end of old cycle, we set
					// startAlreadyFound to 1
					startAlreadyFound=1;
				}
				// loop for potential end of cycle
			}
			// loop to calculate running standard deviation if passed 50 ms
		} 
		Output << ((float)(ctr2.QuadPart-ctr1.QuadPart)/(freq.QuadPart)) << "\n";
		// while read
		//step++;
	}
	// out of while loop

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

	FDI_file.close();
	Extensor_file.close();
	Flexor_file.close();
	Output.close();

	//
	// Release WinSock
	//
	WSACleanup();
}

