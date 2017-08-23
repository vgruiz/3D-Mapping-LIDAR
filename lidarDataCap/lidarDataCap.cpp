/*
* Copyright (c) 1999 - 2005 NetGroup, Politecnico di Torino (Italy)
* Copyright (c) 2005 - 2006 CACE Technologies, Davis (California)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the Politecnico di Torino, CACE Technologies
* nor the names of its contributors may be used to endorse or promote
* products derived from this software without specific prior written
* permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


//Testing
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <pcap.h>
#include <fstream>
#include <cmath>
#include <cstring>
#include <time.h>

using namespace std;

#define LINE_LEN 16

#pragma region "GLOBAL VARIABLES"
/*used to store hex values in the two hex conversion functions*/
int hex0 = 0;
int hex1 = 0;
int hex2 = 0;
int hex3 = 0;
int hex4 = 0;
int hex5 = 0;
int hex6 = 0;
int hex7 = 0;
/*describe this at some point lol*/
int a = 0;
#pragma endregion

#pragma region "FUNCTION PROTOTYPES"
/*takes in the 2 byte values for the azimuth or distance value and returns the calculated value as an integer*/
int TwoByteHexConv(int);
/*takes in the 4 byte values for the time stamp and returns the calculated value as an integer*/
int FourByteHexConv(int);
#pragma endregion


int main(int argc, char **argv)
{
	/*VARIABLES*/
	int curByte = 0;	//the current byte being processed
	int nextByte = 0;	//used in conjunction with curByte
	int blockCounter = 0;	//counter for number of data blocks counted in a packet
	int dataBlockStatus = 0;	//used to facilitate
	/*stores the azimuth value for current block being processed*/
	int azimuth = 0;
	/*temporarily stores the distance value for the current block being processed. Each of the 32 values per block get printed immediately.*/
	int distance = 0;
	/*counter for the number of distance and reflectivity data points processed.*/
	int ctr = 0;
	/*stores the time stamp*/
	int timeStamp = 0;
	bool flag = false;
	int reflFlag = 0;
	int gpsFlag = 0;
	int gpsHeader = false;
	int gpsByte = 0;
	int wait = 0; //seconds before start
	string cur;

#pragma region "PACKET CAPTURE CODE FROM WINPCAP"
	pcap_if_t *alldevs, *d;
	pcap_t *fp;
	u_int inum, i = 0;
	char errbuf[PCAP_ERRBUF_SIZE];
	int res;
	struct pcap_pkthdr *header;
	const u_char *pkt_data;

	printf("pktdump_ex: prints the packets of the network using WinPcap.\n");
	printf("   Usage: pktdump_ex [-s source]\n\n"
		"   Examples:\n"
		"      pktdump_ex -s file://c:/temp/file.acp\n"
		"      pktdump_ex -s rpcap://\\Device\\NPF_{C8736017-F3C3-4373-94AC-9A34B7DAD998}\n\n");

	if (argc < 3)
	{

		printf("\nNo adapter selected: printing the device list:\n");
		/* The user didn't provide a packet source: Retrieve the local device list */
		if (pcap_findalldevs_ex(PCAP_SRC_IF_STRING, NULL, &alldevs, errbuf) == -1)
		{
			fprintf(stderr, "Error in pcap_findalldevs_ex: %s\n", errbuf);
			return -1;
		}

		/* Print the list */
		for (d = alldevs; d; d = d->next)
		{
			printf("%d. %s\n    ", ++i, d->name);

			if (d->description)
				printf(" (%s)\n", d->description);
			else
				printf(" (No description available)\n");
		}

		if (i == 0)
		{
			fprintf(stderr, "No interfaces found! Exiting.\n");
			return -1;
		}

		printf("Enter the interface number (1-%d):", i);
		scanf_s("%d", &inum);

		if (inum < 1 || inum > i)
		{
			printf("\nInterface number out of range.\n");

			/* Free the device list */
			pcap_freealldevs(alldevs);
			return -1;
		}

		/* Jump to the selected adapter */
		for (d = alldevs, i = 0; i< inum - 1; d = d->next, i++);

		/* Open the device */
		if ((fp = pcap_open(d->name,
			100 /*snaplen*/,
			PCAP_OPENFLAG_PROMISCUOUS /*flags*/,
			20 /*read timeout*/,
			NULL /* remote authentication */,
			errbuf)
			) == NULL)
		{
			fprintf(stderr, "\nError opening adapter\n");
			return -1;
		}
	}
	else
	{
		// Do not check for the switch type ('-s')
		if ((fp = pcap_open(argv[2],
			100 /*snaplen*/,
			PCAP_OPENFLAG_PROMISCUOUS /*flags*/,
			20 /*read timeout*/,
			NULL /* remote authentication */,
			errbuf)
			) == NULL)
		{
			fprintf(stderr, "\nError opening source: %s\n", errbuf);
			return -1;
		}
	}
#pragma endregion

	/*Declaration and initialization of the output file that we will be writing to and the input file we will be reading settings from.*/
	ofstream capFile("LIDAR_data.txt");
	ifstream settings("settings.txt");

	getline(settings, cur);
	wait = stoi(cur.substr(0, 3));

	clock_t start = clock();
	double duration = 0;

	while (duration < wait)
	{
		duration = (clock() - start) / (double)CLOCKS_PER_SEC;
		cout << "waiting" << endl;
		cout << wait - duration << endl;
	}

	system("start IMUcap.exe"); 
	  


	/*while */
	while ((res = pcap_next_ex(fp, &header, &pkt_data)) >= 0)
	{
		if (res == 0) //if there is a timeout, continue to the next loop
			continue;


		for (int i = 1; i < (header->caplen + 1); i++)	//this loop is just slightly different from Asher's as he started from i = 1 instead of 0.
		{

			curByte = pkt_data[i - 1];
			nextByte = pkt_data[i];


			switch (dataBlockStatus) {
			case 0:	//0xFFEE has not been found, GPS sentence has not been found
				if (curByte == 255 && nextByte == 238)	//detects 0xFFEE
				{
					dataBlockStatus = 1;
					blockCounter++;
				}
				
				if (curByte == 36 && nextByte == 71)	//detects start of GPS sentence, "$G"
				{
					dataBlockStatus = 4;
				}
				break;
			case 1: //0xFFEE has been found, begin reading and calculating azimuth value
				if (!flag)
				{
					/*the purpose of this if statement is to skip one iteration of the for loop. in the previous loop, nextByte 
					was used to identify the block flag. In the loop after, that byte became curByte and the azimuth calculation
					begins at the byte AFTER that one. hopefully that made sense.*/
					flag = true;
				}
				else
				{
					azimuth = TwoByteHexConv(curByte);

					if (azimuth != -1)
					{
						capFile << endl << "angle= " << setw(10) << azimuth << " ";
						dataBlockStatus = 2;
					}
				}
				break;
			case 2:	//Azimuth value has been read. Now process the next 32 3-byte data points.
				flag = false;
				
				ctr++;	//keeps track of how many bytes have been read within this switch case. 
						//3 bytes per data point * 32 data points = 96 bytes total. this will be used for the logic.

				if (ctr % 3 != 0)
				{
					distance = 2 * TwoByteHexConv(curByte); //multiplied by 2 because the precision is down to 2 millimeters
					if (distance > -1)
					{
						capFile << " " << setw(10) << distance;
					}
					distance = -1; 
				}
				else
				{
					capFile << " " << setw(10) << curByte; //reflectivity value
				}

				if (ctr == 96)
				{
					//TODO::convert to if-else statement
					switch (blockCounter)
					{
					case 12: 
						dataBlockStatus = 3;
						ctr = 0;
						break;
					default:
						dataBlockStatus = 0;
						ctr = 0;
						break;
					}
				}
				break;
			case 3:	//all 12 blocks in this packet have been read, now process the timestamp and reset dataBlockStatus
				timeStamp = FourByteHexConv(curByte);

				if (timeStamp != -1) 
				{
					capFile << endl << "time= " << timeStamp;
					dataBlockStatus = 0;
					blockCounter = 0;
				}
				break;
			case 4:	//Read and immediately print the GPS sentence to the output
				int cB = curByte;
				cout << endl;

				if (!gpsHeader) {
					capFile << "GPS= $G" << flush;
					gpsHeader = true;
					gpsByte = 84;
				}
				else
				{
					if (gpsByte > 0)
					{
						capFile << static_cast<char>(cB) << flush;
						gpsByte--;
					}
					else
					{
						gpsHeader = false;
						dataBlockStatus = 0;
					}
				}

				break;
			}
		}
	}

	capFile.close();
	return 0;
}



int TwoByteHexConv(int hexVal)
{
	int val = 0;

	switch (a) 
	{
	case 0:
		//cout << hexVal << " : ";
		hex1 = floor (hexVal / 16);
		//cout << hex1;
		hex0 = hexVal - (hex1 * 16);
		//cout << "  " << hex0 << endl;
		a++; 
		break;
	case 1: 
		//cout << hexVal << " : ";
		hex3 = floor (hexVal / 16);
		//cout << hex3;
		hex2 = hexVal - (hex3 * 16);
		//cout << "  " << hex2 << endl;

		val = (hex3 * pow(16, 3)) + (hex2 * pow(16, 2)) + (hex1 * 16) + hex0;
		hex0 = hex1 = hex2 = hex3 = 0;
		a = 0;
		return val;
	}
	return -1;
}

int FourByteHexConv(int hexVal)
{
	int val = 0;

	switch (a)
	{
	case 0:
		//cout << "case 0; " << hexVal << endl;
		hex1 = floor(hexVal / 16);
		hex0 = hexVal - (hex1 * 16);
		// << "   hex1, hex0: " << hex1 << ", " << hex0 << endl;
		a++;
		break;
	case 1:
		//cout << "case 1; " << hexVal << endl;
		hex3 = floor(hexVal / 16);
		hex2 = hexVal - (hex3 * 16);
		//cout << "   hex3, hex2: " << hex3 << ", " << hex2 << endl;
		a++;
		break;
	case 2:
		//cout << "case 2; " << hexVal << endl;
		hex5 = floor(hexVal / 16);
		hex4 = hexVal - (hex5 * 16);
		//cout << "   hex5, hex4: " << hex5 << ", " << hex4 << endl;
		a++;
		break;
	case 3:
		//cout << "case 3; " << hexVal << endl;
		hex7 = floor(hexVal / 16);
		hex6 = hexVal - (hex7 * 16);
		//cout << "   hex7, hex6: " << hex7 << ", " << hex6 << endl;
		a++;
		break;
	case 4:
		//cout << "CALCULATING;" << endl << endl;
		//cout << hex7 << ", " << hex6 << ", " << hex5 << ", " << hex4 << ", " << hex3 << ", " << hex2 << ", " << hex1 << ", " << hex0 << endl;
		val = (hex7 * pow(16, 7)) + (hex6 * pow(16, 6)) + (hex5 * pow(16, 5)) + (hex4 * pow(16, 4)) 
				+ (hex3 * pow(16, 3)) + (hex2 * pow(16, 2)) + (hex1 * 16) + hex0;
		hex7 = hex6 = hex5 = hex4 = hex3 = hex2 = hex1 = hex0 = 0;
		a = 0;
		return val;
	}

	return -1;
}