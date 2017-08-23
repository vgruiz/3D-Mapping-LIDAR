#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <vector>

using namespace std;

const double PI = 3.141592653589793238463;

int LineCount(ifstream&);
double ConvertToRadians(double);

int main()
{
	double y_offset = -40 * PI / 180;

	//Opens the files to read from/write to.
	//ifstream lidarIFS("LIDARshort.txt");
	ifstream lidarIFS("LIDAR_8_19.txt.");
	//ifstream imuIFS("IMUshort.txt");
	ifstream imuIFS("IMU_8_19.txt");
	ofstream ptCloudOFS("pt_cloud_xyz.txt");
	ofstream lidarXYZ("lidarXYZ.txt");

	//Counts how many lines are in the file, deletes every 13th line(which is the timestamp) and
	int nLidarLines = LineCount(lidarIFS);
	nLidarLines = (nLidarLines - floor(nLidarLines / 13)) * 2;	//multiplies it by two because each line in the text file takes up two lines in the matrix.
	int nGpsLines = (nLidarLines - (12 * floor(nLidarLines / 13)));
	int nImuLines = LineCount(imuIFS);

	//Resets the the file stream pointer to the beginning of the file.
	lidarIFS.clear();	
	lidarIFS.seekg(0);
	imuIFS.clear();
	imuIFS.seekg(0);

#pragma region ARRAY DECLARATION
	cout << "Creating matrices..." << endl;

	double** lidarData = new double*[nLidarLines];
	for (int i = 0; i < nLidarLines; i++)
	{
		lidarData[i] = new double[50];
	}

	string** lidarGPS = new string*[nGpsLines];
	for (int i = 0; i < nGpsLines; i++)
	{
		lidarGPS[i] = new string[13];
	}

	double** imuData = new double*[nImuLines];
	for (int i = 0; i < nImuLines; i++)
	{
		imuData[i] = new double[11];
	}

	//Angle matrix initialization and conversion to radians.
	//Angles of the 16 individual lasers are provided by Velodyne documentation.
	double laserAngle[16] = { 105, 89, 103, 93, 101, 85, 99, 83, 97, 81, 95, 79, 93, 77, 91, 75 };
	for (int ctr = 0; ctr < 16; ctr++)
	{
		laserAngle[ctr] = laserAngle[ctr] * PI / 180; //conversion to radians
	}
#pragma endregion

#pragma region VARIABLES FOR DATA INPUT
	const string ANGLE_DET = "angle=";	//"angle="
	const string TIME_DET = "time=";	//"time="
	const string GPS_DET = "GPS=";		//"GPS="

	string cur;			//Stores a line from the LIDAR text file. It is replaced with the following line during every looping of the while loop.
	int row = 0;		//Row value for the lidarData two-dimensional array.
	int col = 0;		//Column value "										".
	int gRow = 0;		//Row value for the lidarGPS two-dimensional array.
	int charPos = 0;	//Modified throughout the program to know where to start reading data.
	int curTime = 0;	//Stores the value of the most recently encountered LIDAR time value.
	int gpsTime = 0;	//Stores the value of the most recently encountered GPS time stamp, as identified by the VLP-16 documentation.
#pragma endregion

#pragma region VARIABLES FOR GEOREFERENCING MATH
	double lat0 = 0;		//will store the initial latitude value
	double lon0 = 0;		//will store the initial longitude value
	double alt0 = 0;		//will store the initial altitude value
	double imuTime0 = 0;	//will store the initial IMU time stamp
	double lidarTime0 = 0;	//will store the initial LIDAR time stamp

	double lat = 0;
	double lon = 0;
	double alt = 0;
	double roll = 0;
	double pitch = 0;
	double yaw = 0;

	double latLength = 0;
	double lonLength = 0;

	double latOffset = 0;
	double lonOffset = 0;

	double imuTimeA = 0;	//IMU time stamp A for time stamp synchronization
	double imuTimeB = 0;	//IMU time stamp B for time stamp synchronization
	double lidarTime = 0;	//LIDAR time stamp for time stamp synchronization
	int imuRowSelect = 0;	//will store the row number for which IMU data values to do the georef math with
	int lRow = 0;			//for traversing LIDAR matrix rows
	int lCol = 3;			//for traversing LIDAR matrix columns

	double alpha = 0;
	double distance = 0;
	double timeStamp = 0;
	double omega = 0;
	double X = 0;
	double Y = 0;
	double Z = 0;
	int lzr = 0;
#pragma endregion

	cout << "Processing LIDAR data..." << endl;
	while (getline(lidarIFS, cur))
	{
		//Seeks angle_det at the beginning of a line, stores angle value and the following distance and reflectivity points.
		//Interpolates missing angle values, as described in the VLP-16 documentation
		if (cur.substr(0, 6) == ANGLE_DET)
		{
			lidarData[row][col] = stod(cur.substr(6, 11)); //getting the angle value

			int cursor = 0;

			for (int i = 1; i < 96; i++)
			{
				col++;

				if (i == 49) //this indicates that it is at the end of one sequence of lazer firings
				{
					row++;	//go down one row for the next set of distance+reflectivity values
					col = 1;	//we will not go to 0 because that is where the interpolated angle value will be stored

					//azimuth interpolation
					if (row - 3 >= 0)	//checking to avoid access violation error or index out of bound error
					{
						double azi1 = lidarData[row - 3][0];
						double azi3 = lidarData[row - 1][0];

						if (azi3 < azi1)
						{
							azi3 = azi3 + 36000;
						}

						double azi2 = (azi1 + azi3) / 2;

						if (azi2 > 35999)	//accounting for rollover. values are not to exceed 35999.
						{
							azi2 = azi2 - 36000;
						}

						lidarData[row - 2][0] = azi2; //assign the missing angle value with the interpolated one
					}
				}
				
				if (i % 3 != 0) //This is to avoid any column that is reserved for time stamps. See the lidarData Matrix Organization spreadsheet
				{
					charPos = 18 + (11 * (cursor));	//to move through the text file value by value, which are 11 characters apart
					lidarData[row][col] = stod(cur.substr(charPos, 11)); //getting distance value
					cursor++;
				}
			}
			row++;
			col = 0; //reset this to 0. when it reads an angle value next, col will be set to the first column
		}

		//Seeks time_det at the beginning of a line, stores the time value and calculates the exact time for each data point, as
		//described in the VLP-16 documentation
		if (cur.substr(0, 5) == TIME_DET)
		{
			//cout << "time detected" << endl;
			curTime = stod(cur.substr(5, 11));

			for (int i = 23; i > -1; i--)
			{
				lidarData[row - 24 + i][49] = curTime;

				for (int j = 1; j < 17; j++)
				{ 
					int sequence_index = i;
					int data_pt_index = j - 1;

					double exact_time = curTime + (55.296 * sequence_index) + (2.304 * data_pt_index);
					lidarData[row - 24 + i][j * 3] = exact_time;
				}
			}
		}

		//Seeks GPS_DET at the beginning of a line, stores the entire GPS sentence in a string matrix with each row being it's
		//own sentence. Details are in the VLP-16 documentation and Matrix Organization spreadsheet
		if (cur.substr(0, 4) == GPS_DET)
		{
			if (cur.substr(0, 8) != "GPS= $GP") //this conditional is to avoid an exception from being thrown when the lidar capture code has a typo in the GPS line
			{
				//TODO: have this continue to gather the GPS data after the system typo
				//TODO: turn this into a try-catch block
				cout << "GPS ERROR" << endl;
				break;
			}
			gpsTime = stod(cur.substr(12, 6));

			lidarGPS[gRow][0] = cur.substr(12, 6);	//GPS time
			lidarGPS[gRow][1] = cur.substr(19, 1);	//Validity, A or V
			lidarGPS[gRow][2] = cur.substr(21, 9);	//Current Latitude
			lidarGPS[gRow][3] = cur.substr(31, 1);	//N or S
			lidarGPS[gRow][4] = cur.substr(33, 10);	//Current Longitude
			lidarGPS[gRow][5] = cur.substr(44, 1);	//E or W
			lidarGPS[gRow][6] = cur.substr(46, 5);	//Speed in knots
			lidarGPS[gRow][7] = cur.substr(52, 5);	//True course
			lidarGPS[gRow][8] = cur.substr(58, 6);	//Date Stamp
			lidarGPS[gRow][9] = cur.substr(65, 5);	//Variation
			lidarGPS[gRow][10] = cur.substr(71, 1);	//E or W
			lidarGPS[gRow][11] = cur.substr(73, 4);	//checksum
			lidarGPS[gRow][12] = curTime;			//timestamp from LIDAR

			gRow++;

		}

	}

	//reset for the next while loop that takes in the IMU data
	row = 0;
	col = 0;

	cout << "Processing IMU data..." << endl;
	while (getline(imuIFS, cur))
	{
		//cout << "gathering IMU data into matrix" << endl;
		imuData[row][0] = stod(cur.substr(0, 15));	//latitude
		imuData[row][1] = stod(cur.substr(16, 15));	//longitude
		imuData[row][2] = stod(cur.substr(31, 15));	//altitude
		imuData[row][3] = stod(cur.substr(46, 15)); //w
		imuData[row][4] = stod(cur.substr(61, 15)); //x
		imuData[row][5] = stod(cur.substr(76, 15)); //y
		imuData[row][6] = stod(cur.substr(91, 15)); //z
		imuData[row][7] = stod(cur.substr(106, 15)); //roll
		imuData[row][8] = stod(cur.substr(121, 15)); //pitch
		imuData[row][9] = stod(cur.substr(136, 15)); //yaw
		imuData[row][10] = stod(cur.substr(151, 21)); //time stamp

		//cout << "imu time: " << fixed << imuData[row][10] << endl;

		row++;
	}

#pragma region GEOREFERENCING MATH

	lat0 = ConvertToRadians(imuData[0][0]);	//Initial latitude taken from the beginning of the data capture
	lon0 = ConvertToRadians(imuData[0][1]);	//Initial longitude taken from the beginning of the data capture
	alt0 = ConvertToRadians(imuData[0][2]);	//Initial altitude taken from the beginning of the data capture
	imuTime0 = imuData[0][10];				//Initial time value from the start of the IMU data capture. The unit is milliseconds since the Epoch.
	lidarTime0 = lidarData[0][3];			//Initial time value from the start of the LIDAR data capture. The unit is microseconds past the hour.


	cout << "Start georeferencing math..." << endl;
	
	for (int imuRow = 0; imuRow < nImuLines; imuRow++)
	{
		if (imuRow + 1 >= nImuLines || lRow + 1 >= nLidarLines) { cout << "IOOB SAVE" << endl; break; } //prevents loop from throwing an index oob error
		
		//updating the time values to be compared
		imuTimeA = ( imuData[imuRow][10] - imuTime0 ) / 1000;
		imuTimeB = ( imuData[imuRow + 1][10] - imuTime0 ) / 1000;
		lidarTime = ( lidarData[lRow][lCol] - lidarTime0 ) / 1000000;


		while (lidarTime <= imuTimeA) //go to next lidarTime until it's greater than imuTimeA
		{	
			lCol = lCol + 3;	//The next data point's timestamp is three columns away. Refer to the Matrix organization document
			
			if (lCol > 48) //lCol has reached the end of the row
			{ 
				lRow++; 
				lCol = 3; 
			}

			lidarTime = (lidarData[lRow][lCol] - lidarTime0) / 1000000;	//update lidarTime
		}

		while (lidarTime > imuTimeA && lidarTime <= imuTimeB)		//while the lidarTime is between the two imu ts, keep incrementing through lidarTime
		{
		 	if ( (lidarTime - imuTimeA) <= (imuTimeB - lidarTime) ) //if the lidarTime is closer to imu A than imu B
			{
				imuRowSelect = imuRow; //use imuTimeA
			}
			else if ( (lidarTime - imuTimeA) >= (imuTimeB - lidarTime) )
			{
				imuRowSelect = imuRow + 1;	//use imuTimeB
			}
			
			//begin pt cloud math
			lat = imuData[imuRowSelect][0];
			lon = imuData[imuRowSelect][1];
			alt = imuData[imuRowSelect][2];
			roll = ConvertToRadians( imuData[imuRowSelect][7] );
			pitch = ConvertToRadians( imuData[imuRowSelect][8] );
			yaw = ConvertToRadians( imuData[imuRowSelect][9] );
							
			lat = lat - lat0;
			lon = lon - lon0;
			alt = alt - alt0;

			lat = lat / 60;
			lon = lon / 60 * -1;
			alt = alt / 1000; //convert m to mm

			latLength = 111321.5432;
			lonLength = 111321.5432 * cos(lat * PI / 180);

			latOffset = lat * latLength * 1000;
			lonOffset = lon * lonLength * 1000;

			alpha = lidarData[lRow][0] / 100;
			alpha = 90 - alpha;				 		//this is the math I found in Michael's code and I need to try it
			alpha = alpha * PI / 180;
			distance = lidarData[lRow][lCol - 2];
			timeStamp = lidarData[lRow][lCol];
			lzr = (lCol / 3) - 1;
			omega = laserAngle[lzr];

			X = distance * sin(omega) * cos(alpha);
			Y = distance * sin(omega) * sin(alpha);
			Z = distance * cos(omega);


			//if (!(X == 0 && Y == 0 && Z == 0))
			//{
			//	lidarXYZ << X << " " << Y << " " << Z << " " << fixed << timeStamp << !fixed << endl;
			//}

			//X transform (pitch + y_offset)
			Y = Y * cos(pitch + y_offset) - Z * sin(pitch + y_offset);
			Z = Y * sin(pitch + y_offset) + Z * cos(pitch + y_offset);

			//Y transform (roll)
			X = X * cos(roll) - Z * sin(roll);
			Z = -X * sin(roll) + Z * cos(roll);

			//Z transform ( yaw )
			X = X * cos(yaw) - Y * sin(yaw) + lonOffset;
			Y = X * sin(yaw) + Y * cos(yaw) - latOffset;
			Z = Z + alt;

			ptCloudOFS << setprecision(6) << X << " " << Y << " " << Z << " " << endl;
			//end pt cloud math


			//increment lidarTime here
			lCol = lCol + 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
			if (lCol > 48) { lRow++; lCol = 3; }
			 
			lidarTime = lidarData[lRow][lCol];
			lidarTime = (lidarTime - lidarTime0) / 1000000;
		}
	}

#pragma endregion
	
	//
	////xyz point math and formatting
	//	for (int i = 0; i < lineVal; i++)
	//	{
	//		double alpha = lidarData[i][0] / 100;
	//		alpha = 90 - alpha;						//this is the math I found in Michael's code and I need to try it
	//		alpha = alpha * PI / 180;
	//		//cout << alpha << endl;
	//		double omega = 0;
	//		double distance = 0;
	//		double gpsTime = 0;
	//		double lidar_time = 0;
	//		int lzr = 0;

	//		
	//		for (int lCol = 1; lCol < 49; lCol++)
	//		{
	//			if (lCol % 3 == 1)	//if it's remainder is 1, then it is in a distance column
	//			{
	//				omega = laserAngle[lzr];
	//				lzr++;

	//				distance = lidarData[i][lCol]; //in mm
	//				//cout << distance << endl;
	//			}

	//			if (lCol % 3 == 0) //if it's remainder is 0, then it is in a time column
	//			{
	//				lidar_time = lidarData[i][lCol];

	//				double X = distance * sin(omega) * cos(alpha);
	//				double Y = distance * sin(omega) * sin(alpha);
	//				double Z = distance * cos(omega);

	//				//ptCloud << setw(15) << right << X << setw(15) << Y << setw(15) << Z << endl;
	//				if (lidar_time > 0 && X != 0 && Y != 0 && Z != 0)
	//				{
	//					ptCloud << X << " " << Y << " " << Z << " " << setprecision(14) << lidar_time << setprecision(6) << endl;
	//				}
	//			}
	//		}
	//	}

	imuIFS.close();
	lidarIFS.close();
	ptCloudOFS.close();
	return 0;
}

int LineCount(ifstream& file)
{
	int dot_count = 1;
	int n = 0;
	string s;

	while (getline(file, s))
	{
		n++;
	}

	cout << "Lines counted: " << n << endl;

	return n;
}

double ConvertToRadians(double angle)
{
	return angle * PI / 180;
}