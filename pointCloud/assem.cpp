#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib> strtol(s.c_str(),0,10);

using namespace std;


int main()
{
#pragma region "DETECTION STRINGS"
	string TEST;
	string IMU_Line;
	/*"angle"*/
	string detect_angle = "angle";
	/*"time "*/
	string detect_time = "time ";
	/*"GPS ="*/
	string detect_gps = "GPS  =";
	/*" "*/
	string testspace = " ";
	/*"."*/
	string detect_period = ".";
	/*","*/
	string detect_comma = ",";
	/*"S"*/
	string detect_south = "S";
	/*"W"*/
	string detect_west = "W";
	/*"GPS"*/
	string detect_GPS_IMU = "GPS";
	/*"ATT"*/
	string detect_ATT_IMU = "ATT";
	/*"BARO"*/
	string detect_BARO_IMU = "BARO";
#pragma endregion


	int arb = 6;
	double lidar_time = 0;
	double t_new = 0;
	/* 1 = detected "time " */
	int time_check = 0;
	int time_count = 0;
	int anglestart = 0;
	int anglelength = 0;
	int distancestart = 0;
	int distancelength = 0;
	int reflectivitystart = 0;
	int reflectivitylength = 0;
	int linevar = 0;


	// laser matrix (degrees)
	double laserangle[16] = { 105, 89, 103, 93, 101, 85, 99, 83, 97, 81, 95, 79, 93, 77, 91, 75 };

	double angle;
	double distance;
	int reflectivity;
	int laseranglecounter = 0;
	int angle_deg = 0;
	double x;
	double y;
	double z;
	double x1;
	double y1;
	double z1;
	double pi = atan(1) * 4;

	int column1spacing;
	int column2spacing;
	int column3spacing;
	int column4spacing;
	int column5spacing;
	int stringcounter = 0;

	int placeholder = 1;



	// convert laser matrix to radians
	int laserangleconversioncounter = 0;
	while (laserangleconversioncounter < 16) {
		laserangle[laserangleconversioncounter] = laserangle[laserangleconversioncounter] * pi / 180;
		laserangleconversioncounter++;
	}

	double lat_length;
	double long_length;
	double gps_lat_zero;
	double gps_long_zero;
	double gps_lat_offset;
	double gps_long_offset;
	double gps_angle = 0;
	double lat_deg = 0;
	double lat_min = 0;
	double long_deg = 0;
	double long_min = 0;
	double yaw = 0;
	double roll = 0;
	double pitch = 0;
	double IMU_time = 0;
	double IMU_alt = 0;
	int gps_set = 0;
	int gps_zero = 0;
	double y_offset = 0;
	int IMU_Hold = 0;
	int IMU_Scan = 0;
	int IMU_Start = 0;
	int IMU_Length = 0;
	int comma_count = 0;
	int skip = 0;
	int skip_required = 1;

	ofstream outfile;
	outfile.open("Lidar_Data_XYZATI.txt");


	// open file
	ifstream infile;
	infile.open("Flight_Lidar_Data.txt");
	ifstream imufile;
	imufile.open("IMU_Log.txt");

	//Opening, ask for offsets and program warnings
	cout << "This program uses the Flight_Lidar_Data.txt and IMU_Log.txt files to create the point cloud file Lidar_Data_XYZATI.txt\nDo not use data that goes through an hour change\nGPS is set for US West area\n\n";
	cout << "Please enter your angle offsets,downwards tilt is positive";
	cout << "\nEnter Y offset in deg\n";
	cin >> y_offset;
	y_offset = (-y_offset * (pi / 180));
	cout << "\n" << y_offset << "\n";

	for (int n = 9000000; n > 0; n--)
	{

		getline(infile, TEST);

		//Set time value to current(last seen) value
		if (TEST.substr(0, 5) == detect_time.substr(0, 5))
		{
			t_new = std::stod(TEST.substr(6, 15));
			lidar_time = t_new / 1000000;
			time_check = 1;
		}

		//Get the IMU yaw, pitch, and roll.
		//Advance through IMU file until time matches then use last found values of imu
		if (time_check == 1)
		{
			while (IMU_Hold == 0)
			{
				getline(imufile, IMU_Line);

				if (IMU_Line.substr(0, 3) == detect_GPS_IMU.substr(0, 3)) //if you find "GPS " in the first three characters of the IMU file
				{
					while (comma_count != 9) //keeping track of how many commas have been passed. there are more than 9 commas in the GPS line, but we likely don't need anything else beyond that.
					{
						if (IMU_Line.substr(IMU_Scan, 1) == detect_comma)	//if a comma is detected
						{
							comma_count++;
							IMU_Scan++;
						}
						else
						{
							//if a comma is NOT detected
						}
						{
							IMU_Scan++;
						}

						//reads the string between the 3rd and 4th comma, the time, converts to double
						if (comma_count == 3)
						{
							IMU_Start = IMU_Scan + 1;
							//this while loop finds the length of the substring after the 3rd comma
							while (IMU_Line.substr(IMU_Scan, 1) != detect_comma)
							{
								IMU_Scan++;
								IMU_Length++;
							}
							//immediately after the while loop, the IMU_Start and IMU_Length values are used to get IMU_Time
							IMU_time = std::stod(IMU_Line.substr(IMU_Start, IMU_Length)) / (1000 * 60 * 60 * 24);

							IMU_Length = 0;//this gets reset because it will be used in the next loop
							IMU_time = (IMU_time - std::floor(IMU_time)) * 24;
							IMU_time = (IMU_time - std::floor(IMU_time)) * 3600;
						}

						//reads the string between the 7th and 8th comma, the lat degree, converts to double
						if (comma_count == 7)
						{
							IMU_Start = IMU_Scan + 1;
							while (IMU_Line.substr(IMU_Scan, 1) != detect_comma)
							{
								IMU_Scan++;
								IMU_Length++;
							}
							lat_deg = std::stod(IMU_Line.substr(IMU_Start, IMU_Length));
							cout << "\nLAT is " << lat_deg;
							IMU_Length = 0;
						}

						//reads the string between the 8th and 9th comma, the long degree, converts to double
						//corrects lat and long values by subtracting the "zero" values, which are likely the starting point values
						if (comma_count == 8)
						{
							IMU_Start = IMU_Scan + 1;
							while (IMU_Line.substr(IMU_Scan, 1) != detect_comma)
							{
								IMU_Scan++;
								IMU_Length++;
							}
							long_deg = std::stod(IMU_Line.substr(IMU_Start, IMU_Length));
							cout << "\nLONG is " << long_deg;
							IMU_Length = 0; //reset for next GPS reading
							IMU_Scan = 0;	//reset
							lat_deg = lat_deg - gps_lat_zero;
							long_deg = long_deg - gps_long_zero;
							gps_lat_offset = lat_deg * lat_length * 1000;
							gps_long_offset = long_deg * long_length * 1000;
							comma_count = 9;
						}
					}
					comma_count = 0;
				}

				else if (IMU_Line.substr(0, 3) == detect_BARO_IMU.substr(0, 3))  //detects "BARO" subtring
				{
					while (comma_count != 3)
					{
						if (IMU_Line.substr(IMU_Scan, 1) == detect_comma)
						{
							comma_count++;
							IMU_Scan++;
						}

						else
						{
							IMU_Scan++;
						}

						//this while loop finds the length of the substring after the 2nd comma
						if (comma_count == 2)
						{
							IMU_Start = IMU_Scan + 1;
							while (IMU_Line.substr(IMU_Scan, 1) != detect_comma)
							{
								IMU_Scan++;
								IMU_Length++;
							}

							//gets the string that represents the altitude, converts to double, and scales appropriately
							IMU_alt = std::stod(IMU_Line.substr(IMU_Start, IMU_Length)) * 1000;
							cout << "\nIMU ALt is " << IMU_alt;
							IMU_Length = 0;	//reset
							IMU_Scan = 0;	//reset
							comma_count = 3;//why?
						}
					}
					comma_count = 0;
				}

				else if (IMU_Line.substr(0, 3) == detect_ATT_IMU.substr(0, 3))	//detects "ATT" substring
				{
					while (comma_count != 8)
					{
						if (IMU_Line.substr(IMU_Scan, 1) == detect_comma)
						{
							comma_count++;
							IMU_Scan++;
						}

						else
						{
							IMU_Scan++;
						}

						if (comma_count == 3)
						{
							IMU_Start = IMU_Scan + 1;
							while (IMU_Line.substr(IMU_Scan, 1) != detect_comma)
							{
								IMU_Scan++;
								IMU_Length++;
							}
							//gets the string that represents the roll, converts to double, and converts to radians??
							roll = std::stod(IMU_Line.substr(IMU_Start, IMU_Length));
							roll = (roll * pi / 180);
							IMU_Length = 0;
						}

						if (comma_count == 5)
						{
							IMU_Start = IMU_Scan + 1;
							while (IMU_Line.substr(IMU_Scan, 1) != detect_comma)
							{
								IMU_Scan++;
								IMU_Length++;
							}
							//gets the string that represents pitch, converts to double, and converts to radians??
							pitch = std::stod(IMU_Line.substr(IMU_Start, IMU_Length));
							pitch = (pitch * pi / 180);
							IMU_Length = 0;
						}

						if (comma_count == 7)
						{
							IMU_Start = IMU_Scan + 1;
							while (IMU_Line.substr(IMU_Scan, 1) != detect_comma)
							{
								IMU_Scan++;
								IMU_Length++;
							}
							//gets the string that represents yaw, converts to double, and converts to radians??
							yaw = std::stod(IMU_Line.substr(IMU_Start, IMU_Length));
							yaw = (yaw * pi / 180);
							IMU_Length = 0;
							IMU_Scan = 0;
							comma_count = 8;
						}
					}
					comma_count = 0;
				}

				if (IMU_time >= lidar_time)
				{
					IMU_Hold = 1;
				}

				if (IMU_time < lidar_time)
				{
					IMU_Hold = 0;
				}
			}

			//GPS Stuff
			//Be warned this is set to work on 10-99 deg N, and 100 to 180 deg W
			//AKA California
			//Be wary of gps heading and the angle correction. It can be very wrong.
			if ((gps_set == 0) && (TEST.substr(0, 4) == (detect_gps.substr(0, 4)))) //what is gps_set??? && detect the GPS line in the LIDAR data file
			{
				lat_deg = std::stod(TEST.substr(24, 2));
				lat_min = std::stod(TEST.substr(26, 6));
				lat_deg = lat_deg + lat_min / 60;

				long_deg = std::stod(TEST.substr(36, 3));
				long_min = std::stod(TEST.substr(39, 6));
				long_deg = (long_deg + long_min / 60) * -1;

				lat_length = 111321.5432;
				long_length = 111321.5432 * cos(lat_deg * pi / 180);

				cout << "\nEnter every row number to use, 1 is every row. Use a postive integer\n";
				cin >> skip_required;
				cout << "\nYour GPS is at " << lat_deg << " N " << -long_deg << " W";
				cout << "\n Enter your own zero or accept this one\n";
				cout << "Enter 1 to use, Enter 2 to set your own\n";
				cin >> gps_set;
				if (gps_set == 1)
				{
					"\nUsing the gps as zero";
				}
				else if (gps_set == 2)
				{
					"\nEnter Lat in deg";
					cin >> lat_deg;
					"\nEnter Long in deg";
					cin >> long_deg;

				}
				gps_lat_zero = lat_deg;
				gps_long_zero = long_deg;
				gps_set = 1;
			}

			else if ((TEST.substr(0, 4) == (detect_gps.substr(0, 4))) && (gps_set == 1))
			{
				//lat_deg = std::stod(TEST.substr(24, 2));
				//lat_min = std::stod(TEST.substr(26, 6));
				//lat_deg = lat_deg + lat_min / 60;
				//long_deg = std::stod(TEST.substr(36, 3));
				//long_min = std::stod(TEST.substr(39, 6));
				//long_deg = (long_deg + long_min / 60) * -1;

				//lat_deg = lat_deg - gps_lat_zero;
				//long_deg = long_deg - gps_long_zero;
				//gps_lat_offset = lat_deg * lat_length * 1000;
				//gps_long_offset = long_deg * long_length * 1000;
			}
			//Finds the distance and uses the angles to find the XYZ points of the data
			//Writes data to the file
			if ((TEST.substr(0, 5) == detect_angle.substr(0, 5)) && (skip == 0))
			{
				skip = 1;
				while (TEST.substr(arb, 1) == testspace.substr(0, 1)) {
					arb++;
				}

				anglestart = arb;

				while (TEST.substr(arb, 1) != testspace.substr(0, 1)) {
					arb++;
					anglelength++;
				}

				//cout<<"angle:"<< TEST.substr(anglestart,anglelength)<< endl;
				angle = std::stod(TEST.substr(anglestart, anglelength));
				angle_deg = angle / 100;
				angle = angle / 100;
				angle = 90 - angle;
				angle = angle*pi / 180;

				while (linevar < 32) {
					while (TEST.substr(arb, 1) == testspace.substr(0, 1)) {
						arb++;
					}
					distancestart = arb;
					while (TEST.substr(arb, 1) != testspace.substr(0, 1)) {
						arb++;
						distancelength++;
					}

					distance = std::stoi(TEST.substr(distancestart, distancelength));

					while (TEST.substr(arb, 1) == testspace.substr(0, 1)) {
						arb++;
					}
					reflectivitystart = arb;
					while (TEST.substr(arb, 1) != testspace.substr(0, 1)) {
						arb++;
						reflectivitylength++;

						// prevents an error from occurring
						if (arb > 658) {
							arb = 658;
							break;
						}
					}

					reflectivity = std::stoi(TEST.substr(reflectivitystart, reflectivitylength));

					x = distance*sin(laserangle[laseranglecounter])*cos(angle);
					y = distance*sin(laserangle[laseranglecounter])*sin(angle);
					z = distance*cos(laserangle[laseranglecounter]);

					//X Transform (Pitch + Y_offset)
					x1 = x;
					y1 = y * cos(pitch + y_offset) - z * sin(pitch + y_offset);
					z1 = y * sin(pitch + y_offset) + z * cos(pitch + y_offset);

					//Y Transform (Roll)
					x = x1 * cos(roll) - z1 * sin(roll);
					y = y1;
					z = -x1 * sin(roll) + z1 * cos(roll);

					//Z Transform (Yaw)
					x1 = x * cos(yaw) - y * sin(yaw) + gps_long_offset;
					y1 = x * sin(yaw) + y * cos(yaw) - gps_lat_offset;
					z1 = z + IMU_alt;

					if ((time_check != 0) && (time_count != 15))
					{
						lidar_time = lidar_time + 0.0000023;
						time_count = time_count + 1;
					}
					else if (time_count == 15)
					{
						lidar_time = lidar_time + 0.0000183;
						time_count = 0;
						cout << "\n" << lidar_time;
					}

					string xstring = to_string(x1);
					string ystring = to_string(y1);
					string zstring = to_string(z1);
					string astring = to_string(angle_deg);
					string tstring = to_string(lidar_time);

					column1spacing = xstring.length();
					column2spacing = ystring.length();
					column3spacing = zstring.length();
					column4spacing = astring.length();
					column5spacing = tstring.length();

					//Only writes to the file if the point is not a 0,0,0 point and gps/time has loaded
					if ((distance != 0) && (lidar_time != 0) && (gps_set != 0))
					{
						outfile << xstring;

						while (column1spacing < 20) {
							outfile << " ";
							column1spacing++;
						}

						outfile << ystring;

						while (column2spacing < 20) {
							outfile << " ";
							column2spacing++;
						}

						outfile << zstring;

						while (column3spacing < 20) {
							outfile << " ";
							column3spacing++;
						}

						outfile << astring;

						while (column4spacing < 20) {
							outfile << " ";
							column4spacing++;
						}

						outfile << tstring;

						while (column5spacing < 20) {
							outfile << " ";
							column5spacing++;
						}

						outfile << reflectivity << endl;
					}
					else {}

					laseranglecounter = laseranglecounter + 1;
					if (laseranglecounter == 16)
					{
						laseranglecounter = 0;
					}

					distancelength = 0;
					reflectivitylength = 0;

					linevar++;
				}
				linevar = 0;
				anglelength = 0;
				arb = 6;
			}
			else if (skip != 0)
			{
				skip = skip + 1;
			}

			if (skip == skip_required)
			{
				skip = 0;
			}
		}



		infile.close();
		outfile.close();
	}
}