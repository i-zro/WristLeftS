#define _USE_MATH_DEFINES // PI 쓰기 위해 불러옴
#include <stdio.h>
#include "stdafx.h"
#include "windows.h"
#include "serial.cpp"
#include <time.h>
#include <sys/timeb.h>
#include <locale>
#include <string>
#include <sstream>  
#include <thread>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <chrono>
#include <list>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <math.h>
#include <atlstr.h>

#include <k4a/k4a.h>
#include <k4abt.h>
#include <k4abt.hpp>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

using namespace cv;
using namespace std;

using std::string;
// 라디안 -> 흔히 쓰는 각으로
#define PI       3.14159265358979323846 //M_PI 오류
double Rad2Deg(double rad) { return rad * 180 / PI; }

#define FRAME_NUM 10000
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    } 
#define MY_SERIALPORT  3  // 연결된 시리얼 포트번호

#define SBUF_SIZE 2000

char sbuf[SBUF_SIZE];
signed int sbuf_cnt = 0;
static std::vector<double> ivalues;
static std::vector<double> kvalues;

k4a_float2_t g_fSkeleton2D[K4ABT_JOINT_COUNT] = { 0.0f, };
k4a_device_t device = nullptr;
k4abt_tracker_t tracker = nullptr;
k4abt_frame_t body_frame = nullptr;
k4a_calibration_t sensor_calibration;
k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
k4abt_skeleton_t skeleton;

std::string saveCsv(int which, string csvname) {
	// 파일 이름 생성용
	time_t timer;
	timer = time(NULL);

	struct tm t;
	localtime_s(&t, &timer);
	std::stringstream whatTime;

	CString mm;
	CString dd;
	CString hh;
	CString mn;
	CString ss;
	mm.Format("%02d", t.tm_mon + 1);
	dd.Format("%02d", t.tm_mday);
	hh.Format("%02d", t.tm_hour);
	mn.Format("%02d", t.tm_min);
	ss.Format("%02d", t.tm_sec);

	string yy = to_string(t.tm_year + 1900).substr(2, 2);
	whatTime << mm;
	whatTime << dd;
	whatTime << "_";
	whatTime << hh;
	whatTime << mn;
	whatTime << ss;

	string wTime = whatTime.str();

	if (which == 1) {
		csvname = "KinData/K_" + yy + wTime + ".csv";
		cout << "file name : " + csvname << endl;
	}

	else if (which == 2) {
		csvname = "ImuData/I_" + yy + wTime + ".csv";
		cout << "file name : " + csvname << endl;
	}
	return csvname;
}

int LocalMilli() {
	//localtimer
	time_t timer;
	timer = time(NULL);

	struct tm t;
	localtime_s(&t, &timer);

	string wTime = to_string(t.tm_hour) +
		to_string(t.tm_min) +
		to_string(t.tm_sec); //현재 시간 string 변수

	long long time_last;
	time_last = time(NULL);

	//밀리초 위한 ftimer
	struct timeb t1;
	ftime(&t1);
	time_t ttt = t1.millitm;
	CString tt;
	tt.Format("%03d", int(ttt));
	std::stringstream ss;
	ss << tt;
	std::string ts = wTime + ss.str();
	int nts = stoi(ts);
	return nts;
}

struct Quaternion
{
	double w, x, y, z;
};

struct EulerAngles {
	double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
	EulerAngles angles;

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	angles.roll = Rad2Deg(std::atan2(sinr_cosp, cosr_cosp));

	// pitch (y-axis rotation)
	double sinp = 2 * (q.w * q.y - q.z * q.x);
	if (std::abs(sinp) >= 1)
		angles.pitch = Rad2Deg(std::copysign(PI / 2, sinp)); // use 90 degrees if out of range
	else
		angles.pitch = Rad2Deg(std::asin(sinp));

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	angles.yaw = Rad2Deg(std::atan2(siny_cosp, cosy_cosp));

	return angles;
}


char* my_strtok(char* str, char dm, int* result)
{
	int n;

	*result = 0;

	for (n = 0; n < 100; n++)
	{
		if (str[n] == dm) { *result = 1;  break; }
		if (str[n] == NULL) break;
	}

	return &str[n + 1];
}

int EBimuAsciiParser(int* id, float* item, unsigned int number_of_item)
{
	SERIALREADDATA srd;
	unsigned int n, i;
	char* addr;
	int result = 0;
	int ret;

	//	char *context;

	if (ReadSerialPort(MY_SERIALPORT, &srd) == ERR_OK)
	{
		if (srd.nSize)
		{
			for (n = 0; n < srd.nSize; n++)
			{
				//////////////////////////////////////////////////////////////////////
				sbuf[sbuf_cnt] = srd.szData[n];
				if (sbuf[sbuf_cnt] == '\r')  // 1줄 수신완료
				{
					{

						addr = my_strtok(sbuf, '-', &ret);
						if (ret)
						{
							*id = (float)atoi(addr);

							addr = my_strtok(sbuf, ',', &ret);
							for (i = 0; i < number_of_item; i++)
							{
								item[i] = (float)atof(addr);
								addr = my_strtok(addr, ',', &ret);
							}

							result = 1;
						}

					}
				}
				else if (sbuf[sbuf_cnt] == '\n')
				{
					sbuf_cnt = -1;
				}

				sbuf_cnt++;
				if (sbuf_cnt >= SBUF_SIZE) sbuf_cnt = 0;
				///////////////////////////////////////////////////////////////////////
			}
		}
	}

	return result;
}

int EBimu() {
	int id;
	float item[10];

	if (OpenSerialPort(MY_SERIALPORT, 921600, NOPARITY, 8, ONESTOPBIT) != ERR_OK)
	{
		printf("\n\rSerialport Error...");
		Sleep(2000);
		return 0;

	}

	while (1)
	{
		if (EBimuAsciiParser(&id, item, 5))
		{
			//test << id << endl;
			//test << LocalMilli() << endl;
			printf("\n id time :%d %d %f %f %f %f ", id, LocalMilli(), item[0], item[1], item[2], item[3]);
		}
	}
}

bool imu_quart() {
	int id;
	ivalues.clear();
	float item[10];

	if (OpenSerialPort(MY_SERIALPORT, 921600, NOPARITY, 8, ONESTOPBIT) != ERR_OK)
	{
		printf("\n\rSerialport Error...");
		Sleep(2000);

	}

	while (1)
	{
		if (EBimuAsciiParser(&id, item, 4))
		{
			ivalues.push_back(LocalMilli());
			for (int j = 0; j < 4; j++) {
				ivalues.push_back(item[j]);
			}
			CloseSerialPort(MY_SERIALPORT);
			return true;
		}
	}
}

bool imu_euler() {
	int id;
	
	float item[10];

	if (OpenSerialPort(MY_SERIALPORT, 921600, NOPARITY, 8, ONESTOPBIT) != ERR_OK)
	{
		printf("\n\rSerialport Error...");
		Sleep(2000);

	}

	while (1)
	{
		if (EBimuAsciiParser(&id, item, 4))
		{
			Quaternion q;
			
			q.w = item[0];
			q.x = item[1];
			q.y = item[2];
			q.z = item[3];

			EulerAngles e = ToEulerAngles(q);

			ivalues.push_back(LocalMilli());
			ivalues.push_back(e.pitch);
			ivalues.push_back(e.roll);
			ivalues.push_back(e.yaw);

			CloseSerialPort(MY_SERIALPORT);
			return true;
		}
	}
}

bool imu_eq() {
	int id;

	float item[10];

	if (OpenSerialPort(MY_SERIALPORT, 921600, NOPARITY, 8, ONESTOPBIT) != ERR_OK)
	{
		printf("\n\rSerialport Error...");
		Sleep(2000);

	}

	while (1)
	{
		if (EBimuAsciiParser(&id, item, 4))
		{
			ivalues.push_back(LocalMilli());
			Quaternion q;

			q.w = item[0];
			q.x = item[1];
			q.y = item[2];
			q.z = item[3];

			for (int j = 0; j < 4; j++) {
				ivalues.push_back(item[j]);
			}

			EulerAngles e = ToEulerAngles(q);

			ivalues.push_back(e.pitch);
			ivalues.push_back(e.roll);
			ivalues.push_back(e.yaw);

			CloseSerialPort(MY_SERIALPORT);
			return true;
		}
	}
}


void print_imu() {
	cout << int(ivalues[0]) << endl;
	for (int j = 1; j < ivalues.size(); j++) {
		cout << ivalues[j] << endl;
	}
}

void print_kin() {
	cout << int(kvalues[0]) << endl;
	for (int j = 1; j < kvalues.size(); j++) {
		cout << kvalues[j] << endl;
	}
}

// csv 저장
void write_csv(std::string filename, std::vector<double> dataset) {
	std::ofstream myFile(filename, ios::out | ios::app);

	myFile << int(dataset[0]);
	myFile << ",";
	// Send column names to the stream
	for (int j = 1; j < dataset.size(); j++)
	{
		myFile << dataset[j];
		if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
	}
	myFile << "\n";

	// Close the file
	myFile.close();
}


int main()
{
	int num;

	while (1) {
		cout << "1 : Kinect, IMU 오일러 저장" << endl;
		cout << "2 : Kinect, IMU 쿼터니언 저장" << endl;
		cout << "3 : Kinect, IMU 오일러, 쿼터니언 모두 저장" << endl;
		cout << "기타 : 종료" << endl;
		cout << "입력 : ";
		cin >> num;
		if (num != 1 && num != 2 && num != 3) { return 0; }
			string kin_dir = saveCsv(1, "");
			string imu_dir = saveCsv(2, "");

			k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
			device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
			device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
			k4a_device_t device;
			VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");
			VERIFY(k4a_device_start_cameras(device, &device_config), "Start K4A cameras failed!");
			// Make sure to pass in the correct device config for both depth camera and color camera to get the correct sensor calibration
			k4a_calibration_t sensor_calibration;
			VERIFY(k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &sensor_calibration),
				"Get depth camera calibration failed!");
			// Create transformation handle to perform the body index map space transform
			k4a_transformation_t transformation = NULL;
			transformation = k4a_transformation_create(&sensor_calibration);
			if (transformation == NULL)
			{
				printf("Failed to create transformation from sensor calibration!");
				exit(1);
			}

			k4abt_tracker_t tracker = NULL;
			k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
			VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

			int frame_count = 0;
			do
			{
				k4a_capture_t sensor_capture;
				k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
				if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
				{
					frame_count++;

					printf("Start processing frame %d\n", frame_count);

					k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

					k4a_capture_release(sensor_capture);
					if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
					{
						// It should never hit timeout when K4A_WAIT_INFINITE is set.
						printf("Error! Add capture to tracker process queue timeout!\n");
						break;
					}
					else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
					{
						printf("Error! Add capture to tracker process queue failed!\n");
						break;
					}

					k4abt_frame_t body_frame = NULL;
					k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
					if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
					{

						uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

						for (uint32_t i = 0; i < num_bodies; i++)
						{
							// 벡터 clear
							ivalues.clear();
							kvalues.clear();
							k4abt_skeleton_t skeleton;
							VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &skeleton), "Get body from body frame failed!");

							if (num == 1) {
								printf("Frame[%u]:\n", frame_count);
								kvalues.push_back(LocalMilli());
								imu_euler();
								print_imu();
								write_csv(imu_dir, ivalues);

								Quaternion qk;
								qk.x = skeleton.joints[7].orientation.wxyz.x;
								qk.y = skeleton.joints[7].orientation.wxyz.y;
								qk.z = skeleton.joints[7].orientation.wxyz.z;
								qk.w = skeleton.joints[7].orientation.wxyz.w;
								EulerAngles ek = ToEulerAngles(qk);
								kvalues.push_back(ek.pitch);
								kvalues.push_back(ek.roll);
								kvalues.push_back(ek.yaw);

								print_kin();
								write_csv(kin_dir, kvalues);
								k4abt_frame_release(body_frame);
							}
							else if (num == 2) {
								printf("Frame[%u]:\n", frame_count);
								kvalues.push_back(LocalMilli());
								imu_quart();
								print_imu();
								write_csv(imu_dir, ivalues);

								Quaternion qk;
								qk.x = skeleton.joints[7].orientation.wxyz.x;
								qk.y = skeleton.joints[7].orientation.wxyz.y;
								qk.z = skeleton.joints[7].orientation.wxyz.z;
								qk.w = skeleton.joints[7].orientation.wxyz.w;
								kvalues.push_back(qk.x);
								kvalues.push_back(qk.y);
								kvalues.push_back(qk.z);
								kvalues.push_back(qk.w);

								print_kin();
								write_csv(kin_dir, kvalues);
								k4abt_frame_release(body_frame);
							}
							else if (num == 3) {
								printf("Frame[%u]:\n", frame_count);
								kvalues.push_back(LocalMilli());
								imu_eq();
								print_imu();
								write_csv(imu_dir, ivalues);

								Quaternion qk;
								qk.x = skeleton.joints[7].orientation.wxyz.x;
								qk.y = skeleton.joints[7].orientation.wxyz.y;
								qk.z = skeleton.joints[7].orientation.wxyz.z;
								qk.w = skeleton.joints[7].orientation.wxyz.w;
								EulerAngles ek = ToEulerAngles(qk);

								kvalues.push_back(qk.x);
								kvalues.push_back(qk.y);
								kvalues.push_back(qk.z);
								kvalues.push_back(qk.w);
								kvalues.push_back(ek.pitch);
								kvalues.push_back(ek.roll);
								kvalues.push_back(ek.yaw);

								print_kin();
								write_csv(kin_dir, kvalues);
								k4abt_frame_release(body_frame);
							}
						}
					}
					else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
					{
						//  It should never hit timeout when K4A_WAIT_INFINITE is set.
						printf("Error! Pop body frame result timeout!\n");
						break;
					}
					else
					{
						printf("Pop body frame result failed!\n");
						break;
					}
					}
					else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
					{
						// It should never hit time out when K4A_WAIT_INFINITE is set.
						printf("Error! Get depth frame time out!\n");
						break;
					}
					else
					{
						printf("Get depth capture returned error: %d\n", get_capture_result);
						break;
					}

				} while (frame_count < 100);

				printf("Finished body tracking processing!\n");

				k4a_transformation_destroy(transformation);
				k4abt_tracker_shutdown(tracker);
				k4abt_tracker_destroy(tracker);
				k4a_device_stop_cameras(device);
				k4a_device_close(device);

			}
}