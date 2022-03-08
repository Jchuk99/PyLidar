
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <rplidar.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

typedef struct LidarScan
{
	double** data;
	int size;
}LidarScan;

using namespace sl;
class PyLidar
{

protected:
	// For setting scanmodes
	std::vector<LidarScanMode> myscanModes;
	sl_lidar_response_device_info_t devinfo;
	LidarScanMode myScanMode;
	ILidarDriver* _drv = NULL;
	IChannel* _channel = NULL;

	// some parameters to be set
	const char* _port;
	float frequency = 0.0;

	bool connectSuccess = false;

	// create a buffer to hold the scanned data
	rplidar_response_measurement_node_hq_t nodes[8192];

	bool checkRPLIDARHealth(ILidarDriver* drv)
	{
		u_result     op_result;
		rplidar_response_device_health_t healthinfo;

		op_result = drv->getHealth(healthinfo);
		if (SL_IS_OK(op_result)) {
			printf("%d\n", healthinfo.status);
			if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
				//printf("Error1\n");
			   // enable the following code if you want rplidar to be reboot by software
			   // drv->reset();
				return false;
			}
			else {
				return true;
			}

		}
		else {
			//fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
			//printf("Error2\n");
			return false;
		}
	}


public:
	int _baud_rate;
	// Create the constructor: Here the driver will be created.
	PyLidar(const char* my_port = "/dev/ttyUSB0", int baud_rate = 115200);
	~PyLidar();
	// Setup connection to the rplidar.
	// Connect Lidar
	void connectlidar(void);
	void disconnectlidar(void);
	// destroy driver
	void destroydriver(void);
	bool isConnected(void);
	// rest lidar, return true if succeed
	bool reset(unsigned int timeout);
	// A wrapper code for the checkhealth status
	bool checkhealth(void);
	float getFrequency(void);
	sl_lidar_response_device_info_t getDeviceInfo(void);
	// stopping the motor
	void stopmotor(void);
	// starts the motor
	void startmotor(int my_scanmode = 2);

	/*
	 * This function will be used in fetching the scan data
	 * The output is a vector of vectors.
	 * */
	std::vector<std::vector<double>> get_scan_as_vectors(bool filter_quality = false);
	double ** get_scan_as_pointers(bool filter_quality = false);

	/*
	 * This function will be used in fetching the scan data
	 * The output is a vector of vectors.
	 * */
	std::vector<std::vector<double>> get_scan_as_xy(bool filter_quality = false);
	double** get_scan_as_xy_pointers(bool filter_quality = false);

};


