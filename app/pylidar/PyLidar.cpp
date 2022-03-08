#include "PyLidar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>
#include <corecrt_math_defines.h>
#include <rplidar.h>
using namespace sl;

constexpr double deg2rad = M_PI / 180.0;
PyLidar::PyLidar(const char* port, int baud_rate)
{
    // create/update the driver instance
    _baud_rate = baud_rate;
    _port = port;
    _drv = *createLidarDriver();

    if (!_drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    _channel = (*createSerialPortChannel(_port, _baud_rate));
   // printf("%s\n", _port);
   // printf("%d\n", _baud_rate);
}

PyLidar::~PyLidar()
{
    delete _drv;
    delete _channel;
    _channel = NULL;
    _drv = NULL;
}

void PyLidar::destroydriver(void)
{
    delete _drv;
    delete _channel;
    _channel = NULL;
    _drv = NULL;
}


void PyLidar::connectlidar()
{
    sl_result op_result;
    //printf("in pyLidar");

    if (SL_IS_OK(_drv->connect(_channel))) {
        op_result = _drv->getDeviceInfo(devinfo);

        if (SL_IS_OK(op_result))
        {
            connectSuccess = true;
        }
        else {
            destroydriver();
        }
    }

    if (SL_IS_OK(op_result))
    {
        connectSuccess = true;
    }

    if (!connectSuccess){

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , _port);
        // since failed. It should destroy the created driver
        destroydriver();
    }
}

void PyLidar::disconnectlidar()
{
    _drv->disconnect();  
}

bool PyLidar::isConnected()
{
   return _drv->isConnected();
}


// A wrapper code for the checkheatlh status
bool PyLidar::checkhealth(void)
{
    return checkRPLIDARHealth(_drv);
}

sl_lidar_response_device_info_t PyLidar::getDeviceInfo(void)
{
    sl_lidar_response_device_info_t devInfo;
    _drv->getDeviceInfo(devinfo);
    return devInfo;
}

float PyLidar::getFrequency(void) {
    return frequency;
}

bool PyLidar::reset(unsigned int timeout) {
    sl_result op = _drv->reset(timeout);
    if (IS_OK(op)) {
        return true;
    }
    return false;
}


// for stopping the motor
void PyLidar::stopmotor(void)
{
    _drv->stop();
    printf("setting motor speed to 0");
    _drv->setMotorSpeed(0);
    //destroydriver();

}

void PyLidar::startmotor(int my_scanmode)
{
    _drv->setMotorSpeed();
    _drv->getAllSupportedScanModes(myscanModes);
    myScanMode = myscanModes[my_scanmode];
    _drv->startScanExpress(false, myScanMode.id);

}


std::vector<std::vector<double>> PyLidar::get_scan_as_vectors(bool filter_quality)
{
    int quality; // holder for quality of sample
    std::vector<double> sample(3);
    sl_result op_result;
    std::vector<std::vector<double>> output;  // vector of vectors for converted data
    size_t   count = _countof(nodes);

    // Grab a scan frame
    op_result = _drv->grabScanDataHq(nodes, count);
    output.reserve(count); // allocate space for data

    if (IS_OK(op_result)) {
        // readjust the scan data
        _drv->ascendScanData(nodes, count);

        for (int pos = 0; pos < (int)count; pos++) {
            quality = nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            if (quality > 0 || !filter_quality)
            {
                sample.at(0) = quality;
                sample.at(1) = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                sample.at(2) = nodes[pos].dist_mm_q2 / 4.0f;
                output.push_back(sample);
            }
        }
    }
    _drv->getFrequency(myScanMode, nodes, count, frequency);
    return output;
}

double** PyLidar::get_scan_as_pointers(bool filter_quality)
{
    int quality; // holder for quality of sample
    std::vector<double> sample(3);
    sl_result op_result;
    std::vector<double*> output;  // vector of vectors for converted data
    size_t   count = _countof(nodes);

    // Grab a scan frame
    op_result = _drv->grabScanDataHq(nodes, count);
    output.reserve(count); // allocate space for data

    if (IS_OK(op_result)) {
        // readjust the scan data
        _drv->ascendScanData(nodes, count);

        for (int pos = 0; pos < (int)count; pos++) {
            quality = nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            if (quality > 0 || !filter_quality)
            {
                sample.at(0) = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                sample.at(1) = nodes[pos].dist_mm_q2 / 4.0f;
                sample.at(2) = quality;
                output.push_back(sample.data());
            }
        }
    }
    _drv->getFrequency(myScanMode, nodes, count, frequency);
    return output.data();
}

std::vector<std::vector<double>> PyLidar::get_scan_as_xy(bool filter_quality)
{
    std::vector<std::vector<double>> points = get_scan_as_vectors(filter_quality);

    // allocate output size
    std::vector<std::vector<double>> output(points.size(), std::vector<double>(2));

    for (unsigned int i = 0; i < points.size(); i++)
    {
        std::vector<double> coords(2);
        output.at(i).at(0) = std::cos(deg2rad * points.at(i).at(0)) * points.at(i).at(1);
        output.at(i).at(1) = std::sin(deg2rad * points.at(i).at(0)) * points.at(i).at(1);
    }
    return output;
}

double** PyLidar::get_scan_as_xy_pointers(bool filter_quality)
{
    std::vector<std::vector<double>> points = get_scan_as_vectors(filter_quality);

    // allocate output size
    std::vector<double *> output;
    size_t   count = _countof(nodes);
    output.reserve(count);

    for (unsigned int i = 0; i < points.size(); i++)
    {
        std::vector<double> coords(2);
        coords.at(0) = std::cos(deg2rad * points.at(i).at(0)) * points.at(i).at(1);
        coords.at(1) = std::sin(deg2rad * points.at(i).at(0)) * points.at(i).at(1);
        output.push_back(coords.data());
    }
    return output.data();
}