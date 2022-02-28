#include "PyLidar.h"

#define API __declspec(dllexport)

extern "C" {
    API PyLidar* Lidar_New(const char* port, int baud_rate) { return new PyLidar(port, baud_rate); }
    API void connect(PyLidar* lidar) { lidar->connectlidar(); }
    API bool checkHealth(PyLidar* lidar) { return lidar->checkhealth(); }
    API void stopMotor(PyLidar* lidar) { lidar->stopmotor(); }
    API void startMotor(PyLidar* lidar) { lidar->startmotor(); }
    API lidar_sample* get_scan_as_lidar_samples(PyLidar* lidar, bool filter_quality) {
        return lidar->get_scan_as_lidar_samples(filter_quality);
    }
    API double** get_scan_as_vectors(PyLidar* lidar, bool filter_quality) {
        return lidar->get_scan_as_pointers(filter_quality);
    }
    API double** get_scan_as_xy(PyLidar* lidar, bool filter_quality) {
        return lidar->get_scan_as_xy_pointers(filter_quality);
    }
}