#include "PyLidar.h"
#define PY_SSIZE_T_CLEAN
#include <iostream>

#if defined _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

extern "C" {
    API void* createLidar(const char* port, int baud_rate) {
        printf("returning void pointer");
        return new(std::nothrow) PyLidar(port, baud_rate);
    }
    API void connect(void* ptr) { 
        PyLidar* lidar = reinterpret_cast<PyLidar *>(ptr);
        lidar->connectlidar(); 
    }
    API bool checkHealth(void* ptr) { 
        PyLidar* lidar = reinterpret_cast<PyLidar*>(ptr);
        return lidar->checkhealth(); 
    }
    API void stopMotor(void* ptr) { 
        PyLidar* lidar = reinterpret_cast<PyLidar*>(ptr);
        lidar->stopmotor(); 
    }
    API void startMotor(void* ptr) { 
        PyLidar* lidar = reinterpret_cast<PyLidar*>(ptr);
        lidar->startmotor(); 
    }

    API LidarScan* get_lidar_scan(void* ptr, bool filter_quality) {
        PyLidar* lidar = reinterpret_cast<PyLidar*>(ptr);
        LidarScan* lidar_scan = (LidarScan*) malloc(sizeof(LidarScan));

        std::vector<std::vector<double>> output = lidar->get_scan_as_vectors(filter_quality);
        double** data = (double**) malloc(sizeof(double*) * output.size());

        for (int i = 0; i < output.size(); i++) {
            double* sample = (double*) malloc(sizeof(double) * 3);
            std::vector<double> lidar_sample = output.at(i);
            for (int j = 0; j < lidar_sample.size(); j++) {
                sample[j] = lidar_sample.at(j);
            }
            data[i] = sample;
        }

        printf("C++: ");
        printf("angle: %f ",data[0][0]);
        printf("distance: %f ", data[0][1]);
        printf("quality: %f\n", data[0][2]);

        lidar_scan->data = data;
        lidar_scan->size = output.size();

        return lidar_scan;
    }
    API double** get_scan_as_xy(void* ptr, bool filter_quality) {
        PyLidar* lidar = reinterpret_cast<PyLidar*>(ptr);
        return lidar->get_scan_as_xy_pointers(filter_quality);
    }
}