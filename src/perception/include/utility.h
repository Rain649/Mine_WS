#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

// VLP-16
extern const int Horizon_SCAN = 360/1;
extern const int N_SCAN = 16;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0;
extern const int groundScanInd = 0;
extern const int aboveScanInd = 16;
extern const int colFL = 0;
//extern  const int colBL = 120;
//extern  const int colBR = 240;
extern const int colFR = Horizon_SCAN;


extern const int segmentationRadius_Max = 15;

#endif
