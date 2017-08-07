#pragma once

#include <opencv2/opencv.hpp>

class CircleDetector {
public:
    CircleDetector(int parkcolor=2); //blue=1, red=2
    float m_center[2];
    float m_radius;
    bool detect(cv::Mat &image);
    bool m_parkdetected;
private:
    int m_parkcolor;
    int m_imgrows;
    int m_imgcols;
    cv::Mat m_originalimg;
    cv::Mat m_thresholdimg;
    cv::Mat m_dilateimg;
    bool imgThreshold();
    bool imgDilate();
    bool findParkCircle();
    bool show_result();
};
