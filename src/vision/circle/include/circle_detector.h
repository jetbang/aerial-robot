#pragma once

#include <opencv2/opencv.hpp>

class CircleDetector {
public:
    CircleDetector(int parkcolor=2); //blue=1, red=2
    cv::Point2f m_center;
    float m_radius;
    bool detect(cv::Mat &image);
    bool m_parkdetected;

    int m_parkcolor;
    int m_imgrows;
    int m_imgcols;
    cv::Mat m_originalimg;
    cv::Mat m_thresholdimg;
    cv::Mat m_dilateimg;
    bool imgThreshold();
    bool imgDilate();
    bool findParkCircle();
    void draw(cv::Mat& img);
    bool show_result();
};
