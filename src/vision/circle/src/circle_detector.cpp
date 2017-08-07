#include"circle_detector.h"

CircleDetector::CircleDetector(int parkcolor) {
    m_parkcolor=parkcolor;
//    m_originalimg = cv::Mat::zeros(480, 640, CV_8UC3);
//    m_thresholdimg = cv::Mat::zeros(480, 640, CV_8UC1);
//    m_dilateimg = cv::Mat::zeros(480, 640, CV_8UC1);
    
}


bool CircleDetector::detect(cv::Mat &image) {
    m_originalimg = image.clone();
    m_imgrows = image.rows;
    m_imgcols = image.cols;
    m_parkdetected=false;
    imgThreshold();
    imgDilate();
    findParkCircle();
    debug_vision();
    if(m_parkdetected) return true;
    else return false;
}


bool CircleDetector::imgThreshold() {
    m_thresholdimg = cv::Mat::zeros(m_imgrows, m_imgcols, CV_8UC1);
/*
    cv::Vec3b c = m_originalimg.at<cv::Vec3b>(1,1);
    int k = m_originalimg.at<cv::Vec3b>(1,1)[0];
    std::vector<cv::Mat> bgr;
    cv::split(m_originalimg, bgr);
    cv::Mat b=bgr[0];
    int a = b.at<unsigned char>(1,1);
*/
//    std::cout << int(m_originalimg.at<cv::Vec3b>(0, 0)[0]) << ' ' << int(m_originalimg.at<cv::Vec3b>(0, 0)[1]) << ' ' << int(m_originalimg.at<cv::Vec3b>(0, 0)[2]) << std::endl;  
    for (int i = 0; i < m_imgrows; i++) {
        for (int j = 0; j < m_imgcols; j++) {
            if(m_parkcolor==2) {
                if (m_originalimg.at<cv::Vec3b>(i, j)[0] < 150 && m_originalimg.at<cv::Vec3b>(i, j)[1] < 150 && m_originalimg.at<cv::Vec3b>(i, j)[2]>200) {
                    m_thresholdimg.at<unsigned char>(i, j) = 255;
                }
            
            }    
            else if(m_parkcolor==1) {
                if (m_originalimg.at<cv::Vec3b>(i, j)[0] > 180 && m_originalimg.at<cv::Vec3b>(i, j)[1] > 110 && m_originalimg.at<cv::Vec3b>(i, j)[2]<100) {
                    m_thresholdimg.at<unsigned char>(i, j) = 255;  
                }      
            }    
            else std::cout << "parkcolor error" << std::endl; 
        }
    }
//    cv::imshow("thresholdimg",m_thresholdimg);
    return true;
}


bool CircleDetector::imgDilate() {
    cv::dilate(m_thresholdimg, m_dilateimg, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)), cv::Point(-1, -1), 1);
    return true;
}


bool CircleDetector::findParkCircle() {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i > hierarchy;
    cv::findContours(m_dilateimg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() > 0) {
        int largest_area = 0;
        int largest_contour_index = 0;
        for (int i = 0; i < contours.size(); i++) {
            double s = contourArea(contours[i]);
            if (s > largest_area) {
            largest_area = s;
            largest_contour_index = i;
            }
        }
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contours[largest_contour_index], center, radius);
        float circle_area = 3.14*radius*radius;
        float contour_area = cv::contourArea(contours[largest_contour_index]);			
        if (contour_area / circle_area > 0.9) {
            m_parkdetected = true;
            // std::cout << contour_area / circle_area << std::endl;
            m_center[0] = center.x;
            m_center[1] = center.y;
            std::cout << center.x << ' ' << center.y << std::endl;
            m_radius = radius;
            if(m_parkcolor==1) cv::circle(m_originalimg, center, radius, cv::Scalar(0, 0, 255));
            else if(m_parkcolor==2) cv::circle(m_originalimg, center, radius, cv::Scalar(255, 0, 0));
        }
    }	
    return true;
}

bool CircleDetector::debug_vision() {
    cv::imshow("originalimg", m_originalimg);
    
//    cv::imshow("thresholdimg",m_thresholdimg);
//    cv::imshow("dilateimg",m_dilateimg);
    cv::waitKey(1);
}
