#ifndef WEBCAM_TRACKER_HPP
#define WEBCAM_TRACKER_HPP

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <cstdio>
#include <algorithm>

#include "webcam/vector.hpp"

namespace webcam {
  CvCapture* getCamera();
  cv::Mat getShot(CvCapture* camera);
  cv::Mat threshold(cv::Mat& image);
  std::vector<cv::Vec3f> findCircles(cv::Mat &image_);
  bool getScaledDots(Vector2D *left, Vector2D *middle, Vector2D *right);
  bool circleSort(cv::Vec3f a, cv::Vec3f b);
  void plotCircles(cv::Mat &image, std::vector<cv::Vec3f> &circles);
}

#endif
