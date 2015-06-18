#ifndef WEBCAM_TRACKER_HPP
#define WEBCAM_TRACKER_HPP

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "webcam/util.hpp"

#include <vector>
#include <iostream>
#include <cstdio>
#include <algorithm>

namespace webcam {
bool getScaledDots(Vector2D *left, Vector2D *middle, Vector2D *right);
}

#endif
