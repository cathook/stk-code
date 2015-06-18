#ifndef WEBCAM_TRACKER_HPP
#define WEBCAM_TRACKER_HPP

#include "webcam/vector.hpp"


namespace webcam {

void InitTracker();

double GetAspectRatio();

bool GetScaledDots(Vector2D *left, Vector2D *middle, Vector2D *right);

}  // webcam

#endif  // WEBCAM_TRACKER_HPP
