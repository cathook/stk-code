#ifndef WEBCAM_WEBCAM_HPP
#define WEBCAM_WEBCAM_HPP

#include "webcam/vector.hpp"


namespace webcam {

void Init();

Vector3D GetAdjustedCameraOffset(Vector3D org);


}  // namespace webcam

#endif  // WEBCAM_WEBCAM_HPP
