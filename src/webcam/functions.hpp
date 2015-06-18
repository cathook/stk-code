#ifndef WEBCAM_FUNCTIONS_HPP
#define WEBCAM_FUNCTIONS_HPP

#include "webcam/vector.hpp"
#include "webcam/function.hpp"

namespace webcam {


template <typename Input>
class ConstantFunction : public Function<Input> {
 public:
  ConstantFunction(double value) :
      Function<Input>([value](Input x) { return value; },
                      [](Input x) { return Input(0); }) {}
};


class AbsFunction : public Function<double> {
 public:
  AbsFunction();
};


class ArcCosFunction : public Function<double> {
 public:
  ArcCosFunction();
};


class Distance3DFunction : public Function<Vector3D> {
 public:
  Distance3DFunction(const Vector3D &p0);
};


class IncludedDot3DFunction : public Function<Vector3D> {
 public:
  IncludedDot3DFunction(const Vector3D &p1, const Vector3D &p2);
};


class IncludedCos3DFunction : public Function<Vector3D> {
 public:
  IncludedCos3DFunction(const Vector3D &p1, const Vector3D &p2);
};


class IncludedAngle3DFunction : public Function<Vector3D> {
 public:
  IncludedAngle3DFunction(const Vector3D &p1, const Vector3D &p2);
};

}  // namespace webcam

#endif  // WEBCAM_FUNCTIONS_HPP
