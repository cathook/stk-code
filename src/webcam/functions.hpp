#ifndef WEBCAM_FUNCTIONS_HPP
#define WEBCAM_FUNCTIONS_HPP

#include "webcam/vector.hpp"
#include "webcam/function.hpp"

namespace webcam {


class ArcCosFunction : public Function<double> {
 public:
  ArcCosFunction();
};


class DistanceFunction : public Function<Vector3D> {
 public:
  DistanceFunction(const Vector3D &p0) :
      Function<Vector3D>(
          [p0](Vector3D p) { return (p - p0).Length2(); },
          [p0](Vector3D p) { return (p - p0) * 2; },
          1, 0.5) {}
};


class IncludedDotFunction : public Function<Vector3D> {
 public:
  IncludedDotFunction(const Vector3D &p1, const Vector3D &p2) :
      Function<Vector3D>(
          [p1, p2](Vector3D p) { return (p1 - p).Dot(p2 - p); },
          [p1, p2](Vector3D p) { return p * 2 - (p1 + p2); }) {}
};


class IncludedCosFunction : public Function<Vector3D> {
 public:
  IncludedCosFunction(const Vector3D &p1, const Vector3D &p2) :
      Function<Vector3D>(IncludedDotFunction(p1, p2) /
                         (DistanceFunction(p1) * DistanceFunction(p2))) {}
};


class IncludedAngleFunction : public Function<Vector3D> {
 public:
  IncludedAngleFunction(const Vector3D &p1, const Vector3D &p2) :
      Function<Vector3D>(ArcCosFunction() | IncludedCosFunction(p1, p2)) {}
};

class AbsFunction : public Function<double> {
 public:
  AbsFunction() :
      Function<double>(
          fabs, [](double x) { return (x > 0 ? 1 : (x < 0 ? -1 : 0)); }) {}
};


template <typename Input>
class ConstantFunction : public Function<Input> {
 public:
  ConstantFunction(double value) :
      Function<Input>([value](Input x) { return value; },
                      [](Input x) { return Input(); }) {}
};
}  // namespace webcam

#endif  // WEBCAM_FUNCTIONS_HPP
