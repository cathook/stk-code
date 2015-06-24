#ifndef WEBCAM_VECTOR_HPP
#define WEBCAM_VECTOR_HPP

#include <math.h>

#include "util.hpp"


namespace webcam {


class Vector2D {
 public:
  Vector2D() : Vector2D(0, 0) {}

  Vector2D(double k) : Vector2D(k, k) {}

  Vector2D(double x, double y) : x_(x), y_(y) {}

  double x() const { return x_; }

  double y() const { return y_; }

  double set_x(double val) { x_ = val; return x(); }

  double set_y(double val) { y_ = val; return y(); }

  double Dot(const Vector2D &b) const { return x() * b.x() + y() * b.y(); }

  double Cross(const Vector2D &b) const { return x() * b.y() - y() * b.x(); }

  double Length2() const { return Dot(*this); }

  double Length() const { return sqrt(Length2()); }

  Vector2D Normalize() const { return *this / Length(); }

  Vector2D Rotate(double angle) const {
    Vector2D i(cos(-angle), sin(-angle)), j(~i);
    return Vector2D(Dot(i), Dot(j));
  }

  Vector2D operator~() const { return Vector2D(-y(), x()); }

  Vector2D operator+(const Vector2D &b) const {
    return Vector2D(x() + b.x(), y() + b.y());
  }

  Vector2D operator-(const Vector2D &b) const {
    return Vector2D(x() - b.x(), y() - b.y());
  }

  Vector2D operator*(double k) const {
    return Vector2D(x() * k, y() * k);
  }

  Vector2D operator/(double k) const {
    return Vector2D(x() / k, y() / k);
  }

  std::string ToString() const {
    return FormatString("<%.3f, %.3f>", x(), y());
  }

 private:
  double x_;
  double y_;
};


class Vector3D {
 public:
  Vector3D() : x_(0), y_(0), z_(0) {}

  Vector3D(double k) : Vector3D(k, k, k) {}

  Vector3D(double x, double y, double z) : x_(x), y_(y), z_(z) {}

  Vector3D(const Vector2D &p, double z) : x_(p.x()), y_(p.y()), z_(z) {}

  double x() const { return x_; }

  double y() const { return y_; }

  double z() const { return z_; }

  double set_x(double val) { x_ = val; return x(); }

  double set_y(double val) { y_ = val; return y(); }

  double set_z(double val) { z_ = val; return z(); }

  double Dot(const Vector3D &b) const {
    return x() * b.x() + y() * b.y() + z() * b.z();
  }

  Vector3D Cross(const Vector3D &b) const {
    return Vector3D(y() * b.z() - z() * b.y(),
                    z() * b.x() - x() * b.z(),
                    x() * b.y() - y() * b.x());
  }

  Vector3D Normalize() const {
    return *this / Length();
  }

  double IncludedCos(const Vector3D &b) const {
    return Dot(b) / (Length() * b.Length());
  }

  double IncludedAngle(const Vector3D &b) const {
    return acos(Dot(b) / (Length() * b.Length()));
  }

  double Length2() const { return Dot(*this); }

  double Length() const { return sqrt(Length2()); }

  Vector3D operator+(const Vector3D &b) const {
    return Vector3D(x() + b.x(), y() + b.y(), z() + b.z());
  }

  Vector3D operator-(const Vector3D &b) const {
    return Vector3D(x() - b.x(), y() - b.y(), z() - b.z());
  }

  Vector3D operator*(double k) const {
    return Vector3D(x() * k, y() * k, z() * k);
  }

  Vector3D operator/(double k) const {
    return Vector3D(x() / k, y() / k, z() / k);
  }

  Vector3D &operator+=(const Vector3D &b) {
    set_x(x() + b.x());
    set_y(y() + b.y());
    set_z(z() + b.z());
    return *this;
  }

  Vector3D &operator-=(const Vector3D &b) {
    set_x(x() - b.x());
    set_y(y() - b.y());
    set_z(z() - b.z());
    return *this;
  }


  std::string ToString() const {
    return FormatString("<%.3f, %.3f, %.3f>", x(), y(), z());
  }

 private:
  double x_;
  double y_;
  double z_;
};

}  // namespace webcam

#endif  // WEBCAM_VECTOR_HPP