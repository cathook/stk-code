#ifndef WEBCAM_UTIL_HPP
#define WEBCAM_UTIL_HPP

#include <stdio.h>
#include <math.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>


namespace webcam {


template <typename... Args>
inline std::string FormatString(Args&&... args) {
  int len = snprintf(NULL, 0, std::forward<Args>(args)...);
  std::unique_ptr<char[]> buf(new char[len + 1]);
  snprintf(buf.get(), len + 1, std::forward<Args>(args)...);
  return std::string(buf.get());
}


class Vector2D {
 public:
  Vector2D() : Vector2D(0, 0) {}

  Vector2D(double k) : Vector2D(k, k) {}

  Vector2D(double x, double y) : x_(x), y_(y) {}

  double x() const { return x_; }

  double y() const { return y_; }

  double Cross(const Vector2D &b) const { return x() * b.y() - y() * b.x(); }

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

 private:
  double x_;
  double y_;
  double z_;
};


template <typename X>
inline X Squ(X x) { return x * x; }


template <typename X>
inline X Tri(X x) { return x * x * x; }


inline double GetIncludedCos(const Vector3D &v1, const Vector3D &v2) {
  return v1.Dot(v2) / (v1.Length() * v2.Length());
}


inline double GetIncludedAngle(const Vector3D &v1, const Vector3D &v2) {
  return acos(GetIncludedCos(v1, v2));
}


template <typename Input>
class FunctionBase {
 public:
  virtual ~FunctionBase() {}

  virtual double GetValue(Input x) const = 0;

  virtual Input GetDerivValue(Input x) const = 0;

  virtual FunctionBase *Clone() const = 0;

 protected:
  FunctionBase() {}
};


template <typename Input>
class TrivialFunction : public FunctionBase<Input> {
 public:
  TrivialFunction(std::function<double(Input)> eval_func,
                  std::function<Input(Input)> deriv_func) :
      eval_func_(eval_func), deriv_func_(deriv_func) {}

  double GetValue(Input x) const { return eval_func_(x); }

  Input GetDerivValue(Input x) const { return deriv_func_(x); }

  FunctionBase<Input> *Clone() const {
    return new TrivialFunction(eval_func_, deriv_func_);
  }

 private:
  std::function<double(Input)> eval_func_;
  std::function<Input(Input)> deriv_func_;
};


template <typename Input>
class BinaryAddFunction : public FunctionBase<Input> {
 public:
  BinaryAddFunction(FunctionBase<Input> *f1, FunctionBase<Input> *f2) :
      f1_(f1), f2_(f2) {}

  double GetValue(Input x) const {
    return f1_->GetValue(x) + f2_->GetValue(x);
  }

  Input GetDerivValue(Input x) const {
    return f1_->GetDerivValue(x) + f2_->GetDerivValue(x);
  }

  FunctionBase<Input> *Clone() const {
    return new BinaryAddFunction(f1_->Clone(), f2_->Clone());
  }

 private:
  std::unique_ptr<FunctionBase<Input>> f1_;
  std::unique_ptr<FunctionBase<Input>> f2_;
};


template <typename Input>
class BinaryMultFunction : public FunctionBase<Input> {
 public:
  BinaryMultFunction(FunctionBase<Input> *f1, FunctionBase<Input> *f2) :
      f1_(f1), f2_(f2) {}

  double GetValue(Input x) const {
    return f1_->GetValue(x) * f2_->GetValue(x);
  }

  Input GetDerivValue(Input x) const {
    return (f1_->GetDerivValue(x) * f2_->GetValue(x) +
            f2_->GetDerivValue(x) * f1_->GetValue(x));
  }

  FunctionBase<Input> *Clone() const {
    return new BinaryMultFunction(f1_->Clone(), f2_->Clone());
  }

 private:
  std::unique_ptr<FunctionBase<Input>> f1_;
  std::unique_ptr<FunctionBase<Input>> f2_;
};


template <typename Input>
class BinaryPipeFunction : public FunctionBase<Input> {
 public:
  BinaryPipeFunction(FunctionBase<double> *f1, FunctionBase<Input> *f2) :
      f1_(f1), f2_(f2) {}

  double GetValue(Input x) const {
    return f1_->GetValue(f2_->GetValue(x));
  }

  Input GetDerivValue(Input x) const {
    double m = f1_->GetDerivValue(f2_->GetValue(x));
    return f2_->GetDerivValue(x) * m;
  }

  FunctionBase<Input> *Clone() const {
    return new BinaryPipeFunction(f1_->Clone(), f2_->Clone());
  }

 private:
  std::unique_ptr<FunctionBase<double>> f1_;
  std::unique_ptr<FunctionBase<Input>> f2_;
};


template <typename Input>
class Function : public FunctionBase<Input> {
 public:
  Function(const Function &inst) :
      func_(inst.func_->Clone()), k_(inst.k()), n_(inst.n()) {}

  Function(std::function<double(Input)> eval_func,
           std::function<Input(Input)> deriv_func,
           double k = 1, double n = 1) :
      func_(new TrivialFunction<Input>(eval_func, deriv_func)), k_(k), n_(n) {}

  Function(FunctionBase<Input> *ptr) : func_(ptr), k_(1), n_(1) {}

  virtual ~Function() {}

  double k() const { return k_; }

  double set_k(double val) { k_ = val; return k(); }

  double n() const { return n_; }

  double set_n(double val) { n_ = val; return n(); }

  double GetValue(Input x) const {
    return k_ * pow(func_->GetValue(x), n_);
  }

  Input GetDerivValue(Input x) const {
    double m = k_ * n_ * pow(func_->GetValue(x), n_ - 1);
    return func_->GetDerivValue(x) * m;
  }

  FunctionBase<Input> *Clone() const {
    return new Function(*this);
  }

  double operator()(Input x) const { return GetValue(x); }

  Function operator+(const Function &f2) const {
    return Function(new BinaryAddFunction<Input>(Clone(), f2.Clone()));
  }

  Function operator-(const Function& f2) const {
    Function *inst = new Function(f2);
    inst->set_k(-(inst->k()));
    return Function(new BinaryAddFunction<Input>(Clone(), inst));
  }

  Function operator*(const Function &f2) const {
    return Function(new BinaryMultFunction<Input>(Clone(), f2.Clone()));
  }

  Function operator/(const Function &f2) const {
    Function *inst = new Function(f2);
    inst->set_n(-(inst->n()));
    return Function(new BinaryMultFunction<Input>(Clone(), inst));
  }

  template <typename Input2>
  Function<Input2> operator|(const Function<Input2> &f2) const {
    return Function<Input2>(
        new BinaryPipeFunction<Input2>(Clone(), f2.Clone()));
  }

  Function &operator=(const Function &func) {
    func_.reset(func.func_->Clone());
    set_k(func.k());
    set_n(func.n());
    return *this;
  }

 private:
  std::unique_ptr<FunctionBase<Input>> func_;
  double k_;
  double n_;
};


class ArcCosFunction : public Function<double> {
 public:
  ArcCosFunction() :
      Function<double>(acos, [](double x) { return -1 / sqrt(1 - Squ(x)); }) {}
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

#endif  // WEBCAM_UTIL_HPP
