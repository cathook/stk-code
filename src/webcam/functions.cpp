#include <math.h>

#include <memory>
#include <string>

#include "webcam/functions.hpp"

#include "webcam/function.hpp"
#include "webcam/util.hpp"
#include "webcam/vector.hpp"

namespace webcam {


AbsFunction::AbsFunction() : Function<double>(
    fabs, [](double x) { return (x > 0 ? 1 : (x < 0 ? -1 : 0)); }) {}


ArcCosFunction::ArcCosFunction() :
    Function<double>(acos, [](double x) { return -1 / sqrt(1 - Squ(x)); }) {}


Distance3DFunction::Distance3DFunction(const Vector3D &p0) :
    Function<Vector3D>([p0](Vector3D p) { return (p - p0).Length2(); },
                       [p0](Vector3D p) { return (p - p0) * 2; },
                       1, 0.5) {}


IncludedDot3DFunction::IncludedDot3DFunction(const Vector3D &p1,
                                             const Vector3D &p2) :
    Function<Vector3D>([p1, p2](Vector3D p) { return (p1 - p).Dot(p2 - p); },
                       [p1, p2](Vector3D p) { return p * 2 - (p1 + p2); }) {}


IncludedCos3DFunction::IncludedCos3DFunction(const Vector3D &p1,
                                             const Vector3D &p2) :
    Function<Vector3D>(IncludedDot3DFunction(p1, p2) /
                       (Distance3DFunction(p1) * Distance3DFunction(p2))) {}


IncludedAngle3DFunction::IncludedAngle3DFunction(const Vector3D &p1,
                                                 const Vector3D &p2) :
      Function<Vector3D>(ArcCosFunction() | IncludedCos3DFunction(p1, p2)) {}


#ifdef VR_FINAL_TEST

namespace {

template <typename Input>
class CheckerBase {
 public:
  CheckerBase(Function<Input> *f) : f_(f) {}

  bool Check(const Input &x, double y, const Input &dy,
             double eps_y = 1e-9, double eps_dy = 1e-9) {
    double yy = f_->GetValue(x);
    Input dyy = f_->GetDerivValue(x);

    printf("x = %s, f(x) = %.3f/%.3f, f'(x) = %s/%s",
           ToString(x).c_str(),
           yy, y,
           ToString(dyy).c_str(), ToString(dy).c_str());

    if (NoEPS(yy - y, eps_y) != 0 || NoEPS(Abs(dyy - dy), eps_dy) != 0) {
      printf("  !!\n");
      return false;
    } else {
      printf("\n");
      return true;
    }
  }

  virtual std::string ToString(const Input &p) = 0;

  virtual double Abs(const Input &p) = 0;

 private:
  std::unique_ptr<Function<Input>> f_;
};


class Checker : public CheckerBase<double> {
 public:
  Checker(Function<double> *f) : CheckerBase<double>(f) {}

  std::string ToString(const double &k) { return FormatString("%.3f", &k); }

  double Abs(const double &k) { return fabs(k); }
};


class Checker3D : public CheckerBase<Vector3D> {
 public:
  Checker3D(Function<Vector3D> *f) : CheckerBase<Vector3D>(f) {}

  std::string ToString(const Vector3D &k) { return k.ToString(); }

  double Abs(const Vector3D &k) { return k.Length(); }
};


class TestConstantFunction : public UnitTest, public Checker3D {
 public:
  TestConstantFunction() :
      UnitTest("constant function"),
      Checker3D(new ConstantFunction<Vector3D>(5)) {
    bool ok = true;
    ok = Check(Vector3D(0, 0, 0), 5, Vector3D(0, 0, 0)) && ok;
    ok = Check(Vector3D(0, 2, 1), 5, Vector3D(0, 0, 0)) && ok;
    ok = Check(Vector3D(3, -1, 4), 5, Vector3D(0, 0, 0)) && ok;
    SetResult(ok);
  }

 private:
  static TestConstantFunction _;
};

TestConstantFunction TestConstantFunction::_;


class TestAbsFunction : public UnitTest, public Checker {
 public:
  TestAbsFunction() :
      UnitTest("abs function"), Checker(new AbsFunction()) {
    bool ok = true;
    ok = Check(3, 3, 1) && ok;
    ok = Check(-3, 3, -1) && ok;
    ok = Check(0, 0, 0) && ok;
    SetResult(ok);
  }

 private:
  static TestAbsFunction _;
};

TestAbsFunction TestAbsFunction::_;


class TestArcCosFunction : public UnitTest, public Checker {
 public:
  TestArcCosFunction() :
      UnitTest("acos function"), Checker(new ArcCosFunction()) {
    bool ok = true;
    ok = Check(0.0, acos(0.0), -1 / sqrt(1 - 0.00)) && ok;
    ok = Check(0.5, acos(0.5), -1 / sqrt(1 - 0.25)) && ok;
    ok = Check(0.6, acos(0.6), -1 / sqrt(1 - 0.36)) && ok;
    SetResult(ok);
  }

 private:
  static TestArcCosFunction _;
};

TestArcCosFunction TestArcCosFunction::_;


class TestDistance3DFunction : public UnitTest, public Checker3D {
 public:
  TestDistance3DFunction () :
      UnitTest("dist3d function"),
      Checker3D(new Distance3DFunction(Vector3D(0, 0, 1))) {
    bool ok = true;
    ok = Check(Vector3D(0, 0, 1.1), 0.1, Vector3D(0, 0, 0.1) / 0.1) && ok;
    ok = Check(Vector3D(0, 1, 1), 1, Vector3D(0, 1, 0) / 1) && ok;
    ok = Check(Vector3D(2, 0, 3), sqrt(8), Vector3D(2, 0, 2) / sqrt(8)) && ok;
    SetResult(ok);
  }

 private:
  static TestDistance3DFunction _;
};

TestDistance3DFunction TestDistance3DFunction::_;


class TestIncludedDot3DFunction : public UnitTest, public Checker3D {
 public:
  TestIncludedDot3DFunction () :
      UnitTest("idot function"),
      Checker3D(new IncludedDot3DFunction(
          Vector3D(1, 0, 0), Vector3D(0, 1, 0))) {
    bool ok = true;
    ok = Check(Vector3D(0, 0, 0), 0, Vector3D(-1, -1, 0)) && ok;
    ok = Check(Vector3D(0, 2, 3), 11, Vector3D(-1, 3, 6)) && ok;
    ok = Check(Vector3D(3, 2, 7), 57, Vector3D(5, 3, 14)) && ok;
    SetResult(ok);
  }

 private:
  static TestIncludedDot3DFunction _;
};

TestIncludedDot3DFunction TestIncludedDot3DFunction::_;


}  // namespace

#endif  // VR_FINAL_TEST

}  // namespace webcam
