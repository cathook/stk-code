#ifdef VR_FINAL_TEST

#include <math.h>

#include <memory>
#include <utility>
#include <vector>

#include "webcam/util.hpp"


using namespace webcam;


template <typename X>
inline X Fou(X x) { return x * x * x * x; }


template <typename X>
inline X Fri(X x) { return x * x * x * x * x; }


inline std::string DblToStr(double k) {
  char buf[100];
  sprintf(buf, "%.3f", k);
  return std::string(buf);
}


bool BinaryCombineTest(double eps) {
  // x3 = 5x^3
  Function<double> x3([](double x) { return x; }, [](double x) { return 1; },
                      5, 3);
  // x2 = 6x^2 + 7
  Function<double> x2([](double x) { return 6 * Squ(x) + 7; },
                      [](double x) { return 12 * x; });
  std::vector<std::pair<Function<double>, Function<double>>> v;

  v.push_back(std::pair<Function<double>, Function<double>>(
      x3 + x2, Function<double>(
          [](double x) { return 5 * Tri(x) + 6 * Squ(x) + 7; },
          [](double x) { return 15 * Squ(x) + 12 * x; })));
  v.push_back(std::pair<Function<double>, Function<double>>(
      x3 - x2, Function<double>(
          [](double x) { return 5 * Tri(x) - 6 * Squ(x) - 7; },
          [](double x) { return 15 * Squ(x) - 12 * x; })));
  v.push_back(std::pair<Function<double>, Function<double>>(
      x3 * x2, Function<double>(
          [](double x) { return 30 * Fri(x) + 35 * Tri(x); },
          [](double x) { return 150 * Fou(x) + 105 * Squ(x); })));
  v.push_back(std::pair<Function<double>, Function<double>>(
      x3 / x2, Function<double>(
          [](double x) { return (5 * Tri(x)) / (6 * Squ(x) + 7); },
          [](double x) { return (((15 * Squ(x)) * (6 * Squ(x) + 7) -
                                  (5 * Tri(x)) * (12 * x))
                                 / Squ(6 * Squ(x) + 7)); })));

  v.push_back(std::pair<Function<double>, Function<double>>(
      x3 | x2, Function<double>(
          [](double x) { return 5 * Tri(6 * Squ(x) + 7); },
          [](double x) { return 15 * Squ(6 * Squ(x) + 7) * (12 * x); })));

  bool ret = true;
  for (size_t i = 0; i < v.size(); ++i) {
    for (double x = 0; x <= 5; ++x) {
      printf("x = %.3f, f(x) = %.3f/%.3f, f'(x) = %.3f/%.3f",
             x,
             v[i].first(x), v[i].second(x),
             v[i].first.GetDerivValue(x), v[i].second.GetDerivValue(x));
      bool err = ((fabs(v[i].first(x) - v[i].second(x)) >= eps) &&
                  (fabs(v[i].first.GetDerivValue(x) -
                        v[i].second.GetDerivValue(x)) >= eps));
      printf(err ? "    !!!\n" : "\n");
      ret = ret && !err;
    }
  }
  return ret;
}


template <typename Input>
struct Handler {
  static std::string ToString(double x) { return FormatString("%.3f", x); }

  static double Abs(double x) { return fabs(x); }
};


template <>
struct Handler<Vector3D> {
  static std::string ToString(Vector3D x) {
    return FormatString("<%.3f,%.3f,%.3f>", x.x(), x.y(), x.z());
  }

  static double Abs(Vector3D x) { return x.Length(); }
};


template <typename Input>
class SpecialFunctionChecker {
 public:
  SpecialFunctionChecker(Function<Input> *func) : func_(func), result_(true) {}

  void Check(Input x, double exp_y, Input exp_dy, double eps_y, double eps_dy) {
    double y = func_->GetValue(x);
    Input dy = func_->GetDerivValue(x);
    printf("x = %s, f(x) = %.3f/%.3f, f'(x) = %s/%s",
           Handler<Input>::ToString(x).c_str(),
           y, exp_y,
           Handler<Input>::ToString(dy).c_str(),
           Handler<Input>::ToString(exp_dy).c_str());
    if (fabs(y - exp_y) >= eps_y ||
        Handler<Input>::Abs(dy - exp_dy) >= eps_dy) {
      printf("   !!\n");
      result_ = false;
    } else {
      printf("\n");
    }
  }

  bool GetResult() { return result_; }

 private:
  std::unique_ptr<Function<Input>> func_;
  bool result_;
};


bool SpecialFunctionTest(double eps) {
  bool ret = true;

  printf("\n===========================\n");
  printf("abs:\n");
  SpecialFunctionChecker<double> abs_checker(new AbsFunction());
  abs_checker.Check(5.3, 5.3, 1, eps, eps);
  abs_checker.Check(-5.7, 5.7, -1, eps, eps);
  ret = ret && abs_checker.GetResult();

  printf("\n===========================\n");
  printf("const:\n");
  SpecialFunctionChecker<double> const_checker(new ConstantFunction<double>(7));
  const_checker.Check(0, 7, 0, eps, eps);
  const_checker.Check(1, 7, 0, eps, eps);
  const_checker.Check(3, 7, 0, eps, eps);
  ret = ret && const_checker.GetResult();

  printf("\n===========================\n");
  printf("const:\n");
  SpecialFunctionChecker<double> acos_checker(new ArcCosFunction());
  acos_checker.Check(0.0, acos(0.0), -1 / sqrt(1 - 0.00), eps, eps);
  acos_checker.Check(0.5, acos(0.5), -1 / sqrt(1 - 0.25), eps, eps);
  acos_checker.Check(0.6, acos(0.6), -1 / sqrt(1 - 0.36), eps, eps);
  ret = ret && acos_checker.GetResult();

  printf("\n===========================\n");
  printf("dist:\n");
  SpecialFunctionChecker<Vector3D> dist_checker(
      new DistanceFunction(Vector3D(0, 0, 1)));
  dist_checker.Check(
      Vector3D(0, 0, 1.1), 0.1, Vector3D(0, 0, 0.1) / 0.1, eps, eps * 3);
  dist_checker.Check(
      Vector3D(0, 1, 1), 1, Vector3D(0, 1, 0) / 1, eps, eps * 3);
  dist_checker.Check(
      Vector3D(2, 0, 3), sqrt(8), Vector3D(2, 0, 2) / sqrt(8), eps, eps * 3);
  ret = ret && dist_checker.GetResult();

  printf("\n===========================\n");
  printf("included dot:\n");
  SpecialFunctionChecker<Vector3D> idot_checker(
      new IncludedDotFunction(Vector3D(1, 0, 0), Vector3D(0, 1, 0)));
  idot_checker.Check(
      Vector3D(0, 0, 0), 0, Vector3D(-1, -1, 0), eps, eps * 3);
  idot_checker.Check(
      Vector3D(0, 2, 3), 11, Vector3D(-1, 3, 6), eps, eps * 3);
  idot_checker.Check(
      Vector3D(3, 2, 7), 57, Vector3D(5, 3, 14), eps, eps * 3);
  ret = ret && idot_checker.GetResult();

  return ret;
}


int main() {
  bool ok;
  ok = BinaryCombineTest(1e-8);
  printf("==> %s\n\n", ok ? "True" : "False");

  ok = SpecialFunctionTest(1e-8);
  printf("==> %s\n\n", ok ? "True" : "False");

  return 0;
}

#endif  // VR_FINAL_TEST
