#ifdef VR_FINAL_TEST


#include <utility>
#include <vector>

#include "webcam/function.hpp"
#include "webcam/util.hpp"


namespace webcam {

namespace {


template <typename X>
inline X Fou(X x) { return x * x * x * x; }


template <typename X>
inline X Fri(X x) { return x * x * x * x * x; }


class Test : public UnitTest {
 public:
  Test() : UnitTest("binary combine") {
    double eps = 1e-7;

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
    SetResult(ret);
  }

 private:
  static Test _;
};

Test Test::_;

}  // namespace

}  // namespace webcam


#endif  // VR_FINAL_TEST
