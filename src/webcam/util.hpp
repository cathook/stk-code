#ifndef WEBCAM_UTIL_HPP
#define WEBCAM_UTIL_HPP

#include <stdio.h>

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


inline double NoEPS(double x, double eps) {
  return (-eps < x && x < eps ? 0 : x);
}


template <typename X>
inline X Squ(X x) { return x * x; }


template <typename X>
inline X Tri(X x) { return x * x * x; }


#ifdef VR_FINAL_TEST


class UnitTest {
 public:
  virtual ~UnitTest() {}

  void SetResult(bool result) {
    if (result) {
      printf("test passed\n\n");
    } else {
      printf("test failed\n\n");
    }
    result_ = result_ && result;
  }

  static bool GetResult() {
    return result_;
  }

 protected:
  UnitTest(const std::string test_name) {
    printf("======= unittest -- %s ========\n", test_name.c_str());
  }

 private:
  static bool result_;
};

#endif  // VR_FINAL_TEST


}  // namespace webcam

#endif  // WEBCAM_UTIL_HPP
