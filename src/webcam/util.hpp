#ifndef WEBCAM_UTIL_HPP
#define WEBCAM_UTIL_HPP

#include <stdio.h>

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


template <typename X>
inline X Squ(X x) { return x * x; }


template <typename X>
inline X Tri(X x) { return x * x * x; }


class UnitTest {
 public:
  ~virtual UnitTest() {}

  virtual bool Run() = 0;

 protected:
  UnitTest(const std::string test_name) {
    printf("======= unittest -- %s ========\n", test_name.c_str());
    if (!Run()) {
      printf("test failed\n");
    } else {
      printf("test pass\n");
    }
  }
};


}  // namespace webcam

#endif  // WEBCAM_UTIL_HPP
