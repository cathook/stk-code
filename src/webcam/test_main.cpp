#ifdef VR_FINAL_TEST

#include <stdlib.h>

#include "util.hpp"

int main() {
  exit(webcam::UnitTest::GetResult() ? 0 : 1);
};

#endif  // VR_FINAL_TEST
