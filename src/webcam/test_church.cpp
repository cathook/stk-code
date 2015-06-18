#ifdef VR_FINAL_TEST


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <functional>
#include <vector>

#include "webcam/church_method.hpp"

using namespace webcam;

bool Test1() {
  bool ret = true;
  for (int i = 0; i < 10; ++i) {
    double f = int(1.0 * rand() / RAND_MAX * 5 + 5);
    Vector2D p1, p2, p3;
    while (fabs((p2 - p1).Cross(p3 - p1)) < 2) {
      p1 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
      p2 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
      p3 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
    }
    ChurchMethod church_method(Vector3D(p1 * 10, f * 10),
                               Vector3D(p2 * 10, f * 10),
                               Vector3D(p3 * 10, f * 10),
                               Vector2D(p1.x(), -p1.y()),
                               Vector2D(p2.x(), -p2.y()),
                               Vector2D(p3.x(), -p3.y()),
                               f,
                               5000,
                               3.14159265358979 / 180 * 0.005);
    Vector3D camera = church_method.GetCameraPos();
    printf("<%.3f, %.3f> <%.3f, %.3f> <%.3f, %.3f>  f(%.5f) =>  <%.3f, %.3f, %.3f>",
           p1.x(), p1.y(),
           p2.x(), p2.y(),
           p3.x(), p3.y(),
           f,
           camera.x(), camera.y(), camera.z());
    if (camera.Length() >= 3) {
      ret = false;
      printf("    !!\n");
    } else {
      printf("\n");
    }
  }
  return ret;
};

bool Test2() {
  bool ret = true;
  for (int i = 0; i < 10; ++i) {
    double f = int(1.0 * rand() / RAND_MAX * 5 + 5);
    Vector2D p1, p2, p3;
    while (fabs((p2 - p1).Cross(p3 - p1)) < 2) {
      p1 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
      p2 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
      p3 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
    }
    ChurchMethod2 church_method(Vector3D(p1 * 10, f * 10),
                                Vector3D(p2 * 10, f * 10),
                                Vector3D(p3 * 10, f * 10),
                                Vector2D(p1.x(), -p1.y()),
                                Vector2D(p2.x(), -p2.y()),
                                Vector2D(p3.x(), -p3.y()),
                                f,
                                5000,
                                3.14159265358979 / 180 * 0.005);
    Vector3D camera = church_method.GetCameraPos();
    printf("<%.3f, %.3f> <%.3f, %.3f> <%.3f, %.3f>  f(%.5f) =>  <%.3f, %.3f, %.3f>",
           p1.x(), p1.y(),
           p2.x(), p2.y(),
           p3.x(), p3.y(),
           f,
           camera.x(), camera.y(), camera.z());
    if (camera.Length() >= 3) {
      ret = false;
      printf("    !!\n");
    } else {
      printf("\n");
    }
  }
  return ret;
};


bool Test3() {
  bool ret = true;
  for (int i = 0; i < 10; ++i) {
    double f = int(1.0 * rand() / RAND_MAX * 5 + 5);
    Vector2D p1, p2, p3;
    while (fabs((p2 - p1).Cross(p3 - p1)) < 2) {
      p1 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
      p2 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
      p3 = Vector2D(int(1.0 * rand() / RAND_MAX * 10 - 5),
                    int(1.0 * rand() / RAND_MAX * 10 - 5));
    }
    ChurchMethod3 church_method(Vector3D(p1 * 10, f * 10),
                                Vector3D(p2 * 10, f * 10),
                                Vector3D(p3 * 10, f * 10),
                                Vector2D(p1.x(), -p1.y()),
                                Vector2D(p2.x(), -p2.y()),
                                Vector2D(p3.x(), -p3.y()),
                                f,
                                5000,
                                3.14159265358979 / 180 * 0.001);
    Vector3D camera = church_method.GetCameraPos();
    printf("<%.3f, %.3f> <%.3f, %.3f> <%.3f, %.3f>  f(%.5f) =>  <%.3f, %.3f, %.3f>",
           p1.x(), p1.y(),
           p2.x(), p2.y(),
           p3.x(), p3.y(),
           f,
           camera.x(), camera.y(), camera.z());
    if (camera.Length() >= 3) {
      ret = false;
      printf("    !!\n");
    } else {
      printf("\n");
    }
  }
  return ret;
};

int main() {
  std::vector<std::function<bool()>> cases({Test1, Test2, Test3});

  for (size_t i = 0; i < cases.size(); ++i) {
    bool ok = cases[i]();
    printf("==> %s\n\n", ok ? "True" : "False");
  }
  return 0;
};

#endif  // VR_FINAL_TEST
