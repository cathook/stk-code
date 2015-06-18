#include <math.h>
#include <stdlib.h>

#include <algorithm>
#include <vector>

#include "webcam/church_method.hpp"

#include "webcam/util.hpp"
#include "webcam/vector.hpp"
#include "webcam/functions.hpp"


namespace webcam {

ChurchMethod::ChurchMethod(const Vector3D &world_p1,
                           const Vector3D &world_p2,
                           const Vector3D &world_p3,
                           const Vector2D &photo_p1,
                           const Vector2D &photo_p2,
                           const Vector2D &photo_p3,
                           double focal_length,
                           int max_round,
                           double eps) {
  first_ = true;
  set_world_p(0, world_p1);
  set_world_p(1, world_p2);
  set_world_p(2, world_p3);
  set_photo_p(0, photo_p1);
  set_photo_p(1, photo_p2);
  set_photo_p(2, photo_p3);
  set_focal_length(focal_length);
  set_eps(eps);
  set_max_round(max_round);
}

Vector3D ChurchMethod::set_world_p(size_t i, const Vector3D &p) {
  world_p_[i] = p;
  changed_ = true;
  return world_p_[i];
}

Vector2D ChurchMethod::set_photo_p(size_t i, const Vector2D &p) {
  photo_p_[i].set_x(p.x());
  photo_p_[i].set_y(p.y());
  changed_ = true;
  return p;
}

double ChurchMethod::set_focal_length(double focal_length) {
  for (int i = 0; i < 3; ++i) {
    photo_p_[i].set_z(-focal_length);
  }
  changed_ = true;
  return focal_length;
}

double ChurchMethod::set_eps(double eps) {
  eps_ = eps;
  changed_ = true;
  return eps_;
}

int ChurchMethod::set_max_round(int max_round) {
  max_round_ = max_round;
  changed_ = true;
  return max_round_;
}

Vector3D ChurchMethod::GetCameraPos() const {
  if (changed_) {
    if (first_) {
      FirstUpdate_();
      first_ = false;
    } else {
      Update_();
    }
    changed_ = false;
  }
  return camera_p_;
}

void ChurchMethod::InitialGuest_() const {
  double r = 10;
  for (double d = 0; d <= 5; ) {
    camera_p_.set_x(1.0 * rand() / RAND_MAX * r * 2 - r);
    camera_p_.set_y(1.0 * rand() / RAND_MAX * r * 2 - r);
    camera_p_.set_z(1.0 * rand() / RAND_MAX * r * 2 - r);
    d = 10000000;
    for (int i = 0; i < 3; ++i) {
      d = std::min(d, (camera_p_ - world_p_[i]).Length());
    }
  }
}

bool ChurchMethod::FirstUpdate_() const {
  InitialGuest_();
  return Update_();
}

bool ChurchMethod::Update_() const {
  std::vector<Function<Vector3D>> funcs;
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      funcs.push_back(
          AbsFunction() | (IncludedAngle3DFunction(world_p_[i], world_p_[j]) -
                           ConstantFunction<Vector3D>(
                               photo_p_[i].IncludedAngle(photo_p_[j]))));
    }
  }
  size_t ok_ct = 0;
  for (int i = 0; ok_ct < funcs.size() && i < max_round_; ++i) {
    int id = i % funcs.size();
    bool ok = false;
    for (int j = 0; j < max_round_ && !ok; ++j) {
      double value = funcs[id].GetValue(camera_p_);
      if (value < eps_) {
        ok = true;
      } else {
        camera_p_ -= funcs[id].GetDerivValue(camera_p_) * 70;
      }
    }
    if (ok) {
      ok_ct += 1;
    } else {
      ok_ct = 0;
    }
  }
  return (ok_ct == funcs.size());
}


bool ChurchMethod2::Update_() const {
  std::vector<Function<Vector3D>> funcs;
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      auto theta = IncludedAngle3DFunction(world_p_[i], world_p_[j]);
      auto phi = photo_p_[i].IncludedAngle(photo_p_[j]);
      funcs.push_back(theta - ConstantFunction<Vector3D>(phi));
      funcs.back().set_n(2);
    }
  }
  Function<Vector3D> func = funcs[0];
  for (size_t i = 1; i < funcs.size(); ++i) {
    func = func + funcs[i];
  }
  for (int i = 0; i < max_round_; ++i) {
    if (func(camera_p_) < eps_) {
      return true;
    } else {
      camera_p_ -= func.GetDerivValue(camera_p_) * 100;
    }
  }
  return false;
}


bool ChurchMethod3::FirstUpdate_() const {
  bool ok = false;
  for (int i = 0; i < 100 && !ok; ++i) {
    InitialGuest_();
    ok = Update_();
  }
  return ok;
}

#ifdef VR_FINAL_TEST

namespace {

class Test1 : public UnitTest {
 public:
  Test1() : UnitTest("church method 1") {
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
    SetResult(ret);
  }

 private:
  static Test1 _;
};

Test1 Test1::_;

}  // namespace

#endif  // VR_FINAL_TEST

};  // namespace webcam
