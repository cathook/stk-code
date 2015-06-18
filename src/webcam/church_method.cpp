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
  double r = 30;
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
//      funcs.push_back(
//          AbsFunction() | (IncludedAngle3DFunction(world_p_[i], world_p_[j]) -
//                           ConstantFunction<Vector3D>(
//                               photo_p_[i].IncludedAngle(photo_p_[j]))));
      funcs.push_back(IncludedAngle3DFunction(world_p_[i], world_p_[j]) -
                      ConstantFunction<Vector3D>(
                          photo_p_[i].IncludedAngle(photo_p_[j])));
      funcs[funcs.size() - 1].set_n(2);
    }
  }
  size_t ok_ct = 0;
  for (int i = 0; ok_ct < funcs.size() && i < max_round_; ++i) {
    int id = i % funcs.size();
    int ok_j = -1;
    for (int j = 0; j < max_round_ && ok_j < 0; ++j) {
      double value = funcs[id].GetValue(camera_p_);
//      printf("camera %s   ", camera_p_.ToString().c_str());
      if (value < eps_) {
//        printf("\n");
        ok_j = j;
      } else {
//        printf(" delta = %s\n", funcs[id].GetDerivValue(camera_p_).ToString().c_str());
        camera_p_ -= funcs[id].GetDerivValue(camera_p_) * 60;
      }
    }
//    printf("next\n");
    if (ok_j == 0) {
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
      Vector3D p[4], x0, y0;
      Vector2D w[3];
      for (bool ok = false; !ok; ) {
        for (int j = 0; j < 4; ++j) {
          double ra = j < 3 ? 50 : 15, rb = j < 3 ? 10 : 5;
          for (double dist = 0; dist < 5; ) {
            p[j].set_x(int(1.0 * rand() / RAND_MAX * ra + rb));
            p[j].set_y(int(1.0 * rand() / RAND_MAX * ra + rb));
            p[j].set_z(int(1.0 * rand() / RAND_MAX * ra + rb));
            dist = 10000;
            for (int k = 0; k < j; ++k) {
              dist = std::min(dist, (p[j] - p[k]).Length());
            }
          }
        }
        x0 = p[3].Cross(Vector3D(0, 0, 1)).Normalize();
        y0 = x0.Cross(p[3]).Normalize();
        for (int j = 0; j < 3; ++j) {
          double t = p[3].Dot(p[3]) / p[j].Dot(p[3]);
          Vector3D delta(p[j] * t - p[3]);
          w[j].set_x(x0.Dot(delta));
          w[j].set_y(y0.Dot(delta));
        }
        ok = true;
        for (int j = 0; j < 3; ++j) {
          for (int k = j + 1; k < 3; ++k) {
            if ((w[j] - w[k]).Length() < 2) {
              ok = false;
            }
          }
        }
        double area = (w[2] - w[0]).Cross(w[1] - w[0]);
        if (fabs(area) < 10) {
          ok = false;
        }
      }
      printf("World: %s   %s   %s\n",
             p[0].ToString().c_str(), p[1].ToString().c_str(),
             p[2].ToString().c_str());
      printf("F %s\n", p[3].ToString().c_str());
      printf("Photo: %s   %s   %s\n",
             w[0].ToString().c_str(), w[1].ToString().c_str(),
             w[2].ToString().c_str());
      printf("Focal Length = %.3f\n", p[3].Length());
      ChurchMethod church_method(p[0], p[1], p[2], w[0], w[1], w[2],
                                 p[3].Length(),
                                 500,
                                 3.14159265358979 / 180 * 0.0002);
      Vector3D camera = church_method.GetCameraPos();
      printf("Camera: %s", camera.ToString().c_str());
      if (camera.Length() >= 3) {
        ret = false;
        printf("    !!\n\n");
      } else {
        printf("\n\n");
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
