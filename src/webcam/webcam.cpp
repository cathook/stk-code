#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

#include "webcam/webcam.hpp"

#include "webcam/church_method.hpp"
#include "webcam/tracker.hpp"
#include "webcam/util.hpp"
#include "webcam/vector.hpp"

namespace webcam {

namespace {

// const double FOCAL_LENGTH = 16.5 / 7.5;  // 7.5 / 16.5;
const double FOCAL_LENGTH = 7.5 / 16.5;

// const Vector3D world_p1(9, 0, 0);
// const Vector3D world_p2(0, 11, 0);
// const Vector3D world_p3(-9, 0, 0);

const Vector3D world_p1(5, 0, 0);
const Vector3D world_p2(0, 5, 0);
const Vector3D world_p3(-5, 0, 0);

// const Vector3D world_p1( 5, 0, 0);
// const Vector3D world_p2( 0, 7.3, 0);
// const Vector3D world_p3(-5, 0, 0);


class AngleInfo_ {
 public:
  AngleInfo_() {
    pthread_mutex_init(&lock_, NULL);
  }

  void GetAngle(double *hori, double *vert) {
    pthread_mutex_lock(&lock_);
    *hori = hori_angle_;
    *vert = vert_angle_;
    pthread_mutex_unlock(&lock_);
  }

  void SetAngle(double hori, double vert) {
    pthread_mutex_lock(&lock_);
    hori_angle_ = hori;
    vert_angle_ = vert;
    pthread_mutex_unlock(&lock_);
  }

 private:
  double hori_angle_;
  double vert_angle_;

  pthread_mutex_t lock_;
};

AngleInfo_ info_;


ChurchMethod *church_;

void *MainThread_(void *) {
  Vector2D p1, p2, p3;
  double last_hori = 0, rat = 0.6;
  while (true) {
    if (GetScaledDots2(&p1, &p2, &p3)) {
      p1.set_y(p1.y() * GetAspectRatio());
      p2.set_y(p2.y() * GetAspectRatio());
      p3.set_y(p3.y() * GetAspectRatio());

      church_->set_photo_p(0, p1);
      church_->set_photo_p(1, p2);
      church_->set_photo_p(2, p3);

      Vector3D pos = church_->GetCameraPos();

      double l = Vector2D(pos.x(), pos.z()).Length();

      double hori = atan2(-pos.x(), -pos.z());
      double vert = atan2(-pos.y(), l);

      // hori *= 1.5;
      vert = 0;

      hori = (1 - rat) * last_hori + rat * hori;

      info_.SetAngle(hori, vert);

      printf("camera pos %s  angle=%.3f, %.3f\n",
             pos.ToString().c_str(),
             hori / 3.14159 * 180, vert / 3.14159 * 180);
      last_hori = hori;
    }

    usleep(8000);
  }
  return NULL;
}


}  // namespace


void Init() {
  InitTracker();

  church_ = new ChurchMethodFirstGuess(
      world_p1, world_p2, world_p3,
      Vector2D(0), Vector2D(0), Vector2D(0),
      Vector3D(0, 0, -40),
      FOCAL_LENGTH,
      350,
      3.141592653589 / 180 * 0.0008);

  pthread_t thr;
  pthread_create(&thr, NULL, MainThread_, NULL);
}


Vector3D GetAdjustedCameraOffset(Vector3D org) {
  double hori, vert;
  info_.GetAngle(&hori, &vert);

  Vector2D xy(org.x(), org.z());
  Vector2D z(xy.Length(), org.y());
  z = z.Rotate(vert);
  xy = (xy.Normalize() * z.x()).Rotate(hori);
  return Vector3D(xy.x(), z.y(), xy.y());
}


#ifdef VR_FINAL_TEST

class Test : public UnitTest {
 public:
  Test() : UnitTest("webcam integral test") {
  }
};

#endif  // VR_FINAL_TEST

}  // namespace webcam
