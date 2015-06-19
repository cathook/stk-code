#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

#include "webcam/webcam.hpp"

#include "utils/vec3.hpp"
#include "webcam/church_method.hpp"
#include "webcam/tracker.hpp"
#include "webcam/vector.hpp"

namespace webcam {

namespace {

const double FOCAL_LENGTH = 7.5 / 14.5;

struct AngleInfo_ {
  double hori_angle;
  double vert_angle;

  AngleInfo_() : AngleInfo_(0, 0) {}

  AngleInfo_(double hori_angle, double vert_angle) :
      hori_angle(hori_angle), vert_angle(vert_angle) {}
};

pthread_mutex_t lock_;
AngleInfo_ info_;

AngleInfo_ GetAngleInfo_() {
  pthread_mutex_lock(&lock_);
  AngleInfo_ ret = info_;
  pthread_mutex_unlock(&lock_);
  return ret;
}

void SetAngleInfo_(const AngleInfo_ &info) {
  pthread_mutex_lock(&lock_);
  info_ = info;
  pthread_mutex_unlock(&lock_);
}


ChurchMethod *church_;
pthread_t thr_;

void *MainThread_(void *) {
  Vector2D p1, p2, p3;
  while (true) {
    if (GetScaledDots(&p1, &p2, &p3)) {
      church_->set_photo_p(0, p1);
      church_->set_photo_p(1, p2);
      church_->set_photo_p(2, p3);
      Vector3D pos = church_->GetCameraPos();

      double l = Vector2D(pos.x(), pos.z()).Length();

      SetAngleInfo_(AngleInfo_(atan2(-pos.z(), -pos.x()), atan2(l, pos.y())));
    }
  }
  return NULL;
}


}  // namespace

void Init() {
  InitTracker();

  // TODO: initial value
  church_ = new ChurchMethod(
      Vector3D(0), Vector3D(0), Vector3D(0),
      Vector2D(0), Vector2D(0), Vector2D(0),
      FOCAL_LENGTH,
      500,
      3.141592653589 / 180 * 0.0003);

  pthread_mutex_init(&lock_, NULL);
  pthread_create(&thr_, NULL, MainThread_, NULL);

  printf("Webcam initiailzed!!\n");
}

void AdjustCameraOffset(Vec3 *offs) {
  AngleInfo_ info = GetAngleInfo_();

  Vector2D xy(offs->getX(), offs->getY());
  Vector2D z(xy.Length(), offs->getZ());
  z = z.Rotate(info.vert_angle);
  xy = (xy.Normalize() * z.x()).Rotate(info.hori_angle);
  *offs = Vec3(xy.x(), xy.y(), z.y());
}

}  // namespace webcam
