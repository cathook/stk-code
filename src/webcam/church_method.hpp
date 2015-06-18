#ifndef WEBCAM_CHURCH_METHOD_HPP
#define WEBCAM_CHURCH_METHOD_HPP

#include "webcam/vector.hpp"


namespace webcam {


class ChurchMethod {
 public:
  ChurchMethod(const Vector3D &world_p1,
               const Vector3D &world_p2,
               const Vector3D &world_p3,
               const Vector2D &photo_p1,
               const Vector2D &photo_p2,
               const Vector2D &photo_p3,
               double focal_length,
               int max_round,
               double eps = 1e-1);

  virtual ~ChurchMethod() {}

  Vector3D set_world_p(size_t i, const Vector3D &p);

  Vector2D set_photo_p(size_t i, const Vector2D &p);

  double set_focal_length(double focal_length);

  double set_eps(double eps);

  int set_max_round(int max_round);

  Vector3D GetCameraPos() const;

 protected:
  virtual void InitialGuest_() const;

  virtual bool FirstUpdate_() const;

  virtual bool Update_() const;

  Vector3D world_p_[3];
  Vector3D photo_p_[3];
  double focal_length_;
  double eps_;
  int max_round_;

  mutable bool first_;
  mutable bool changed_;
  mutable Vector3D camera_p_;
};


class ChurchMethod2 : public ChurchMethod {
 public:
  ChurchMethod2(const Vector3D &world_p1,
                const Vector3D &world_p2,
                const Vector3D &world_p3,
                const Vector2D &photo_p1,
                const Vector2D &photo_p2,
                const Vector2D &photo_p3,
                double focal_length,
                int max_round,
                double eps = 1e-1) :
      ChurchMethod(world_p1, world_p2, world_p3,
                   photo_p1, photo_p2, photo_p3,
                   focal_length, max_round, eps) {}

 protected:
  bool Update_() const;
};


class ChurchMethod3 : public ChurchMethod {
 public:
  ChurchMethod3(const Vector3D &world_p1,
                const Vector3D &world_p2,
                const Vector3D &world_p3,
                const Vector2D &photo_p1,
                const Vector2D &photo_p2,
                const Vector2D &photo_p3,
                double focal_length,
                int max_round,
                double eps = 1e-1) :
      ChurchMethod(world_p1, world_p2, world_p3,
                   photo_p1, photo_p2, photo_p3,
                   focal_length, max_round, eps) {}

 protected:
  bool FirstUpdate_() const;
};

}

#endif  // WEBCAM_CHURCH_METHOD_HPP
