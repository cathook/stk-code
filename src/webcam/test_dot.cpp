#include "webcam/util.hpp"
#include "webcam/tracker.hpp"

using namespace webcam;
using namespace std;

int main() {
  Vector2D a, b, c;

  getCamera();

  if (getScaledDots(&a, &b, &c)) {
    printf("%f %f\n", a.x(), a.y());
    printf("%f %f\n", b.x(), b.y());
    printf("%f %f\n", c.x(), c.y());
  }
}
