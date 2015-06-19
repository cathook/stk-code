#include <stdio.h>

#include <algorithm>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>

#include "webcam/tracker.hpp"

#include "webcam/disjoint_set.hpp"
#include "webcam/util.hpp"
#include "webcam/vector.hpp"

using namespace std;
using namespace cv;

namespace {

CvCapture *camera;

double aspect_ratio;

}  // namespace


namespace webcam {


Mat GetShot() {
  Mat frame, rFrame;

  if (!camera) {
    printf("No camera, please call webcam::InitTracker()\n");
    return frame;
  }

  IplImage* iplImg = cvQueryFrame(camera);

  frame = cvarrToMat(iplImg);
  if (frame.empty()) return frame;
  if (iplImg->origin == IPL_ORIGIN_TL)  return frame;
  else {
    flip(frame, rFrame, 0);
    return rFrame;
  }
}


void InitTracker() {
  camera =  cvCaptureFromCAM(0);
  if (!camera) printf("No camera found\n");
  else {
    IplImage* iplImage = cvQueryFrame(camera);
    aspect_ratio = (double)iplImage->height / iplImage->width;
  }
  cvNamedWindow("result", CV_WINDOW_AUTOSIZE);
}

double GetAspectRatio() {
  return aspect_ratio;
}

Mat Threshold(Mat& image) {
  Mat hsvImage, bitmapLow, bitmapHigh, bitmap;
  cvtColor(image, hsvImage, COLOR_BGR2HSV);
  inRange(hsvImage, Scalar(90,100,100), Scalar(180,255,255), bitmap);
  GaussianBlur(bitmap, bitmap, Size(9, 9), 2, 2);
  return bitmap;
}


bool CircleSort(Vec3f a, Vec3f b) {
  return a[0] == b[0] ? (a[1] < b[1]) : (a[0] < b[0]);
}


vector<Vec3f> FindCircles(Mat &image_) {
  Mat image;
  image_.convertTo(image, CV_8U);
  vector<Vec3f> circles;
  HoughCircles(
      image, circles, CV_HOUGH_GRADIENT, 1, image.rows / 10, 100, 30, 0, 0);
  // printf("find %lu circles\n", circles.size());
  sort(circles.begin(), circles.end(), CircleSort);
  return circles;
}


void PlotCircles(Mat &image, vector<Vec3f> &circles) {
  for (size_t i = 0; i < circles.size() ; i++) {
    Point center(round(circles[i][0]), round(circles[i][1]));
    int radius = round(circles[i][2]);
    circle(image, center, radius, Scalar(0, 255, 0), 5);
  }
}


bool GetScaledDots(Vector2D *left, Vector2D *middle, Vector2D *right) {
  Mat rawImage = GetShot();
  Mat image = Threshold(rawImage);
  vector<Vec3f> circles = FindCircles(image);

  PlotCircles(rawImage, circles);
//  imshow("result", image);
#ifdef VR_FINAL_TEST
  waitKey(0);
#endif  // VR_FINAL_TEST
  imshow("result", rawImage);
#ifdef VR_FINAL_TEST
  waitKey(0);
#endif  // VR_FINAL_TEST
  if (circles.size() != 3) return false;

  double normX = image.cols / 2;
  double normY = image.rows / 2;
  for (int i = 0; i < 3; i++) {
    circles[i][0] = (circles[i][0] - normX) / normX;
    circles[i][1] = (circles[i][1] - normY) / normY;
  }

  *left = Vector2D(circles[0][0], circles[0][1]);
  *middle = Vector2D(circles[1][0], circles[1][1]);
  *right = Vector2D(circles[2][0], circles[2][1]);

  return true;
}

bool GetScaledDots2(Vector2D *left, Vector2D *middle, Vector2D *right) {
  Mat raw_image = GetShot();
  
  GaussianBlur(raw_image, raw_image, Size(7, 7), 1.5, 1.5);

  size_t h = raw_image.size().height;
  size_t w = raw_image.size().width;

  DisjointSet dsj(h * w);

  std::vector<std::vector<double>> val(h, std::vector<double>(w));
  for (size_t i = 0; i < h; ++i) {
    for (size_t j = 0; j < w; ++j) {
      Vec3b pixel = raw_image.at<Vec3b>(i, j);
      double b = pixel.val[0], g = pixel.val[1], r = pixel.val[2];
      val[i][j] = b * .8 - g - r;
      if (val[i][j] > 0) {
        if (i > 0 && val[i - 1][j] > 0) {
          dsj.Union(i * w + j, (i - 1) * w + j);
        }
        if (j > 0 && val[i][j - 1] > 0) {
          dsj.Union(i * w + j, i * w + j - 1);
        }
      }
    }
  }
  struct Point {
    double x_sum, y_sum;
    int count;

    Point() : x_sum(0), y_sum(0), count(0) {}

    bool operator<(const Point &p) const { return (count < p.count); }
  };
  std::map<size_t, Point> ps;
  for (size_t i = 0; i < h; ++i) {
    for (size_t j = 0; j < w; ++j) {
      if (val[i][j] > 0) {
        size_t id = dsj.GetRoot(i * w + j);
        ps[id].x_sum += 2.0 * j / w - 1;
        ps[id].y_sum += 2.0 * i / h - 1;
        ps[id].count += 1;
      }
    }
  }
  std::vector<Point> pos;
  for (auto it = ps.begin(); it != ps.end(); ++it) {
    pos.push_back(it->second);
  }
  std::sort(pos.begin(), pos.end());
  bool ret = true;
  if (pos.size() >= 3) {
    std::vector<Vector2D> result;
    for (size_t i = 0; i < 3; ++i) {
      result.push_back(Vector2D(pos[i].x_sum / pos[i].count,
                                pos[i].y_sum / pos[i].count));
    }
    std::sort(result.begin(), result.end(),
              [](const Vector2D &a, const Vector2D &b) { return a.x() < b.x(); });
    *left = result[0];
    *middle = result[1];
    *right = result[2];
    for (int k = 0; k < 3; ++k) {
      double x = (result[k].x() + 1) / 2 * w;
      double y = (result[k].y() + 1) / 2 * h;
      for (size_t i = 0; i < h; ++i) {
        for (size_t j = 0; j < w; ++j) {
          if (sqrt(Squ(x - j) + Squ(y - i)) <= w / 20) {
            raw_image.at<Vec3b>(i, j)[0] = 0;
            raw_image.at<Vec3b>(i, j)[1] = 0;
            raw_image.at<Vec3b>(i, j)[2] = 0;
          }
        }
      }
    }
    ret = true;
  } else {
    ret = false;
  }
  imshow("result", raw_image);
  return ret;
}


#ifdef VR_FINAL_TEST

class Test : public UnitTest {
 public:
  Test() : UnitTest("camera test") {
    return;
    Vector2D a, b, c;

    InitTracker();

    printf("Aspect ratio is : %f \n", aspect_ratio);

    if (GetScaledDots(&a, &b, &c)) {
      printf("%6.3f %6.3f\n", a.x(), a.y());
      printf("%6.3f %6.3f\n", b.x(), b.y());
      printf("%6.3f %6.3f\n", c.x(), c.y());
    }

    printf("if okey, press [y/Y]: ");
    char s[100];
    scanf("%s", s);
    SetResult(s[0] == 'y' || s[0] == 'Y');
  }

 private:
  static Test _;
};

Test Test::_;


class Test2 : public UnitTest {
 public:
  Test2() : UnitTest("camera test") {
    Vector2D a, b, c;

    InitTracker();

    int key;
    while ((key = waitKey(20)) == -1) {
      if (GetScaledDots2(&a, &b, &c)) {
        printf("<%6.3f %6.3f>  <%6.3f %6.3f>  <%6.3f %6.3f>\n",
               a.x(), a.y(), b.x(), b.y(), c.x(), c.y());
      } else {
        printf("no\n");
      }
    }

    SetResult(key == 'y' || key == 'Y');
  }

 private:
  static Test2 _;
};

Test2 Test2::_;

#endif  // VR_FINAL_TEST

}
