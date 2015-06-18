//tracker.hpp


#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <cstdio>
#include <algorithm>

#include "webcam/tracker.hpp"

#include "webcam/util.hpp"
#include "webcam/vector.hpp"

using namespace std;
using namespace cv;

namespace webcam {

Mat getShot(CvCapture* camera) {
  Mat frame, rFrame;

  if (!camera) {
    printf("No camera\n");
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

CvCapture* getCamera() {
  return cvCaptureFromCAM(0);
}



Mat threshold(Mat& image) {
  Mat hsvImage, bitmapLow, bitmapHigh, bitmap;
  cvtColor(image, hsvImage, COLOR_BGR2HSV);
  inRange(hsvImage, Scalar(50,100,100), Scalar(160,255,255), bitmap);
  GaussianBlur(bitmap, bitmap, Size(9, 9), 2, 2);
  return bitmap;
}

bool circleSort(Vec3f a, Vec3f b) {
  return a[0] == b[0] ? (a[1] < b[1]) : (a[0] < b[0]);
}

vector<Vec3f> findCircles(Mat &image_) {
  Mat image;
  image_.convertTo(image, CV_8U);
  vector<Vec3f> circles;
  HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, image.rows / 100, 100, 25, 0, 0);
  printf("find %lu circles\n", circles.size());
  sort(circles.begin(), circles.end(), circleSort);
  return circles;
}

void plotCircles(Mat &image, vector<Vec3f> &circles) {
  for (size_t i = 0; i < circles.size() ; i++) {
    Point center(round(circles[i][0]), round(circles[i][1]));
    int radius = round(circles[i][2]);
    circle(image, center, radius, Scalar(0, 255, 0), 5);
  }
}

bool getScaledDots(Vector2D *left, Vector2D *middle, Vector2D *right) {
  Mat rawImage = getShot(getCamera());
  cvNamedWindow("result", CV_WINDOW_AUTOSIZE);
  Mat image = threshold(rawImage);
  vector<Vec3f> circles = findCircles(image);

  plotCircles(rawImage, circles);
  imshow("result", image);
  waitKey(0);
  imshow("result", rawImage);
  waitKey(0);
  cvDestroyWindow("result");
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
  //imshow("result", image);
  //waitKey(0);
  //cvDestroyWindow("result");

  return true;
}


#ifdef VR_FINAL_TEST

class Test : public UnitTest {
 public:
  Test() : UnitTest("camera test") {
    Vector2D a, b, c;

    getCamera();

    if (getScaledDots(&a, &b, &c)) {
      printf("%f %f\n", a.x(), a.y());
      printf("%f %f\n", b.x(), b.y());
      printf("%f %f\n", c.x(), c.y());
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

#endif  // VR_FINAL_TEST

}
