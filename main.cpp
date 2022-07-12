#include <iostream>
#include <fstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types_c.h>

using namespace std;
using namespace cv;
// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
 
    return  norm(I, shouldBeIdentity) < 1e-6;
 
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));
 
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

Vec3f RotattionVectorToEulerAngles(std::vector<double> vec)
{
  //https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac

  cv::Mat rotation_metrix;
  Vec3f EulerAngles{ 0,0,0 };
  cv::Mat rotation_vector{ vec }; 
  cv::Rodrigues(rotation_vector, rotation_metrix); 
  // cout  << "旋转矩阵 = "  << rotation_metrix << endl;
  int result = isRotationMatrix(rotation_metrix);
  // cout << "是否是旋转矩阵 = " << result << endl;

  // cout.precision(4);     //调整精确度
  // cout.setf(ios::scientific);

  if (result == 1){
    EulerAngles = rotationMatrixToEulerAngles(rotation_metrix);
  }
  
  return EulerAngles;
}

int main(int, char**) {
  std::cout << "Hello, World!\n\n\n\n";

  std::vector<double> vec2{1.8063778750802852e+00, 1.6230459121882656e-02,
       1.1101101768446519e-02 };
  std::vector<double> vec3{1.8046314743119014e+00, 1.4557382856810670e-02,
       8.3065651962448833e-03 };
  std::vector<double> vec4{1.8053390554112343e+00, 1.3612879526161765e-02,
       7.3313673460015903e-03  };
  Vec3f EulerAngles_result2 = RotattionVectorToEulerAngles(vec2);
  Vec3f EulerAngles_result3 = RotattionVectorToEulerAngles(vec3);
  Vec3f EulerAngles_result4 = RotattionVectorToEulerAngles(vec4);
  cout << "L2 Front 欧拉角 xyz = " << EulerAngles_result2 << endl; 
  cout << "L3 Front 欧拉角 xyz = " << EulerAngles_result3 << endl; 
  cout << "L4 Front 欧拉角 xyz = " << EulerAngles_result4 << endl; 
}

