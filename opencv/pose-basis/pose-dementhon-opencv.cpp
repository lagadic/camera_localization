//! \example pose-dementhon-opencv.cpp

//! [Include]
#include <iostream>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/core.hpp>
#if defined(HAVE_OPENCV_CALIB3D)
#include <opencv2/calib3d/calib3d.hpp>
#endif
#if defined(HAVE_OPENCV_CALIB)
#include <opencv2/calib.hpp>
#endif
//! [Include]

//! [Estimation function]
void pose_dementhon(const std::vector< cv::Point3d >& wX,
                    const std::vector< cv::Point2d >& x,
                    cv::Mat& ctw, cv::Mat& cRw)
  //! [Estimation function]
  {
  //! [POSIT]
  int npoints = (int)wX.size();
  cv::Mat r1, r2, r3;
  cv::Mat A(npoints, 4, CV_64F);
  for (int i = 0; i < npoints; i++) {
    A.at<double>(i, 0) = wX[i].x;
    A.at<double>(i, 1) = wX[i].y;
    A.at<double>(i, 2) = wX[i].z;
    A.at<double>(i, 3) = 1;
    }
  cv::Mat Ap = A.inv(cv::DECOMP_SVD);

  cv::Mat eps(npoints, 1, CV_64F, cv::Scalar(0)); // Initialize epsilon_i = 0
  cv::Mat Bx(npoints, 1, CV_64F);
  cv::Mat By(npoints, 1, CV_64F);
  double tx, ty, tz;
  cv::Mat I, J;
  cv::Mat Istar(3, 1, CV_64F), Jstar(3, 1, CV_64F);

  // POSIT loop
  for (unsigned int iter = 0; iter < 20; iter++) {
    for (int i = 0; i < npoints; i++) {
      Bx.at<double>(i, 0) = x[i].x * (eps.at<double>(i, 0) + 1.);
      By.at<double>(i, 0) = x[i].y * (eps.at<double>(i, 0) + 1.);
      }

    I = Ap * Bx; // Notice that the pseudo inverse
    J = Ap * By; // of matrix A is a constant that has been precompiled.

    for (int i = 0; i < 3; i++) {
      Istar.at<double>(i, 0) = I.at<double>(i, 0);
      Jstar.at<double>(i, 0) = J.at<double>(i, 0);
      }

    // Estimation of the rotation matrix
    double normI = 0, normJ = 0;
    for (int i = 0; i < 3; i++) {
      normI += Istar.at<double>(i, 0) * Istar.at<double>(i, 0);
      normJ += Jstar.at<double>(i, 0) * Jstar.at<double>(i, 0);
      }
    normI = sqrt(normI);
    normJ = sqrt(normJ);
    r1 = Istar / normI;
    r2 = Jstar / normJ;
    r3 = r1.cross(r2);

    // Estimation of the translation
    tz = 1 / normI;
    tx = tz * I.at<double>(3, 0);
    ty = tz * J.at<double>(3, 0);

    // Update epsilon_i
    for (int i = 0; i < npoints; i++) {
      eps.at<double>(i, 0) = (r3.at<double>(0, 0) * wX[i].x
                              + r3.at<double>(1, 0) * wX[i].y
                              + r3.at<double>(2, 0) * wX[i].z) / tz;
      }
    }
  //! [POSIT]

  //! [Update homogeneous matrix]
  ctw.at<double>(0, 0) = tx; // Translation tx
  ctw.at<double>(1, 0) = ty; // Translation tx
  ctw.at<double>(2, 0) = tz; // Translation tx
  for (int i = 0; i < 3; i++) { // Rotation matrix
    cRw.at<double>(0, i) = r1.at<double>(i, 0);
    cRw.at<double>(1, i) = r2.at<double>(i, 0);
    cRw.at<double>(2, i) = r3.at<double>(i, 0);
    }
  //! [Update homogeneous matrix]
  }

//! [Main function]
int main()
//! [Main function]
  {
  //! [Create data structures]
  std::vector< cv::Point3d > wX;
  std::vector< cv::Point2d >  x;
  //! [Create data structures]

  //! [Simulation]
  // Ground truth pose used to generate the data
  cv::Mat ctw_truth = (cv::Mat_<double>(3, 1) << -0.1, 0.1, 0.5); // Translation vector
  cv::Mat crw_truth = (cv::Mat_<double>(3, 1) << CV_PI / 180 * (5), CV_PI / 180 * (0), CV_PI / 180 * (45)); // Rotation vector
  cv::Mat cRw_truth(3, 3, cv::DataType<double>::type); // Rotation matrix
  cv::Rodrigues(crw_truth, cRw_truth);

  // Input data: 3D coordinates of at least 4 non coplanar points
  double L = 0.2;
  wX.push_back(cv::Point3d(-L, -L, 0)); // wX_0 (-L, -L, 0  )^T
  wX.push_back(cv::Point3d(L, -L, 0.2)); // wX_1 ( L, -L, 0.2)^T
  wX.push_back(cv::Point3d(L, L, -0.1)); // wX_2 ( L,  L,-0.1)^T
  wX.push_back(cv::Point3d(-L, L, 0)); // wX_3 (-L,  L, 0  )^T

  // Input data: 2D coordinates of the points on the image plane
  for (int i = 0; i < wX.size(); i++) {
    cv::Mat cX = cRw_truth * cv::Mat(wX[i]) + ctw_truth; // Update cX, cY, cZ
    x.push_back(cv::Point2d(cX.at<double>(0, 0) / cX.at<double>(2, 0),
                            cX.at<double>(1, 0) / cX.at<double>(2, 0))); // x = (cX/cZ, cY/cZ)
    }
  //! [Simulation]

  //! [Call function]
  cv::Mat ctw(3, 1, CV_64F); // Translation vector
  cv::Mat cRw(3, 3, CV_64F); // Rotation matrix

  pose_dementhon(wX, x, ctw, cRw);
  //! [Call function]

  std::cout << "ctw (ground truth):\n" << ctw_truth << std::endl;
  std::cout << "ctw (computed with Dementhon):\n" << ctw << std::endl;
  std::cout << "cRw (ground truth):\n" << cRw_truth << std::endl;
  std::cout << "cRw (computed with Dementhon):\n" << cRw << std::endl;

  return 0;
  }
