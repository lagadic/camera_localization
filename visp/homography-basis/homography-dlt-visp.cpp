//! \example homography-dlt-visp.cpp
//! [Include]
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
//! [Include]

//! [Estimation function]
vpMatrix homography_dlt(const std::vector< vpColVector > &x1, const std::vector< vpColVector > &x2)
//! [Estimation function]
{
  //! [DLT]
  int npoints = (int)x1.size();
  vpMatrix A(2*npoints, 9, 0.);

  // We need here to compute the SVD on a (n*2)*9 matrix (where n is
  // the number of points). if n == 4, the matrix has more columns
  // than rows. This kind of matrix is not supported by GSL for
  // SVD. The solution is to add an extra line with zeros
  if (npoints == 4)
    A.resize(2*npoints+1, 9);

  // Since the third line of matrix A is a linear combination of the first and second lines
  // (A is rank 2) we don't need to implement this third line
  for(int i = 0; i < npoints; i++) {      // Update matrix A using eq. 33
    A[2*i][3] = -x1[i][0];                // -xi_1
    A[2*i][4] = -x1[i][1];                // -yi_1
    A[2*i][5] = -x1[i][2];                // -1
    A[2*i][6] =  x2[i][1] * x1[i][0];     //  yi_2 * xi_1
    A[2*i][7] =  x2[i][1] * x1[i][1];     //  yi_2 * yi_1
    A[2*i][8] =  x2[i][1] * x1[i][2];     //  yi_2

    A[2*i+1][0] =  x1[i][0];              //  xi_1
    A[2*i+1][1] =  x1[i][1];              //  yi_1
    A[2*i+1][2] =  x1[i][2];              //  1
    A[2*i+1][6] = -x2[i][0] * x1[i][0];   // -xi_2 * xi_1
    A[2*i+1][7] = -x2[i][0] * x1[i][1];   // -xi_2 * yi_1
    A[2*i+1][8] = -x2[i][0] * x1[i][2];   // -xi_2
  }

  // Add an extra line with zero.
  if (npoints == 4) {
    for (int i=0; i < 9; i ++) {
      A[2*npoints][i] = 0;
    }
  }

  vpColVector D;
  vpMatrix V;

  A.svd(D, V);

  double smallestSv = D[0];
  unsigned int indexSmallestSv = 0 ;
  for (unsigned int i = 1; i < D.size(); i++) {
    if ((D[i] < smallestSv) ) {
      smallestSv = D[i];
      indexSmallestSv = i ;
    }
  }

#if VISP_VERSION_INT >= VP_VERSION_INT(2, 10, 0)
  vpColVector h = V.getCol(indexSmallestSv);
#else
  vpColVector h = V.column(indexSmallestSv + 1); // Deprecated since ViSP 2.10.0
#endif
  if (h[8] < 0) // tz < 0
    h *=-1;
  //! [DLT]

  //! [Update homography matrix]
  vpMatrix _2H1(3, 3);
  for (int i = 0 ; i < 3 ; i++)
    for (int j = 0 ; j < 3 ; j++)
      _2H1[i][j] = h[3*i+j];
  //! [Update homography matrix]

  return _2H1;
}

//! [Main function]
int main()
//! [Main function]
{
  //! [Create data structures]
  int npoints = 4;

  std::vector< vpColVector > x1(npoints);
  std::vector< vpColVector > x2(npoints);

  std::vector< vpColVector > wX(npoints);  // 3D points in the world plane
  std::vector< vpColVector > c1X(npoints); // 3D points in the camera frame 1
  std::vector< vpColVector > c2X(npoints); // 3D points in the camera frame 2

  for (int i = 0; i < npoints; i++) {
    x1[i].resize(3);
    x2[i].resize(3);

    wX[i].resize(4);
    c1X[i].resize(4);
    c2X[i].resize(4);
  }
  //! [Create data structures]

  //! [Simulation]
  // Ground truth pose used to generate the data
  vpHomogeneousMatrix c1Tw_truth(-0.1, 0.1, 1.2, vpMath::rad(5), vpMath::rad(0), vpMath::rad(45));
  vpHomogeneousMatrix c2Tc1(0.01, 0.01, 0.2, vpMath::rad(0), vpMath::rad(3), vpMath::rad(5));

  // Input data: 3D coordinates of at least 4 coplanar points
  double L = 0.2;
  wX[0][0] =   -L; wX[0][1] = -L; wX[0][2] = 0; wX[0][3] = 1; // wX_0 ( -L, -L, 0, 1)^T
  wX[1][0] =  2*L; wX[1][1] = -L; wX[1][2] = 0; wX[1][3] = 1; // wX_1 (-2L, -L, 0, 1)^T
  wX[2][0] =    L; wX[2][1] =  L; wX[2][2] = 0; wX[2][3] = 1; // wX_2 (  L,  L, 0, 1)^T
  wX[3][0] =   -L; wX[3][1] =  L; wX[3][2] = 0; wX[3][3] = 1; // wX_3 ( -L,  L, 0, 1)^T

  // Input data: 2D coordinates of the points on the image plane
  for(int i = 0; i < npoints; i++) {
    c1X[i] = c1Tw_truth * wX[i];          // Update c1X, c1Y, c1Z
    x1[i][0] = c1X[i][0] / c1X[i][2];     // x1 = c1X/c1Z
    x1[i][1] = c1X[i][1] / c1X[i][2];     // y1 = c1Y/c1Z
    x1[i][2] = 1;

    c2X[i] = c2Tc1 * c1Tw_truth * wX[i];  // Update cX, cY, cZ
    x2[i][0] = c2X[i][0] / c2X[i][2];     // x2 = c1X/c1Z
    x2[i][1] = c2X[i][1] / c2X[i][2];     // y2 = c1Y/c1Z
    x2[i][2] = 1;
  }
  //! [Simulation]

  //! [Call function]
  vpMatrix _2H1 = homography_dlt(x1, x2);
  //! [Call function]

  std::cout << "2H1 (computed with DLT):\n" << _2H1 << std::endl;

  return 0;
}
