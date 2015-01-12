//! \example pose-from-homography-dlt.cpp
//! [Include]
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
//! [Include]

vpMatrix homography_dlt(const std::vector< vpColVector > &x1, const std::vector< vpColVector > &x2)
{
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
    for (unsigned int i=0; i < 9; i ++) {
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

  vpMatrix _2H1(3, 3);
  for (int i = 0 ; i < 3 ; i++)
    for (int j = 0 ; j < 3 ; j++)
      _2H1[i][j] = h[3*i+j] ;

  return _2H1;
}

//! [Estimation function]
vpHomogeneousMatrix pose_from_homography_dlt(const std::vector< vpColVector > &xw, const std::vector< vpColVector > &xo)
{
  //! [Estimation function]
  //! [Homography estimation]
  vpMatrix oHw = homography_dlt(xw, xo);
  //! [Homography estimation]

  //! [Homography normalization]
  // Normalization to ensure that ||c1|| = 1
  double norm = sqrt(vpMath::sqr(oHw[0][0]) + vpMath::sqr(oHw[1][0]) + vpMath::sqr(oHw[2][0])) ;
  oHw /= norm;
  //! [Homography normalization]

  //! [Compute M]
  vpMatrix PI_inverse(4, 3, 0);
  for (int i=0; i < 3; i++)
    PI_inverse[i][i] = 1.;

  vpMatrix M = PI_inverse * oHw;
  //! [Compute M]

  //! [Extract c1, c2, otw]
  vpColVector c1(3), c2(3), otw(3);
  for(int i=0; i < 3; i++) {
    c1[i] = M[i][0];
    c2[i] = M[i][1];
    otw[i] = M[i][2];
  }
  //! [Extract c1, c2, otw]
  //! [Compute c3]
  vpColVector c3 = vpColVector::crossProd(c1, c2);
  //! [Compute c3]

  //! [Update pose]
  vpHomogeneousMatrix oTw;
  for(int i=0; i < 3; i++) {
    oTw[i][0] = c1[i];
    oTw[i][1] = c2[i];
    oTw[i][2] = c3[i];
    oTw[i][3] = otw[i];
  }
  //! [Update pose]

  return oTw;
}

//! [Main function]
int main()
{
  //! [Main function]
  //! [Create data structures]
  int npoints = 4;

  std::vector< vpColVector > wX(npoints);  // 3D points in the world plane
  std::vector< vpColVector > oX(npoints);  // 3D points in the camera frame

  std::vector< vpColVector > xw(npoints);  // Normalized coordinates in the object frame
  std::vector< vpColVector > xo(npoints);  // Normalized coordinates in the image plane

  for (int i = 0; i < npoints; i++) {
    xw[i].resize(3);
    xo[i].resize(3);

    wX[i].resize(4);
    oX[i].resize(4);
  }
  //! [Create data structures]

  //! [Simulation]
  // Ground truth pose used to generate the data
  vpHomogeneousMatrix oTw_truth(-0.1, 0.1, 1.2, vpMath::rad(5), vpMath::rad(0), vpMath::rad(45));

  // Input data: 3D coordinates of at least 4 coplanar points
  double L = 0.2;
  wX[0][0] =   -L; wX[0][1] = -L; wX[0][2] = 0; wX[0][3] = 1; // wX_0 ( -L, -L, 0, 1)^T
  wX[1][0] =  2*L; wX[1][1] = -L; wX[1][2] = 0; wX[1][3] = 1; // wX_1 (-2L, -L, 0, 1)^T
  wX[2][0] =    L; wX[2][1] =  L; wX[2][2] = 0; wX[2][3] = 1; // wX_2 (  L,  L, 0, 1)^T
  wX[3][0] =   -L; wX[3][1] =  L; wX[3][2] = 0; wX[3][3] = 1; // wX_3 ( -L,  L, 0, 1)^T

  // Input data: 2D coordinates of the points on the image plane
  for(int i = 0; i < npoints; i++) {
    oX[i] = oTw_truth * wX[i];          // Update oX, oY, oZ
    xo[i][0] = oX[i][0] / oX[i][2];     // xo = oX/oZ
    xo[i][1] = oX[i][1] / oX[i][2];     // yo = oY/oZ
    xo[i][2] = 1;

    xw[i][0] = wX[i][0];                // xw = wX
    xw[i][1] = wX[i][1];                // xw = wY
    xw[i][2] = 1;
  }
  //! [Simulation]

  //! [Call function]
  vpHomogeneousMatrix oTw = pose_from_homography_dlt(xw, xo);
  //! [Call function]

  std::cout << "oTw (ground truth):\n" << oTw_truth << std::endl;
  std::cout << "oTw (computed with homography DLT):\n" << oTw << std::endl;

  return 0;
}
