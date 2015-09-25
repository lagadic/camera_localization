//! \example pose-dlt-visp.cpp

//! [Include]
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
//! [Include]

//! [Estimation function]
vpHomogeneousMatrix pose_dlt(const std::vector< vpColVector > &wX, const std::vector< vpColVector > &x)
//! [Estimation function]
{
  //! [DLT]
  int npoints = (int)wX.size();
  vpMatrix A(2*npoints, 12, 0.);
  for(int i = 0; i < npoints; i++) { // Update matrix A using eq. 5
    A[2*i][0] = wX[i][0];
    A[2*i][1] = wX[i][1];
    A[2*i][2] = wX[i][2];
    A[2*i][3] = 1;

    A[2*i+1][4] = wX[i][0];
    A[2*i+1][5] = wX[i][1];
    A[2*i+1][6] = wX[i][2];
    A[2*i+1][7] = 1;

    A[2*i][8]  = -x[i][0] * wX[i][0];
    A[2*i][9]  = -x[i][0] * wX[i][1];
    A[2*i][10] = -x[i][0] * wX[i][2];
    A[2*i][11] = -x[i][0];

    A[2*i+1][8]  = -x[i][1] * wX[i][0];
    A[2*i+1][9]  = -x[i][1] * wX[i][1];
    A[2*i+1][10] = -x[i][1] * wX[i][2];
    A[2*i+1][11] = -x[i][1];
  }

  vpColVector D;
  vpMatrix V;

  A.svd(D, V);

  double smallestSv = D[0];
  unsigned int indexSmallestSv = 0 ;
  for (unsigned int i = 1; i < D.size(); i++) {
    if ((D[i] < smallestSv) ) {
      smallestSv = D[i];
      indexSmallestSv = i;
    }
  }

#if VISP_VERSION_INT >= VP_VERSION_INT(2, 10, 0)
  vpColVector h = V.getCol(indexSmallestSv);
#else
  vpColVector h = V.column(indexSmallestSv + 1); // Deprecated since ViSP 2.10.0
#endif

  if (h[11] < 0) // tz < 0
    h *=-1;

  // Normalization to ensure that ||r3|| = 1
  double norm = sqrt(vpMath::sqr(h[8]) + vpMath::sqr(h[9]) + vpMath::sqr(h[10]));

  h /= norm;
  //! [DLT]

  //! [Update homogeneous matrix]
  vpHomogeneousMatrix cTw;
  for (int i = 0 ; i < 3 ; i++)
    for (int j = 0 ; j < 4 ; j++)
      cTw[i][j] = h[4*i+j];
  //! [Update homogeneous matrix]

  return cTw;
}

//! [Main function]
int main()
//! [Main function]
{
  //! [Create data structures]
  int npoints = 6;

  std::vector< vpColVector > wX(npoints); // 3D points
  std::vector< vpColVector >  x(npoints); // Their 2D coordinates in the image plane

  for (int i = 0; i < npoints; i++) {
    wX[i].resize(4);
    x[i].resize(3);
  }
  //! [Create data structures]

  //! [Simulation]
  // Ground truth pose used to generate the data
  vpHomogeneousMatrix cTw_truth(-0.1, 0.1, 1.2, vpMath::rad(5), vpMath::rad(0), vpMath::rad(45));

  // Input data: 3D coordinates of at least 6 non coplanar points
  double L = 0.2;
  wX[0][0] =   -L; wX[0][1] = -L; wX[0][2] = 0;   wX[0][3] = 1; // wX_0 ( -L, -L, 0,   1)^T
  wX[1][0] =  2*L; wX[1][1] = -L; wX[1][2] = 0.2; wX[1][3] = 1; // wX_1 (-2L, -L, 0.2, 1)^T
  wX[2][0] =    L; wX[2][1] =  L; wX[2][2] = 0.2; wX[2][3] = 1; // wX_2 (  L,  L, 0.2, 1)^T
  wX[3][0] =   -L; wX[3][1] =  L; wX[3][2] = 0;   wX[3][3] = 1; // wX_3 ( -L,  L, 0,   1)^T
  wX[4][0] = -2*L; wX[4][1] =  L; wX[4][2] = 0;   wX[4][3] = 1; // wX_4 (-2L,  L, 0,   1)^T
  wX[5][0] =    0; wX[5][1] =  0; wX[5][2] = 0.5; wX[5][3] = 1; // wX_5 (  0,  0, 0.5, 1)^T

  // Input data: 2D coordinates of the points on the image plane
  for(int i = 0; i < npoints; i++) {
    vpColVector cX = cTw_truth * wX[i];     // Update cX, cY, cZ
    x[i][0] = cX[0] / cX[2]; // x = cX/cZ
    x[i][1] = cX[1] / cX[2]; // y = cY/cZ
    x[i][2] = 1;
  }
  //! [Simulation]

  //! [Call function]
  vpHomogeneousMatrix cTw = pose_dlt(wX, x);
  //! [Call function]

  std::cout << "cTw (ground truth):\n" << cTw_truth << std::endl;
  std::cout << "cTw (computed with DLT):\n" << cTw << std::endl;

  return 0;
}
