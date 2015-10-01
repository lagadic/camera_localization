//! \example pose-dementhon-visp.cpp

//! [Include]
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
//! [Include]

//! [Estimation function]
vpHomogeneousMatrix pose_dementhon(const std::vector< vpColVector > &wX, const std::vector< vpColVector > &x)
//! [Estimation function]
{
  //! [POSIT]
  int npoints = (int)wX.size();
  vpColVector r1, r2, r3;
  vpMatrix A(npoints, 4);
  for(int i = 0; i < npoints; i++) {
    for (int j = 0; j < 4; j++) {
      A[i][j] = wX[i][j];
    }
  }
  vpMatrix Ap = A.pseudoInverse();

  vpColVector eps(npoints);
  eps = 0; // Initialize epsilon_i = 0

  vpColVector Bx(npoints);
  vpColVector By(npoints);
  double tx, ty, tz;
  vpMatrix I, J;
  vpColVector Istar(3), Jstar(3);

  // POSIT loop
  for(unsigned int iter = 0; iter < 20; iter ++) {
    for(int i = 0; i < npoints; i++) {
      Bx[i] = x[i][0] * (eps[i] + 1.);
      By[i] = x[i][1] * (eps[i] + 1.);
    }

    I = Ap * Bx; // Notice that the pseudo inverse
    J = Ap * By; // of matrix A is a constant that has been precompiled.

    for (int i = 0; i < 3; i++) {
      Istar[i] = I[i][0];
      Jstar[i] = J[i][0];
    }

    // Estimation of the rotation matrix
    double normI = sqrt( Istar.sumSquare() );
    double normJ = sqrt( Jstar.sumSquare() );
    r1 = Istar / normI;
    r2 = Jstar / normJ;
    r3 = vpColVector::crossProd(r1, r2);

    // Estimation of the translation
    tz = 1/normI;
    tx = tz * I[3][0];
    ty = tz * J[3][0];

    // Update epsilon_i
    for(int i = 0; i < npoints; i++)
      eps[i] = (r3[0] * wX[i][0] + r3[1] * wX[i][1] + r3[2] * wX[i][2]) / tz;
  }
  //! [POSIT]

  //! [Update homogeneous matrix]
  vpHomogeneousMatrix cTw;
  // Update translation vector
  cTw[0][3] = tx;
  cTw[1][3] = ty;
  cTw[2][3] = tz;
  cTw[3][3] = 1.;
  // update rotation matrix
  for(int i = 0; i < 3; i++) {
    cTw[0][i] = r1[i];
    cTw[1][i] = r2[i];
    cTw[2][i] = r3[i];
  }
  //! [Update homogeneous matrix]

  return cTw;
}

//! [Main function]
int main()
//! [Main function]
{
  //! [Create data structures]
  int npoints = 4;
  std::vector< vpColVector > wX(npoints); // 3D points
  std::vector< vpColVector >  x(npoints); // Their 2D coordinates in the image plane

  for (int i = 0; i < npoints; i++) {
    wX[i].resize(4);
    x[i].resize(3);
  }
  //! [Create data structures]

  //! [Simulation]
  // Ground truth pose used to generate the data
  vpHomogeneousMatrix cTw_truth(-0.1, 0.1, 0.5, vpMath::rad(5), vpMath::rad(0), vpMath::rad(45));

  // Input data: 3D coordinates of at least 4 non coplanar points
  double L = 0.2;
  wX[0][0] =   -L; wX[0][1] = -L; wX[0][2] =  0;   wX[0][3] = 1; // wX_0 ( -L, -L,  0,   1)^T
  wX[1][0] =    L; wX[1][1] = -L; wX[1][2] =  0.2; wX[1][3] = 1; // wX_1 (  L, -L,  0.2, 1)^T
  wX[2][0] =    L; wX[2][1] =  L; wX[2][2] = -0.1; wX[2][3] = 1; // wX_2 (  L,  L, -0.1, 1)^T
  wX[3][0] =   -L; wX[3][1] =  L; wX[3][2] =  0;   wX[3][3] = 1; // wX_3 ( -L,  L,  0,   1)^T

  // Input data: 2D coordinates of the points on the image plane
  for(int i = 0; i < npoints; i++) {
    vpColVector cX = cTw_truth * wX[i];     // Update cX, cY, cZ
    x[i][0] = cX[0] / cX[2]; // x = cX/cZ
    x[i][1] = cX[1] / cX[2]; // y = cY/cZ
    x[i][2] = 1.;
  }
  //! [Simulation]

  //! [Call function]
  vpHomogeneousMatrix cTw = pose_dementhon(wX, x);
  //! [Call function]

  std::cout << "cTw (ground truth):\n" << cTw_truth << std::endl;
  std::cout << "cTw (from Dementhon):\n" << cTw << std::endl;

  return 0;
}
