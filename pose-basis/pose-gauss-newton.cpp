//! \example pose-gauss-newton.cpp
//! [Include]
#include <visp/vpColVector.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
//! [Include]

//! [Estimation function]
vpHomogeneousMatrix pose_gauss_newton(
    #if VISP_VERSION_INT >= VP_VERSION_INT(2, 10, 0)
    const
    #endif
    std::vector< vpColVector > &wX, const std::vector< vpColVector > &x, const vpHomogeneousMatrix &cTw)
{
  //! [Estimation function]
  //! [Gauss-Newton]
  int npoints = (int)wX.size();
  vpMatrix J(2*npoints, 6);
  std::vector< vpColVector > cX(npoints);
  double lambda = 0.25;
  vpColVector err, sd(2*npoints), s(2*npoints), xq(npoints*2), xn(npoints*2);
  vpHomogeneousMatrix cTw_ = cTw;
  double residual=0, residual_prev;
  vpMatrix Jp;

  // From input vector x = (x, y, 1)^T we create a new one xn = (x, y)^T to ease computation of e_q
  for (int i = 0; i < x.size(); i ++) {
    xn[i*2]   = x[i][0]; // x
    xn[i*2+1] = x[i][1]; // y
  }

  // Iterative Gauss-Newton minimization loop
  do {
    for (int i = 0; i < npoints; i++) {
      cX[i] = cTw_ * wX[i];                               // Update cX, cY, cZ

      // Update x(q)
      xq[i*2]   = cX[i][0] / cX[i][2];                    // x(q) = cX/cZ
      xq[i*2+1] = cX[i][1] / cX[i][2];                    // y(q) = cY/cZ

      // Update J using equation (22)
      J[i*2][0] = -1/cX[i][2];                            // -1/cZ
      J[i*2][1] = 0;
      J[i*2][2] = x[i][0] / cX[i][2];                     // x/cZ
      J[i*2][3] = x[i][0] * x[i][1];                      // xy
      J[i*2][4] = -(1 + x[i][0] * x[i][0]);               // -(1+x^2)
      J[i*2][5] = x[i][1];                                // y

      J[i*2+1][0] = 0;
      J[i*2+1][1] = -1/cX[i][2];                          // -1/cZ
      J[i*2+1][2] = x[i][1] / cX[i][2];                   // y/cZ
      J[i*2+1][3] = 1 + x[i][1] * x[i][1];                // 1+y^2
      J[i*2+1][4] = -x[i][0] * x[i][1];                   // -xy
      J[i*2+1][5] = -x[i][1];                             // -x
    }

    vpColVector e_q = xq - xn;                            // Equation (18)

    J.pseudoInverse(Jp);                                  // Compute pseudo inverse of the Jacobian
    vpColVector dq = -lambda * Jp * e_q;                  // Equation (21)

    cTw_ = vpExponentialMap::direct(dq).inverse() * cTw_; // Update the pose

    residual_prev = residual;                             // Memorize previous residual
    residual = e_q.sumSquare();                           // Compute the actual residual

  } while (fabs(residual - residual_prev) > 0);
  //! [Gauss-Newton]
  return cTw_;
}

//! [Main function]
int main()
{
  //! [Main function]
  //! [Create data structures]
  int npoints = 4;
  std::vector< vpColVector > wX(npoints);
  std::vector< vpColVector > cX(npoints);
  std::vector< vpColVector >  x(npoints);

  for (int i = 0; i < npoints; i++) {
    wX[i].resize(4);
    cX[i].resize(4);
    x[i].resize(3);
  }
  //! [Create data structures]

  //! [Simulation]
  // Ground truth pose used to generate the data
  vpHomogeneousMatrix cTw_truth(-0.1, 0.1, 0.5, vpMath::rad(5), vpMath::rad(0), vpMath::rad(45));

  // Input data: 3D coordinates of at least 4 points
  double L = 0.2;
  wX[0][0] =  -L; wX[0][1] = -L; wX[0][2] = 0; wX[0][3] = 1; // wX_0 ( -L, -L, 0, 1)^T
  wX[1][0] = 2*L; wX[1][1] = -L; wX[1][2] = 0; wX[1][3] = 1; // wX_1 (-2L, -L, 0, 1)^T
  wX[2][0] =   L; wX[2][1] =  L; wX[2][2] = 0; wX[2][3] = 1; // wX_2 (  L,  L, 0, 1)^T
  wX[3][0] =  -L; wX[3][1] =  L; wX[3][2] = 0; wX[3][3] = 1; // wX_3 ( -L,  L, 0, 1)^T

  // Input data: 2D coordinates of the points on the image plane
  for(int i = 0; i < npoints; i++) {
    cX[i] = cTw_truth * wX[i];     // Update cX, cY, cZ
    x[i][0] = cX[i][0] / cX[i][2]; // x = cX/cZ
    x[i][1] = cX[i][1] / cX[i][2]; // y = cY/cZ
    x[i][2] = 1;
  }
  //! [Simulation]

  //! [Set pose initial value]
  // Initialize the pose to estimate near the solution
  vpHomogeneousMatrix cTw(-0.05, 0.05, 0.45, vpMath::rad(1), vpMath::rad(0), vpMath::rad(35));
  //! [Set pose initial value]

  //! [Call function]
  cTw = pose_gauss_newton(wX, x, cTw);
  //! [Call function]

  std::cout << "cTw (ground truth):\n" << cTw_truth << std::endl;
  std::cout << "cTw (from non linear method):\n" << cTw << std::endl;

  return 0;
}
