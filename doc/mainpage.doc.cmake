/*! 

\mainpage Pose estimation for augmented reality: a hands-on survey

\tableofcontents

\section intro_sec Introduction

Augmented Reality (AR) has now progressed to the point where real-time applications are being considered and needed. At the same time it is important that synthetic elements are rendered and aligned in the scene in an accurate and visually acceptable way. In order to address these issues, real-time, robust and efficient tracking algorithms have to be considered. The tracking of objects in the scene amounts to calculating the location (or pose) between the camera and the scene.

 In the paper <a href="http://rainbow-doc.irisa.fr/pdf/2016_ieeetvcg_marchand.pdf">[pdf]</a>:

\code
E. Marchand, H. Uchiyama and F. Spindler. Pose estimation for augmented reality:
a hands-on survey. submitted
\endcode

a brief but almost self contented introduction to the most important approaches dedicated to camera localization along with a survey of the extension that have been proposed in the recent years. We also try to link these methodological concepts to the main libraries and SDK available on the market. 

The aim of this paper is then to provide researchers and practitioners with an almost  comprehensive and consolidate 
introduction to effective tools to facilitate  research in augmented reality. It is also dedicated to academics involved in teaching augmented reality at the undergraduate and graduate level. 

For most of the presented approaches, we also provide links to code of short examples. This should allow readers to easily bridge the gap between theoretical aspects and practice.  These examples have been written using <a href="http://opencv.org">OpenCV</a> but also <a href="http://visp.inria.fr">ViSP</a> developed at Inria.
This page contains the documentation of these documented source code   proposed as a supplementary material of the paper.

We hope this article and source code will be accessible and interesting to experts and students alike.

\section install_sec Installation

\subsection prereq_subsec Prerequisities

The source code available from http://github.com/lagadic/camera_localization was designed to work with <a href="http://opencv.org">OpenCV</a> or with <a href="http://visp.inria.fr">ViSP</a>. During CMake configuration the user can choose which of these two 3rd parties are used.
Prior to build this project, the user has to download and install OpenCV and/or ViSP.
For example under Ubuntu OpenCV installation is done using:
\code
sudo apt-get libopencv-dev
\endcode
while ViSP installation is performed using:
\code
sudo apt-get libvisp-dev
\endcode


\subsection build_subsec How to build

Once ViSP is installed, download the lastest source code release from github <https://github.com/lagadic/camera_localization/releases>.

Unzip the archive:
\code
$ unzip camera_localization-2.0.1.zip
\endcode

or extract the code from tarball:
\code
$ tar xvzf camera_localization-2.0.1.tar.gz
\endcode


Using cmake run:

\code
$ cd camera_localization
$ cmake .
$ make
\endcode

To generate the documentation you can run:
\code
$ make doc
\endcode

\section pose_3d_model_sec Pose estimation relying on a 3D model

\subsection pose_known_model_sec Pose estimation from a known model
 
In this section we give base algorithm for camera localization.

- <b>Pose from Direct Linear Transform method</b> using \ref tutorial-pose-dlt-opencv "OpenCV" or using \ref tutorial-pose-dlt-visp "ViSP"<br>In this first tutorial a simple solution known as Direct Linear Transform (DLT) \cite HZ01 \cite Sut74 based on the resolution of a linear system is considered to estimate the pose of the camera from at least 6 non coplanar points.

- <b>Pose from Dementhon's POSIT method</b> using \ref tutorial-pose-dementhon-opencv "OpenCV" or using \ref tutorial-pose-dementhon-visp "ViSP"<br>In this second tutorial we give Dementhon's POSIT method \cite DD95 \cite ODD96 used to estimate the pose based on the resolution of a linear system introducing additional constraints on the rotation matrix. The pose is estimated from at least 4 non coplanar points.

- <b>Pose from homography estimation</b> using \ref tutorial-pose-dlt-planar-opencv "OpenCV" or using \ref tutorial-pose-dlt-planar-visp "ViSP" <br>In this tutorial we explain how to decompose the homography to estimate the pose from at least 4 coplanar points.

- <b>Pose from a non-linear minimization method</b> using \ref tutorial-pose-gauss-newton-opencv "OpenCV" or using \ref tutorial-pose-gauss-newton-visp "ViSP" <br>In this other tutorial we give a non-linear minimization method to estimate the pose from at least 4 points. This method requires an initialization of the pose to estimate. Depending on the points planarity, this initialization could be performed using one of the previous pose algorithms.

\subsection pose_mbt_sec Extension to markerless model-based tracking

- <b>Pose from markerless model-based tracking</b> using \ref tutorial-pose-mbt-visp "ViSP" <br>This tutorial focuses on markerless model-based tracking that allows to estimate the pose of the camera.

\section motion_estimation_sec Pose estimation relying on an image model

\subsection homography_from_point_sec Homography estimation

- <b>Homography estimation</b> using \ref tutorial-homography-opencv "OpenCV" or using \ref tutorial-homography-visp "ViSP" <br>In this tutorial we describe the estimation of an homography using Direct Linear Transform (DLT) algorithm. At least 4 coplanar points are requested to achieve the estimation.

\subsection homography_from_template_tracker_sec Direct motion estimation through template matching

- <b>Direct motion estimation through template matching</b> using \ref tutorial-template-matching-visp "ViSP" <br>In this other tutorial we propose a direct motion estimation through template matching approach. Here the reference template should be planar.



*/

