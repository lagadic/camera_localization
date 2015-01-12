//! \example pose-mbt.cpp
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
//! [Include]
#include <visp/vpMbEdgeTracker.h>
//! [Include]
#include <visp/vpVideoReader.h>

int main(int argc, char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100) || defined(VISP_HAVE_FFMPEG)
  try {
    std::string videoname = "teabox.mpg";

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--name")
        videoname = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n" << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(videoname);
    std::string objectname = vpIoTools::getNameWE(videoname);

    if(! parentname.empty())
       objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname
              << ".[init,"
#ifdef VISP_HAVE_XML2
              << "xml,"
#endif
              << "cao or wrl]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;

    //! [Image]
    vpImage<unsigned char> I;
    vpCameraParameters cam;
    //! [Image]
    //! [cTw]
    vpHomogeneousMatrix cTw;
    //! [cTw]

    vpVideoReader g;
    g.setFileName(videoname);
    g.open(I);

#if defined(VISP_HAVE_X11)
    vpDisplayX display(I,100,100,"Model-based edge tracker");;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display(I,100,100,"Model-based edge tracker");;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I,100,100,"Model-based edge tracker");;
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    //! [Constructor]
    vpMbEdgeTracker tracker;
    //! [Constructor]
    bool usexml = false;
#ifdef VISP_HAVE_XML2
    //! [Load xml]
    if(vpIoTools::checkFilename(objectname + ".xml")) {
      tracker.loadConfigFile(objectname + ".xml");
      usexml = true;
    }
    //! [Load xml]
#endif
    if (! usexml) {
      //! [Set parameters]
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      me.setNbTotalSample(250);
      tracker.setMovingEdge(me);
      cam.initPersProjWithoutDistortion(839, 839, 325, 243);
      tracker.setCameraParameters(cam);
      //! [Set parameters]
    }
    //! [Load cao]
    if(vpIoTools::checkFilename(objectname + ".cao"))
      tracker.loadModel(objectname + ".cao");
    //! [Load cao]
    //! [Load wrl]
    else if(vpIoTools::checkFilename(objectname + ".wrl"))
      tracker.loadModel(objectname + ".wrl");
    //! [Load wrl]
    //! [Set display]
    tracker.setDisplayFeatures(true);
    //! [Set display]
    //! [Init]
    tracker.initClick(I, objectname + ".init", true);
    //! [Init]

    while(! g.end()){
      g.acquire(I);
      vpDisplay::display(I);
      //! [Track]
      tracker.track(I);
      //! [Track]
      //! [Get pose]
      tracker.getPose(cTw);
      //! [Get pose]
      //! [Display]
      tracker.getCameraParameters(cam);
      tracker.display(I, cTw, cam, vpColor::red, 2);
      vpDisplay::displayFrame(I, cTw, cam, 0.025, vpColor::none, 3);
      //! [Display]
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
    vpDisplay::getClick(I);
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV or ffmpeg and rebuild ViSP to use this example." << std::endl;
#endif
}
