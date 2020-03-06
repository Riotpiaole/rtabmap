#include <rtabmap/core/Odometry.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <QApplication>
#include <stdio.h>

#include "MapBuilder.h"
#include "util.h"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

using namespace rtabmap;
int main(int argc, char *argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    int driver = 0;
    if (argc < 2)
    {
        showUsage();
    }
    else
    {
        driver = atoi(argv[argc - 1]);
        if (driver < 0 || driver > 8)
        {
            UERROR("driver should be between 0 and 8.");
            showUsage();
        }
    }
    Camera* camera = 0;
    // Here is the pipeline that we will use:
    // CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

    // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
    // Set transform to camera so z is up, y is left and x going forward
    Transform opticalRotation(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    if (driver == 1)
    {
        if (!CameraOpenNI2::available())
        {
            UERROR("Not built with OpenNI2 support...");
            exit(-1);
        }
        camera = new CameraOpenNI2("", CameraOpenNI2::kTypeColorDepth, 0, opticalRotation);
    }
    else if (driver == 2)
    {
        if (!CameraFreenect::available())
        {
            UERROR("Not built with Freenect support...");
            exit(-1);
        }
        camera = new CameraFreenect(0, CameraFreenect::kTypeColorDepth, 0, opticalRotation);
    }
    else if (driver == 3)
    {
        if (!CameraOpenNICV::available())
        {
            UERROR("Not built with OpenNI from OpenCV support...");
            exit(-1);
        }
        camera = new CameraOpenNICV(false, 0, opticalRotation);
    }
    else if (driver == 4)
    {
        if (!CameraOpenNICV::available())
        {
            UERROR("Not built with OpenNI from OpenCV support...");
            exit(-1);
        }
        camera = new CameraOpenNICV(true, 0, opticalRotation);
    }
    else if (driver == 5)
    {
        if (!CameraFreenect2::available())
        {
            UERROR("Not built with Freenect2 support...");
            exit(-1);
        }
        camera = new CameraFreenect2(0, CameraFreenect2::kTypeColor2DepthSD, 0, opticalRotation);
    }
    else if (driver == 6)
    {
        if (!CameraStereoZed::available())
        {
            UERROR("Not built with ZED SDK support...");
            exit(-1);
        }
        camera = new CameraStereoZed(0, 2, 1, 1, 100, false, 0, opticalRotation);
    }
    else if (driver == 7)
    {
        if (!CameraRealSense::available())
        {
            UERROR("Not built with RealSense support...");
            exit(-1);
        }
        camera = new CameraRealSense(0, 0, 0, false, 0, opticalRotation);
    }
    else if (driver == 8)
    {
        if (!CameraRealSense2::available())
        {
            UERROR("Not built with RealSense2 support...");
            exit(-1);
        }
        camera = new CameraRealSense2("", 0, opticalRotation);
    }
    else
    {
        camera = new rtabmap::CameraOpenni("", 0, opticalRotation);
    }

    if (!camera->init())
    {
        UERROR("Camera init failed!");
    }

    if (!camera->init())
    {
        UERROR("Camera init failed!");
        exit(-1);
    }

    // CameraThread cameraThread(camera);
    // // GUI stuff, there the handler will receive RtabmapEvent and construct the map
    // // We give it the camera so the GUI can pause/resume the camera
    // QApplication app(argc, argv);
    // MapBuilder mapBuilder(&cameraThread);

    // // Create an odometry thread to process camera events, it will send OdometryEvent.
    // OdometryThread odomThread(Odometry::create());

    // ParametersMap params;
    // //param.insert(ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true")); // uncomment to create local occupancy grids

    // // Create RTAB-Map to process OdometryEvent
    // Rtabmap *rtabmap = new Rtabmap();
    // rtabmap->init(params);
    // RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

    // // Setup handlers
    // odomThread.registerToEventsManager();
    // rtabmapThread.registerToEventsManager();
    // mapBuilder.registerToEventsManager();

    // // The RTAB-Map is subscribed by default to CameraEvent, but we want
    // // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
    // // We can do that by creating a "pipe" between the camera and odometry, then
    // // only the odometry will receive CameraEvent from that camera. RTAB-Map is
    // // also subscribed to OdometryEvent by default, so no need to create a pipe between
    // // odometry and RTAB-Map.
    // UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

    // // Let's start the threads
    // rtabmapThread.start();
    // odomThread.start();
    // cameraThread.start();

    // mapBuilder.show();
    // app.exec(); // main loop

    // // remove handlers
    // mapBuilder.unregisterFromEventsManager();
    // rtabmapThread.unregisterFromEventsManager();
    // odomThread.unregisterFromEventsManager();

    // // Kill all threads
    // cameraThread.kill();
    // odomThread.join(true);
    // rtabmapThread.join(true);

    // rtabmap->close(false);
    rs2::pipeline p;
    auto profile = p.start();
    auto dev = profile.get_device();

    auto dbg = dev.as<rs2::debug_protocol>();
    std::vector<uint8_t> cmd = {0x14, 0, 0xab, 0xcd, 0x2a, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    auto res = dbg.send_and_receive_raw_data(cmd);
    int temp = res[4];
    std::cout << temp << std::endl;

    return 0;
}