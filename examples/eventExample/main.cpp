#include "MapBuilder.h"

#include <rtabmap/core/Odometry.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/util3d_registration.h"

#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UProcessInfo.h"
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/core/RtabmapThread.h"

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

#include <rtabmap/core/Odometry.h>
#include <QApplication>
#include <stdio.h>
#include <signal.h>

using namespace std;

void showUsage()
{
    printf("\nUsage:\n"
           "event_demo [options] path\n"
           "  path               Folder of the sequence (e.g., \"~/KITTI/dataset/sequences/07\")\n"
           "                        containing least calib.txt, times.txt, image_0 and image_1 folders.\n"
           "                        Optional image_2, image_3 and velodyne folders.\n"
           "  --output           Output directory. By default, results are saved in \"path\".\n"
           "  --output_name      Output database name (default \"rtabmap\").\n"
           "  --gt \"path\"        Ground truth path (e.g., ~/KITTI/devkit/cpp/data/odometry/poses/07.txt)\n"
           "  --quiet            Don't show log messages and iteration updates.\n"
           "  --color            Use color images for stereo (image_2 and image_3 folders).\n"
           "  --height           Add car's height to camera local transform (1.67m).\n"
           "  --disp             Generate full disparity.\n"
           "  --exposure_comp    Do exposure compensation between left and right images.\n"
           "  --scan             Include velodyne scan in node's data.\n"
           "  --scan_step #      Scan downsample step (default=1).\n"
           "  --scan_voxel #.#   Scan voxel size (default 0.5 m).\n"
           "  --scan_k           Scan normal K (default 0).\n"
           "  --scan_radius      Scan normal radius (default 0).\n\n"
           "%s\n"
           "Example:\n\n"
           "   $ rtabmap-kitti_dataset \\\n"
           "       --Rtabmap/PublishRAMUsage true\\\n"
           "       --Rtabmap/DetectionRate 2\\\n"
           "       --Rtabmap/CreateIntermediateNodes true\\\n"
           "       --RGBD/LinearUpdate 0\\\n"
           "       --GFTT/QualityLevel 0.01\\\n"
           "       --GFTT/MinDistance 7\\\n"
           "       --OdomF2M/MaxSize 3000\\\n"
           "       --Mem/STMSize 30\\\n"
           "       --Kp/MaxFeatures 750\\\n"
           "       --Vis/MaxFeatures 1500\\\n"
           "       --gt \"~/KITTI/devkit/cpp/data/odometry/poses/07.txt\"\\\n"
           "       ~/KITTI/dataset/sequences/07\n\n",
           rtabmap::Parameters::showUsage());
    exit(1);
}

// catch ctrl-c
bool g_forever = true;
void sighandler(int sig)
{
    printf("\nSignal %d caught...\n", sig);
    g_forever = false;
}

int main(int argc, char** argv)
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    ParametersMap parameters;
    std::string path;
    std::string output;
    std::string outputName = "rtabmap";
    std::string seq;
    bool color = false;
    bool height = false;
    bool scan = false;
    bool disp = false;
    bool exposureCompensation = false;
    bool vis = false;
    int scanStep = 1;
    float scanVoxel = 0.5f;
    int scanNormalK = 0;
    float scanNormalRadius = 1.0f;
    std::string gtPath;
    bool quiet = false;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--output") == 0)
        {
            output = argv[++i];
        }
        else if (std::strcmp(argv[i], "--output_name") == 0)
        {
            outputName = argv[++i];
        }
        else if (std::strcmp(argv[i], "--quiet") == 0)
        {
            quiet = true;
        }
        else if (std::strcmp(argv[i], "--scan_step") == 0)
        {
            scanStep = atoi(argv[++i]);
            if (scanStep <= 0)
            {
                printf("scan_step should be > 0\n");
                showUsage();
            }
        }
        else if (std::strcmp(argv[i], "--scan_voxel") == 0)
        {
            scanVoxel = atof(argv[++i]);
            if (scanVoxel < 0.0f)
            {
                printf("scan_voxel should be >= 0.0\n");
                showUsage();
            }
        }
        else if (std::strcmp(argv[i], "--scan_k") == 0)
        {
            scanNormalK = atoi(argv[++i]);
            if (scanNormalK < 0)
            {
                printf("scanNormalK should be >= 0\n");
                showUsage();
            }
        }
        else if (std::strcmp(argv[i], "--scan_radius") == 0)
        {
            scanNormalRadius = atof(argv[++i]);
            if (scanNormalRadius < 0.0f)
            {
                printf("scanNormalRadius should be >= 0\n");
                showUsage();
            }
        }
        else if (std::strcmp(argv[i], "--gt") == 0)
        {
            gtPath = argv[++i];
        }
        else if (std::strcmp(argv[i], "--color") == 0)
        {
            color = true;
        }
        else if (std::strcmp(argv[i], "--height") == 0)
        {
            height = true;
        }
        else if (std::strcmp(argv[i], "--scan") == 0)
        {
            scan = true;
        }
        else if (std::strcmp(argv[i], "--disp") == 0)
        {
            disp = true;
        }
        else if (std::strcmp(argv[i], "--exposure_comp") == 0)
        {
            exposureCompensation = true;
        }
        else if (std::strcmp(argv[i], "--vis") == 0)
        {
            vis = true;
        }
    }
    UWARN("visualization tag %b", vis);
    parameters = Parameters::parseArguments(argc, argv);
    path = argv[argc - 1];
    path = uReplaceChar(path, '~', UDirectory::homeDir());
    path = uReplaceChar(path, '\\', '/');
    if (output.empty())
    {
        output = path;
    }
    else
    {
        output = uReplaceChar(output, '~', UDirectory::homeDir());
        UDirectory::makeDir(output);
    }
    parameters.insert(ParametersPair(Parameters::kRtabmapWorkingDirectory(), output));
    parameters.insert(ParametersPair(Parameters::kRtabmapPublishRAMUsage(), "true"));

    std::string pathLeftImages = path + (color ? "/image_02/data" : "/image_00/data");
    std::string pathRightImages = path + (color ? "/image_03/data" : "/image_01/data");
    std::string pathCalib = path + "/calib.txt";
    std::string pathTimes = path + "/image_03/timestamps.txt";
    std::string pathScan;

    printf("Paths:\n"
           "   Sequence number:  %s\n"
           "   Sequence path:    %s\n"
           "   Output:           %s\n"
           "   Output name:      %s\n"
           "   left images:      %s\n"
           "   right images:     %s\n"
           "   calib.txt:        %s\n"
           "   times.txt:        %s\n",
           seq.c_str(),
           path.c_str(),
           output.c_str(),
           outputName.c_str(),
           pathLeftImages.c_str(),
           pathRightImages.c_str(),
           pathCalib.c_str(),
           pathTimes.c_str());
    printf("   Exposure Compensation: %s\n", exposureCompensation ? "true" : "false");
    printf("\tDisparity:         %s\n", disp ? "true" : "false");

    // convert calib.txt to rtabmap format (yaml)
    FILE *pFile = 0;
    pFile = fopen(pathCalib.c_str(), "r");
    if (!pFile)
    {
        UERROR("Cannot open calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    cv::Mat_<double> P0(3, 4);
    cv::Mat_<double> P1(3, 4);
    cv::Mat_<double> P2(3, 4);
    cv::Mat_<double> P3(3, 4);
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P0(0, 0), &P0(0, 1), &P0(0, 2), &P0(0, 3),
               &P0(1, 0), &P0(1, 1), &P0(1, 2), &P0(1, 3),
               &P0(2, 0), &P0(2, 1), &P0(2, 2), &P0(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P1(0, 0), &P1(0, 1), &P1(0, 2), &P1(0, 3),
               &P1(1, 0), &P1(1, 1), &P1(1, 2), &P1(1, 3),
               &P1(2, 0), &P1(2, 1), &P1(2, 2), &P1(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P2(0, 0), &P2(0, 1), &P2(0, 2), &P2(0, 3),
               &P2(1, 0), &P2(1, 1), &P2(1, 2), &P2(1, 3),
               &P2(2, 0), &P2(2, 1), &P2(2, 2), &P2(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P3(0, 0), &P3(0, 1), &P3(0, 2), &P3(0, 3),
               &P3(1, 0), &P3(1, 1), &P3(1, 2), &P3(1, 3),
               &P3(2, 0), &P3(2, 1), &P3(2, 2), &P3(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    fclose(pFile);
    // get image size
    UDirectory dir(pathLeftImages);
    std::string firstImage = dir.getNextFileName();
    cv::Mat image = cv::imread(dir.getNextFilePath());
    if (image.empty())
    {
        UERROR("Failed to read first image of \"%s\"", firstImage.c_str());
        return -1;
    }

    StereoCameraModel model(outputName + "_calib",
                            image.size(), P0.colRange(0, 3), cv::Mat(), cv::Mat(), P0,
                            image.size(), P1.colRange(0, 3), cv::Mat(), cv::Mat(), P1,
                            cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat());

    if (!model.save(output, true))
    {
        UERROR("Could not save calibration!");
        return -1;
    }
    printf("Saved calibration \"%s\" to \"%s\"\n", (outputName + "_calib").c_str(), output.c_str());

    if (!parameters.empty())
    {
        printf("Parameters:\n");
        for (ParametersMap::iterator iter = parameters.begin(); iter != parameters.end(); ++iter)
        {
            printf("   %s=%s\n", iter->first.c_str(), iter->second.c_str());
        }
    }

    printf("RTAB-Map version: %s\n", RTABMAP_VERSION);

    if (quiet)
    {
        ULogger::setLevel(ULogger::kError);
    }

    //======================
    // begin odometry
    //======================
    // Transform opticalRotation(0, 0, 1, 0, -1, 0, 0, color ? -0.06 : 0, 0, -1, 0, height ? 1.67 : 0.0);
    Transform opticalRotation(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    CameraThread cameraThread(
        new CameraStereoImages(
                pathLeftImages, pathRightImages,
                false, // assume that images are already rectified
                0.0f,
                opticalRotation),
            parameters);
    ((CameraStereoImages *)cameraThread.camera())->setTimestamps(false, pathTimes, false);
    if (exposureCompensation)
    {
        cameraThread.setStereoExposureCompensation(true);
    }
    if (disp)
    {
        cameraThread.setStereoToDepth(true);
    }
    if (!gtPath.empty())
    {
        ((CameraStereoImages *)cameraThread.camera())->setGroundTruthPath(gtPath, 2);
    }
    if (!pathScan.empty())
    {
        ((CameraStereoImages *)cameraThread.camera())->setScanPath(pathScan, 130000, Transform(-0.27f, 0.0f, 0.08 + (height ? 1.67f : 0.0f), 0.0f, 0.0f, 0.0f));
        cameraThread.setScanParameters(
            false,
            scanStep,
            0,
            scanVoxel,
            scanNormalK,
            scanNormalRadius,
            true);
    }
    float detectionRate = Parameters::defaultRtabmapDetectionRate();
    bool intermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
    int odomStrategy = Parameters::defaultOdomStrategy();
    // int odomStrategy = 2;
    Parameters::parse(parameters, Parameters::kOdomStrategy(), odomStrategy);
    Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), detectionRate);
    Parameters::parse(parameters, Parameters::kRtabmapCreateIntermediateNodes(), intermediateNodes);

    int cameraRate = 10;
    int odomUpdate = 2;

    // assuming source is 10 Hz
    int mapUpdate = detectionRate > 0 ? 10 / detectionRate : 1;
    if (mapUpdate < 1)
    {
        mapUpdate = 1;
    }

    QApplication app(argc, argv);
    MapBuilder mapBuilder(&cameraThread);
    printf("Camera rate = %d Hz\n", cameraRate);
    printf("Odometry update rate = %d Hz\n", cameraRate / odomUpdate);
    printf("Map update rate = %d Hz\n", (cameraRate / odomUpdate) / mapUpdate);

    std::string databasePath = output + "/" + outputName + ".db";
    UFile::erase(databasePath);

    if (!cameraThread.camera()->init(output, outputName + "_calib"))
    {
        printf("Camera Thread doesn't work");
        return -1;
    }

    // Create an odometry thread to process camera events, it will send OdometryEvent.
    ParametersMap odomParameters = parameters;
    odomParameters.erase(Parameters::kRtabmapPublishRAMUsage()); // as odometry is in the same process than rtabmap, don't get RAM usage in odometry.

    OdometryThread odomThread(Odometry::create(odomParameters));

    Rtabmap * rtabmap = new Rtabmap();
    rtabmap->init(parameters, databasePath);

    RtabmapThread rtabmapThread(rtabmap);

    // Setup handlers
    odomThread.registerToEventsManager();
    rtabmapThread.registerToEventsManager();
    mapBuilder.registerToEventsManager();

    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

    // Let's start the threads
    rtabmapThread.start();
    odomThread.start();
    cameraThread.start();

    mapBuilder.show();
    app.exec(); // main loopxx

    // remove handlers
    mapBuilder.unregisterFromEventsManager();
    rtabmapThread.unregisterFromEventsManager();
    odomThread.unregisterFromEventsManager();

     // Kill all threads
    cameraThread.kill();
    odomThread.join(true);
    rtabmapThread.join(true);

    rtabmap->close(false);
}