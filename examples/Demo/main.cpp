#include "MapBuilder.h"

#include <stdio.h>

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

void showUsage()
{
    printf("\nUsage:\n"
           "rtabmap-kitti_dataset [options] path\n"
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


using namespace std;

int main(int argc, char *argv[])
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
        }else if(std::strcmp(argv[i], "--vis") == 0 )
        {
            vis = true;
        }
    }
    UWARN("visualization tag %b", vis);
    parameters = Parameters::parseArguments(argc, argv);
    path = argv[argc - 1];
    path = uReplaceChar(path, '~', UDirectory::homeDir());

    const string middle_dir = "/sequences/";

    vector<string> seq_datasets_dirs(22);
    char seq_num[2];
    for (int i = 0; i < 22 ;i++){
        std::sprintf(seq_num ,"%02d", i);
        seq_datasets_dirs[i] = path + middle_dir + seq_num;
    }


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

    std::string pathLeftImages = path + middle_dir + "00"+ (color ? "/image_2" : "/image_0");
    std::string pathRightImages = path + middle_dir + "00" + (color ? "/image_3" : "/image_1");
    std::string pathCalib = path + middle_dir + "00/calib.txt";
    std::string pathTimes = path + middle_dir + "/00/times.txt";

    std::string pathScan;

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

    UWARN("Check Buggy code %s", pathLeftImages.c_str());
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

    float detectionRate = Parameters::defaultRtabmapDetectionRate();
    bool intermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
    // int odomStrategy = Parameters::defaultOdomStrategy();
    int odomStrategy = 2;
    Parameters::parse(parameters, Parameters::kOdomStrategy(), odomStrategy);
    Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), detectionRate);
    Parameters::parse(parameters, Parameters::kRtabmapCreateIntermediateNodes(), intermediateNodes);

    int cameraRate = 30;
    int odomUpdate = 2;

    // assuming source is 10 Hz
    int mapUpdate = detectionRate > 0 ? 30 / detectionRate : 1;
    if (mapUpdate < 1)
    {
        mapUpdate = 1;
    }

    QApplication app(argc, argv);
    MapBuilder mapBuilder;
    printf("Camera rate = %d Hz\n", cameraRate);
    printf("Odometry update rate = %d Hz\n", cameraRate / odomUpdate);
    printf("Map update rate = %d Hz\n", (cameraRate / odomUpdate) / mapUpdate);

    std::string databasePath = output + "/" + outputName + ".db";
    UFile::erase(databasePath);

    //======================
    // begin odometry
    //======================
    Transform opticalRotation(0, 0, 1, 0, -1, 0, 0, color ? -0.06 : 0, 0, -1, 0, height ? 1.67 : 0.0);
    Rtabmap rtabmap;

    UTimer totalTime;
    UTimer timer;

    ParametersMap odomParameters = parameters;
    odomParameters.erase(Parameters::kRtabmapPublishRAMUsage()); // as odometry is in the same process than rtabmap, don't get RAM usage in odometry.
    Odometry *odom = Odometry::create(odomParameters);

    rtabmap.init(parameters, databasePath);

    // for (int i = 0; i < 0 ; i++ ){
        int i = 0;
        // char seq_num[2];
        sprintf(seq_num,"/%02d", i);
        // std::string pathLeftImages = path + middle_dir + seq_num  + (color ? "/image_2" : "/image_0");
        // std::string pathRightImages = path + middle_dir + seq_num + (color ? "/image_3" : "/image_1");
        // std::string pathCalib = path + middle_dir + seq_num + "/calib.txt";
        // std::string pathTimes = path + middle_dir + seq_num + "/times.txt";
        pathLeftImages = path + middle_dir + seq_num + (color ? "/image_2" : "/image_0");
        pathRightImages = path + middle_dir + seq_num + (color ? "/image_3" : "/image_1");
        pathCalib = path + middle_dir + seq_num + "/calib.txt";
        pathTimes = path + middle_dir + seq_num + "/times.txt";
        CameraThread *cameraThread= new CameraThread(
            new CameraStereoImages(
                pathLeftImages, pathRightImages,
                false, // assume that images are already rectified
                0.0f,
                opticalRotation));
        ((CameraStereoImages *)cameraThread->camera())->setTimestamps(false, pathTimes, false);
        vector<string> all_imgs = ((CameraStereoImages *)cameraThread->camera())->filenames();
        int totalImages = (int)((CameraStereoImages *)cameraThread->camera())->filenames().size();

        UWARN("Operating currently with %s \r", seq_datasets_dirs[i].c_str());
        UWARN("Processing %d images...\n", totalImages);

        if (exposureCompensation)
        {
            cameraThread->setStereoExposureCompensation(true);
        }
        if (disp)
        {
            cameraThread->setStereoToDepth(true);
        }
        if (!gtPath.empty())
        {
            ((CameraStereoImages *)cameraThread->camera())->setGroundTruthPath(gtPath, 2);
        }
        if (!pathScan.empty())
        {
            ((CameraStereoImages *)cameraThread->camera())->setScanPath(pathScan, 130000, Transform(-0.27f, 0.0f, 0.08 + (height ? 1.67f : 0.0f), 0.0f, 0.0f, 0.0f));
            cameraThread->setScanParameters(
                false,
                scanStep,
                0,
                scanVoxel,
                scanNormalK,
                scanNormalRadius,
                true);
        }

        if (cameraThread->camera()->init(output, outputName + "_calib"))
        {
            // setup visualization
            if(vis)
            {
                mapBuilder.show();
                QApplication::processEvents();
            }

            CameraInfo cameraInfo;
            SensorData data = cameraThread->camera()->takeImage(&cameraInfo);
            int iteration = 0;

            /////////////////////////////
            // Processing dataset begin
            /////////////////////////////

            cv::Mat covariance;
            int odomKeyFrames = 0;
            printf("Press \"Space\" in the window to pause\n");
            printf("Check for vis tag %s",vis ? "true" : "false");
            while (data.isValid() && g_forever)
            {
                cameraThread->postUpdate(&data, &cameraInfo);
                cameraInfo.timeTotal = timer.ticks();

                OdometryInfo odomInfo;
                Transform pose = odom->process(data, &odomInfo);
                float speed = 0.0f;
                if (odomInfo.interval > 0.0)
                    speed = odomInfo.transform.x() / odomInfo.interval * 3.6;
                if (odomInfo.keyFrameAdded)
                {
                    ++odomKeyFrames;
                }

                if (odomStrategy == 2)
                {
                    //special case for FOVIS, set covariance 1 if 9999 is detected
                    if (!odomInfo.reg.covariance.empty() && odomInfo.reg.covariance.at<double>(0, 0) >= 9999)
                    {
                        odomInfo.reg.covariance = cv::Mat::eye(6, 6, CV_64FC1);
                    }
                }

                bool processData = true;
                if (iteration % mapUpdate != 0)
                {
                    // set negative id so rtabmap will detect it as an intermediate node
                    data.setId(-1);
                    data.setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat()); // remove features
                    processData = intermediateNodes;
                }
                if (covariance.empty() || odomInfo.reg.covariance.at<double>(0, 0) > covariance.at<double>(0, 0))
                {
                    covariance = odomInfo.reg.covariance;
                }

                timer.restart();
                if (processData)
                {
                    std::map<std::string, float> externalStats;
                    // save camera statistics to database
                    externalStats.insert(std::make_pair("Camera/BilateralFiltering/ms", cameraInfo.timeBilateralFiltering * 1000.0f));
                    externalStats.insert(std::make_pair("Camera/Capture/ms", cameraInfo.timeCapture * 1000.0f));
                    externalStats.insert(std::make_pair("Camera/Disparity/ms", cameraInfo.timeDisparity * 1000.0f));
                    externalStats.insert(std::make_pair("Camera/ImageDecimation/ms", cameraInfo.timeImageDecimation * 1000.0f));
                    externalStats.insert(std::make_pair("Camera/Mirroring/ms", cameraInfo.timeMirroring * 1000.0f));
                    externalStats.insert(std::make_pair("Camera/ExposureCompensation/ms", cameraInfo.timeStereoExposureCompensation * 1000.0f));
                    externalStats.insert(std::make_pair("Camera/ScanFromDepth/ms", cameraInfo.timeScanFromDepth * 1000.0f));
                    externalStats.insert(std::make_pair("Camera/TotalTime/ms", cameraInfo.timeTotal * 1000.0f));
                    externalStats.insert(std::make_pair("Camera/UndistortDepth/ms", cameraInfo.timeUndistortDepth * 1000.0f));
                    // save odometry statistics to database
                    externalStats.insert(std::make_pair("Odometry/LocalBundle/ms", odomInfo.localBundleTime * 1000.0f));
                    externalStats.insert(std::make_pair("Odometry/LocalBundleConstraints/", odomInfo.localBundleConstraints));
                    externalStats.insert(std::make_pair("Odometry/LocalBundleOutliers/", odomInfo.localBundleOutliers));
                    externalStats.insert(std::make_pair("Odometry/TotalTime/ms", odomInfo.timeEstimation * 1000.0f));
                    externalStats.insert(std::make_pair("Odometry/Registration/ms", odomInfo.reg.totalTime * 1000.0f));
                    externalStats.insert(std::make_pair("Odometry/Speed/kph", speed));
                    externalStats.insert(std::make_pair("Odometry/Inliers/", odomInfo.reg.inliers));
                    externalStats.insert(std::make_pair("Odometry/Features/", odomInfo.features));
                    externalStats.insert(std::make_pair("Odometry/DistanceTravelled/m", odomInfo.distanceTravelled));
                    externalStats.insert(std::make_pair("Odometry/KeyFrameAdded/", odomInfo.keyFrameAdded));
                    externalStats.insert(std::make_pair("Odometry/LocalKeyFrames/", odomInfo.localKeyFrames));
                    externalStats.insert(std::make_pair("Odometry/LocalMapSize/", odomInfo.localMapSize));
                    externalStats.insert(std::make_pair("Odometry/LocalScanMapSize/", odomInfo.localScanMapSize));

                    OdometryEvent e(SensorData(), Transform(), odomInfo);
                    rtabmap.process(data, pose, covariance, e.velocity(), externalStats);
                    covariance = cv::Mat();
                }

                ++iteration;
                if (!quiet || iteration == totalImages)
                {
                    double slamTime = timer.ticks();

                    float rmse = -1;
                    if (rtabmap.getStatistics().data().find(Statistics::kGtTranslational_rmse()) != rtabmap.getStatistics().data().end())
                    {
                        rmse = rtabmap.getStatistics().data().at(Statistics::kGtTranslational_rmse());
                    }

                    if (data.keypoints().size() == 0 && data.laserScanRaw().size())
                    {
                        if (rmse >= 0.0f)
                        {
                            //printf("Iteration %d/%d: speed=%dkm/h camera=%dms, odom(quality=%f, kfs=%d)=%dms, slam=%dms, rmse=%fm, noise stddev=%fm %frad",
                            //		iteration, totalImages, int(speed), int(cameraInfo.timeTotal*1000.0f), odomInfo.reg.icpInliersRatio, odomKeyFrames, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f), rmse, sqrt(odomInfo.reg.covariance.at<double>(0,0)), sqrt(odomInfo.reg.covariance.at<double>(3,3)));
                            printf("Iteration %d/%d: speed=%dkm/h camera=%dms, odom(quality=%f, kfs=%d)=%dms, slam=%dms, rmse=%fm",
                                iteration, totalImages, int(speed), int(cameraInfo.timeTotal * 1000.0f), odomInfo.reg.icpInliersRatio, odomKeyFrames, int(odomInfo.timeEstimation * 1000.0f), int(slamTime * 1000.0f), rmse);
                        }
                        else
                        {
                            printf("Iteration %d/%d: speed=%dkm/h camera=%dms, odom(quality=%f, kfs=%d)=%dms, slam=%dms",
                                iteration, totalImages, int(speed), int(cameraInfo.timeTotal * 1000.0f), odomInfo.reg.icpInliersRatio, odomKeyFrames, int(odomInfo.timeEstimation * 1000.0f), int(slamTime * 1000.0f));
                        }
                    }
                    else
                    {
                        if (rmse >= 0.0f)
                        {
                            //printf("Iteration %d/%d: speed=%dkm/h camera=%dms, odom(quality=%d/%d, kfs=%d)=%dms, slam=%dms, rmse=%fm, noise stddev=%fm %frad",
                            //		iteration, totalImages, int(speed), int(cameraInfo.timeTotal*1000.0f), odomInfo.reg.inliers, odomInfo.features, odomKeyFrames, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f), rmse, sqrt(odomInfo.reg.covariance.at<double>(0,0)), sqrt(odomInfo.reg.covariance.at<double>(3,3)));
                            printf("Iteration %d/%d: speed=%dkm/h camera=%dms, odom(quality=%d/%d, kfs=%d)=%dms, slam=%dms, rmse=%fm",
                                iteration, totalImages, int(speed), int(cameraInfo.timeTotal * 1000.0f), odomInfo.reg.inliers, odomInfo.features, odomKeyFrames, int(odomInfo.timeEstimation * 1000.0f), int(slamTime * 1000.0f), rmse);
                        }
                        else
                        {
                            printf("Iteration %d/%d: speed=%dkm/h camera=%dms, odom(quality=%d/%d, kfs=%d)=%dms, slam=%dms",
                                iteration, totalImages, int(speed), int(cameraInfo.timeTotal * 1000.0f), odomInfo.reg.inliers, odomInfo.features, odomKeyFrames, int(odomInfo.timeEstimation * 1000.0f), int(slamTime * 1000.0f));
                        }
                    }

                    if (processData && rtabmap.getLoopClosureId() > 0)
                    {
                        printf(" *");
                    }
                    printf("\r");
                    fflush(stdout);
                }


                if (vis)
                {
                    mapBuilder.processStatistics(rtabmap.getStatistics());
                    mapBuilder.processOdometry(data, pose, odomInfo);
                    QApplication::processEvents();

                    while (mapBuilder.isPaused() && mapBuilder.isVisible())
                    {
                        uSleep(100);
                        QApplication::processEvents();
                    }
                }
                cameraInfo = CameraInfo();
                timer.restart();
                data = cameraThread->camera()->takeImage(&cameraInfo);
            }
        }
        else
        {
            UERROR("Camera init failed!");
        }
        delete cameraThread;
    // }

    //=====================
    // end of the odometry
    //=====================


    delete odom;
    printf("Total time=%fs\n", totalTime.ticks());

    if (mapBuilder.isVisible())
    {
        printf("Processed all frames\n");
        app.exec();
    }
    /////////////////////////////
    // Processing dataset end
    /////////////////////////////

    // Save trajectory
    printf("Saving trajectory ...\n");
    std::map<int, Transform> poses;
    std::multimap<int, Link> links;
    rtabmap.getGraph(poses, links, true, true);
    std::string pathTrajectory = output + "/" + outputName + "_poses.txt";
    if (poses.size() && graph::exportPoses(pathTrajectory, 2, poses, links))
    {
        printf("Saving %s... done!\n", pathTrajectory.c_str());
    }
    else
    {
        printf("Saving %s... failed!\n", pathTrajectory.c_str());
    }

    if (!gtPath.empty())
    {
        // Log ground truth statistics
        std::map<int, Transform> groundTruth;

        for (std::map<int, Transform>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
        {
            Transform o, gtPose;
            int m, w;
            std::string l;
            double s;
            std::vector<float> v;
            GPS gps;
            EnvSensors sensors;
            rtabmap.getMemory()->getNodeInfo(iter->first, o, m, w, l, s, gtPose, v, gps, sensors, true);
            if (!gtPose.isNull())
            {
                groundTruth.insert(std::make_pair(iter->first, gtPose));
            }
        }

        // compute KITTI statistics
        float t_err = 0.0f;
        float r_err = 0.0f;
        graph::calcKittiSequenceErrors(uValues(groundTruth), uValues(poses), t_err, r_err);
        printf("Ground truth comparison:\n");
        printf("   KITTI t_err = %f %%\n", t_err);
        printf("   KITTI r_err = %f deg/m\n", r_err);

        // compute RMSE statistics
        float translational_rmse = 0.0f;
        float translational_mean = 0.0f;
        float translational_median = 0.0f;
        float translational_std = 0.0f;
        float translational_min = 0.0f;
        float translational_max = 0.0f;
        float rotational_rmse = 0.0f;
        float rotational_mean = 0.0f;
        float rotational_median = 0.0f;
        float rotational_std = 0.0f;
        float rotational_min = 0.0f;
        float rotational_max = 0.0f;
        graph::calcRMSE(
            groundTruth,
            poses,
            translational_rmse,
            translational_mean,
            translational_median,
            translational_std,
            translational_min,
            translational_max,
            rotational_rmse,
            rotational_mean,
            rotational_median,
            rotational_std,
            rotational_min,
            rotational_max);

        printf("   translational_rmse=   %f m\n", translational_rmse);
        printf("   rotational_rmse=      %f deg\n", rotational_rmse);

        pFile = 0;
        std::string pathErrors = output + "/" + outputName + "_rmse.txt";
        pFile = fopen(pathErrors.c_str(), "w");
        if (!pFile)
        {
            UERROR("could not save RMSE results to \"%s\"", pathErrors.c_str());
        }
        fprintf(pFile, "Ground truth comparison:\n");
        fprintf(pFile, "  KITTI t_err =         %f %%\n", t_err);
        fprintf(pFile, "  KITTI r_err =         %f deg/m\n", r_err);
        fprintf(pFile, "  translational_rmse=   %f\n", translational_rmse);
        fprintf(pFile, "  translational_mean=   %f\n", translational_mean);
        fprintf(pFile, "  translational_median= %f\n", translational_median);
        fprintf(pFile, "  translational_std=    %f\n", translational_std);
        fprintf(pFile, "  translational_min=    %f\n", translational_min);
        fprintf(pFile, "  translational_max=    %f\n", translational_max);
        fprintf(pFile, "  rotational_rmse=      %f\n", rotational_rmse);
        fprintf(pFile, "  rotational_mean=      %f\n", rotational_mean);
        fprintf(pFile, "  rotational_median=    %f\n", rotational_median);
        fprintf(pFile, "  rotational_std=       %f\n", rotational_std);
        fprintf(pFile, "  rotational_min=       %f\n", rotational_min);
        fprintf(pFile, "  rotational_max=       %f\n", rotational_max);
        fclose(pFile);
    }

    std::map<int, Signature> nodes;
    std::map<int, Transform> optimizedPoses;

    rtabmap.get3DMap(nodes, optimizedPoses, links, true, true);


    printf("Saving rtabmap database (with all statistics) to \"%s\"\n", (output + "/" + outputName + ".db").c_str());
    printf("Do:\n"
           " $ rtabmap-databaseViewer %s\n\n",
           (output + "/" + outputName + ".db").c_str());
    return 0;
}