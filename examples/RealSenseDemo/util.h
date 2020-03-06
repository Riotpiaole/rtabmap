#include <stdio.h>


#include "MapBuilder.h"

// void showUsage()
// {
//     printf("\nUsage:\n"
//            "rtabmap-kitti_dataset [options] path\n"
//            "  path               Folder of the sequence (e.g., \"~/KITTI/dataset/sequences/07\")\n"
//            "                        containing least calib.txt, times.txt, image_0 and image_1 folders.\n"
//            "                        Optional image_2, image_3 and velodyne folders.\n"
//            "  --output           Output directory. By default, results are saved in \"path\".\n"
//            "  --output_name      Output database name (default \"rtabmap\").\n"
//            "  --gt \"path\"        Ground truth path (e.g., ~/KITTI/devkit/cpp/data/odometry/poses/07.txt)\n"
//            "  --quiet            Don't show log messages and iteration updates.\n"
//            "  --color            Use color images for stereo (image_2 and image_3 folders).\n"
//            "  --height           Add car's height to camera local transform (1.67m).\n"
//            "  --disp             Generate full disparity.\n"
//            "  --exposure_comp    Do exposure compensation between left and right images.\n"
//            "  --scan             Include velodyne scan in node's data.\n"
//            "  --scan_step #      Scan downsample step (default=1).\n"
//            "  --scan_voxel #.#   Scan voxel size (default 0.5 m).\n"
//            "  --scan_k           Scan normal K (default 0).\n"
//            "  --scan_radius      Scan normal radius (default 0).\n\n"
//            "%s\n"
//            "Example:\n\n"
//            "   $ rtabmap-kitti_dataset \\\n"
//            "       --Rtabmap/PublishRAMUsage true\\\n"
//            "       --Rtabmap/DetectionRate 2\\\n"
//            "       --Rtabmap/CreateIntermediateNodes true\\\n"
//            "       --RGBD/LinearUpdate 0\\\n"
//            "       --GFTT/QualityLevel 0.01\\\n"
//            "       --GFTT/MinDistance 7\\\n"
//            "       --OdomF2M/MaxSize 3000\\\n"
//            "       --Mem/STMSize 30\\\n"
//            "       --Kp/MaxFeatures 750\\\n"
//            "       --Vis/MaxFeatures 1500\\\n"
//            "       --gt \"~/KITTI/devkit/cpp/data/odometry/poses/07.txt\"\\\n"
//            "       ~/KITTI/dataset/sequences/07\n\n",
//            rtabmap::Parameters::showUsage());
//     exit(1);
// }

void showUsage()
{
    printf("\nUsage:\n"
           "rtabmap-rgbd_mapping driver\n"
           "  driver       Driver number to use: 0=OpenNI-PCL, 1=OpenNI2, 2=Freenect, 3=OpenNI-CV, 4=OpenNI-CV-ASUS, 5=Freenect2, 6=ZED SDK, 7=RealSense, 8=RealSense2\n\n");
    exit(1);
}
