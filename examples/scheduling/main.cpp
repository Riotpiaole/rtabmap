#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/Link.h"

#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UProcessInfo.h"
#include "rtabmap/utilite/UEventsManager.h"

using namespace rtabmap;
using namespace std;
int main(int argc , char *argv[]){

    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);
    // std::string path = "./dataset/rtabmap.db";
    // std::string path = "../stereo_20Hz/stereo_one_map_no_loop.db";
    // std::string path = "../stereo_20Hz/stereo_loop.db";
    if (argc < 1){
        printf("Please provide path to database");
        return -1;
    }

    std::string path = argv[1];

    ParametersMap parameters;

    // parameters.insert(ParametersPair(Parameters::kRtabmapWorkingDirectory(), ""));
    parameters.insert(ParametersPair(Parameters::kRtabmapPublishRAMUsage(), "true"));

    Rtabmap rtabmap;

    rtabmap.init(parameters, path);

    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, Link> links;
    std::map<int, Signature> nodes;
    std::map<int, rtabmap::Transform> optimizedPoses;

    rtabmap.get3DMap(nodes, optimizedPoses, links, true, true);

    for ( auto signature : nodes){
        printf("In map %d Signature id %d \n", signature.second.mapId() ,signature.second.id());
    }
    printf("Link Table\n---------------------------------\n");
    printf("\nKey \t Type \t From \t To\n");
    for (auto map_link : links){
        printf("%d \t %d \t %d \t %d \n",
               map_link.first,
               map_link.second.type(),
               map_link.second.from(),
               map_link.second.to());
    }

    return 0;
}