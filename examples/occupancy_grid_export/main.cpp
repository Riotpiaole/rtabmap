#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/utilite/UtiLite.h"
#include "rtabmap/core/OccupancyGrid.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace rtabmap;
using namespace cv;


int main(int argc , char* argv[]){

    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    if( argc != 2 ){
        UERROR("Invalid input size please provide a database path");
        return -1;
    }
    ParametersMap parameters;
    Memory* memory = new Memory(parameters);
    memory->init(argv[1], false,parameters, true);
    OccupancyGrid* grid = memory->exportOccupancyGrid();
    float cond = 0, ymin = 0;
    cv::Mat map =grid->getProbMap(cond , ymin);

    // namedWindow("window", CV_WINDOW_AUTOSIZE); // Create a window for display.
    // imshow("window", map);                     // Show our image inside it.

    // waitKey(0);
    // UDEBUG("Check cell size %i", grid->getCellSize());

    return 0;
}