#include "rtabmap_ros/OccupancyGridMapWrapper.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_grid_map");

    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    // process "--params" argument
    for(int i = 1; i < argc; ++i)
    {
        if(strcmp(argv[i], "--udebug") == 0)
        {
            ULogger::setLevel(ULogger::kDebug);
        }
        else if(strcmp(argv[i], "--uinfo") == 0)
        {
            ULogger::setLevel(ULogger::kInfo);
        }
    }

    rtabmap_ros::OccupancyGridMapWrapper occupancyGridMapWrapper(argc, argv);

    constexpr double WAIT_FOR_TF = 0.3;
    std::cout << "Wait for transforms for " << WAIT_FOR_TF << " seconds...\n";
    ros::Duration(WAIT_FOR_TF).sleep();  // wait for tf to accumulate
    std::cout << "Done.\n";

    ros::spin();
    return 0;
}
