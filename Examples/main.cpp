#include "StandAloneSensors.hpp"
#include <iostream>
#include <string>

int main(int argc, const char *argv[])
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <out_file_name> <period_ms> <total_duration_sec>" << std::endl;
        return 1;
    }

    float period = 30E-3f;
    if (argc >= 3)
        period = std::stof(argv[2]) * 1E-3f;

    float total_duration = 3600;
    if (argc >= 4)
        total_duration = std::stof(argv[3]);

    std::cout << "Period is " << period << "sec" << std::endl;
    std::cout << "Total duration is " << total_duration << "sec" << std::endl;


    using namespace msr::airlib;

    //GeoPoint testLocation(47.763160, -122.068534, 120.6f);

	//GeoPoint testLocation(47.7037051477, -122.1415384809, 9.93f); // 60 acres
	//GeoPoint testLocation(47.662804385, -122.1167039875, 9.93f); // marymoore
	GeoPoint testLocation(47.7631699747, -122.0685655406, 9.93f); // woodinville
	float yawOffset = 0;// static_cast<float>(91.27622  * M_PI / 180.0); // I was aligned with the road...

    std::ofstream out_file(argv[1]);
    //StandALoneSensors::createStaticData(out_file, period, total_duration, testLocation);
    //StandALoneSensors::generateBarometerStaticData(out_file, period, total_duration, testLocation);
    //StandALoneSensors::generateBarometerDynamicData(out_file, period, total_duration, testLocation);
    StandALoneSensors::generateMagnetometer2D(out_file, period, total_duration, testLocation, yawOffset, true);
	//StandALoneSensors::generateMagnetometerMap(out_file);
}