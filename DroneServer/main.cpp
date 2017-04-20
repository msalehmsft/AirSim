// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <string>
#include "rpc/RpcLibServer.hpp"
#include "controllers/MavLinkDroneController.hpp"
#include "controllers/Settings.hpp"

using namespace std;
using namespace msr::airlib;

void printUsage() {
    cout << "Usage: DroneServer" << endl;
    cout << "Start the DroneServer using the 'Pixhawk' settings in ~/Documents/AirSim/settings.json." << endl;
}

int main(int argc, const char* argv[])
{
    MavLinkDroneController::ConnectionInfo connection_info;
    connection_info.vehicle_name = "Pixhawk";
    
    // read settings and override defaults
    Settings& settings = Settings::singleton();
    Settings child;
    auto settings_filename = Settings::singleton().getFileName();
    if (!settings_filename.empty()) {
        settings.getChild(connection_info.vehicle_name, child);

        // allow json overrides on a per-vehicle basis.
        connection_info.sim_sysid = static_cast<msr::airlib::uint8_t>(child.getInt("SimSysID", connection_info.sim_sysid));
        connection_info.sim_compid = child.getInt("SimCompID", connection_info.sim_compid);

        connection_info.vehicle_sysid = static_cast<msr::airlib::uint8_t>(child.getInt("VehicleSysID", connection_info.vehicle_sysid));
        connection_info.vehicle_compid = child.getInt("VehicleCompID", connection_info.vehicle_compid);

        connection_info.offboard_sysid = static_cast<msr::airlib::uint8_t>(child.getInt("OffboardSysID", connection_info.offboard_sysid));
        connection_info.offboard_compid = child.getInt("OffboardCompID", connection_info.offboard_compid);

        connection_info.logviewer_ip_address = child.getString("LogViewerHostIp", connection_info.logviewer_ip_address);
        connection_info.logviewer_ip_port = child.getInt("LogViewerPort", connection_info.logviewer_ip_port);

        connection_info.qgc_ip_address = child.getString("QgcHostIp", connection_info.qgc_ip_address);
        connection_info.qgc_ip_port = child.getInt("QgcPort", connection_info.qgc_ip_port);

        connection_info.sitl_ip_address = child.getString("SitlIp", connection_info.sitl_ip_address);
        connection_info.sitl_ip_port = child.getInt("SitlPort", connection_info.sitl_ip_port);

        connection_info.local_host_ip = child.getString("LocalHostIp", connection_info.local_host_ip);

        connection_info.use_serial = child.getBool("UseSerial", connection_info.use_serial);
        connection_info.ip_address = child.getString("UdpIp", connection_info.ip_address);
        connection_info.ip_port = child.getInt("UdpPort", connection_info.ip_port);
        connection_info.serial_port = child.getString("SerialPort", connection_info.serial_port);
        connection_info.baud_rate = child.getInt("SerialBaudRate", connection_info.baud_rate);
    }

    MavLinkDroneController mav_drone;
    mav_drone.initialize(connection_info, nullptr, true);   //TODO: need to review how is_simulation flag might affect here
    mav_drone.start();

    DroneControllerCancelable server_wrapper(&mav_drone);
    msr::airlib::RpcLibServer server(&server_wrapper, connection_info.local_host_ip);
    
    auto v = std::vector<msr::airlib::uint8_t>{ 5, 4, 3 };
    server_wrapper.setImageForCamera(3, DroneControllerBase::ImageType::Depth, v);
    server_wrapper.setImageForCamera(4, DroneControllerBase::ImageType::Scene, std::vector<msr::airlib::uint8_t>{6, 5, 4, 3, 2});
    
    std::cout << "Server connected to MavLink endpoint at " << connection_info.local_host_ip << ":" << connection_info.ip_port << std::endl;
    server.start(true);
    return 0;
}
