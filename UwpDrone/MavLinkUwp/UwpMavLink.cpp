#include "pch.h"
#include "MavLinkConnection.hpp"
#include "UwpMavLink.h"

using namespace MavLinkUwp;
using namespace Platform;
using namespace Windows::Foundation;
using namespace mavlinkcom;

static const int pixhawkVendorId = 9900;   ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
static const int pixhawkFMUV4ProductId = 18;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2ProductId = 17;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2OldBootloaderProductId = 22;     ///< Product ID for Bootloader on older Pixhawk V2 boards
static const int pixhawkFMUV1ProductId = 16;     ///< Product ID for PX4 FMU V1 board


UwpMavLink::UwpMavLink()
{
}

std::string findPixhawk() 
{

    auto result = MavLinkConnection::findSerialPorts(0, 0);
    for (auto iter = result.begin(); iter != result.end(); iter++)
    {
        SerialPortInfo info = *iter;
        if (info.vid == pixhawkVendorId) {
            if (info.pid == pixhawkFMUV4ProductId || info.pid == pixhawkFMUV2ProductId || info.pid == pixhawkFMUV2OldBootloaderProductId)
            {
                return std::string(info.portName.begin(), info.portName.end());
            }
        }
    }
    return "";
}

std::shared_ptr<MavLinkConnection> connectSerial()
{

    std::string name = findPixhawk();
    return MavLinkConnection::connectSerial("drone", name);
}

bool UwpMavLink::connectToMavLink()
{
    _com = connectSerial();

    return _com != nullptr;
}

