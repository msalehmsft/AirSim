#include "pch.h"
#include "MavLinkConnection.hpp"
#include "MavLinkVehicle.hpp"
#include "UwpMavLink.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace MavLinkUwp;
using namespace Platform;
using namespace Windows::Foundation;
using namespace mavlinkcom;

static const int pixhawkVendorId = 9900;   ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
static const int pixhawkFMUV4ProductId = 18;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2ProductId = 17;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2OldBootloaderProductId = 22;     ///< Product ID for Bootloader on older Pixhawk V2 boards
static const int pixhawkFMUV1ProductId = 16;     ///< Product ID for PX4 FMU V1 board

const int LocalSystemId = 166;
const int LocalComponentId = 1;


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
    _vehicle = std::make_shared<MavLinkVehicle>(LocalSystemId, LocalComponentId);

    _com = connectSerial();

    _vehicle->connect(_com);

    _vehicle->startHeartbeat();


    return _com != nullptr;
}

bool UwpMavLink::arm()
{
    _vehicle->armDisarm(true);
    return true;
}

bool UwpMavLink::disarm()
{
    _vehicle->armDisarm(false);
    return true;
}

bool UwpMavLink::takeoff(float z)
{
    _vehicle->takeoff(z);
    return true;
}

bool UwpMavLink::land()
{
    const VehicleState& state = _vehicle->getVehicleState();
    _vehicle->land(state.global_est.heading, state.home.global_pos.lat, state.home.global_pos.lon, state.home.global_pos.alt);

    return true;
}

bool UwpMavLink::Goto(float x, float y, float z)
{
    requestedControl = false;
    hasControl = false;
    targetReached = false;
    settled = false;
    hasLocalPosition = false;
    targetPosition = false;
    targetVelocity = false;
    targetVelocityAltHold = false;
    targetSpeed = 0;
    theading = 0;
    paused = false;
    tx = x;
    ty = y;
    if (z > 0)
    {
        tz = -z;    // Negative coordinates
    }
    else
    {
        tz = z;
    }

    int subscription = _vehicle->getConnection()->subscribe([=](std::shared_ptr<MavLinkConnection> con, const MavLinkMessage& msg) {
        switch (msg.msgid)
        {
        case MavLinkLocalPositionNed::kMessageId: // MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            // The filtered local position
            MavLinkLocalPositionNed localPos;
            localPos.decode(msg);

            this->vx = localPos.vx;
            this->vy = localPos.vy;
            this->vz = localPos.vz;
            this->x = localPos.x;
            this->y = localPos.y;
            this->z = localPos.z;

            if (paused) {
                return;
            }

            if (_vehicle->hasOffboardControl()) {
                this->hasControl = true;
            }
            else
            {
                // not ready for offboard control, or we have lost offboard control.
                if (this->hasControl) {
                    this->hasControl = false;
                    _vehicle->releaseControl();
                    this->requestedControl = false;
                }
                return;
            }


            if (!this->hasLocalPosition) {
                this->hasLocalPosition = true;
                HasLocalPosition();
            }

            if (targetPosition) {
                // must send these regularly to keep offboard control.
                _vehicle->moveToLocalPosition(tx, ty, tz, is_yaw, static_cast<float>(theading * M_PI / 180));

                if (this->hasLocalPosition) {
                    if (!targetReached && fabsf(x - tx) < nearDelta && fabsf(y - ty) < nearDelta)
                    {
                        targetReached = true;
                    }
                    if (targetReached && !settled && (fabs(this->vx) + fabsf(this->vy) + fabsf(this->vz) < almostStationery)) {
                        settled = true;
                    }
                }
            }
            else if (targetVelocity) {

                // must send these regularly to keep offboard control.
                // integrate the heading so it is smoother.
                _vehicle->moveByLocalVelocity(tvx, tvy, tvz, is_yaw, static_cast<float>(theading * M_PI / 180));
            }
            else if (targetVelocityAltHold) {

                // must send these regularly to keep offboard control.
                // integrate the heading so it is smoother.
                _vehicle->moveByLocalVelocityWithAltHold(tvx, tvy, tz, is_yaw, static_cast<float>(theading * M_PI / 180));
            }
            break;
        }
        case MavLinkAttitude::kMessageId: // MAVLINK_MSG_ID_ATTITUDE:
        {
            MavLinkAttitude att;
            att.decode(msg);
            pitch = att.pitch;
            pitchSpeed = att.pitchspeed;
            roll = att.roll;
            rollSpeed = att.rollspeed;
            yaw = att.yaw;
            yawSpeed = att.yawspeed;
            break;
        }
        default:
            break;
        }
    });

    // control works better if we get about 50 of these per second (20ms interval, if we can).
    _vehicle->setMessageInterval(static_cast<int>(MavLinkMessageIds::MAVLINK_MSG_ID_LOCAL_POSITION_NED), 50);
    _vehicle->requestControl();




    return true;
}

void UwpMavLink::HasLocalPosition() {
    Goto(tx, ty, tz, -1.0f, yaw, true);
}

void UwpMavLink::Goto(float targetX, float targetY, float targetZ, float speed, float heading, bool isYaw)
{
    targetPosition = true;
    targetVelocity = false;
    targetVelocityAltHold = false;
    tx = targetX;
    ty = targetY;
    tz = targetZ;
    is_yaw = isYaw;
    theading = heading;
    targetSpeed = speed;
    targetReached = false;
}

void UwpMavLink::Move(float targetvx, float targetvy, float targetvz, float heading, bool isYaw)
{
    targetPosition = false;
    targetVelocity = true;
    targetVelocityAltHold = false;
    tvx = targetvx;
    tvy = targetvy;
    tvz = targetvz;
    is_yaw = isYaw;
    theading = heading;
}

void UwpMavLink::MoveAltHold(float targetvx, float targetvy, float targetZ, float heading, bool isYaw)
{
    targetPosition = false;
    targetVelocity = false;
    targetVelocityAltHold = true;
    tvx = targetvx;
    tvy = targetvy;
    tz = targetZ;
    is_yaw = isYaw;
    theading = heading;
}

