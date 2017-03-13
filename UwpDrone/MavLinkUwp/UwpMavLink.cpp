#include "pch.h"
#include "MavLinkConnection.hpp"
#include "MavLinkVehicle.hpp"
#include "src/serial_com/Port.h"
#include "UwpMavLink.h"
#include "ppltasks.h"
#include "Semaphore.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace concurrency;
using namespace MavLinkUwp;
using namespace Platform;
using namespace Windows::Foundation;
using namespace mavlinkcom;
using namespace Windows::Devices::SerialCommunication;
using namespace Windows::Devices::Enumeration;
using namespace Windows::Storage::Streams;

static const int pixhawkVendorId = 9900;   ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
static const int pixhawkFMUV4ProductId = 18;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2ProductId = 17;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2OldBootloaderProductId = 22;     ///< Product ID for Bootloader on older Pixhawk V2 boards
static const int pixhawkFMUV1ProductId = 16;     ///< Product ID for PX4 FMU V1 board

static const std::wstring intelMavLink(L"UART0");

const int LocalSystemId = 166;
const int LocalComponentId = 1;


void UwpMavLink::UwpMavLinkPort::connect(DataWriter^ w, DataReader^ r)
{
	_writer = w;
	_reader = r;
	/*
	Platform::String^ selector = SerialDevice::GetDeviceSelector();
	DeviceInformationCollection^ deviceCollection;
	create_task(DeviceInformation::FindAllAsync(selector)).then([&](DeviceInformationCollection^ dc)
	{
		deviceCollection = dc;
	}).wait();
		

	if (deviceCollection->Size == 0)
		return;

	for (unsigned int i = 0; i < deviceCollection->Size; ++i)
	{
		std::wstring name(deviceCollection->GetAt(i)->Name->Data());
		std::wstring id(deviceCollection->GetAt(i)->Id->Data());

		if (name.find_first_of(identifyingSubStr) != std::wstring::npos || 
			id.find_first_of(identifyingSubStr) != std::wstring::npos)
		{
			create_task(SerialDevice::FromIdAsync(deviceCollection->GetAt(i)->Id)).then([&](SerialDevice^ device)
			{
				_device = device;
				if (_device != nullptr)
				{
					TimeSpan ts;
					ts.Duration = 5 * 1000000;

					_device->BaudRate = 115200;
					_device->Parity = SerialParity::None;
					_device->DataBits = 8;
					_device->StopBits = SerialStopBitCount::One;
					_device->Handshake = SerialHandshake::None;
					_device->ReadTimeout = ts;
					_device->WriteTimeout = ts;
					_device->IsRequestToSendEnabled = false;
					//_device->IsDataTerminalReadyEnabled = false;

					writer = ref new DataWriter(_device->OutputStream);
					reader = ref new DataReader(_device->InputStream);
					reader->InputStreamOptions = InputStreamOptions::Partial;

				}
			}).then([&]()
			{
				auto ret = reader->LoadAsync(1);
				create_task(ret).then([&](unsigned int bytes)
				{
					uint8_t buf[50];
					reader->ReadBytes(Platform::ArrayReference<BYTE>(buf, 50));
				});

			});

			return;
		}
	}
	*/
}

// write to the port, return number of bytes written or -1 if error.
int UwpMavLink::UwpMavLinkPort::write(const uint8_t* ptr, int count)
{
	if (count < 0)
	{
		return 0;
	}

	Platform::Array<uint8_t>^ arr = ref new Platform::Array<uint8_t>(const_cast<uint8_t*>(ptr), (unsigned int)count);
	_writer->WriteBytes(arr);

	create_task(_writer->StoreAsync()).wait();

	return count;
}

// read a given number of bytes from the port (blocking until the requested bytes are available).
// return the number of bytes read or -1 if error.
int UwpMavLink::UwpMavLinkPort::read(uint8_t* buffer, int bytesToRead)
{
	ZeroMemory(buffer, bytesToRead);

	auto ret = _reader->LoadAsync(bytesToRead);
	create_task(ret).then([&](unsigned int bytes)
	{
		_reader->ReadBytes(Platform::ArrayReference<BYTE>(buffer, bytes));
		bytesToRead = bytes;
	}).wait();

	return bytesToRead;
}

// close the port.
void UwpMavLink::UwpMavLinkPort::close()
{
	// ack!
}

bool UwpMavLink::UwpMavLinkPort::isClosed()
{
	return false;
}

UwpMavLink::UwpMavLink()
{
}

bool UwpMavLink::connectToMavLink(DataWriter^ w, DataReader^ r)
{
    _vehicle = std::make_shared<MavLinkVehicle>(LocalSystemId, LocalComponentId);

	std::shared_ptr<UwpMavLink::UwpMavLinkPort> mp = std::make_shared<UwpMavLink::UwpMavLinkPort>();
	mp->connect(w ,r);

	_com = MavLinkConnection::connectPort("drone", mp);

    _vehicle->connect(_com);

    _vehicle->startHeartbeat();

	//_vehicle->requestControl();


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

