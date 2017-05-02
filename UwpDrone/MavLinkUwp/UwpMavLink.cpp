#include "pch.h"
#include "MavLinkConnection.hpp"
#include "MavLinkVehicle.hpp"
#include "src/serial_com/Port.h"
#include "UwpMavLink.h"
#include "ppltasks.h"
#include "Semaphore.hpp"
#include "geo.h"


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
using namespace Windows::Storage;

static const int pixhawkVendorId = 9900;   ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
static const int pixhawkFMUV4ProductId = 18;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2ProductId = 17;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2OldBootloaderProductId = 22;     ///< Product ID for Bootloader on older Pixhawk V2 boards
static const int pixhawkFMUV1ProductId = 16;     ///< Product ID for PX4 FMU V1 board

static const std::wstring intelMavLink(L"UART0");

const int kLocalSystemId = 166;
const int kLocalComponentId = 1;

const double kMyLat = 47.644230;
const double kMyLon = -122.139070;

static unsigned long getTimeSinceEpochMillis(std::time_t* t = nullptr)
{
	std::time_t st = std::time(t);
	auto millies = static_cast<std::chrono::milliseconds>(st).count();
	return static_cast<unsigned long>(millies);
}
//high precision time in seconds since epoch
static double getTimeSinceEpoch(std::chrono::high_resolution_clock::time_point* t = nullptr)
{
	using Clock = std::chrono::high_resolution_clock;
	return std::chrono::duration<double>((t != nullptr ? *t : Clock::now()).time_since_epoch()).count();
}


UwpMavLink::UwpMavLink()
{
	// nobody will ever use more than 64 bits...
	globallocalconverter_init(0, 0, 0, GetTickCount64());
}

void UwpMavLink::UwpMavLinkPort::connect(DataWriter^ w, DataReader^ r)
{
	_writer = w;
	_reader = r;
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
		auto bytesFromSerial = Platform::ArrayReference<BYTE>(buffer, bytes);
		_reader->ReadBytes(bytesFromSerial);
		bytesToRead = bytes;

		auto bytesToLog = ref new Platform::Array<BYTE>(bytesFromSerial);

		auto writer = ref new DataWriter();
		writer->WriteBytes(bytesToLog);

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

bool UwpMavLink::connectToMavLink(DataWriter^ w, DataReader^ r)
{
	_vehicle = std::make_shared<MavLinkVehicle>(kLocalSystemId, kLocalComponentId);

	std::shared_ptr<UwpMavLink::UwpMavLinkPort> mp = std::make_shared<UwpMavLink::UwpMavLinkPort>();
	mp->connect(w ,r);

	_com = MavLinkConnection::connectPort("drone", mp);

	_vehicle->connect(_com);
    _vehicle->startHeartbeat();

	_subscription = _vehicle->getConnection()->subscribe([=](std::shared_ptr<MavLinkConnection> con, const MavLinkMessage& msg) {
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

    return _com != nullptr;
}

bool UwpMavLink::proxy(Platform::String^ localIp, Platform::String^ remoteIp, int port)
{
	stdext::cvt::wstring_convert<std::codecvt_utf8<wchar_t>> convert;
	std::string stdLocalIp = convert.to_bytes(localIp->Data());
	std::string stdRemoteIp = convert.to_bytes(remoteIp->Data());

	std::shared_ptr<MavLinkConnection> proxyConnection = MavLinkConnection::connectRemoteUdp("Uwp", stdLocalIp, stdRemoteIp, port);
	_com->join(proxyConnection);

	return true;
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

bool UwpMavLink::takeoff(double z)
{
	const VehicleState& state = _vehicle->getVehicleState();
	_vehicle->takeoff((float)z, 0.0f, state.global_est.heading);

	return true;
}

bool UwpMavLink::land()
{
    const VehicleState& state = _vehicle->getVehicleState();
    _vehicle->land(state.global_est.heading, state.home.global_pos.lat, state.home.global_pos.lon, state.home.global_pos.alt);

    return true;
}

void UwpMavLink::FlyToHeight(float z)
{
	const VehicleState& state = _vehicle->getVehicleState();
	MoveAltHold(state.local_est.pos.x, state.local_est.pos.x, z, state.global_est.heading, false);
}

double UwpMavLink::getLatOrigin()
{
	return kMyLat;
}

double UwpMavLink::getLonOrigin()
{
	return kMyLon;
}


double UwpMavLink::getAltitudeLocal()
{
	const VehicleState& state = _vehicle->getVehicleState();

	return state.altitude.altitude_local;
}

double UwpMavLink::getAltitudeGlobal()
{
	const VehicleState& state = _vehicle->getVehicleState();

	return state.global_est.alt_ground;
}

double UwpMavLink::getBatteryVoltage()
{
	const VehicleState& state = _vehicle->getVehicleState();

	double voltsInMillivolts = (double)state.stats.voltage_battery;
	return voltsInMillivolts / 1000.0;	// return volts
}

double UwpMavLink::getBatteryRemaining()
{
	const VehicleState& state = _vehicle->getVehicleState();

	return state.stats.battery_remaining;
}

double UwpMavLink::getPitch()
{
	const VehicleState& state = _vehicle->getVehicleState();

	return state.attitude.pitch;
}

double UwpMavLink::getRoll()
{
	const VehicleState& state = _vehicle->getVehicleState();

	return state.attitude.roll;
}

double UwpMavLink::getLocalX()
{
	return x;
}

double UwpMavLink::getLocalY()
{
	return y;
}

double UwpMavLink::getHeading()
{
	const VehicleState& state = _vehicle->getVehicleState();

	return state.global_est.heading;
}

void UwpMavLink::TakeControl()
{
	if (!requestedControl) 
	{
		// control works better if we get about 50 of these per second (20ms interval, if we can).
		_vehicle->requestControl();
		requestedControl = true;
	}
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

	TakeControl();

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
	paused = false;
	tx = targetX;
    ty = targetY;
    tz = targetZ;
    is_yaw = isYaw;
    theading = heading;
    targetSpeed = speed;
    targetReached = false;

	TakeControl();
}

void UwpMavLink::Move(float targetvx, float targetvy, float targetvz, float heading, bool isYaw)
{
    targetPosition = false;
    targetVelocity = true;
    targetVelocityAltHold = false;
	paused = false;
	tvx = targetvx;
    tvy = targetvy;
    tvz = targetvz;
    is_yaw = isYaw;
    theading = heading;
	TakeControl();
}

void UwpMavLink::MoveAltHold(float targetvx, float targetvy, float targetZ, float heading, bool isYaw)
{
    targetPosition = false;
    targetVelocity = false;
    targetVelocityAltHold = true;
	paused = false;
    tvx = targetvx;
    tvy = targetvy;
    tz = targetZ;
    is_yaw = isYaw;
    theading = heading;
	TakeControl();
}

void UwpMavLink::setGPS(double xCM, double yCM, double zCM)
{
	double lat, lon;
	double alt;
	MavLinkHilGps hilGPS;

	// coordinates in converted from meters to lat/lon/alt
	globallocalconverter_toglobal(xCM / 100.0, yCM / 100.0, zCM / 100.0, &lat, &lon, &alt);

	lat += kMyLat;
	lon += kMyLon;

	hilGPS.compid = _com->getTargetComponentId();
	hilGPS.sysid = _com->getTargetSystemId();
	hilGPS.alt = (int32_t) std::round(alt  * 1000.0f);	// alt is now in meters
	hilGPS.lat = (int32_t)(lat * 1e7);
	hilGPS.lon = (int32_t)(lon * 1e7);
	hilGPS.eph = static_cast<uint16_t>(0.01 * 100);
	hilGPS.epv = static_cast<uint16_t>(0.01 * 100);
	hilGPS.vel = 0; // unknown
	hilGPS.cog = 0; // unknown
	hilGPS.fix_type = 3;	// because Chris says so
	hilGPS.satellites_visible = 10;	// because Chris says so
	hilGPS.timestamp = static_cast<uint64_t>(getTimeSinceEpochMillis());
	hilGPS.time_usec = static_cast<uint64_t>(getTimeSinceEpochMillis() * 1000);

	_com->sendMessage(hilGPS);
}

