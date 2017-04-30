#include "AirSim.h"
#include "MultiRotorConnector.h"
#include "AirBlueprintLib.h"
#include <exception>

using namespace msr::airlib;

void MultiRotorConnector::initialize(AFlyingPawn* vehicle_pawn, msr::airlib::MultiRotorParams* vehicle_params, bool enable_rpc, std::string api_server_address)
{
    enable_rpc_ = enable_rpc;
    api_server_address_ = api_server_address;
    vehicle_pawn_ = vehicle_pawn;
    vehicle_pawn_->initialize();

    vehicle_params_ = vehicle_params;

    //init physics vehicle
    auto initial_kinematics = Kinematics::State::zero();
    initial_kinematics.pose = vehicle_pawn_->getPose();
    msr::airlib::Environment::State initial_environment;
    initial_environment.position = initial_kinematics.pose.position;
    initial_environment.geo_point = vehicle_pawn_->getHomePoint();
    initial_environment.min_z_over_ground = vehicle_pawn_->getMinZOverGround();
    environment_.initialize(initial_environment);

    vehicle_params_->initialize();
    vehicle_.initialize(vehicle_params_, initial_kinematics, &environment_);

    //pass ground truth to some controllers who needs it (usually the ones without state estimation capabilities)
    vehicle_params_->initializePhysics(&environment_, &vehicle_.getKinematics());

    controller_ = static_cast<msr::airlib::DroneControllerBase*>(vehicle_.getController());

    if (controller_->getRemoteControlID() >= 0)
        detectUsbRc();
}

MultiRotorConnector::~MultiRotorConnector()
{
    stopApiServer();
}

void MultiRotorConnector::beginPlay()
{
    last_pose = Pose::nanPose();
    last_debug_pose = Pose::nanPose();

    //connect to HIL
    try {

        controller_->start();
    }
    catch (std::exception ex) {

        UAirBlueprintLib::LogMessage(FString("Vehicle controller cannot be started, please check your settings.json"), FString(ex.what()), LogDebugLevel::Failure, 180);
        UAirBlueprintLib::LogMessage(FString(ex.what()), TEXT(""), LogDebugLevel::Failure, 180);
    }
}

msr::airlib::VehicleControllerBase* MultiRotorConnector::getController()
{
    return controller_;
}

void MultiRotorConnector::endPlay()
{
    controller_->stop();
}

void MultiRotorConnector::detectUsbRc()
{
    joystick_.getJoyStickState(controller_->getRemoteControlID(), joystick_state_);

    rc_data_.is_connected = joystick_state_.is_connected;

    if (rc_data_.is_connected)
        UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB: "), "Detected", LogDebugLevel::Informational);
    else
        UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB: "), "Not detected", LogDebugLevel::Informational);
}

const msr::airlib::RCData& MultiRotorConnector::getRCData()
{
    joystick_.getJoyStickState(controller_->getRemoteControlID(), joystick_state_);

    rc_data_.is_connected = joystick_state_.is_connected;

    if (rc_data_.is_connected) {
        rc_data_.throttle = joyStickToRC(joystick_state_.left_y);
        rc_data_.yaw = joyStickToRC(joystick_state_.left_x);
        rc_data_.roll = joyStickToRC(joystick_state_.right_x);
        rc_data_.pitch = joyStickToRC(joystick_state_.right_y);

        rc_data_.switch1 = joystick_state_.left_trigger ? 1 : 0;
        rc_data_.switch2 = joystick_state_.right_trigger ? 1 : 0;

        UAirBlueprintLib::LogMessage(FString("Joystick: "), 
            FString::SanitizeFloat(rc_data_.throttle) + ", " + FString::SanitizeFloat(rc_data_.roll) + ", " + FString::SanitizeFloat(rc_data_.pitch) + ", " + FString::SanitizeFloat(rc_data_.yaw), 
            LogDebugLevel::Informational);
    }
    //else don't waste time

    return rc_data_;
}

float MultiRotorConnector::joyStickToRC(int16_t val)
{
    float valf = static_cast<float>(val);
    return (valf - Utils::min<int16_t>()) / Utils::max<uint16_t>();
}


void MultiRotorConnector::updateRenderedState()
{
    //move collison info from rendering engine to vehicle
    vehicle_.setCollisionInfo(vehicle_pawn_->getCollisonInfo());
    //update ground level
    environment_.getState().min_z_over_ground = vehicle_pawn_->getMinZOverGround();
    //update pose of object for rendering engine
    last_pose = vehicle_.getPose();
    last_debug_pose = controller_->getDebugPose();

    //update rotor poses
    for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
        const auto& rotor_output = vehicle_.getRotorOutput(i);
        rotor_speeds_[i] = rotor_output.speed;
        rotor_directions_[i] = static_cast<int>(rotor_output.turning_direction);
        rotor_thrusts_[i] = rotor_output.thrust;
        rotor_controls_filtered_[i] = rotor_output.control_signal_filtered;
    }

    controller_->getStatusMessages(controller_messages_);

    if (controller_->getRemoteControlID() >= 0)
        controller_->setRCData(getRCData());
}

void MultiRotorConnector::updateRendering(float dt)
{
    try {
        controller_->reportTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    if (!VectorMath::hasNan(last_pose.position)) {
        vehicle_pawn_->setPose(last_pose, last_debug_pose);
    }
    
    //update rotor animations
    for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
        vehicle_pawn_->setRotorSpeed(i, rotor_speeds_[i] * rotor_directions_[i]);
    }

    for (auto i = 0; i < controller_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(controller_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }
}


void MultiRotorConnector::startApiServer()
{
    if (enable_rpc_) {
        controller_cancelable_.reset(new msr::airlib::DroneControllerCancelable(
            vehicle_.getController()));
        rpclib_server_.reset(new msr::airlib::RpcLibServer(controller_cancelable_.get(), api_server_address_));
        rpclib_server_->start();
    }

}
void MultiRotorConnector::stopApiServer()
{
    if (rpclib_server_ != nullptr) {
        controller_cancelable_->cancelAllTasks();
        rpclib_server_->stop();
        rpclib_server_.reset(nullptr);
        controller_cancelable_.reset(nullptr);
    }
}

bool MultiRotorConnector::isApiServerStarted()
{
    return rpclib_server_ != nullptr;
}

//*** Start: UpdatableState implementation ***//
void MultiRotorConnector::reset()
{
    rc_data_ = RCData();
    vehicle_pawn_->reset();    //we do flier resetPose so that flier is placed back without collisons
    vehicle_.reset();
}

void MultiRotorConnector::update()
{
    //this is high frequency physics tick, flier gets ticked at rendering frame rate
    vehicle_.update();
}

void MultiRotorConnector::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    if (vehicle_pawn_ != nullptr) {
        FVector unrealPosition = vehicle_pawn_->getPosition();
        reporter.writeValue("unreal pos", AVehiclePawnBase::toVector3r(unrealPosition, 1.0f, false));
        vehicle_.reportState(reporter);
    }
}

MultiRotorConnector::UpdatableObject* MultiRotorConnector::getPhysicsBody()
{
    return vehicle_.getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

