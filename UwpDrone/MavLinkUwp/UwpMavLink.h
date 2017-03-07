﻿#pragma once

namespace MavLinkUwp
{
    public ref class UwpMavLink sealed
    {
        std::shared_ptr<mavlinkcom::MavLinkConnection> _com;
        std::shared_ptr<mavlinkcom::MavLinkVehicle> _vehicle;

        bool hasLocalPosition;
        bool requestedControl;
        bool hasControl;
        bool targetReached;
        bool targetPosition;
        bool targetVelocity;
        bool targetVelocityAltHold;
        bool settled; // after target reached.

        float x, y, z; // current
        float tx, ty, tz; // target position
        float vx, vy, vz; // current speed

                          // current attitude
        float pitch;
        float pitchSpeed;
        float roll;
        float rollSpeed;
        float yaw;
        float yawSpeed;

        // targets for planned movement.
        float tvx, tvy, tvz; // target velocities
        bool is_yaw; // is target a heading or a rate (true=heading, false=rate).
        float theading; // target heading
        float targetSpeed;
        bool paused = false;
        const float nearDelta = 0.5f; // meters
        const float almostStationery = 0.6f;

        void HasLocalPosition();

    public:
        UwpMavLink();
        bool connectToMavLink();
        bool arm();
        bool disarm();
        bool takeoff(float z);
        bool land();
        bool Goto(float x, float y, float z);

        void MoveAltHold(float targetvx, float targetvy, float targetZ, float heading, bool isYaw);
        void Move(float targetvx, float targetvy, float targetvz, float heading, bool isYaw);
        void Goto(float targetX, float targetY, float targetZ, float speed, float heading, bool isYaw);

    };
}
