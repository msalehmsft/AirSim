#pragma once

namespace MavLinkUwp
{
    public ref class UwpMavLink sealed
    {
		class UwpMavLinkPort : public Port
		{
			Windows::Devices::SerialCommunication::SerialDevice^ _device;
			Windows::Storage::Streams::DataWriter^ _writer;
			Windows::Storage::Streams::DataReader^ _reader;

			mavlink_utils::Semaphore dataReceived;

		public:
			void connect(Windows::Storage::Streams::DataWriter^ w, Windows::Storage::Streams::DataReader^ r);
			int write(const uint8_t* ptr, int count);
			int read(uint8_t* buffer, int bytesToRead);
			void close();
			bool isClosed();
			int getRssi(const char*)
			{
				return 0;
			}
		};

		std::unique_ptr<UwpMavLinkPort> linkPort;
        std::shared_ptr<mavlinkcom::MavLinkConnection> _com;
        std::shared_ptr<mavlinkcom::MavLinkVehicle> _vehicle;

        bool hasLocalPosition = false;
        bool requestedControl = false;
        bool hasControl = false;
        bool targetReached = false;
        bool targetPosition = false;
        bool targetVelocity = false;
        bool targetVelocityAltHold = false;
        bool settled = false; // after target reached.

        float x = 0.0f, y = 0.0f, z = 0.0f; // current
        float tx = 0.0f, ty = 0.0f, tz = 0.0f; // target position
        float vx = 0.0f, vy = 0.0f, vz = 0.0f; // current speed

                          // current attitude
        float pitch = 0.0f;
        float pitchSpeed = 0.0f;
        float roll = 0.0f;
        float rollSpeed = 0.0f;
        float yaw = 0.0f;
        float yawSpeed = 0.0f;

        // targets for planned movement.
        float tvx = 0.0f, tvy = 0.0f, tvz = 0.0f; // target velocities
        bool is_yaw = false; // is target a heading or a rate (true=heading, false=rate).
        float theading = 0.0f; // target heading
        float targetSpeed = 0.0f;
        bool paused = true;
        const float nearDelta = 0.5f; // meters
        const float almostStationery = 0.6f;

		int _subscription = 0;

        void HasLocalPosition();
		void TakeControl();

    public:
        UwpMavLink();

		double getLatOrigin();
		double getLonOrigin();

		double getAltitudeLocal();
		double getAltitudeGlobal();
		double getBatteryVoltage();
		double getBatteryRemaining();

		int GetVehicleStateVersion();


		double getLocalX();
		double getLocalY();
		double getHeading();
		double getPitch();
		double getRoll();

		bool connectToMavLink(Windows::Storage::Streams::DataWriter^ w, Windows::Storage::Streams::DataReader^ r);
        bool arm();
        bool disarm();
        bool takeoff(double z);
        bool land();
        bool Goto(float x, float y, float z);

		bool proxy(Platform::String^ localIp, Platform::String^ remoteIp, int port);

		void FlyToHeight(float z);
        void MoveAltHold(float targetvx, float targetvy, float targetZ, float heading, bool isYaw);
        void Move(float targetvx, float targetvy, float targetvz, float heading, bool isYaw);
        void Goto(float targetX, float targetY, float targetZ, float speed, float heading, bool isYaw);

		void setGPS(double xCM, double yCM, double zCM);
    };
}
