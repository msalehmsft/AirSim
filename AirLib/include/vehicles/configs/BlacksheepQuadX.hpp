// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_BlacksheepQuadX_hpp
#define msr_airlib_vehicles_BlacksheepQuadX_hpp

#include "vehicles/MultiRotorParams.hpp"
#include "controllers/MavLinkDroneController.hpp"


namespace msr {
    namespace airlib {

        class BlacksheepQuadX : public MultiRotorParams {
        public:
            BlacksheepQuadX(const MavLinkDroneController::ConnectionInfo& connection_info)
                : connection_info_(connection_info)
            {
            }

        protected:
            virtual void setup(Params& params, SensorCollection& sensors, unique_ptr<DroneControllerBase>& controller) override
            {
                /*
                Motor placement:
                x
                (2)  |   (0)
                |
                ------------ y
                |
                (1)  |   (3)
                |

                */
                //set up arm lengths
                //dimensions are for Team Blacksheep Discovery (http://team-blacksheep.com/products/product:98)
                params.rotor_count = 4;
                std::vector<real_T> arm_lengths;

                Vector3r unit_z(0, 0, -1);  //NED frame

                // relative to Forward vector in the order (0,3,1,2) required by quad X pattern
                // http://ardupilot.org/copter/_images/MOTORS_QuadX_QuadPlus.jpg
                arm_lengths.push_back(0.22);
                arm_lengths.push_back(0.255);
                arm_lengths.push_back(0.22);
                arm_lengths.push_back(0.255);

                // note: the Forward vector is actually the "x" axis, and the AngleAxisr rotation is pointing down and is left handed, so this means the rotation
                // is counter clockwise, so the vector (arm_lengths[i], 0) is the X-axis, so the CCW rotations to position each arm correctly are listed below:
                // See measurements here: http://diydrones.com/profiles/blogs/arducopter-tbs-discovery-style (angles reversed because we are doing CCW rotation)
                std::vector<real_T> arm_angles;
                arm_angles.push_back(-55);
                arm_angles.push_back(125);
                arm_angles.push_back(55);
                arm_angles.push_back(-125);

                // quad X pattern 
                std::vector<RotorTurningDirection> rotor_directions;
                rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
                rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
                rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);
                rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);

                // data from
                // http://dronesvision.net/team-blacksheep-750kv-motor-esc-set-for-tbs-discovery-fpv-quadcopter/
                //set up mass
                params.mass = 2.0f; //can be varied from 0.800 to 1.600
                real_T motor_assembly_weight = 0.052f;  // weight for TBS motors 
                real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

                // the props we are using a E-Prop, which I didn't find in UIUC database, but this one is close:
                // http://m-selig.ae.illinois.edu/props/volume-2/plots/ef_130x70_static_ctcp.png
                params.rotor_params.C_T = 0.11;
                params.rotor_params.C_P = 0.047;
                params.rotor_params.max_rpm = 9500;
                params.rotor_params.calculateMaxThrust();

                //set up dimensions of core body box or abdomen (not including arms).
                params.body_box.x = 0.20f; params.body_box.y = 0.12f; params.body_box.z = 0.04f;
                real_T rotor_z = 2.5f / 100;

                //computer rotor poses
                params.rotor_poses.clear();
                for (uint i = 0; i < 4; i++)
                {
                    Quaternionr angle(AngleAxisr(arm_angles[i] * M_PIf / 180, unit_z));
                    params.rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(arm_lengths[i], 0, rotor_z), angle, true), unit_z, rotor_directions[i]);
                };

                //compute inertia matrix
                computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
                //create sensors
                createStandardSensors(sensors, params.enabled_sensors);
                //create MavLink controller for PX4
                createMavController(controller, sensors);

                //leave everything else to defaults
            }

        private:
            void createMavController(unique_ptr<DroneControllerBase>& controller, SensorCollection& sensors)
            {
                controller.reset(new MavLinkDroneController());
                auto mav_controller = static_cast<MavLinkDroneController*>(controller.get());
                mav_controller->initialize(connection_info_, &sensors, true);
            }

            void createStandardSensors(SensorCollection& sensors, const EnabledSensors& enabled_sensors)
            {
                sensor_storage_.clear();
                if (enabled_sensors.imu)
                    sensors.insert(createSensor<ImuSimple>(), SensorCollection::SensorType::Imu);
                if (enabled_sensors.magnetometer)
                    sensors.insert(createSensor<MagnetometerSimple>(), SensorCollection::SensorType::Magnetometer);
                if (enabled_sensors.gps)
                    sensors.insert(createSensor<GpsSimple>(), SensorCollection::SensorType::Gps);
                if (enabled_sensors.barometer)
                    sensors.insert(createSensor<BarometerSimple>(), SensorCollection::SensorType::Barometer);
            }

            template<typename SensorClass>
            SensorBase* createSensor()
            {
                sensor_storage_.emplace_back(unique_ptr<SensorClass>(new SensorClass()));
                return sensor_storage_.back().get();
            }

        private:
            vector<unique_ptr<SensorBase>> sensor_storage_;
            MavLinkDroneController::ConnectionInfo connection_info_;
        };

    }
} //namespace
#endif
