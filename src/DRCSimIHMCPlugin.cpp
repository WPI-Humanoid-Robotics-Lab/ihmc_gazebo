#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/thread/condition_variable.hpp>

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/math/Angle.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>

#include <gazebo/sensors/ForceTorqueSensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/physics/Collision.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>

#include <stdio.h>
#include <string>

#include "ByteBuffer.h"
#include "TCPServer.h"

const uint32_t BUFFER_SIZE = 1460;  // MTU = 1500, TCP overhead = 40

// depending on controller architecture (e.g. ticks per sim tick)
// a number of messages might need to be sent prior initializing after the handshake
// until receiving a certain number of updates
const int NUM_MSGS_TO_CONTROLLER_AFTER_HANDSHAKE = 25;
namespace gazebo
{
class DRCSimIHMCPlugin : public ModelPlugin
{
private:
  physics::ModelPtr model;

  event::ConnectionPtr updateConnection;

  std::vector<sensors::ImuSensorPtr> imus;
  std::vector<physics::JointPtr> forceSensors;
  physics::Joint_V joints, controlledJoints;
  physics::JointControllerPtr jointController;

  boost::mutex robotControlLock;
  boost::condition_variable condition;

  TCPServer tcpDataServer;
  TCPServer tcpCommandListener;

  bool initialized;
  bool receivedHandshake;
  bool receivedControlMessage;
  int cyclesRemainingTillControlMessage;
  int64_t lastReceivedTimestamp;

  int simulationCyclesPerControlCycle;
  std::map<std::string, double> desiredPositionsOrTorques;
  std::vector<std::string> jointControlModes;

  ros::NodeHandlePtr rosNode;
  double gzPhysicsTimeStep;
  double estimatorFrequencyInHz;
  int tcpSendModulus;
  int modulusCounter;
  std::map<std::string, bool> jointsToIgnore, jointsToSkip;
  void PopulateJointsToIgnore();
  void populateJointsToSkipData();

public:
  DRCSimIHMCPlugin()
    : tcpDataServer(1234, BUFFER_SIZE)
    , tcpCommandListener(1235, BUFFER_SIZE)
    , initialized(false)
    , receivedHandshake(false)
    , receivedControlMessage(false)
    , cyclesRemainingTillControlMessage(0)
    , simulationCyclesPerControlCycle(-1)
    , tcpSendModulus(1)
  {
    std::cout << "IHMC Plugin Loaded" << std::endl;
  }

  void SendDataToController(int64_t localLastReceivedTimestamp)
  {
    // ROS_INFO("Sending data to Controller");
    ByteBuffer data;
    gazebo::common::Time time = this->model->GetWorld()->GetSimTime();
    uint64_t timeStamp = (uint64_t)time.sec * 1000000000ull + (uint64_t)time.nsec;

    data.put(timeStamp);
    data.put(localLastReceivedTimestamp);

    for (unsigned int i = 0; i < controlledJoints.size(); i++)
    {
      physics::JointPtr joint = controlledJoints[i];
      data.put(joint->GetAngle(0).Radian());
      data.put(joint->GetVelocity(0));
      // std::cout << "Sending Name : " << joint->GetName() << " Angle : " << joint->GetAngle(0).Radian() << std::endl;
    }

    for (unsigned int i = 0; i < imus.size(); i++)
    {
      sensors::ImuSensorPtr imu = imus[i];

      math::Quaternion imuRotation = imu->Orientation();
      math::Vector3 angularVelocity = imu->AngularVelocity();
      math::Vector3 linearAcceleration = imu->LinearAcceleration();

      data.put(imuRotation.w);
      data.put(imuRotation.x);
      data.put(imuRotation.y);
      data.put(imuRotation.z);

      data.put(angularVelocity.x);
      data.put(angularVelocity.y);
      data.put(angularVelocity.z);

      data.put(linearAcceleration.x);
      data.put(linearAcceleration.y);
      data.put(linearAcceleration.z);
    }

    for (unsigned int i = 0; i < 4; i++)
    {
      gazebo::physics::JointWrench wrench = forceSensors[i]->GetForceTorque((unsigned int)0);

      data.put(wrench.body2Torque.x);
      data.put(wrench.body2Torque.y);
      data.put(wrench.body2Torque.z);

      data.put(wrench.body2Force.x);
      data.put(wrench.body2Force.y);
      data.put(wrench.body2Force.z);
    }

    if (this->modulusCounter >= this->tcpSendModulus)
    {
      tcpDataServer.send(data);
      this->modulusCounter = 0;
    }

    this->modulusCounter++;
  }

  static bool sortJoints(const physics::JointPtr& a, const physics::JointPtr& b)
  {
    return a->GetName().compare(b->GetName()) < 0;
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
  {
    PopulateJointsToIgnore();
    populateJointsToSkipData();
    this->model = _parent;

    this->gzPhysicsTimeStep = model->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();
    this->tcpSendModulus = (int)((1.0 / (this->gzPhysicsTimeStep * 1e3)) + 0.5);
    this->modulusCounter = 3;

    joints = this->model->GetJoints();
    std::sort(joints.begin(), joints.end(), DRCSimIHMCPlugin::sortJoints);

    for (auto it = joints.begin(); it != joints.end(); it++)
    {
      if (jointsToIgnore.count((*it)->GetName()) > 0)
      {
        continue;
      }
      controlledJoints.push_back(*it);
    }

    std::cout << "\n\n********************\nTotal Joints " << joints.size() << "\nControlled joints "
              << controlledJoints.size() << "\n**********************************\n\n ";
    jointController = this->model->GetJointController();

    this->rosNode = ros::NodeHandlePtr(new ros::NodeHandle(""));
    sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
    for (unsigned int i = 0; i < sensors.size(); i++)
    {
      gazebo::sensors::ImuSensorPtr imu = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensors[i]);
      if (imu)
      {
        if (imu->ParentName() == "atlas::pelvis")
        {
          imus.push_back(imu);
        }
      }
    }

    forceSensors.push_back(this->model->GetJoint("l_arm_wrx"));
    forceSensors.push_back(this->model->GetJoint("r_arm_wrx"));
    forceSensors.push_back(this->model->GetJoint("l_leg_akx"));
    forceSensors.push_back(this->model->GetJoint("r_leg_akx"));

    tcpCommandListener.addReadListener(boost::bind(&DRCSimIHMCPlugin::readControlMessage, this, _1, _2));

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DRCSimIHMCPlugin::OnUpdate, this, _1));

    jointControlModes.resize(controlledJoints.size(), "");

    for (size_t i = 0; i < controlledJoints.size(); i++)
    {
      if (jointsToSkip.count(controlledJoints[i]->GetName()) > 0 || controlledJoints[i]->GetName() == "hokuyo_joint")
      {
        continue;
      }
      std::string keyRoot = "atlas/" + controlledJoints[i]->GetName() + "_effort_controller";
      double initialAngle;
      std::string initAngleKey = keyRoot + "/initial_angle";
      this->rosNode->getParam(initAngleKey, initialAngle);

      std::string controlMode;
      std::string ctrlModeKey = keyRoot + "/type";
      this->rosNode->getParam(ctrlModeKey, controlMode);
      this->jointControlModes[i] = controlMode;

      if (controlMode == "pid")
      {
        double kp, ki, kd, imax, imin, cmdMax, cmdMin;
        this->rosNode->getParam(keyRoot + "/pid/p", kp);
        this->rosNode->getParam(keyRoot + "/pid/i", ki);
        this->rosNode->getParam(keyRoot + "/pid/d", kd);
        this->rosNode->getParam(keyRoot + "/pid/imax", imax);
        this->rosNode->getParam(keyRoot + "/pid/imin", imin);
        this->rosNode->getParam(keyRoot + "/pid/cmdMax", cmdMax);
        this->rosNode->getParam(keyRoot + "/pid/cmdMin", cmdMin);
        jointController->SetPositionPID(controlledJoints[i]->GetName(),
                                        common::PID(kp, ki, kd, imax, imin, cmdMax, cmdMin));
      }

      controlledJoints[i]->SetPosition(0, initialAngle);
      controlledJoints[i]->SetUpperLimit(0, initialAngle);
      controlledJoints[i]->SetLowerLimit(0, initialAngle);

      jointController->SetPositionTarget(controlledJoints[i]->GetName(), initialAngle);
      desiredPositionsOrTorques[controlledJoints[i]->GetName()] = initialAngle;
    }
  }

  void OnUpdate(const common::UpdateInfo& info)
  {
    boost::unique_lock<boost::mutex> lock(robotControlLock);
    {
      if (!initialized)
      {
        if (!receivedHandshake)
        {
          // wait for controller to connect
        }
        else
        {
          std::cout << "Unlocking joints. Starting control." << std::endl;

          for (int j = 0; j < NUM_MSGS_TO_CONTROLLER_AFTER_HANDSHAKE; j++)
          {
            DRCSimIHMCPlugin::SendDataToController(0);
          }

          while (!receivedControlMessage)
          {
            condition.wait(lock);
          }

          for (unsigned int i = 0; i < controlledJoints.size(); i++)
          {
            if (jointsToSkip.count(controlledJoints[i]->GetName()) > 0 || controlledJoints[i]->GetName() == "hokuyo_"
                                                                                                            "joint")
            {
              continue;
            }
            if (jointControlModes[i] == "position_controllers/joint_position_controller")
            {
              continue;
            }
            if (jointControlModes[i] == "effort_controllers/joint_effort_controller")
            {
              controlledJoints[i]->SetLowerLimit(0, -3.14);
              controlledJoints[i]->SetUpperLimit(0, 3.14);
              controlledJoints[i]->SetEffortLimit(0, -1);
              controlledJoints[i]->SetDamping(0, 0.1);  // TODO do we need to set damping?
            }
          }
          initialized = true;
        }
        lock.unlock();
        return;
      }

      if (cyclesRemainingTillControlMessage == 0)
      {
        // Block for control message
        while (!receivedControlMessage)
        {
          condition.wait(lock);
        }
        cyclesRemainingTillControlMessage = simulationCyclesPerControlCycle;
        receivedControlMessage = false;
      }

      for (size_t i = 0; i < controlledJoints.size(); i++)
      {
        if (jointsToSkip.count(controlledJoints[i]->GetName()) > 0 || controlledJoints[i]->GetName() == "hokuyo_joint")
        {
          continue;
        }
        else if (jointControlModes[i] == "effort_controllers/joint_effort_controller")
        {
          controlledJoints[i]->SetForce(0, desiredPositionsOrTorques[controlledJoints[i]->GetName()]);
        }
      }

      jointController->Update();
      cyclesRemainingTillControlMessage--;
    }

    int64_t localLastReceivedTimestamp = lastReceivedTimestamp;
    lock.unlock();

    DRCSimIHMCPlugin::SendDataToController(localLastReceivedTimestamp);
  }

  void readControlMessage(char* buffer, std::size_t bytes_transffered)
  {
    if (!receivedHandshake)
    {
      receivedHandshake = true;
    }
    else
    {
      boost::unique_lock<boost::mutex> lock(robotControlLock);
      {
        int64_t* longBuffer = ((int64_t*)(buffer));

        int estimatorTicksPerControlTick = longBuffer[0];
        lastReceivedTimestamp = longBuffer[1];
        estimatorFrequencyInHz = longBuffer[2];

        simulationCyclesPerControlCycle =
            estimatorTicksPerControlTick / (this->gzPhysicsTimeStep * estimatorFrequencyInHz);

        double* jointPositionsOrTorques = (double*)(buffer + 24);

        for (unsigned int i = 0; i < controlledJoints.size(); i++)
        {
          desiredPositionsOrTorques[controlledJoints[i]->GetName()] = jointPositionsOrTorques[i];
          // ROS_INFO("Recieved data for joint %s : %.2f", controlledJoints[i]->GetName().c_str(),
          //          desiredPositionsOrTorques[controlledJoints[i]->GetName()]);
        }

        receivedControlMessage = true;

        condition.notify_all();
      }

      lock.unlock();
    }
  }
};

/**
 * @brief Joints that should be skipped while sending the control input to gazebo
 *
 */
void DRCSimIHMCPlugin::populateJointsToSkipData()
{
  jointsToSkip["l_fixed"] = true;
  jointsToSkip["l_fixed_dummy"] = true;
  jointsToSkip["l_fixed_dummy2"] = true;
  jointsToSkip["r_fixed"] = true;
  jointsToSkip["r_fixed_dummy"] = true;
  jointsToSkip["r_fixed_dummy2"] = true;
  jointsToSkip["l_palm_to_finger_1_1"] = true;
  jointsToSkip["l_palm_to_finger_2_1"] = true;
  jointsToSkip["l_palm_to_finger_3_1"] = true;
  jointsToSkip["r_palm_to_finger_1_1"] = true;
  jointsToSkip["r_palm_to_finger_2_1"] = true;
  jointsToSkip["r_palm_to_finger_3_1"] = true;
  jointsToSkip["l_finger_1_1_to_finger_1_2"] = true;
  jointsToSkip["l_finger_2_1_to_finger_2_2"] = true;
  jointsToSkip["l_finger_3_1_to_finger_3_2"] = true;
  jointsToSkip["l_finger_1_2_to_finger_1_3"] = true;
  jointsToSkip["l_finger_2_2_to_finger_2_3"] = true;
  jointsToSkip["l_finger_3_2_to_finger_3_3"] = true;
  jointsToSkip["r_finger_1_1_to_finger_1_2"] = true;
  jointsToSkip["r_finger_2_1_to_finger_2_2"] = true;
  jointsToSkip["r_finger_3_1_to_finger_3_2"] = true;
  jointsToSkip["r_finger_1_2_to_finger_1_3"] = true;
  jointsToSkip["r_finger_2_2_to_finger_2_3"] = true;
  jointsToSkip["r_finger_3_2_to_finger_3_3"] = true;
  jointsToSkip["l_finger_1_joint_1"] = true;
  jointsToSkip["l_finger_1_joint_2"] = true;
  jointsToSkip["l_finger_1_joint_3"] = true;
  jointsToSkip["l_finger_2_joint_1"] = true;
  jointsToSkip["l_finger_2_joint_2"] = true;
  jointsToSkip["l_finger_2_joint_3"] = true;
  jointsToSkip["l_finger_middle_joint_1"] = true;
  jointsToSkip["l_finger_middle_joint_2"] = true;
  jointsToSkip["l_finger_middle_joint_3"] = true;
  jointsToSkip["l_palm_finger_1_joint"] = true;
  jointsToSkip["l_palm_finger_2_joint"] = true;
  jointsToSkip["l_palm_finger_middle_joint"] = true;
  jointsToSkip["r_finger_1_joint_1"] = true;
  jointsToSkip["r_finger_1_joint_2"] = true;
  jointsToSkip["r_finger_1_joint_3"] = true;
  jointsToSkip["r_finger_2_joint_1"] = true;
  jointsToSkip["r_finger_2_joint_2"] = true;
  jointsToSkip["r_finger_2_joint_3"] = true;
  jointsToSkip["r_finger_middle_joint_1"] = true;
  jointsToSkip["r_finger_middle_joint_2"] = true;
  jointsToSkip["r_finger_middle_joint_3"] = true;
  jointsToSkip["r_palm_finger_1_joint"] = true;
  jointsToSkip["r_palm_finger_2_joint"] = true;
  jointsToSkip["r_palm_finger_middle_joint"] = true;
}

/**
 * @brief Joints that are supposed to be ignored to match the joint list coming from java controllers
 *
 */
void DRCSimIHMCPlugin::PopulateJointsToIgnore()
{
  jointsToIgnore["l_finger_1_joint_median_actuating_hinge"] = true;
  jointsToIgnore["l_finger_1_joint_median_actuating_hinge_median_bar"] = true;
  jointsToIgnore["l_finger_1_joint_paradistal_hinge"] = true;
  jointsToIgnore["l_finger_1_joint_paramedian_hinge"] = true;
  jointsToIgnore["l_finger_1_joint_paramedian_hinge_median_bar_underactuated"] = true;
  jointsToIgnore["l_finger_1_joint_paraproximal_actuating_hinge"] = true;
  jointsToIgnore["l_finger_1_joint_paraproximal_bar"] = true;
  jointsToIgnore["l_finger_1_joint_proximal_actuating_bar"] = true;
  jointsToIgnore["l_finger_1_joint_proximal_actuating_hinge"] = true;
  jointsToIgnore["l_finger_1_link_median_bar_link_3_couple"] = true;
  jointsToIgnore["l_finger_1_link_paradistal_hinge_link_3_couple"] = true;
  jointsToIgnore["l_finger_1_link_paramedian_bar_paradistal_hinge_couple"] = true;
  jointsToIgnore["l_finger_1_link_paraproximal_bar_paramedian_hinge_couple"] = true;
  jointsToIgnore["l_finger_1_link_proximal_actuating_bar_median_actuating_hinge_couple"] = true;
  jointsToIgnore["l_finger_2_joint_median_actuating_hinge"] = true;
  jointsToIgnore["l_finger_2_joint_median_actuating_hinge_median_bar"] = true;
  jointsToIgnore["l_finger_2_joint_paradistal_hinge"] = true;
  jointsToIgnore["l_finger_2_joint_paramedian_hinge"] = true;
  jointsToIgnore["l_finger_2_joint_paramedian_hinge_median_bar_underactuated"] = true;
  jointsToIgnore["l_finger_2_joint_paraproximal_actuating_hinge"] = true;
  jointsToIgnore["l_finger_2_joint_paraproximal_bar"] = true;
  jointsToIgnore["l_finger_2_joint_proximal_actuating_bar"] = true;
  jointsToIgnore["l_finger_2_joint_proximal_actuating_hinge"] = true;
  jointsToIgnore["l_finger_2_link_median_bar_link_3_couple"] = true;
  jointsToIgnore["l_finger_2_link_paradistal_hinge_link_3_couple"] = true;
  jointsToIgnore["l_finger_2_link_paramedian_bar_paradistal_hinge_couple"] = true;
  jointsToIgnore["l_finger_2_link_paraproximal_bar_paramedian_hinge_couple"] = true;
  jointsToIgnore["l_finger_2_link_proximal_actuating_bar_median_actuating_hinge_couple"] = true;
  jointsToIgnore["l_finger_middle_joint_median_actuating_hinge"] = true;
  jointsToIgnore["l_finger_middle_joint_median_actuating_hinge_median_bar"] = true;
  jointsToIgnore["l_finger_middle_joint_paradistal_hinge"] = true;
  jointsToIgnore["l_finger_middle_joint_paramedian_hinge"] = true;
  jointsToIgnore["l_finger_middle_joint_paramedian_hinge_median_bar_underactuated"] = true;
  jointsToIgnore["l_finger_middle_joint_paraproximal_actuating_hinge"] = true;
  jointsToIgnore["l_finger_middle_joint_paraproximal_bar"] = true;
  jointsToIgnore["l_finger_middle_joint_proximal_actuating_bar"] = true;
  jointsToIgnore["l_finger_middle_joint_proximal_actuating_hinge"] = true;
  jointsToIgnore["l_finger_middle_link_median_bar_link_3_couple"] = true;
  jointsToIgnore["l_finger_middle_link_paradistal_hinge_link_3_couple"] = true;
  jointsToIgnore["l_finger_middle_link_paramedian_bar_paradistal_hinge_couple"] = true;
  jointsToIgnore["l_finger_middle_link_paraproximal_bar_paramedian_hinge_couple"] = true;
  jointsToIgnore["l_finger_middle_link_proximal_actuating_bar_median_actuating_hinge_couple"] = true;
  jointsToIgnore["r_finger_1_joint_median_actuating_hinge"] = true;
  jointsToIgnore["r_finger_1_joint_median_actuating_hinge_median_bar"] = true;
  jointsToIgnore["r_finger_1_joint_paradistal_hinge"] = true;
  jointsToIgnore["r_finger_1_joint_paramedian_hinge"] = true;
  jointsToIgnore["r_finger_1_joint_paramedian_hinge_median_bar_underactuated"] = true;
  jointsToIgnore["r_finger_1_joint_paraproximal_actuating_hinge"] = true;
  jointsToIgnore["r_finger_1_joint_paraproximal_bar"] = true;
  jointsToIgnore["r_finger_1_joint_proximal_actuating_bar"] = true;
  jointsToIgnore["r_finger_1_joint_proximal_actuating_hinge"] = true;
  jointsToIgnore["r_finger_1_link_median_bar_link_3_couple"] = true;
  jointsToIgnore["r_finger_1_link_paradistal_hinge_link_3_couple"] = true;
  jointsToIgnore["r_finger_1_link_paramedian_bar_paradistal_hinge_couple"] = true;
  jointsToIgnore["r_finger_1_link_paraproximal_bar_paramedian_hinge_couple"] = true;
  jointsToIgnore["r_finger_1_link_proximal_actuating_bar_median_actuating_hinge_couple"] = true;
  jointsToIgnore["r_finger_2_joint_median_actuating_hinge"] = true;
  jointsToIgnore["r_finger_2_joint_median_actuating_hinge_median_bar"] = true;
  jointsToIgnore["r_finger_2_joint_paradistal_hinge"] = true;
  jointsToIgnore["r_finger_2_joint_paramedian_hinge"] = true;
  jointsToIgnore["r_finger_2_joint_paramedian_hinge_median_bar_underactuated"] = true;
  jointsToIgnore["r_finger_2_joint_paraproximal_actuating_hinge"] = true;
  jointsToIgnore["r_finger_2_joint_paraproximal_bar"] = true;
  jointsToIgnore["r_finger_2_joint_proximal_actuating_bar"] = true;
  jointsToIgnore["r_finger_2_joint_proximal_actuating_hinge"] = true;
  jointsToIgnore["r_finger_2_link_median_bar_link_3_couple"] = true;
  jointsToIgnore["r_finger_2_link_paradistal_hinge_link_3_couple"] = true;
  jointsToIgnore["r_finger_2_link_paramedian_bar_paradistal_hinge_couple"] = true;
  jointsToIgnore["r_finger_2_link_paraproximal_bar_paramedian_hinge_couple"] = true;
  jointsToIgnore["r_finger_2_link_proximal_actuating_bar_median_actuating_hinge_couple"] = true;
  jointsToIgnore["r_finger_middle_joint_median_actuating_hinge"] = true;
  jointsToIgnore["r_finger_middle_joint_median_actuating_hinge_median_bar"] = true;
  jointsToIgnore["r_finger_middle_joint_paradistal_hinge"] = true;
  jointsToIgnore["r_finger_middle_joint_paramedian_hinge"] = true;
  jointsToIgnore["r_finger_middle_joint_paramedian_hinge_median_bar_underactuated"] = true;
  jointsToIgnore["r_finger_middle_joint_paraproximal_actuating_hinge"] = true;
  jointsToIgnore["r_finger_middle_joint_paraproximal_bar"] = true;
  jointsToIgnore["r_finger_middle_joint_proximal_actuating_bar"] = true;
  jointsToIgnore["r_finger_middle_joint_proximal_actuating_hinge"] = true;
  jointsToIgnore["r_finger_middle_link_median_bar_link_3_couple"] = true;
  jointsToIgnore["r_finger_middle_link_paradistal_hinge_link_3_couple"] = true;
  jointsToIgnore["r_finger_middle_link_paramedian_bar_paradistal_hinge_couple"] = true;
  jointsToIgnore["r_finger_middle_link_paraproximal_bar_paramedian_hinge_couple"] = true;
  jointsToIgnore["r_finger_middle_link_proximal_actuating_bar_median_actuating_hinge_couple"] = true;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DRCSimIHMCPlugin)
}  // namespace gazebo
