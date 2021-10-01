#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_msgs/Float32MultiArray.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class ScoutHardwareInterface : public hardware_interface::RobotHW
{
    static constexpr unsigned int K_NUM_WHEELS = 4;

public:
    ScoutHardwareInterface();
    bool init(ros::NodeHandle& rootNh, ros::NodeHandle& robotHardwareNh) override;
    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;

private:
    bool AppendWheelNames(ros::NodeHandle& nodeHandle, const std::string& paramName, std::vector<std::string>& wheelNames);

private:
    hardware_interface::JointStateInterface m_JointStateInterface;
    hardware_interface::VelocityJointInterface m_VelocityJointInterface;
    double m_Commands[K_NUM_WHEELS];
    double m_Positions[K_NUM_WHEELS];
    double m_Velocities[K_NUM_WHEELS];
    double m_Effort[K_NUM_WHEELS];

    ros::Publisher m_WheelVelocityPublisher;
    std_msgs::Float32MultiArray m_WheelVelocityMessage;
};



//Implementation
//////////////////////////////////////////////////////////////////////////////////

ScoutHardwareInterface::ScoutHardwareInterface()
{
    m_WheelVelocityMessage.data.resize(K_NUM_WHEELS);
}

bool ScoutHardwareInterface::init(ros::NodeHandle& rootNh, ros::NodeHandle& robotHardwareNh)
{
    std::fill_n(m_Positions, K_NUM_WHEELS, 0.0);
    std::fill_n(m_Velocities, K_NUM_WHEELS, 0.0);
    std::fill_n(m_Effort, K_NUM_WHEELS, 0.0);
    std::fill_n(m_Commands, K_NUM_WHEELS, 0.0);

    std::string diffDriveControllerName;
    if(!robotHardwareNh.getParam("diffDriveControllerName", diffDriveControllerName))
    {
        ROS_ERROR("diffDriveControllerName not set");
        return false;
    }

    diffDriveControllerName = rootNh.getNamespace() + "/" + diffDriveControllerName;

    std::string leftWheelsParamName = diffDriveControllerName + "/left_wheel";
    std::string rightWheelsParamName = diffDriveControllerName + "/right_wheel";

    std::vector<std::string> jointNames;
    if(!AppendWheelNames(rootNh, leftWheelsParamName, jointNames))
    {
        ROS_ERROR("%s not found", leftWheelsParamName.c_str());
        return false;
    }

    if(!AppendWheelNames(rootNh, rightWheelsParamName, jointNames))
    {
        ROS_ERROR("%s not found", rightWheelsParamName.c_str());
        return false;
    }

    if(jointNames.size() != K_NUM_WHEELS)
    {
        ROS_ERROR("Wrong number of wheels provided. Expected %d but got %ld:", K_NUM_WHEELS, jointNames.size());
        for(std::string& jointName : jointNames)
        {
            ROS_ERROR("\t%s", jointName.c_str());
        }

        return false;
    }

    // connect and register the joint state and velocity interfaces
    for (unsigned int i = 0; i < K_NUM_WHEELS; i++)
    {
        ROS_INFO("Registering interface for joint %s", jointNames[i].c_str());

        hardware_interface::JointStateHandle stateHandle(jointNames[i], &m_Positions[i], &m_Velocities[i], &m_Effort[i]);
        m_JointStateInterface.registerHandle(stateHandle);

        hardware_interface::JointHandle velocityHandle(m_JointStateInterface.getHandle(jointNames[i]), &m_Commands[i]);
        m_VelocityJointInterface.registerHandle(velocityHandle);
    }
    
    registerInterface(&m_JointStateInterface);
    registerInterface(&m_VelocityJointInterface);

    m_WheelVelocityPublisher = rootNh.advertise<std_msgs::Float32MultiArray>("wheel_velocities/", 1);

    return true;
}

bool ScoutHardwareInterface::AppendWheelNames(ros::NodeHandle& nodeHandle, const std::string& paramName, std::vector<std::string>& wheelNames)
{
    std::vector<std::string> currentWheelNames;
    if(!nodeHandle.getParam(paramName, currentWheelNames))
    {
        return false;
    }

    wheelNames.insert(wheelNames.end(), currentWheelNames.begin(), currentWheelNames.end());
    return true;
}

void ScoutHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
    //TODO read wheel states
    for(unsigned int i = 0; i < K_NUM_WHEELS; i++)
    {
        m_Velocities[i] = m_Commands[i];
        m_Positions[i] += m_Velocities[i] * period.toSec();
    }
}

void ScoutHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
    for(unsigned int i = 0; i < K_NUM_WHEELS; i++)
    {
        m_WheelVelocityMessage.data[i] = m_Commands[i];
    }

    m_WheelVelocityPublisher.publish(m_WheelVelocityMessage);
}