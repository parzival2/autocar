#include <ackermann_msgs/AckermannDrive.h>
#include <autocar_ackermann_plugin/autocar_pidConfig.h>
#include <dynamic_reconfigure/server.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ignition/math/Vector3.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <thread>

namespace gazebo
{
class AckermannPlugin : public ModelPlugin
{
  public:
    // aliases
    using PIDConfig = autocar_ackermann_plugin::autocar_pidConfig;
    /**
     *   Reset all the values if the AckermannDrive message doesnt arrive after this time
     *   has passed
     */
    static constexpr double DEFAULT_RESET_TIME = 5.0;
    /**
     * @brief The DriveJoints enum Enum representing the drivable joints in our vehicle.
     */
    enum class DriveJoints : uint8_t
    {
        LF = 0,
        RF = 1,
        LR = 2,
        RR = 3,
        Count = 4
    };
    /**
     * @brief The SteerJoints enum Enum representing the steerable joints in our vehicle.
     */
    enum class SteerJoints : uint8_t
    {
        LF = 0,
        RF = 1,
        Count = 2
    };

    AckermannPlugin();
    ~AckermannPlugin();
    // Overload the load method
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  private:
    // Variables
    /**
     * @brief mGazeboRosPtr Pointer to the gazebo ros
     */
    GazeboRosPtr mGazeboRosPtr;
    // Model
    physics::ModelPtr mModelPtr;
    /**
     * @brief mRosNodeHandle Ros node handle
     */
    ros::NodeHandle mRosNodeHandle;
    /**
     * @brief mRosQueue Callback queue for ros messages
     */
    ros::CallbackQueue mRosQueue;
    // Subscribers
    /**
     * @brief AckermannDrive command subscriber for receiving messages.
     */
    ros::Subscriber mAckermannMsgSub;
    /**
     * @brief mRosQueueThread A Thread to process messages in queue.
     */
    std::thread mRosQueueThread;
    /**
     * @brief mUpdateConnection Pointer to the update event
     */
    event::ConnectionPtr mUpdateConnection;
    /**
     * @brief mAckermannCommandTopic The topic on which to listen for AckermannDrive messages.
     */
    std::string mAckermannCommandTopic;
    /**
     * @brief mDriveJointNames Array of strings that represent the drivable joints in the model.
     */
    std::string mDriveJointNames[static_cast<uint8_t>(DriveJoints::Count)];
    /**
     * @brief mSteerJointNames Array of strings that represent the steerable joints in the model.
     */
    std::string mSteerJointNames[static_cast<uint8_t>(SteerJoints::Count)];
    /**
     * @brief mSteerJoints Vector of steerable joints represented by JointPtr
     */
    std::vector<physics::JointPtr> mSteerJoints;
    /**
     * @brief mDriveJoints Vector of drivable joints represented by JointPtr
     */
    std::vector<physics::JointPtr> mDriveJoints;
    /**
     * @brief mDrivePids PID controllers used to control the drivable joints LR, RR
     */
    std::vector<common::PID> mDrivePids;
    /**
     * @brief mSteerPids PID controllers used to control the steerable joints LF, RF
     */
    std::vector<common::PID> mSteerPids;
    /**
     * @brief mReferenceWheelSpeeds Reference wheel speeds that we will try to acheive.
     * These will be calculated when we receive the AckermannDrive message.
     */
    std::vector<double> mReferenceWheelSpeeds;
    /**
     * @brief mReferenceSteerAngles Reference steering wheel angles that we will try to acheive
     * These will be calculated when we receive the AckermannDrive message.
     */
    std::vector<double> mReferenceSteerAngles;
    /**
     * @brief mReferenceAckermannMsg The message that is received on the callback. This will be
     * the reference state that we will try to achieve.
     */
    ackermann_msgs::AckermannDrive::ConstPtr mReferenceAckermannMsgPtr;
    /**
     * @brief mLastUpdateTime The last update time to calculate the time difference between the
     * updates.
     */
    common::Time mLastUpdateTime;
    /**
     * @brief mLastAckermannCmdTime Just for keeping track of whether we have received any ackermann
     * command message.
     */
    common::Time mLastAckermannCmdTime;
    /**
     * @brief mWheelBase Wheelbase that we receive from model plugin parameters.
     */
    double mWheelBase;
    /**
     * @brief mWheelSeperation Wheel seperation that we receive from model plugin parameters.
     */
    double mWheelSeperation;
    /**
     * @brief mWheelDiameter Wheel diameter.
     */
    double mWheelDiameter;
    /**
     * @brief mReconfigureServer Dynamic reconfigure server for tuning PID parameters
     */
    boost::shared_ptr<dynamic_reconfigure::Server<PIDConfig>> mReconfigureServer;
    /**
     * @brief mCallbackType Callback function type for the reconfigure server.
     */
    dynamic_reconfigure::Server<PIDConfig>::CallbackType mCallbackType;
    /**
     * @brief dynamicReconfigureCallback Callback function for the dynamic reconfigure server.
     * @param config The config that we are going to receive
     * @param level Log level.
     */
    void dynamicReconfigureCallback(PIDConfig& config, uint32_t level);
    /**
     * @brief QueueThread Function to process messages in queue
     */
    void QueueThread();
    /**
     * @brief ackermannCallback Callback for the AckermannDrive message
     * @param ackermann_msg AckermannDrive message
     */
    void ackermannCallback(const ackermann_msgs::AckermannDrive::ConstPtr& ackermann_msg);
    /**
     * @brief OnUpdate Will be called when the gazebo simulation updates.
     */
    void OnUpdate();
};
} // namespace gazebo
