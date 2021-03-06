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
// Publishers
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

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
        LF	= 0,
        RF	= 1,
        LR	= 2,
        RR	= 3,
        Count = 4
    };
    /**
     * @brief The SteerJoints enum Enum representing the steerable joints in our vehicle.
     */
    enum class SteerJoints : uint8_t
    {
        LF	= 0,
        RF	= 1,
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
     * @brief mImuMessageSub Subscriber for IMU messages.
     */
    ros::Subscriber mImuMessageSub;
    // Publishers
    /**
     * @brief mJointStatePublisher The publisher that publishes the joint state.
     */
    ros::Publisher mJointStatePublisher;
    /**
     * @brief mWheelOdomPublisher The publisher that publishes wheel odometry for
     * robot_localization.
     */
    ros::Publisher mWheelOdomPublisher;
    /**
     * @brief mOdomTransformPublisher Transform for odometry frame to base_link frame.
     */
    tf::TransformBroadcaster mOdomTransformBroadcaster;
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
     * @brief mJointState The joint state that needs to be published.
     */
    sensor_msgs::JointState mJointState;
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
     * @brief mLastImuMessageTime The last noted time when the IMU message has been received.
     */
    common::Time mLastImuMessageTime;
    /**
     * @brief mVehicleCartesianXPos The cartesian x-position calculated by integrating wheel
     * velocities from encoder.
     */
    double mVehicleCartesianXPos;
    /**
     * @brief mVehicleCartesianYPos The cartesian y-position calculated by integrating wheel
     * velocities from encoder.
     */
    double mVehicleCartesianYPos;
    /**
     * @brief mVehicleIntegratedHeading The vehicle heading calculated by integrating angular
     * velocity.
     */
    double mVehicleIntegratedHeading;
    /**
     * @brief mAckermannCommandReceived Flag whether an Ackermann message has been received.
     * Think of it like a throttle, when you left it, it goes back but that doesn't mean
     * the car will stop immediately. Due to Interia, the car will travel for certain distance
     * before coming to halt.
     * TODO : Use a circular buffer to enque reference values and deque them as soon as the
     * reference value is processed. If the buffer gets empty dont set the PID values.
     */
    bool mAckermannCommandReceived;
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
     * @brief mXPosition Current cartesian position
     */
    double mXPosition;
    /**
     * @brief mYPosition Current cartesian y coordinate
     */
    double mYPosition;
    /**
     * @brief mHeading Current heading of the vehicle.
     */
    double mHeading;
    /**
     * @brief mCurrentOdometry The current odometry result of the robot.
     */
    nav_msgs::Odometry mCurrentOdometry;
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
     * @brief imuInterruptCallback Callback for the IMU message. The interrupt might be a bit
     * misleading as there are no interrupts involved. Its named so as to replicate the behavior on
     * the real robot.
     * @param imuMessage The Imu message.
     */
    void imuInterruptCallback(const sensor_msgs::Imu::ConstPtr& imuMessage);
    /**
     * @brief OnUpdate Will be called when the gazebo simulation updates.
     */
    void OnUpdate();
    /**
     * @brief updatePIDs Updates the PIDs based on the reference received. Will be called on every
     * update depending on whether we have received any new message.
     */
    void updatePIDs(const common::Time& deltaTime);
};
} // namespace gazebo
