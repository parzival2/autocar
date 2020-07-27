#include "ackermann_plugin.hh"
#include "std_msgs/String.h"

namespace gazebo
{
// Constructor
AckermannPlugin::AckermannPlugin()
    : mAckermannCommandReceived(false)
    , mVehicleCartesianXPos(0.)
    , mVehicleCartesianYPos(0.)
    , mVehicleIntegratedHeading(0.)
    , mXPosition(0.0)
    , mYPosition(5.0)
    , mHeading(0.0)

{
}

// Destructor
AckermannPlugin::~AckermannPlugin() {}

/**
 * @brief AckermannPlugin::Load This function will be called when Gazebo first loads the plugin.
 * @param _model Pointer to the model.
 * @param _sdf Pointer to the SDF element tree.
 */
void AckermannPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Gazebo ros instance.
    this->mGazeboRosPtr = GazeboRosPtr(new GazeboRos(_model, _sdf, "AckermannPlugin"));

    // Model
    this->mModelPtr = _model;

    // Check if ros is initialized.
    this->mGazeboRosPtr->isInitialized();

    this->mUpdateConnection =
        event::Events::ConnectWorldUpdateBegin(std::bind(&AckermannPlugin::OnUpdate, this));

    // Get the strings representing the drivable joints in our vehicle.
    /// Left Front wheel drivable joint
    this->mDriveJointNames[static_cast<uint8_t>(DriveJoints::LF)] =
        _sdf->Get<std::string>("LF_driveJoint");
    ROS_INFO_STREAM("AckermannPlugin : Provided Left Front drivable joint : "
                    << mDriveJointNames[static_cast<uint8_t>(DriveJoints::LF)]);
    /// Right Front wheel drivable joint
    this->mDriveJointNames[static_cast<uint8_t>(DriveJoints::RF)] =
        _sdf->Get<std::string>("RF_driveJoint");
    ROS_INFO_STREAM("AckermannPlugin : Provided Right Front drivable joint : "
                    << mDriveJointNames[static_cast<uint8_t>(DriveJoints::RF)]);
    /// Left Rear wheel drivable joint
    this->mDriveJointNames[static_cast<uint8_t>(DriveJoints::LR)] =
        _sdf->Get<std::string>("LR_driveJoint");
    ROS_INFO_STREAM("AckermannPlugin : Provided Left Rear drivable joint : "
                    << mDriveJointNames[static_cast<uint8_t>(DriveJoints::LR)]);
    /// Right Rear wheel drivable joint
    this->mDriveJointNames[static_cast<uint8_t>(DriveJoints::RR)] =
        _sdf->Get<std::string>("RR_driveJoint");
    ROS_INFO_STREAM("AckermannPlugin : Provided Right Rear drivable joint : "
                    << mDriveJointNames[static_cast<uint8_t>(DriveJoints::RR)]);
    ///
    /// Steering joints
    ///
    this->mSteerJointNames[static_cast<uint8_t>(SteerJoints::LF)] =
        _sdf->Get<std::string>("LF_steerJoint");
    ROS_INFO_STREAM("AckermannPlugin : Provided Left Front steering joint : "
                    << mSteerJointNames[static_cast<uint8_t>(SteerJoints::LF)]);
    this->mSteerJointNames[static_cast<uint8_t>(SteerJoints::RF)] =
        _sdf->Get<std::string>("RF_steerJoint");
    ROS_INFO_STREAM("AckermannPlugin : Provided Right Front steering joint : "
                    << mSteerJointNames[static_cast<uint8_t>(SteerJoints::RF)]);
    ////
    //// Wheel base and Wheel seperation
    ////
    mWheelBase = _sdf->Get<double>("wheelBase");
    ROS_INFO_STREAM("AckermannPlugin : Wheel base received : " << mWheelBase);
    mWheelSeperation = _sdf->Get<double>("wheelSeperation");
    ROS_INFO_STREAM("AckermannPlugin : Wheel seperation received : " << mWheelSeperation);

    ////
    //// Wheel diameter
    ////
    mWheelDiameter = _sdf->Get<double>("wheelDiameter");
    ROS_INFO_STREAM("AckermannPlugin : Wheel diameter received : " << mWheelDiameter);

    // Initialize the drivable joints
    mDriveJoints.resize(static_cast<uint8_t>(DriveJoints::Count));
    mReferenceWheelSpeeds.resize(static_cast<uint8_t>(DriveJoints::Count));
    mReferenceWheelSpeeds.assign(static_cast<uint8_t>(DriveJoints::Count), 0.0);
    mSteerJoints.resize(static_cast<uint8_t>(SteerJoints::Count));
    mReferenceSteerAngles.resize(static_cast<uint8_t>(SteerJoints::Count));
    mReferenceSteerAngles.assign(static_cast<uint8_t>(SteerJoints::Count), 0.0);
    // PIDS
    mDrivePids.resize(static_cast<uint8_t>(DriveJoints::Count));
    mSteerPids.resize(static_cast<uint8_t>(SteerJoints::Count));
    /// Note that these are already tuned parameters.
    // Fill up the pointers for drive joints
    for(size_t i = 0; i < mDriveJoints.size(); i++)
    {
        mDriveJoints[i] = mModelPtr->GetJoint(mDriveJointNames[i]);
        mDrivePids[i].Init(0.75, 0.1, 0.0, 5.0, 0.0, 10.0, -10.0);
    }
    // Fill up the pointers for steer joints
    for(size_t j = 0; j < mSteerJoints.size(); j++)
    {
        mSteerJoints[j] = mModelPtr->GetJoint(mSteerJointNames[j]);
        mSteerPids[j].Init(50.0, 0.1, 0.09, 5.0, 0.0, 1.5, -1.5);
    }
    mJointState.name.resize(mModelPtr->GetJoints().size());
    mJointState.position.resize(mModelPtr->GetJoints().size());
    mJointState.velocity.resize(mModelPtr->GetJoints().size());
    for(int i = 0; i < mJointState.name.size(); i++)
    {
        ROS_INFO_STREAM("AckermannPlugin : Joint found = " << mModelPtr->GetJoints()[i]->GetName());
        mJointState.name[i] = mModelPtr->GetJoints()[i]->GetName();
    }
    // Create a new node
    if(!ros::isInitialized())
    {
        int argc	= 0;
        char** argv = NULL;
        ros::init(argc, argv, "ackermann_plugin_node", ros::init_options::NoSigintHandler);
    }
    mRosNodeHandle = ros::NodeHandle("ackermann_plugin_node");
    // Initialize the last update time
    mLastUpdateTime		  = mModelPtr->GetWorld()->SimTime();
    mLastAckermannCmdTime = mModelPtr->GetWorld()->SimTime();
    mLastImuMessageTime   = mModelPtr->GetWorld()->SimTime();
    // Initialize the reconfigure server
    mReconfigureServer.reset(new dynamic_reconfigure::Server<PIDConfig>(mRosNodeHandle));
    mCallbackType = boost::bind(&AckermannPlugin::dynamicReconfigureCallback, this, _1, _2);
    mReconfigureServer->setCallback(mCallbackType);
    // Ackermann callback
    ros::SubscribeOptions so = ros::SubscribeOptions::create<ackermann_msgs::AckermannDrive>(
        "/ackermann_cmd", 1, boost::bind(&AckermannPlugin::ackermannCallback, this, _1),
        ros::VoidPtr(), &this->mRosQueue);
    this->mAckermannMsgSub = this->mRosNodeHandle.subscribe(so);
    // In the real robot, we publish wheel sensor speeds when the interrupt has been fired on IMU.
    // So here as well we will do the same.
    ros::SubscribeOptions wheelVelSub = ros::SubscribeOptions::create<sensor_msgs::Imu>(
        "/imu", 1, boost::bind(&AckermannPlugin::imuInterruptCallback, this, _1), ros::VoidPtr(),
        &this->mRosQueue);
    this->mImuMessageSub = this->mRosNodeHandle.subscribe(wheelVelSub);
    // Publisher
    this->mJointStatePublisher =
        this->mRosNodeHandle.advertise<sensor_msgs::JointState>("/joint_states", 10);
    this->mWheelOdomPublisher =
        this->mRosNodeHandle.advertise<nav_msgs::Odometry>("/odom", 10);
    ROS_INFO_STREAM("Plugin Initialized");
    ROS_INFO_STREAM("Name of the model : " << this->mModelPtr->GetName().c_str());
    this->mRosQueueThread = std::thread(std::bind(&AckermannPlugin::QueueThread, this));
}

/**
 * @brief AckermannPlugin::dynamicReconfigureCallback Callback that will be called.
 * @param config
 * @param level
 */
void AckermannPlugin::dynamicReconfigureCallback(AckermannPlugin::PIDConfig& config, uint32_t level)
{
    ROS_INFO_STREAM("++++++++++++++ AckermannPlugin : Received new PID values ++++++++++++++");
    ROS_INFO_STREAM("AckermannPlugin : Drive P value : " << config.drive_p);
    ROS_INFO_STREAM("AckermannPlugin : Drive I value : " << config.drive_i);
    ROS_INFO_STREAM("AckermannPlugin : Drive D value : " << config.drive_d);
    // Set the PID values
    for(size_t i = 0; i < static_cast<uint8_t>(DriveJoints::Count); i++)
    {
        mDrivePids[i].SetPGain(config.drive_p);
        mDrivePids[i].SetIGain(config.drive_i);
        mDrivePids[i].SetDGain(config.drive_d);
    }
    // Set the PID values for steer joints
    ROS_INFO_STREAM("AckermannPlugin : Steer P value : " << config.steer_p);
    ROS_INFO_STREAM("AckermannPlugin : Steer I value : " << config.steer_i);
    ROS_INFO_STREAM("AckermannPlugin : Steer D value : " << config.steer_d);
    // Set the PID values
    for(size_t i = 0; i < static_cast<uint8_t>(SteerJoints::Count); i++)
    {
        mSteerPids[i].SetPGain(config.steer_p);
        mSteerPids[i].SetIGain(config.steer_i);
        mSteerPids[i].SetDGain(config.steer_d);
    }
}

/**
 * @brief AckermannPlugin::QueueThread Function to process messages in queue.
 */
void AckermannPlugin::QueueThread()
{
    static const double timeout = 0.01;
    while(this->mRosNodeHandle.ok())
    {
        this->mRosQueue.callAvailable((ros::WallDuration(timeout)));
    }
}

/**
 * @brief AckermannPlugin::OnUpdate This function will be called whenever gazebo updates its
 * simulation.
 */
void AckermannPlugin::OnUpdate()
{
    common::Time currentTime	   = mModelPtr->GetWorld()->SimTime();
    common::Time deltaTime		   = currentTime - mLastUpdateTime;
    common::Time deltaAckremannMsg = currentTime - mLastAckermannCmdTime;
    if(deltaAckremannMsg.Double() > DEFAULT_RESET_TIME)
    {
        mReferenceWheelSpeeds.assign(static_cast<uint8_t>(DriveJoints::Count), 0.0);
        mReferenceSteerAngles.assign(static_cast<uint8_t>(SteerJoints::Count), 0.0);
    }

    if(deltaTime > 0.01)
    {
        // Only set the PIDs if we have received a new ackermann command message
        // or else let the wheel joint friction slow the vehicle.
        if(mAckermannCommandReceived)
        {
            this->updatePIDs(deltaTime);
        }
        ///
        /// Steering angles
        /// Left front
        for(size_t i = 0; i < mSteerJoints.size(); i++)
        {
            double currentSteeringAngle = mSteerJoints[static_cast<uint8_t>(i)]->Position(0);
            double steeringAngleError =
                currentSteeringAngle - mReferenceSteerAngles[static_cast<uint8_t>(i)];
            double steerCommandEffort =
                mSteerPids[static_cast<uint8_t>(i)].Update(steeringAngleError, deltaTime);
            mSteerJoints[static_cast<uint8_t>(i)]->SetForce(0, steerCommandEffort);
        }
        ///
        /// Publish joint states
        ///
        // Set header
        mJointState.header.stamp = ros::Time::now();
        for(size_t i = 0; i < mJointState.position.size(); i++)
        {
            physics::JointPtr joint = mModelPtr->GetJoints()[i];
            mJointState.name[i]		= joint->GetName();
            mJointState.position[i] = joint->Position(0);
            mJointState.velocity[i] = joint->GetVelocity(0);
        }
        mJointStatePublisher.publish(mJointState);
        mLastUpdateTime = currentTime;
        // Set the flag to false indicating that we have processed the message.
        mAckermannCommandReceived = false;
    }
}

/**
 * @brief AckermannPlugin::updatePIDs Updates the PIDs based on the reference value received.
 */
void AckermannPlugin::updatePIDs(const common::Time& deltaTime)
{
    double wheelDia_2 = 0.5 * mWheelDiameter;
    // Set the reference speeds for wheels
    // Convert rad/sec to m/sec by multiplying the angular velocity with radius
    // v = r x omega
    /// Wheels
    /// Left rear
    double leftRearSpeed =
        mDriveJoints[static_cast<uint8_t>(DriveJoints::LR)]->GetVelocity(2) * wheelDia_2;
    double leftRearSpeedError =
        leftRearSpeed - mReferenceWheelSpeeds[static_cast<uint8_t>(DriveJoints::LR)];
    double leftRearCommandEffort =
        mDrivePids[static_cast<uint8_t>(DriveJoints::LR)].Update(leftRearSpeedError, deltaTime);
    mDriveJoints[static_cast<uint8_t>(DriveJoints::LR)]->SetForce(2, leftRearCommandEffort);
    /// Right rear
    double rightRearSpeed =
        mDriveJoints[static_cast<uint8_t>(DriveJoints::RR)]->GetVelocity(2) * wheelDia_2;
    double rightRearSpeedError =
        rightRearSpeed - mReferenceWheelSpeeds[static_cast<uint8_t>(DriveJoints::RR)];
    double rightRearCommandEffort =
        mDrivePids[static_cast<uint8_t>(DriveJoints::RR)].Update(rightRearSpeedError, deltaTime);
    mDriveJoints[static_cast<uint8_t>(DriveJoints::RR)]->SetForce(2, rightRearCommandEffort);
}

/**
 * @brief AckermannPlugin::ackermannCallback Callback message for ackermann_cmd topic
 * @param ackermann_msg AckermannDrive message.
 */
void AckermannPlugin::ackermannCallback(
    const ackermann_msgs::AckermannDrive::ConstPtr& ackermann_msg)
{
    // Set the time for ackermann command message
    mLastAckermannCmdTime			= mModelPtr->GetWorld()->SimTime();
    this->mReferenceAckermannMsgPtr = ackermann_msg;
    // Calculate the steering angles taking into account the wheel base and wheel seperation.
    double numerator = static_cast<double>(2.0) * mWheelBase * sin(ackermann_msg->steering_angle);
    double denomLF = (static_cast<double>(2.0) * mWheelBase * cos(ackermann_msg->steering_angle)) -
                     (mWheelSeperation * sin(ackermann_msg->steering_angle));
    double denomRF = (static_cast<double>(2.0) * mWheelBase * cos(ackermann_msg->steering_angle)) +
                     (mWheelSeperation * sin(ackermann_msg->steering_angle));
    mReferenceSteerAngles.assign(static_cast<uint8_t>(SteerJoints::Count), 0.0);
    mReferenceSteerAngles[static_cast<uint8_t>(SteerJoints::LF)] = atan2(numerator, denomLF);
    mReferenceSteerAngles[static_cast<uint8_t>(SteerJoints::RF)] = atan2(numerator, denomRF);
    ROS_DEBUG_STREAM("Received steering angle : " << ackermann_msg->steering_angle);
    ROS_DEBUG_STREAM("Calculated steering angle LF : "
                     << mReferenceSteerAngles[static_cast<uint8_t>(SteerJoints::LF)]);
    ROS_DEBUG_STREAM("Calculated steering angle RF : "
                     << mReferenceSteerAngles[static_cast<uint8_t>(SteerJoints::RF)]);
    // Calculate the wheel speeds
    mReferenceWheelSpeeds.assign(static_cast<uint8_t>(DriveJoints::Count), 0.);
    double spNumLR =
        static_cast<double>(1.0) - (mWheelSeperation * tan(ackermann_msg->steering_angle));
    double spNumRR =
        static_cast<double>(1.0) + (mWheelSeperation * tan(ackermann_msg->steering_angle));
    double spDenom = static_cast<double>(2.0) * mWheelBase;
    mReferenceWheelSpeeds[static_cast<uint8_t>(DriveJoints::LR)] =
        ackermann_msg->speed * (spNumLR / spDenom);
    mReferenceWheelSpeeds[static_cast<uint8_t>(DriveJoints::RR)] =
        ackermann_msg->speed * (spNumRR / spDenom);
    // Set the flag that we receive a new ackermann command message
    mAckermannCommandReceived = true;
    ROS_DEBUG_STREAM("Received ackermann message");
    ROS_DEBUG_STREAM("Reference speed : " << ackermann_msg->speed);
    ROS_DEBUG_STREAM("Reference angle : " << ackermann_msg->steering_angle);
}

/**
 * @brief AckermannPlugin::imuInterruptCallback The callback that will be called when the imu
 * message is published from Gazebo.
 * @param imuMessage The imu message that has been published.
 */
void AckermannPlugin::imuInterruptCallback(const sensor_msgs::Imu::ConstPtr& imuMessage)
{
    common::Time currentTime = mModelPtr->GetWorld()->SimTime();
    double deltaTime		 = (currentTime - mLastImuMessageTime).Double();
    // Calculate the velocity of the wheels.
    double wheelDia_2 = 0.5 * mWheelDiameter;
    double rearLeftVel =
        mDriveJoints[static_cast<uint8_t>(DriveJoints::LR)]->GetVelocity(2) * wheelDia_2;
    double rearRightVel =
        mDriveJoints[static_cast<uint8_t>(DriveJoints::RR)]->GetVelocity(2) * wheelDia_2;
    double currentSpeed = (rearLeftVel + rearRightVel) * 0.5;
    // Steering angle will be the average of the two
    double frontLeftSteer  = mSteerJoints[static_cast<uint8_t>(SteerJoints::LF)]->Position(0);
    double frontRightSteer = mSteerJoints[static_cast<uint8_t>(SteerJoints::RF)]->Position(0);
    double steeringAngle   = (frontLeftSteer + frontRightSteer) * 0.5;
    mCurrentOdometry.twist.twist.linear.x  = currentSpeed;
    mCurrentOdometry.twist.twist.linear.y  = 0;
    mCurrentOdometry.twist.twist.angular.z = currentSpeed * std::tan(steeringAngle) / mWheelBase;
    // Integrate wheel velocities and heading of the vehicle
    mXPosition += (currentSpeed * std::cos(mHeading) * deltaTime);
    mYPosition += (currentSpeed * std::sin(mHeading) * deltaTime);
    mHeading += (mCurrentOdometry.twist.twist.angular.z * deltaTime);
    // Set pose in odometry
    mCurrentOdometry.pose.pose.position.x = mXPosition;
    mCurrentOdometry.pose.pose.position.y = mYPosition;
    mCurrentOdometry.pose.pose.position.z = 0.0;
    // Transform broadcast
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp				= ros::Time::now();
    odomTrans.header.frame_id			= "odom";
    odomTrans.child_frame_id			= "base_link";
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(mHeading);
    odomTrans.transform.translation.x   = mXPosition;
    odomTrans.transform.translation.y   = mYPosition;
    odomTrans.transform.rotation		= odom_quat;
    // Also set the covariance matrix
    mCurrentOdometry.twist.covariance = boost::array<double, 36>({0.25, 0., 0., 0., 0., 0.,
                                                                  0., 0.25, 0., 0., 0., 0.,
                                                                  0., 0., 0.25, 0., 0., 0.,
                                                                  0., 0., 0., 0.25, 0., 0.,
                                                                  0., 0., 0., 0., 0.25, 0.,
                                                                  0., 0., 0., 0., 0., 0.25});
    //
    mCurrentOdometry.pose.covariance = boost::array<double, 36>({0.25, 0., 0., 0., 0., 0.,
                                                                  0., 0.25, 0., 0., 0., 0.,
                                                                  0., 0., 0.25, 0., 0., 0.,
                                                                  0., 0., 0., 0.25, 0., 0.,
                                                                  0., 0., 0., 0., 0.25, 0.,
                                                                  0., 0., 0., 0., 0., 0.25});
    mOdomTransformBroadcaster.sendTransform(odomTrans);
    // Set header and child frame ids
    mCurrentOdometry.header.stamp = ros::Time::now();
    mCurrentOdometry.header.frame_id = "odom";
    mCurrentOdometry.child_frame_id  = "base_link";
    mWheelOdomPublisher.publish(mCurrentOdometry);
    mLastImuMessageTime = currentTime;
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AckermannPlugin)
} // namespace gazebo
