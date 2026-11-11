#include "ros/ros.h"
#include <inttypes.h>
#include "assignment1_setup/Sonars.h"
#include "geometry_msgs/Twist.h"
#include <chrono>
#include <cmath>
#include <mutex>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "tf/transform_datatypes.h"
#include <Eigen/Dense>
#include "assignment1_setup/ModelState.h"

bool task_1 = false;
bool task_2 = false;
// If none are set, then it assumes task 3

// Global variables
double target;                                      // Target distance from the bowl
double init_yaw;                                    // Initial yaw angle
double volatile current_position_estimate_filtered; // Filtered current position estimate
double bowl_x, bowl_y;                              // Bowl position
ros::NodeHandle *n;                                 // Node handle pointer

// Kalman Filter class
class KalmanFilter
{
public:
    Eigen::Vector2d x; // State vector
    Eigen::Matrix2d P; // Error covariance matrix
    Eigen::Matrix2d A; // Transition matrix
    Eigen::Matrix2d Q; // Process noise covariance matrix
    Eigen::Matrix2d H; // Observation matrix
    Eigen::Matrix2d R; // Measurement noise covariance matrix
    Eigen::Matrix2d I; // Identity matrix

    // Constructor
    KalmanFilter(const Eigen::Matrix2d &initialR) : R(initialR)
    {
        // Initialize matrices
        A << 1, 0, 0, 1;
        H << 1, 0, 0, 1;
        Q << 0.1, 0, 0, 0.1;
        I.setIdentity();

        // Initial state estimate
        x << 0, 0;

        // Initial covariance estimate
        P << 1, 0, 0, 1;
    }

    // Update method
    void update(const Eigen::Vector2d &sensorMeasurement)
    {
        double robot_x, robot_y;
        double robot_x_variance, robot_xy_covariance, robot_y_variance;
        if (task_2)
        {
            // Use the /gazebo/get_model_state service to retrieve the robot position and covariance
            ros::ServiceClient get_state_client = n->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
            gazebo_msgs::GetModelState get_robot_state_srv;
            get_robot_state_srv.request.model_name = "turtlebot3_burger";
            if (get_state_client.call(get_robot_state_srv))
            {
                // Retrieve robot position and covariance from service response
                robot_x = get_robot_state_srv.response.pose.position.x;
                robot_y = get_robot_state_srv.response.pose.position.y;
                robot_x_variance = 0.01;
                robot_xy_covariance = 0.01;
                robot_y_variance = 0.01;
            }
            else
            {
                ROS_ERROR("Failed to retrieve the position of the robot!");
                return;
            }
        }
        else
        {
            ros::ServiceClient get_state_client = n->serviceClient<assignment1_setup::ModelState>("/turtlebot_position");
            assignment1_setup::ModelState get_bowl_state_srv;

            if (get_state_client.call(get_bowl_state_srv))
            {
                // Retrieve robot position and covariance from service response
                robot_x = get_bowl_state_srv.response.location.x;
                robot_y = get_bowl_state_srv.response.location.y;
                robot_x_variance = get_bowl_state_srv.response.covariance[0].x;
                robot_xy_covariance = get_bowl_state_srv.response.covariance[0].y;
                robot_y_variance = get_bowl_state_srv.response.covariance[1].y;
            }
            else
            {
                ROS_ERROR("Failed to retrieve the position of the robot!");
                return;
            }
        }

        // Create prediction covariance matrix from sensor measurements
        Eigen::Matrix2d predictionCovariance;
        predictionCovariance << robot_x_variance, robot_xy_covariance, robot_xy_covariance, robot_y_variance;

        Eigen::Vector2d noisyPrediction(robot_x, robot_y);

        // Integrate external noisy prediction
        Eigen::Matrix2d predictedP = A * P * A.transpose() + Q;                  // Predicted error covariance
        Eigen::Matrix2d totalCovariance = predictedP + predictionCovariance;     // Combine covariances
        Eigen::Matrix2d predictionGain = predictedP * totalCovariance.inverse(); // Compute gain

        x = A * x + predictionGain * (noisyPrediction - A * x); // Updated state with noisy prediction
        P = (I - predictionGain) * predictedP;                  // Updated covariance

        // Update phase based on sensor measurement
        Eigen::Matrix2d S = H * P * H.transpose() + R;
        Eigen::Matrix2d updateGain = P * H.transpose() * S.inverse(); // Kalman gain

        x = x + updateGain * (sensorMeasurement - H * x); // Update the state
        P = (I - updateGain * H) * P;                     // Update the error covariance
    }

    // Get state estimate method
    double getStateEstimate() const
    {
        return sqrt(x(0) * x(0) + x(1) * x(1));
    }
};

KalmanFilter *kalman_filter; // Kalman Filter object pointer

// PID Controller class
class PIDController
{
private:
    double Kp;             // Proportional gain
    double Ki;             // Integral gain
    double Kd;             // Derivative gain
    double previous_error; // Previous error for derivative term
    double integral;       // Integral of error
    double last_time;      // Timestamp of the last update
    double previous_pv;    // Previous position value

public:
    // Constructor
    PIDController(double kp, double ki, double kd)
        : Kp(kp), Ki(ki), Kd(kd), previous_error(0.0), integral(0.0), last_time(0.0) {}

    // Update method
    double update(double setpoint, double pv, double current_time)
    {
        if (last_time == 0.0)
        { // Initialize the last_time on first call
            last_time = current_time;
            previous_pv = pv;
        }

        double dt = current_time - last_time; // Calculate the time elapsed since the last update
        if (dt <= 0)
        {
            ROS_INFO("Non-positive time step detected. Skipping this update.");
            return 0.0;
        }

        double error = setpoint - pv;
        integral += error * dt; // Error integrated over the time period dt

        double derivative = (error - previous_error) / dt; // Derivative term adjusted for variable time steps
        previous_error = error;
        previous_pv = pv;         // Update the previous position value
        last_time = current_time; // Update the last update time

        return Kp * error + Ki * integral + Kd * derivative;
    }
};

// Callback function for sonar data
void sonars_callback(const assignment1_setup::Sonars::ConstPtr &sonars)
{
    double front_sonar_distance_m = sonars->distance1 / 100.0; // Convert cm to meters

    ROS_INFO("Front sonar distance: %f meters", front_sonar_distance_m);

    if (task_1)
    {
        current_position_estimate_filtered = target - front_sonar_distance_m;
        return;
    }

    // Calculate robot position based on sonar data
    double x = front_sonar_distance_m * cos(init_yaw);
    double y = front_sonar_distance_m * sin(init_yaw);

    Eigen::Vector2d z(bowl_x - x, bowl_y - y);

    // Update Kalman filter with new measurement
    kalman_filter->update(z);
    current_position_estimate_filtered = kalman_filter->getStateEstimate();

    ROS_INFO("Current position estimate: %f meters", current_position_estimate_filtered);
}

// Function to control robot movement using PID
void pid_move_towards_bowl(ros::Publisher &pub)
{
    ros::Rate rate(5); // Control loop at 5 Hz

    PIDController pid(0.1, 0, 0); // PID controller with specified gains
    double last_pid_output = 0.0;

    while (ros::ok())
    {
        // Break if the robot is not facing the target anymore
        if (current_position_estimate_filtered < -1)
        {
            ROS_INFO("Robot not facing the target anymore. Stopping the robot.");
            geometry_msgs::Twist velocity;
            velocity.linear.x = 0.0;
            pub.publish(velocity);
            rate.sleep();
            break;
        }

        // Break from the loop if the target is reached and makes sure this isn't the first iteration
        // if (fabs(target - current_position_estimate_filtered) < 0.1 && last_pid_output != 0)
        // {
        //     ROS_INFO("Target reached! Stopping the robot.");
        //     geometry_msgs::Twist velocity;
        //     velocity.linear.x = 0.0;
        //     pub.publish(velocity);
        //     rate.sleep();
        //     break;
        // }

        // Calculate PID output
        double pid_output = pid.update(target, current_position_estimate_filtered, ros::Time::now().toSec());

        // Cap velocity
        pid_output = std::min(std::max(pid_output, -0.15), 0.15);

        ROS_INFO("PID output: %f", pid_output);

        // Slowly accelerate the robot at the beginning using the last PID output up to the current one
        // if (last_pid_output < pid_output)
        // {
        //     pid_output = std::min(pid_output, last_pid_output + 0.001);
        // }

        // Publish velocity command
        geometry_msgs::Twist velocity;
        velocity.linear.x = pid_output;
        pub.publish(velocity);

        last_pid_output = pid_output;

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    n = new ros::NodeHandle();

    Eigen::Matrix2d initialR;
    initialR << 0.1, 0.0, 0.0, 0.1;

    kalman_filter = new KalmanFilter(initialR);

    ros::Subscriber sub = n->subscribe("sonars", 1, sonars_callback);
    ros::Publisher pub = n->advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Wait until we have subscribers for the publisher
    while (pub.getNumSubscribers() == 0)
    {
        ros::Duration(0.1).sleep();
    }

    // Set the initial velocity of the robot to 0
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0.0;
    velocity.angular.z = 0.0;
    pub.publish(velocity);

    // Wait for the robot to stop
    // ros::Duration(2.0).sleep();

    // Set the initial position and orientation of the robot
    gazebo_msgs::SetModelState set_model_state_srv;
    set_model_state_srv.request.model_state.model_name = "turtlebot3_burger";
    set_model_state_srv.request.model_state.pose.position.x = 0.0;
    set_model_state_srv.request.model_state.pose.position.y = 0.0;
    set_model_state_srv.request.model_state.pose.position.z = 0.0;

    set_model_state_srv.request.model_state.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0); // Initialize orientation

    ros::ServiceClient set_state_client = n->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    if (!set_state_client.call(set_model_state_srv))
    {
        ROS_ERROR("Failed to set initial state for turtlebot3_burger!");
        return EXIT_FAILURE;
    }

    // Retrieve initial orientation based on bowl position
    gazebo_msgs::GetModelState get_bowl_state_srv;
    get_bowl_state_srv.request.model_name = "bowl";
    ros::ServiceClient get_state_client = n->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    if (get_state_client.call(get_bowl_state_srv))
    {
        // Calculate initial yaw angle to face the bowl
        bowl_x = get_bowl_state_srv.response.pose.position.x;
        bowl_y = get_bowl_state_srv.response.pose.position.y;
        init_yaw = atan2(bowl_y, bowl_x);

        // Logging for debugging
        ROS_INFO("Initial yaw calculated as: %f radians to face the bowl at position x: %f, y: %f", init_yaw, bowl_x, bowl_y);

        // Set initial orientation towards the bowl
        set_model_state_srv.request.model_state.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, init_yaw);
        if (!set_state_client.call(set_model_state_srv))
        {
            ROS_ERROR("Failed to set initial orientation towards the bowl for turtlebot3_burger!");
            return EXIT_FAILURE;
        }

        // ros::Duration(2.0).sleep();
    }
    else
    {
        ROS_ERROR("Failed to retrieve the position of the bowl!");
        return EXIT_FAILURE;
    }

    // Start the PID controlled movement towards the bowl
    target = sqrt(bowl_x * bowl_x + bowl_y * bowl_y); // Adjust target to stop 0.5 meter from the bowl

    ROS_INFO("Setting target distance to %f meters from the robot's current position towards the bowl", target);

    uint8_t task = std::stoi(argv[1]);

    task_1 = task == 1;
    task_2 = task == 2;

    // Print if it is task 1, 2, or 3 and give it a short break for 2 seconds before we start
    if (task_1)
    {
        ROS_INFO("Task 1");
    }
    else if (task_2)
    {
        ROS_INFO("Task 2");
    }
    else
    {
        ROS_INFO("Task 3");
    }

    ros::Duration(2.0).sleep();

    pid_move_towards_bowl(pub);
    delete kalman_filter;
    delete n;
    return 0;
}
