// --------------------- LIBRARIES --------------------- //
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>  // ROS related
#include <cartesian_interface/CartesianInterfaceImpl.h> // For the solver
#include <RobotInterfaceROS/ConfigFromParam.h>  // Model param config
#include <XBotInterface/ModelInterface.h>   // Model generation
#include <XBotInterface/RobotInterface.h>   // Robot generation
#include <iostream> // Standard
#include <thread>   // Standar
#include <xbot2/xbot2.h>
#include <xbot2/hal/dev_ft.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Twist.h>
#include <chrono>
#include <basestategetter.h>
#include <emafilter.h>
#include <lowpassfilter.h>
#include <fstream>
#include <awesome_utils/awesome_utils/sign_proc_utils.hpp>
#include <grahamscanconvexhull.h>

// --------------------- NAMESPACE --------------------- //
using namespace XBot;
using namespace XBot::Cartesian;
using namespace std;
using namespace Eigen;

// ------------------ GLOBAL VARIABLE ------------------ //
//TODO: change in a struct for the equation of the plane
//TODO: Fai sparire ste cose obrobriose
Vector3d pivot;
double d;


/*
========================================================================================================
                                                FUNCTIONS
========================================================================================================
*/
//TODO: all this function can be defined inside a separate class

/**
 * @brief Generate the type message PointStamped to publish
 * @param com_postion vector containing the coordinate of the CoM, w.r.t the base link
 * @return A variable of the type PointStamped ready to be published
 */
geometry_msgs::PointStamped COM_msg_generator (Vector3d com_postion){

    geometry_msgs::PointStamped com_point;

    //cout << com_postion << endl << "-------\n";

    com_point.header.frame_id = "base_link";  // Specify the frame ID
    com_point.point.x = com_postion.x();
    com_point.point.y = com_postion.y();
    com_point.point.z = com_postion.z();

    return com_point;

}

geometry_msgs::PointStamped COM_msg_generator_acc (Vector3d com_postion){

    geometry_msgs::PointStamped com_point;

    //cout << com_postion << endl << "-------\n";

    com_point.header.frame_id = "base_link";  // Specify the frame ID
    com_point.point.x = com_postion.x();
    com_point.point.y = com_postion.y();
    com_point.point.z = com_postion.z();

    return com_point;

}

/**
 * @brief CVX_HULL_msg_generator
 * @param convex_hull
 * @return A variable of the type PolygonStamped ready to be published
 */
geometry_msgs::PolygonStamped CVX_HULL_msg_generator (vector<Vector3d> convex_hull){

    geometry_msgs::PolygonStamped cvx_hull;

    cvx_hull.header.frame_id = "base_link";

    for (const Vector3d& convex_hull_point : convex_hull){

        geometry_msgs::Point32 point;
        point.x = convex_hull_point.x();
        point.y = convex_hull_point.y();
        point.z = convex_hull_point.z();

        cvx_hull.polygon.points.push_back(point);
    }

    return cvx_hull;

}

/**
 * @brief Compute the plane given a set of three points
 * @param points
 * @return the vector normal to the plane
 */
Vector3d normal_to_plane (vector<Vector3d> points){

    // Let's take the first three points in the vector of all the contact points between the robot and the floor
    // The a check is implemented to verify that the forth point is part of the plane
    /* How to compute a plane from three points:
     *
     * Compute the vectors that goes from point p0 to point p1 and from point p0 to point p2.
     * These vectors will lay on the same plane, and their cross product will be normal to the vector and also to the plane identified by these vectors.
     *  v = p1 - p0
     *  w = p2 - p0
     *  n = v x w
     * The equation of a plane is written as ax + by + cz + d = 0, where a,b,c are the coefficients of the normal n to the plane.
     * The equation and d can be obtained substituting one of the points that generate the plane and the normal coefficient into the equation:
     *  a(x - x_p) + b(y - y_p) + c(z - z_p) = 0
    */

    Vector3d v = points[1] - points[0]; // v = p1 - p0
    Vector3d w = points[2] - points[0]; // w = p2 - p0
    Vector3d n = v.cross(w);    //n = v x w
    n.normalize();
    d = -n.dot(points[0]);

    if (points.size() > 3){
        // Check if the forth point is part of the plane. The cast to int is done in order to round the number
        if (std::round(n.dot(points[3]) + d) != 0)
            cout << "Point: \n" << points[3] << endl << "is not part of the plane\n";
    }

    return n;

}

/**
 * @brief projection will return the coordinates of the point p projected on the plane defined by the normal n
 * @param n normal of the plane on which project the point p
 * @param p point to be projected
 * @return coordinates of the projected point
 */
Vector3d projection(Vector3d n, Vector3d p){

/*
 * Consider an hyperplane defined as H = {z ∈ R^n, a'z = b}, where a is the normal of the plane and b is
 * the scalar, while z is a point that is on the plane. Now the projection of a point onto this hyperplane
 * is defined by the formula:
 *
 *                              p* = p - [(a'p - b)/(||a||^2)] * a
 *
 * where ||a||^2 is the square of 2-norm, computed as (x)² + (y)² + (z)²
*/

    double alpha = (n.dot(p)+d)/(pow(n.x(),2) + pow(n.y(),2) + pow(n.z(),2));
    Vector3d prj_p = p - alpha*n;
    return prj_p;
}

/**
 * @brief check_contact_points is a function that check the position of the contact point w.r.t. the world.
 *
 * If a leg is lifted up, the z-coordinate w.r.t. the world will positive and different from zero. In this
 * condition, that leg, and in particular that point is not used in the definition of the support polygon.
 * The check is done with robustness, since the value of the z-coordinate is rounded to 2 decimal after the
 * dot, so only significant chance on position will cause an elimination of the point.
 *
 * @param contact_points will contain the contact points that will be the vertex of the support polygon
 * @param model
 */
void check_contact_points(vector<Vector3d>& contact_points, ModelInterface::Ptr model){

    for (const string& con : {"contact_1", "contact_2", "contact_4", "contact_3"}){
        Affine3d con_wrt_world;
        Affine3d contact_point_pose;

        if (model->getPose(con, "base_link", contact_point_pose) == false ||
            model->getPose(con, con_wrt_world) == false)
            throw std::runtime_error("getPose() return False");

        double rounded_number = std::round(con_wrt_world.translation().z() * 10.0) / 10.0;
        // Check to exclude a point if it is lifted w.r.t. the world frame
        if (rounded_number == 0)
            contact_points.push_back(contact_point_pose.translation());
        else{
            std::string message = string("Frame ") + con + string(" is raised from the ground of ") + to_string(con_wrt_world.translation().z());
            cout << message << endl;
        }
    }

    if (contact_points.size() < 3){
        string exception = string("Impossible to compute the support polygon, number of contact point equal to ") + to_string(contact_points.size());
        throw std::runtime_error(exception);
    }
}

/*
========================================================================================================
                                                MAIN
========================================================================================================
*/
int main (int argc, char **argv){

    // ROS node initialization
    ros::init(argc, argv, "stability_node");

    // Defining all the needed configuration parameter for the robot
    ros::NodeHandle n("/xbotcore");

    BaseStateGetter base_getter(n, "pelvis");

    XBot::ConfigOptions xbot_cfg = XBot::ConfigOptionsFromParamServer(n);
    auto model = XBot::ModelInterface::getModel(xbot_cfg); //Creation of the model class

    XBot::RobotInterface::Ptr robot;

    try
    {
        robot = XBot::RobotInterface::getRobot(xbot_cfg);
        model->syncFrom(*robot);
        robot->setControlMode(XBot::ControlMode::Position());
        robot->setControlMode(
            {
                {"j_wheel_1", XBot::ControlMode::Velocity()},
                {"j_wheel_2", XBot::ControlMode::Velocity()},
                {"j_wheel_3", XBot::ControlMode::Velocity()},
                {"j_wheel_4", XBot::ControlMode::Velocity()}
            });
    }
    catch (...)
    {
        // Setting homing postion of the robot
        Eigen::VectorXd qhome;
        model->getRobotState("home", qhome);
        model->setJointPosition(qhome);
        model->update();
    }

    const double dt = 0.01; // sampling time

    auto pub_com = n.advertise<geometry_msgs::PointStamped>("/com_point", 1);
    auto pub_poly = n.advertise<geometry_msgs::PolygonStamped>("/supp_poly", 1);
    auto pub_com_prj = n.advertise<geometry_msgs::PointStamped>("/com_prj_point", 1);
    auto pub_zmp = n.advertise<geometry_msgs::PointStamped>("/zmp_point", 1);
    auto pub_acc_com = n.advertise<geometry_msgs::PointStamped>("/com_acc", 1);
    auto pub_vel = n.advertise<geometry_msgs::PointStamped>("/com_vel", 1);
    auto pub_vel_filtered = n.advertise<geometry_msgs::PointStamped>("/com_vel_filtered", 1);

    VectorXd joint_position;
    VectorXd joint_velocity;
    robot->getJointPosition(joint_position);
    robot->getJointVelocity(joint_velocity);

    double n_joints = joint_position.rows();
    VectorXd full_q = VectorXd::Zero(n_joints+6);   // Initialize full joint position vector (floating base joint + actuated joint)
    VectorXd full_v = VectorXd::Zero(n_joints+6);   // Initialize full joint velocity vector (floating base joint + actuated joint)

    Vector3d initial_velocity, final_velocity = Vector3d::Zero();

    SignProcUtils::MovAvrgFilt filter_vel(3, dt, 2.0);
    SignProcUtils::MovAvrgFilt filter_acc(3, dt, 2.0);

    while(true){

        auto start_time = std::chrono::high_resolution_clock::now();    // cycle starting time
        robot ->sense();
        ros::spinOnce();

        // Take states value
        robot->getJointPosition(joint_position);
        robot->getJointVelocity(joint_velocity);

        full_q << 0,0,0,0,0,0,
            joint_position; // unactuated + actuated joint value

        full_v << base_getter.getTwist(),
            joint_velocity; // unactuated + actuated twist value

        //cout << full_q << endl << "-----\n";

        // Update info
        model->setJointPosition(full_q);
        model->setJointVelocity(full_v);
        model->setFloatingBaseState(base_getter.getPose(), base_getter.getTwist());
        model->update();

        // COM
        Vector3d com_postion;
        model->getCOM("base_link", com_postion);
        pub_com.publish(COM_msg_generator(com_postion));

        vector<Vector3d> contact_points_pos;
        Vector3d com_prj;

        try {

            // Support polygon
            check_contact_points(contact_points_pos, model);
            pub_poly.publish(CVX_HULL_msg_generator(contact_points_pos));

            // COM projection on support polygon
            com_prj = projection(normal_to_plane(contact_points_pos), com_postion);
            pub_com_prj.publish(COM_msg_generator(com_prj));

        } catch (const std::exception& e) {

            std::cerr << "Error: " << e.what() << std::endl;

        }

        Vector3d com_acc, com_vel, com_vel_filtered, com_acc_filtered;

        // COM velocity
        model->getCOMVelocity(com_vel); // raw values
        pub_vel.publish(COM_msg_generator(com_vel));    // for debugging purpose

        VectorXd vect = com_vel.head(3); // casting the data in a VectorXd object needed for the filter function
        filter_vel.add_sample(vect);
        filter_vel.get(vect);  // filtering velocity of the CoM
        com_vel_filtered = vect.head(3);
        pub_vel_filtered.publish(COM_msg_generator(com_vel_filtered));

        // COM Acceleration from velocity (numerical differentiation)
        final_velocity = com_vel_filtered;
        com_acc = (final_velocity - initial_velocity)/dt;   // compute the acceleration as a = dv/dt

        VectorXd temp = com_acc.head(3);
        filter_acc.add_sample(temp);
        filter_acc.get(temp);   // filtering acceleration of the CoM
        com_acc_filtered = temp.head(3);
        pub_acc_com.publish(COM_msg_generator_acc(com_acc_filtered));
        initial_velocity = final_velocity;  // update the velocity for the next sample

        //ZMP or CoP
        Vector3d zmp = com_prj - ((abs(com_prj.z()))/(9.81))*com_acc;   // ZMP computation using basic formula related to the CoM acceleration
        pub_zmp.publish(COM_msg_generator(zmp));

        // Chronos
        auto current_time = std::chrono::high_resolution_clock::now();  //cycle ending time
        std::chrono::duration<double> cycle_duration = current_time - start_time;   // the initialization of this variable can be done outside the while loop
        std::chrono::duration<double> standard_duration(dt);    // the initialization of this variable can be done outside the while loop

        if (standard_duration.count() - cycle_duration.count() > 0 )
            std::this_thread::sleep_for(standard_duration - cycle_duration);

    }

    return 0;

}

