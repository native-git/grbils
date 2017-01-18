#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


// This code is designed to provide a way for programmers who are comfortable with C++ to access the position information from the GrBILS system


int main(int argc, char** argv){
  ros::init(argc, argv, "position_finder_cpp");

    ros::NodeHandle node;

  tf::TransformListener listener;

    ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/hd_cam_new",  
                               ros::Time(0), transform);
        }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    float x, y, z;

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    z = transform.getOrigin().z();

    // If you would like to work with quaternions instead of euler angles, access the values in the following way:
    
    /*
    float, q_x, q_y, q_z, q_w;
    q_x = transform.getRotation().x();
    q_y = transform.getRotation().y();
    q_z = transform.getRotation().z();
    q_w = transform.getRotation().w();
    */

    tf::Matrix3x3 m(transform.getRotation());

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); //Calculate roll, pitch, and yaw from quaternion

    // Note that units for translational components are meters, and rotational components are radians

    fprintf(stdout, "TRANSLATIONAL COMPONENTS\n");
    fprintf(stdout, "X: %f\n", x);
    fprintf(stdout, "Y: %f\n", y);
    fprintf(stdout, "Z: %f\n", z);
    fprintf(stdout, "\n");
    fprintf(stdout, "ROTATIONAL COMPONENTS\n");
    fprintf(stdout, "ROLL: %f\n", roll);
    fprintf(stdout, "PITCH: %f\n", pitch);
    fprintf(stdout, "YAW: %f\n", yaw);
    fprintf(stdout, "------------------------------\n");

        rate.sleep();
  }
  return 0;
};