#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <tf/transform_broadcaster.h>
#include <golfi/ukf_states.h>
#include <geometry_msgs/Twist.h>

using namespace Eigen;

geometry_msgs::Twist vel;
sensor_msgs::Imu imu;
nav_msgs::Odometry utm;
golfi::ukf_states golfi_msg;

int N = 5.; // number of states
int N_sigma = 2 * N + 1; // number of sigma points
int kappa = 3 - N;
MatrixXd X(5,1); // state variables: x, y, vx, vy, yaw
VectorXd u(3); // input variables: ax, ay, vyaw
MatrixXd Q(5,5); // process noise covariance
double y_tach = 0; // to store measurement v from tachometer
double y_imu = 0;
double P_tach = 0;
double P_imu = 0;
double vyaw, yaw;
VectorXd y_tach_estimate(N_sigma);
VectorXd y_imu_estimate(N_sigma);
VectorXd y_gnss(2); //to store measurement x and y from GNSS
MatrixXd y_gnss_estimate(2,N_sigma);
MatrixXd P_gnss(2,2);
double R_tach = 10.0; // measurement noise covariance for tachometer
double R_imu = 5;
MatrixXd R_gnss(2,2); // measurement noise covariance for GNSS
MatrixXd P(5,5); // state covariance
MatrixXd sigma_points(5,11); // to store sigma points
double dt;
MatrixXd F(5,5); // motion model
MatrixXd u_mat(5,3); // input matrix in motion model
VectorXd weights(N_sigma);
VectorXd Pxy_tach(5);
VectorXd Pxy_imu(5);
MatrixXd Pxy_gnss(5,2);
VectorXd K_tach(5);
VectorXd K_imu(5);
MatrixXd K_gnss(5,2);
VectorXd utm_gnss(2);
int i; // for iteration


// BUAT DAPETIN PARAMETER-PARAMETER
// double int_var;
// double double_var;
// std::string string_var;
// ros::param::get("/my_integer", int_var);
// ros::param::get("/my_float", double_var);
// ros::param::get("/my_string", string_var);
// ROS_INFO("Int: %d, Float: %lf, String: %s", int_var, double_var, string_var.c_str());

void tachCallback(const geometry_msgs::Twist &vel_msg) {
    // vel = vel_msg;
    // // vel = vel_msg.linear.x;
    // LLT<MatrixXd> lltOfA(P); // compute the Cholesky decomposition of P
    // MatrixXd L = lltOfA.matrixL();
    // ROS_INFO("7");
    // // Redrawn sigma points
    // sigma_points.col(0) = X;
    // ROS_INFO("Z");
    // for (i = 1; i <= N; i++) {
    //   sigma_points.col(i) = X + sqrt(N + kappa) * L.col(i-1);
    //   sigma_points.col(i + N) = X - sqrt(N + kappa) * L.col(i-1);
    // }
    // ROS_INFO("A");
    // y_tach_estimate.setZero();
    // for(i = 0; i < N_sigma; i++) {
    //   y_tach_estimate[i] = sqrt(pow(sigma_points.col(i)[2], 2) + pow(sigma_points.col(i)[3], 2));
    // }
    // y_tach = 0;
    // ROS_INFO("B");
    // for(i = 0; i < N_sigma; i++) {
    //   y_tach += weights[i] * y_tach_estimate[i];
    // }
    // P_tach = 0;
    // for(i = 0; i < N_sigma; i++) {
    //   P_tach += weights[i] * (y_tach_estimate[i] - y_tach) * (y_tach_estimate[i] - y_tach);
    // }
    // P_tach += R_tach;
    // Pxy_tach.setZero();
    // for(i = 0; i < N_sigma; i++) {
    //   Pxy_tach += weights[i] * (sigma_points.col(i) - X) * (y_tach_estimate[i] - y_tach);
    // }
    // K_tach = Pxy_tach / P_tach;
    // X = X + K_tach * (vel.linear.x - y_tach);
    // P = P - K_tach * P_tach * K_tach.transpose();
    // ROS_INFO("X KEUBAHHHH TACHO");
    // std::cout << "X = " << X << std::endl;
}

void imuCallback(const sensor_msgs::Imu &imu_msg) {
    imu = imu_msg;
    yaw = tf::getYaw(imu.orientation);
    LLT<MatrixXd> lltOfA(P); // compute the Cholesky decomposition of P
    MatrixXd L = lltOfA.matrixL();
    ROS_INFO("39");
    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = X - sqrt(N + kappa) * L.col(i-1);
    }
    ROS_INFO("40");
    y_imu_estimate.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_imu_estimate[i] = sigma_points.col(i)[4];
    }
    y_imu = 0;
    ROS_INFO("41");
    for(i = 0; i < N_sigma; i++) {
      y_imu += weights[i] * y_imu_estimate[i];
    }
    P_imu = 0;
    for(i = 0; i < N_sigma; i++) {
      P_imu += weights[i] * (y_imu_estimate[i] - y_imu) * (y_imu_estimate[i] - y_imu);
    }
    P_imu += R_imu;
    Pxy_imu.setZero();
    ROS_INFO("42");
    for(i = 0; i < N_sigma; i++) {
      Pxy_imu += weights[i] * (sigma_points.col(i) - X) * (y_imu_estimate[i] - y_imu);
    }
    K_imu = Pxy_imu / P_imu;
    X = X + K_imu * (yaw - y_imu);
    P = P - K_imu * P_imu * K_imu.transpose();
    // std::cout << "yaw = " << yaw << std::endl;
    // std::cout << "X = " << X << std::endl;
    // X(4,0) = yaw;
    ROS_INFO("X KEUBAHHHH IMU");
    std::cout << "X = " << X << std::endl;
}

void gnssCallback(const nav_msgs::Odometry &utm_msg) {
    utm = utm_msg;
    ROS_INFO("GNSS received is already set to true.");
    LLT<MatrixXd> lltOfA(P); // compute the Cholesky decomposition of P
    MatrixXd L = lltOfA.matrixL();
    ROS_INFO("39");
    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = X - sqrt(N + kappa) * L.col(i-1);
    }
    ROS_INFO("40");
    y_gnss_estimate.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_gnss_estimate.col(i)[0] = sigma_points.col(i)[0];
      y_gnss_estimate.col(i)[1] = sigma_points.col(i)[1];
    }
    y_gnss.setZero();
    ROS_INFO("41");
    for(i = 0; i < N_sigma; i++) {
      y_gnss += weights[i] * y_gnss_estimate.col(i);
    }
    P_gnss.setZero();
    for(i = 0; i < N_sigma; i++) {
      P_gnss += weights[i] * (y_gnss_estimate.col(i) - y_gnss) * (y_gnss_estimate.col(i) - y_gnss).transpose();
    }
    P_gnss += R_gnss;
    Pxy_gnss.setZero();
    ROS_INFO("42");
    for(i = 0; i < N_sigma; i++) {
      Pxy_gnss += weights[i] * (sigma_points.col(i) - X) * (y_gnss_estimate.col(i) - y_gnss).transpose();
    }
    K_gnss = Pxy_gnss * P_gnss.inverse();
    utm_gnss << utm.pose.pose.position.x, utm.pose.pose.position.y;
    X = X + K_gnss * (utm_gnss - y_gnss);
    P = P - K_gnss * P_gnss * K_gnss.transpose();
    ROS_INFO("X KEUBAHHHH GNSS");
    std::cout << "X = " << X << std::endl;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "ukf_ule");
  ros::NodeHandle n;
  // ros::Subscriber tach_sub = n.subscribe("/sensor_velocity", 10, tachCallback);
  ros::Subscriber imu_sub = n.subscribe("/imu", 10, imuCallback);
  ros::Subscriber gnss_sub = n.subscribe("/utm", 10, gnssCallback);
  ros::Publisher ukf_pub = n.advertise<golfi::ukf_states>("ukf_states", 10);
  ros::Rate loop_rate(50);

  Q << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 5, 0, 0,
      0, 0, 0, 5, 0,
      0, 0, 0, 0, 1.5;
  R_gnss << 0.0001, 0,
            0, 0.0001;

  bool initialized = false;
  ros::Time current_time;
  ros::Time last_time;
  last_time = ros::Time::now();

  while(ros::ok()) {
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    //********** INITIALIZATION **********//

    if(!initialized && (utm.header.seq > 0) && (imu.header.seq > 0)) {
      // while (!GNSS_received) {
      //   ROS_INFO("Waiting for GNSS data to set the initial location");
      // }
      // ROS_INFO("Already received the first GNSS data. Continuing to estimate the location..");
      // X << utm.pose.pose.position.x,
      //     utm.pose.pose.position.y,
      //     vel.linear.x * cos (yaw),
      //     vel.linear.x * sin (yaw),
      //     yaw;

      X << utm.pose.pose.position.x,
          utm.pose.pose.position.y,
          1,
          1,
          yaw;
      std::cout << "X = " << X << std::endl;

      P << 0.0005, 0, 0, 0, 0,
          0, 0.0005, 0, 0, 0,
          0, 0, 5, 0, 0,
          0, 0, 0, 5, 0,
          0, 0, 0, 0, 0.8;
      initialized = true;
      ROS_INFO("DAH INITIALIZED.");
    }

    else if (initialized) {
      //********** PREDICTION **********//

      F << 1, 0, dt, 0, 0,
          0, 1, 0, dt, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
      std::cout << "F = " << F << std::endl;
      u_mat << pow(dt, 2) / 2., 0., 0.,
              0., pow(dt, 2) / 2., 0.,
              dt, 0., 0.,
              0, dt, 0.,
              0, 0., dt;
      std::cout << "u_mat = " << u_mat << std::endl;
      std::cout << "P = " << P << std::endl;
      // Compute Cholesky decomposition of matrix P
      ROS_INFO("1");
      LLT<MatrixXd> lltOfA(P);
      MatrixXd L = lltOfA.matrixL();
      std::cout << "L = " << L << std::endl;

      // Compute sigma points
      ROS_INFO("2");
      sigma_points.col(0) = X;
      for (i = 1; i < N; i++) {
        sigma_points.col(i) = X + sqrt(N + kappa) * L.col(i);
        sigma_points.col(i + N) = X - sqrt(N + kappa) * L.col(i);
      }
      std::cout << "sigma points = " << sigma_points << std::endl;
      ROS_INFO("3");

      // Propagate sigma points
      u << imu.linear_acceleration.x, imu.linear_acceleration.y, imu.angular_velocity.z;
      // u << 0.1, 0.1, imu.angular_velocity.z;
      for(i = 0; i < N_sigma; i++) {
        sigma_points.col(i) = F * sigma_points.col(i) + u_mat * u;
      }
      std::cout << "sigma points = " << sigma_points << std::endl;
      weights[0] = (float) kappa / (N + kappa);
      // weights[0] = -weights[0];
      for (i = 1; i < N_sigma; i++) {
        weights[i] = (float) 1 / (2 * (N + kappa));
      }
      std::cout << "weights = " << weights << std::endl;
      ROS_INFO("4");
      // Compute predicted mean and covariance
      X.setZero();
      for (i = 0; i < N_sigma; i++) {
        X = X + weights[i] * sigma_points.col(i);
        ROS_INFO("X KEUBAHHHH PREDICT");
      }
      std::cout << "X = " << X << std::endl;
      P.setZero();
      for(i = 0; i < N_sigma; i++) {
        P += weights[i] * (sigma_points.col(i) - X) * (sigma_points.col(i) - X).transpose();
      }
      P += Q;
      ROS_INFO("5");
    }
    ROS_INFO("14");
    golfi_msg.x = X(0,0);
    golfi_msg.y = X(1,0);
    golfi_msg.vx = X(2,0);
    golfi_msg.vy = X(3,0);
    golfi_msg.yaw = X(4,0);
    golfi_msg.init = initialized;
    ukf_pub.publish(golfi_msg);
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO("15");
    last_time = current_time;
  }
  return 0;
}
