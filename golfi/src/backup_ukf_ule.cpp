#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <golfi/ukf_states.h>
#include <geometry_msgs/Twist.h>
#include <fstream>

using namespace Eigen;
using namespace std;

geometry_msgs::Twist vel;
sensor_msgs::Imu imu;
nav_msgs::Odometry utm;
golfi::ukf_states golfi_msg;

int N = 5.; // number of states
int N_sigma = 2 * N + 1; // number of sigma points
int kappa = 3 - N;

// Mat X(5,1); // state variables: x, y, vx, vy, yaw
typedef Matrix< long double, Dynamic, Dynamic > Mat;
typedef Matrix< long double, Dynamic, 1 > Vec;
Mat X(5,1);
Vec u(3); // input variables: ax, ay, vyaw
Mat Q(5,5); // process noise covariance
long double y_tach = 0; // to store measurement v from tachometer
long double y_imu = 0;
long double P_tach = 0;
long double P_imu = 0;
long double vyaw, yaw;
Vec y_tach_estimate(N_sigma);
Vec y_imu_estimate(N_sigma);
Vec y_gnss(2); //to store measurement x and y from GNSS
Mat y_gnss_estimate(2,N_sigma);
Mat P_gnss(2,2);
long double R_tach = 10.0; // measurement noise covariance for tachometer
long double R_imu = 5.0;
Mat R_gnss(2,2); // measurement noise covariance for GNSS
Mat P(5,5); // state covariance
Mat sigma_points(5,11); // to store sigma points
long double dt;
Mat F(5,5); // motion model
Mat u_mat(5,3); // input matrix in motion model
Vec weights(N_sigma);
Vec Pxy_tach(5);
Vec Pxy_imu(5);
Mat Pxy_gnss(5,2);
Vec K_tach(5);
Vec K_imu(5);
Mat K_gnss(5,2);
Vec utm_gnss(2);
int i, j; // for iteration
ofstream myfile;

void tachCallback(const geometry_msgs::Twist &vel_msg) {
    // vel = vel_msg;
    // // vel = vel_msg.linear.x;
    // LLT<Mat> lltOfA(P); // compute the Cholesky decomposition of P
    // Mat L = lltOfA.matrixL();
    // Mat L = P.llt().matrixL();
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
    // myfile << "X KEUBAHHHH TACHO\n";
    // std::cout << "X = " << setprecision(12) << X << std::endl;
}

void imuCallback(const sensor_msgs::Imu &imu_msg) {
    imu = imu_msg;
    yaw = tf::getYaw(imu.orientation);
    Mat L = P.llt().matrixL();
    myfile << "L =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N; j++) {
        myfile << "\t" << L(i,j);
      }
      myfile << endl;
    }
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
    myfile << "X KEUBAHHHH IMU\n";
    myfile << "X =";
    for (i = 0; i < N; i++) {
      myfile << "\t" << setprecision(12) << X(i,0) << endl;
    }
    myfile << "P =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N; j++) {
        myfile << "\t" << P(i,j);
      }
      myfile << endl;
    }
    std::cout << "X = " << setprecision(12) << X << std::endl;
}

void gnssCallback(const nav_msgs::Odometry &utm_msg) {
    utm = utm_msg;
    ROS_INFO("GNSS received is already set to true.");
    Mat L = P.llt().matrixL();
    myfile << "L =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N; j++) {
        myfile << "\t" << L(i,j);
      }
      myfile << endl;
    }
    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = X - sqrt(N + kappa) * L.col(i-1);
    }
    myfile << "Sigma points =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N_sigma; j++) {
        myfile << "\t" << setprecision(12) << sigma_points(i,j);
      }
      myfile << endl;
    }

    y_gnss_estimate.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_gnss_estimate.col(i)[0] = sigma_points.col(i)[0];
      y_gnss_estimate.col(i)[1] = sigma_points.col(i)[1];
    }
    myfile << "y_gnss estimate =";
    for(i = 0; i < 2; i++) {
      for(j = 0; j < N_sigma; j++) {
        myfile << "\t" << setprecision(12) << y_gnss_estimate(i,j);
      }
      myfile << endl;
    }
    y_gnss.setZero();
    ROS_INFO("41");
    for(i = 0; i < N_sigma; i++) {
      y_gnss = y_gnss + weights[i] * y_gnss_estimate.col(i);
    }
    myfile << "y_gnss =";
    for (i = 0; i < 2; i++) {
      myfile << "\t" << setprecision(12) << y_gnss[i] << endl;
    }
    P_gnss.setZero();
    for(i = 0; i < N_sigma; i++) {
      P_gnss = P_gnss + weights[i] * (y_gnss_estimate.col(i) - y_gnss) * (y_gnss_estimate.col(i) - y_gnss).transpose();
    }
    P_gnss += R_gnss;
    myfile << "P_gnss =";
    for(i = 0; i < 2; i++) {
      for(j = 0; j < 2; j++) {
        myfile << "\t" << setprecision(12) << P_gnss(i,j);
      }
      myfile << endl;
    }
    Pxy_gnss.setZero();
    ROS_INFO("42");
    for(i = 0; i < N_sigma; i++) {
      Pxy_gnss += weights[i] * (sigma_points.col(i) - X) * (y_gnss_estimate.col(i) - y_gnss).transpose();
    }
    myfile << "Pxy_gnss =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < 2; j++) {
        myfile << "\t" << setprecision(12) << Pxy_gnss(i,j);
      }
      myfile << endl;
    }
    K_gnss = Pxy_gnss * P_gnss.inverse();
    myfile << "K_gnss =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < 2; j++) {
        myfile << "\t" << setprecision(12) << K_gnss(i,j);
      }
      myfile << endl;
    }
    utm_gnss << utm.pose.pose.position.x, utm.pose.pose.position.y;
    myfile << "utm_gnss =";
    for (i = 0; i < 2; i++) {
      myfile << "\t" << setprecision(12) << utm_gnss[i] << endl;
    }
    X = X + K_gnss * (utm_gnss - y_gnss);
    P = P - K_gnss * P_gnss * K_gnss.transpose();
    ROS_INFO("X KEUBAHHHH GNSS");
    myfile << "X KEUBAHHHH GNSS\n";
    myfile << "X =";
    for (i = 0; i < N; i++) {
      myfile << "\t" << setprecision(12) << X(i,0) << endl;
    }
    myfile << "P =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N; j++) {
        myfile << "\t" << P(i,j);
      }
      myfile << endl;
    }
    std::cout << "X = " << setprecision(12) << X << std::endl;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "ukf_ule");
  ros::NodeHandle n;
  // ros::Subscriber tach_sub = n.subscribe("/sensor_velocity", 10, tachCallback);
  ros::Subscriber imu_sub = n.subscribe("/imu", 10, imuCallback);
  ros::Subscriber gnss_sub = n.subscribe("/utm", 10, gnssCallback);
  ros::Publisher ukf_pub = n.advertise<golfi::ukf_states>("ukf_states", 10);
  ros::Rate loop_rate(50);

  // n.getParam("kappa", kappa);
  // n.getParam("R_tach", R_tach);
  // n.getParam("R_imu", R_imu);

  Q << 5, 0, 0, 0, 0,
      0, 5, 0, 0, 0,
      0, 0, 5, 0, 0,
      0, 0, 0, 5, 0,
      0, 0, 0, 0, 1.5;
  R_gnss << 0.0001, 0,
            0, 0.0001;

  bool initialized = false;
  ros::Time current_time;
  ros::Time last_time;
  last_time = ros::Time::now();
  // ofstream myfile;
  myfile.open("myfile.txt");

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

      P << 1e-3, 0, 0, 0, 0,
          0, 1e-3, 0, 0, 0,
          0, 0, 0.5, 0, 0,
          0, 0, 0, 0.5, 0,
          0, 0, 0, 0, 0.8;
      initialized = true;
      ROS_INFO("DAH INITIALIZED.");
      myfile << "DAH INITIALIZED\n";
      myfile << "X =";
      for (i = 0; i < N; i++) {
        myfile << "\t" << X(i,0) << endl;
      }
      myfile << "P =";
      for(i = 0; i < N; i++) {
        for(j = 0; j < N; j++) {
          myfile << "\t" << P(i,j);
        }
        myfile << endl;
      }
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
      // LLT<Mat> lltOfA(P);
      // Mat L = lltOfA.matrixL();
      Mat L = P.llt().matrixL();
      std::cout << "L = " << L << std::endl;
      myfile << "L =";
      for(i = 0; i < N; i++) {
        for(j = 0; j < N; j++) {
          myfile << "\t" << L(i,j);
        }
        myfile << endl;
      }
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
      weights[0] = (long double) kappa / (N + kappa);
      // weights[0] = -weights[0];
      for (i = 1; i < N_sigma; i++) {
        weights[i] = (long double) 1 / (2 * (N + kappa));
      }
      std::cout << "weights = " << weights << std::endl;
      ROS_INFO("4");
      // Compute predicted mean and covariance
      X.setZero();
      for (i = 0; i < N_sigma; i++) {
        X = X + weights[i] * sigma_points.col(i);
        // ROS_INFO("X KEUBAHHHH PREDICT");
      }
      std::cout << "X = " << X << std::endl;
      P.setZero();
      for(i = 0; i < N_sigma; i++) {
        P += weights[i] * (sigma_points.col(i) - X) * (sigma_points.col(i) - X).transpose();
      }
      P += Q;
      ROS_INFO("5");
      myfile << "X KEUBAHHHH PREDICT\n";
      myfile << "X =";
      for (i = 0; i < N; i++) {
        myfile << "\t" << setprecision(12) << X(i,0) << endl;
      }
      myfile << "P =";
      for(i = 0; i < N; i++) {
        for(j = 0; j < N; j++) {
          myfile << "\t" << P(i,j);
        }
        myfile << endl;
      }
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
  myfile.close();
  return 0;
}
