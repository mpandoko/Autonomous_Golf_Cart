//udah bagus deh pokoknya, tapi measurement model tacho masih yang v = akar(vx^2 + vy^2)

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
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
#include <cmath>
#include <pkg_ta/LogArduino.h>

using namespace Eigen;
using namespace std;

sensor_msgs::Imu imu; // to store IMU data
nav_msgs::Odometry utm; //to store UTM data from GNSS
golfi::ukf_states golfi_msg; // custom message to store UKF state values
nav_msgs::Odometry odom; // published odometry message
float enc;

int N = 5.; // number of states
int N_sigma = 2 * N + 1; // number of sigma points
int kappa; // parameter to compute sigma points

typedef Matrix<long double, Dynamic, Dynamic> Mat;
typedef Matrix<long double, Dynamic, 1> Vec;
Mat X(5,1); // state variables
Vec u(3); // input variables: ax, ay, vyaw
Mat Q(5,5); // process noise covariance
long double y_tach = 0; // mean of predicted velocity measurements from tachometer
long double y_imu = 0; // mean of predicted yaw measurements from imu
long double y_enc = 0;
Vec y_gnss(2); // mean of predicted x and y measurements from GNSS
long double P_tach = 0; // covariance of predicted tachometer measurement
long double P_imu = 0; // covariance of predicted IMU measurement
long double P_enc = 0;
Mat P_gnss(2,2); // covariance of predicted GNSS measurement
long double vel; // magnitude velocity measurement from tachometer
long double yaw;  // yaw measurement from imu
long double yawprev;// yaw of previous measurement
Vec utm_gnss(2); // UTM measurement from GNSS
Vec y_tach_estimate(N_sigma); //predicted measurements from tachometer
Vec y_imu_estimate(N_sigma); // predicted measurements from IMU
Vec y_enc_estimate(N_sigma); // predicted measurement from Absolute Encoder
Mat y_gnss_estimate(2,N_sigma); // predicted x and y measurements from GNSS
double R_tach; // measurement noise covariance for tachometer
double R_imu; // measurement noise covariance for IMU
double R_enc; // measurement noise covariance for Absolute Encoder
Mat R_gnss(2,2); // measurement noise covariance for GNSS
Mat P(5,5); // state covariance
Mat sigma_points(5,11); // to store sigma points
long double dt; // delta time every 1 loop
Mat F(5,5); // motion model matrix
Mat u_mat(5,3); // input matrix in motion model
Vec weights(N_sigma);
Vec Pxy_tach(5); // cross covariance from predicted tachometer measurements
Vec Pxy_imu(5); // cross covariance from predicted IMU measurements
Vec Pxy_enc(5); // cross covariance from predicted Encoder measurement
Mat Pxy_gnss(5,2); // cross covariance from predicted GNSS measurements
Vec K_tach(5); // Kalman gain for tachometer
Vec K_imu(5); // Kalman gain for IMU
Vec K_enc(5); // Kalman gain for Absolute Encoder
Mat K_gnss(5,2); // Kalman gain for GNSS
Mat rotation_mat(2,2); // to transform between world/map frame and imu frame
Vec u_accel(2); // temporary variable
Vec u_accel_temp(2);
long double x_init, y_init, yaw_init; // to store the first x and y measurement from GNSS as the initial point
int i, j; // for iteration
ofstream myfile; // for the purpose of debugging. File "myfile.txt" can be found at .ros directory that's hidden in home
long double pi = 3.14159265359;
long double yaw_odom;
long double yaw_gnss = 0;
long double x_gnss_old, y_gnss_old, x_gnss_new, y_gnss_new, vx_gnss, vy_gnss, v_gnss;
long double dy,dx;
long double v_tach_old = 0;
long double v_tach_new;
bool first_tach = false;

void setProcessNoiseCovarianceQ(const Mat &get_Q) {
  // sets the process noise covariance Q (used when getting Q from ROS param server such as yaml file)
  Q = get_Q;
}

void setRgnss (const Mat &get_R_gnss) {
  // sets the measurement noise covariance for GNSS (used when getting R_gnss from ROS param server such as yaml file)
  R_gnss = (Mat) get_R_gnss;
}

void setInitialCovarianceP (const Mat &get_P) {
  // sets the initial state covariance (used when getting P from ROS param server such as yaml file)
  P = get_P;
}

long double wrap_angle(long double yaw) {
  while (yaw > pi) {
    yaw -= 2 * pi;
  }
  while (yaw < -pi) {
    yaw += 2 * pi;
  }
  return (yaw);
}
void encoderCallback(const pkg_ta::LogArduino &enc_msg) {
    enc = enc_msg.steering_angle;

    // Compute Cholesky decomposition of matrix P
    Mat L = P.llt().matrixL();

    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
    }
    myfile << "Sigma points =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N_sigma; j++) {
        myfile << "\t" << setprecision(12) << sigma_points(i,j);
      }
      myfile << endl;
    }

    // predict measurement(s)
    y_enc_estimate.setZero();
  
    for(i = 0; i < N_sigma; i++) {
      y_enc_estimate[i] = ((sigma_points.col(i)[4] - yawprev)/ dt);
    }
    myfile << "y_enc_estimate =";
    for (i = 0; i < N_sigma; i++) {
      myfile << "\t" << setprecision(12) << y_enc_estimate[i] << endl;
    }

    // calculate the mean of predicted measurements
    y_enc = 0;
    for(i = 0; i < N_sigma; i++) {
      y_enc += (long double) weights[i] * y_enc_estimate[i];
    }
    myfile << "y_enc = " << setprecision(12) << y_enc << endl;

    // calculate the covariance of predicted measurements
    P_enc = 0;
    for(i = 0; i < N_sigma; i++) {
      P_enc += (long double) weights[i] * (y_enc_estimate[i] - y_enc) * (y_enc_estimate[i] - y_enc);
    }
    P_enc += R_enc;
    myfile << "P_enc = " << setprecision(12) << P_enc << endl;

    // calculate the cross-covariance of predicted measurements
    Pxy_enc.setZero();
    for(i = 0; i < N_sigma; i++) {
      Pxy_enc += (Vec) ((long double) weights[i] * (sigma_points.col(i) - X) * (y_enc_estimate[i] - y_enc));
    }
    myfile << "Pxy_enc =";
    for(i = 0; i < N; i++) {
      myfile << "\t" << setprecision(12) << Pxy_enc[i] << endl;
    }

    // calculate the Kalman gain of predicted measurements
    K_enc = (Vec) Pxy_enc / P_enc;
    myfile << "K_enc =";
    for(i = 0; i < N; i++) {
      myfile << "\t" << setprecision(12) << K_enc[i] << endl;
    }

    // update states
    X = (Mat) X + K_enc * (((tan(enc)*X(2,0))/2.945) - y_enc);

    // update state covariances
    P = (Mat) P - K_enc * P_enc * K_enc.transpose();

    // std::cout << "yaw = " << yaw << std::endl;
    // std::cout << "X = " << X << std::endl;
    // X(4,0) = yaw;

    ROS_INFO("X KEUBAHHHH ENCODER");
    // write to external file for debugging
    myfile << "X KEUBAHHHH ENCODER\n";
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

void tachCallback(const geometry_msgs::Twist &tach_msg) {
  vel = tach_msg.linear.x;
  if (vel <= 4) {
    myfile << "vel = " << vel << endl;

    // Compute Cholesky decomposition of matrix P
    Mat L = P.llt().matrixL();

    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
    }
    myfile << "Sigma points =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N_sigma; j++) {
        myfile << "\t" << setprecision(12) << sigma_points(i,j);
      }
      myfile << endl;
    }

    // predict measurement(s)
    y_tach_estimate.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_tach_estimate[i] = sqrt(pow(sigma_points.col(i)[2],2) + pow(sigma_points.col(i)[3],2));
      // y_tach_estimate[i] = (sigma_points.col(i)[2] / cos(sigma_points.col(i)[4]) + sigma_points.col(i)[3] / sin(sigma_points.col(i)[4])) / 2;
    }
    myfile << "y_tach_estimate =";
    for (i = 0; i < N_sigma; i++) {
      myfile << "\t" << setprecision(12) << y_tach_estimate[i] << endl;
    }

    // calculate the mean of predicted measurements
    y_tach = 0;
    j = 0;
    for(i = 0; i < N_sigma; i++) {
      if(abs(y_tach_estimate[i]) <= 6) {
          j++;
          y_tach += y_tach_estimate[i];
      }
      // y_tach += (long double) weights[i] * y_tach_estimate[i];
    }
    if (j == 0) {
      y_tach = 2;
    }
    else {
      y_tach = (long double) y_tach / j;
    }
    myfile << "y_tach = " << setprecision(12) << y_tach << endl;

    // calculate the covariance of predicted measurements
    P_tach = 0;
    for(i = 0; i < N_sigma; i++) {
      P_tach += (long double) weights[i] * (y_tach_estimate[i] - y_tach) * (y_tach_estimate[i] - y_tach);
      // ini gadipake
      // if(abs(y_tach_estimate[i]) <= 10) {
      //   P_tach += (long double) (y_tach_estimate[i] - y_tach) * (y_tach_estimate[i] - y_tach);
      //   // P_tach += (long double) weights[i] * (y_tach_estimate[i] - y_tach) * (y_tach_estimate[i] - y_tach);
      // }
    }
    // P_tach = P_tach / j;
    P_tach += R_tach;
    myfile << "P_tach = " << setprecision(12) << P_tach << endl;

    // calculate the cross-covariance of predicted measurements
    Pxy_tach.setZero();
    for(i = 0; i < N_sigma; i++) {
      if(abs(y_tach_estimate[i]) <= 10) {
        Pxy_tach += (Vec) ((long double) weights[i] * (sigma_points.col(i) - X) * (y_tach_estimate[i] - y_tach));
      }
    }
    myfile << "Pxy_tach =";
    for(i = 0; i < N; i++) {
      myfile << "\t" << setprecision(12) << Pxy_tach[i] << endl;
    }

    // calculate the Kalman gain of predicted measurements
    K_tach = (Vec) Pxy_tach / P_tach;
    myfile << "K_tach =";
    for(i = 0; i < N; i++) {
      myfile << "\t" << setprecision(12) << K_tach[i] << endl;
    }

    // update states
    X = (Mat) X + K_tach * (vel - y_tach);

    // update state covariances
    P = (Mat) P - K_tach * P_tach * K_tach.transpose();

    ROS_INFO("X KEUBAHHHH TACHO");
    // write to external file for debugging
    myfile << "X KEUBAHHHH TACHO\n";
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
}

void imuCallback(const sensor_msgs::Imu &imu_msg) {
    imu = imu_msg;
    // Get yaw from quaternion measurement
    yaw = tf::getYaw(imu.orientation);
    // yaw = (long double) atan2(2 * (imu.orientation.w * imu.orientation.z + imu.orientation.x * imu.orientation.y), 1 - 2 * (imu.orientation.y*imu.orientation.y + imu.orientation.z*imu.orientation.z));
    myfile << "yaw = " << yaw << endl;

    // Compute Cholesky decomposition of matrix P
    Mat L = P.llt().matrixL();

    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
    }
    myfile << "Sigma points =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N_sigma; j++) {
        myfile << "\t" << setprecision(12) << sigma_points(i,j);
      }
      myfile << endl;
    }

    // predict measurement(s)
    y_imu_estimate.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_imu_estimate[i] = sigma_points.col(i)[4];
    }
    myfile << "y_imu_estimate =";
    for (i = 0; i < N_sigma; i++) {
      myfile << "\t" << setprecision(12) << y_imu_estimate[i] << endl;
    }

    // calculate the mean of predicted measurements
    y_imu = 0;
    for(i = 0; i < N_sigma; i++) {
      y_imu += (long double) weights[i] * y_imu_estimate[i];
    }
    myfile << "y_imu = " << setprecision(12) << y_imu << endl;

    // calculate the covariance of predicted measurements
    P_imu = 0;
    for(i = 0; i < N_sigma; i++) {
      P_imu += (long double) weights[i] * (y_imu_estimate[i] - y_imu) * (y_imu_estimate[i] - y_imu);
    }
    P_imu += R_imu;
    myfile << "P_imu = " << setprecision(12) << P_imu << endl;

    // calculate the cross-covariance of predicted measurements
    Pxy_imu.setZero();
    for(i = 0; i < N_sigma; i++) {
      Pxy_imu += (Vec) ((long double) weights[i] * (sigma_points.col(i) - X) * (y_imu_estimate[i] - y_imu));
    }
    myfile << "Pxy_imu =";
    for(i = 0; i < N; i++) {
      myfile << "\t" << setprecision(12) << Pxy_imu[i] << endl;
    }

    // calculate the Kalman gain of predicted measurements
    K_imu = (Vec) Pxy_imu / P_imu;
    myfile << "K_imu =";
    for(i = 0; i < N; i++) {
      myfile << "\t" << setprecision(12) << K_imu[i] << endl;
    }

    // update states
    X = (Mat) X + K_imu * (yaw - y_imu);

    // update state covariances
    P = (Mat) P - K_imu * P_imu * K_imu.transpose();

    // std::cout << "yaw = " << yaw << std::endl;
    // std::cout << "X = " << X << std::endl;
    // X(4,0) = yaw;

    ROS_INFO("X KEUBAHHHH IMU");
    // write to external file for debugging
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
    if(utm.header.seq > 0) {
      x_gnss_new = utm.pose.pose.position.x;
      y_gnss_new = utm.pose.pose.position.y;
      dx = x_gnss_new - x_gnss_old;
      dy = y_gnss_new - y_gnss_old;
      yaw_gnss = atan2(dy,dx);
      vx_gnss = dx / 0.2;
      vy_gnss = dy / 0.2;
      v_gnss = sqrt(vx_gnss * vx_gnss + vy_gnss * vy_gnss);
    }

    // Compute Cholesky decomposition of matrix P
    Mat L = P.llt().matrixL();

    // write to external file for debugging
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
      sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
    }
    myfile << "Sigma points =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < N_sigma; j++) {
        myfile << "\t" << setprecision(12) << sigma_points(i,j);
      }
      myfile << endl;
    }

    // predict measurement(s)
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

    // calculate the mean of predicted measurements
    y_gnss.setZero();
    ROS_INFO("41");
    for(i = 0; i < N_sigma; i++) {
      y_gnss = (Vec) y_gnss + weights[i] * y_gnss_estimate.col(i);
    }
    myfile << "y_gnss =";
    for (i = 0; i < 2; i++) {
      myfile << "\t" << setprecision(12) << y_gnss[i] << endl;
    }

    // calculate the covariance of predicted measurements
    P_gnss.setZero();
    for(i = 0; i < N_sigma; i++) {
      P_gnss = (Mat) P_gnss + weights[i] * (y_gnss_estimate.col(i) - y_gnss) * (y_gnss_estimate.col(i) - y_gnss).transpose();
    }
    P_gnss = P_gnss + R_gnss;
    myfile << "P_gnss =";
    for(i = 0; i < 2; i++) {
      for(j = 0; j < 2; j++) {
        myfile << "\t" << setprecision(12) << P_gnss(i,j);
      }
      myfile << endl;
    }

    // calculate the cross-covariance of predicted measurements
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

    // calculate the Kalman gain of predicted measurements
    K_gnss = (Mat) Pxy_gnss * P_gnss.inverse();
    myfile << "K_gnss =";
    for(i = 0; i < N; i++) {
      for(j = 0; j < 2; j++) {
        myfile << "\t" << setprecision(12) << K_gnss(i,j);
      }
      myfile << endl;
    }

    // assign the actual measurement value
    utm_gnss << utm.pose.pose.position.x, utm.pose.pose.position.y;
    myfile << "utm_gnss =";
    for (i = 0; i < 2; i++) {
      myfile << "\t" << setprecision(12) << utm_gnss[i] << endl;
    }

    // update the states
    X = (Mat) X + K_gnss * (utm_gnss - y_gnss);

    // update the state covariances
    P = (Mat) P - K_gnss * P_gnss * K_gnss.transpose();

    ROS_INFO("X KEUBAHHHH GNSS");
    // write to external file for debugging
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
    x_gnss_old = x_gnss_new;
    y_gnss_old = y_gnss_new;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "ukf_ule");
  ros::NodeHandle n;
  ros::Subscriber tach_sub = n.subscribe("/sensor_velocity", 10, tachCallback);
  ros::Subscriber imu_sub = n.subscribe("/imu", 10, imuCallback);
  ros::Subscriber gnss_sub = n.subscribe("/utm", 10, gnssCallback);
  ros::Subscriber encoder_sub = n.subscribe("/logging_arduino",10,encoderCallback);
  ros::Publisher ukf_pub = n.advertise<golfi::ukf_states>("ukf_states", 10);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
  ros::Rate loop_rate(50); // UKF frequency
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster broadcaster;

  // get parameters from ROS param server
  n.getParam("kappa", kappa);
  n.getParam("R_tach", R_tach);
  n.getParam("R_imu", R_imu);
  n.getParam("R_enc",R_enc);

  // get matrix parameters from ROS param server
  Mat get_R_gnss(2,2);
  get_R_gnss.setZero();
  XmlRpc::XmlRpcValue param_R_gnss;
  if (n.hasParam("R_gnss"))
  {
    try
    {
      n.getParam("R_gnss", param_R_gnss);

      ROS_ASSERT(param_R_gnss.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for (i = 0; i < 2; i++)
      {
        for (j = 0; j < 2; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << param_R_gnss[2 * i + j];
            std::istringstream istr(ostr.str());
            istr >> get_R_gnss(i, j);
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch(...) {}
    setRgnss(get_R_gnss);
  }

  Mat get_Q(5,5);
  get_Q.setZero();
  XmlRpc::XmlRpcValue param_Q;
  if (n.hasParam("process_noise_covariance_Q"))
  {
    try
    {
      n.getParam("process_noise_covariance_Q", param_Q);

      ROS_ASSERT(param_Q.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for (i = 0; i < 5; i++)
      {
        for (j = 0; j < 5; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << param_Q[5 * i + j];
            std::istringstream istr(ostr.str());
            istr >> get_Q(i, j);
          }
          catch(...) {}
        }
      }
    }
    catch(...) {
      throw;
    }
    setProcessNoiseCovarianceQ(get_Q);
  }

  bool initialized = false;
  ros::Time current_time;
  ros::Time last_time;
  last_time = ros::Time::now();
  myfile.open("myfile.txt"); //external file for debug

  // compute weights
  weights[0] = (long double) kappa / (N + kappa);
  for (i = 1; i < N_sigma; i++) {
    weights[i] = (long double) 1 / (2 * (N + kappa));
  }

  while(ros::ok()) {
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    //********** INITIALIZATION **********//

    if(!initialized && (utm.header.seq > 0) && (imu.header.seq > 0)) {
      x_init = utm.pose.pose.position.x;
      y_init = utm.pose.pose.position.y;
      yaw_init = yaw;
      X << x_init,
          y_init,
          0,
          0,
          yaw_init;
      yawprev = X(4,0);

      std::cout << "X = " << X << std::endl;

      // Get initial state covariance P from param server
      Mat get_P(5,5);
      get_P.setZero();
      XmlRpc::XmlRpcValue param_P;
      if (n.hasParam("initial_estimate_covariance_P"))
      {
        try
        {
          n.getParam("initial_estimate_covariance_P", param_P);

          ROS_ASSERT(param_P.getType() == XmlRpc::XmlRpcValue::TypeArray);

          for (i = 0; i < 5; i++)
          {
            for (j = 0; j < 5; j++)
            {
              try
              {
                // These matrices can cause problems if all the types
                // aren't specified with decimal points. Handle that
                // using string streams.
                std::ostringstream ostr;
                ostr << param_P[5 * i + j];
                std::istringstream istr(ostr.str());
                istr >> get_P(i, j);
              }
              catch(...) {}
            }
          }
        }
        catch(...) {
          throw;
        }
        setInitialCovarianceP(get_P);
        rotation_mat << (long double) cos(wrap_angle(yaw + pi/2)), (long double) -sin(wrap_angle(yaw + pi/2)),
                        (long double) sin(wrap_angle(yaw + pi/2)), (long double) cos(wrap_angle(yaw + pi/2));
        u_accel << imu.linear_acceleration.x, -imu.linear_acceleration.y;
        u_accel = rotation_mat * u_accel;
        u << u_accel[0], u_accel[1], -imu.angular_velocity.z;
      }

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
      odom.header.frame_id  = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = x_init;
      odom.pose.pose.position.y = y_init;
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(wrap_angle(yaw_init + pi / 2));
    }

    else if (initialized) {
      //********** PREDICTION **********//
      yawprev = X(4,0);

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
      sigma_points.col(0) = X;
      for (i = 1; i <= N; i++) {
        sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
        sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
      }
      

      //INI DIUNCOMMENT KALO MAU BALIK KAYAK YG BIASA
      // rotation_mat << (long double) cos(wrap_angle(yaw + pi/2)), (long double) -sin(wrap_angle(yaw + pi/2)),
      //                 (long double) sin(wrap_angle(yaw + pi/2)), (long double) cos(wrap_angle(yaw + pi/2));
      // u_accel << imu.linear_acceleration.x, -imu.linear_acceleration.y;
      // u_accel = rotation_mat * u_accel;
      // u << u_accel[0], u_accel[1], -imu.angular_velocity.z;

      // INI TTP DICOMMENT
      // u << 0, 0, imu.angular_velocity.z;
      // u << 0.1, 0.1, imu.angular_velocity.z;
      // myfile << "input u =";
      // for(i = 0; i < 3; i++) {
      //   myfile << "\t" << setprecision(12) << u[i] << endl;
      // }

      // u_accel << imu.linear_acceleration.x, imu.linear_acceleration.y;
      for(i = 0; i < N_sigma; i++) {
        // u_accel_temp.setZero();
        // rotation_mat << (long double) cos(wrap_angle(sigma_points.col(i)[4])), (long double) -sin(wrap_angle(sigma_points.col(i)[4])),
        //                 (long double) sin(wrap_angle(sigma_points.col(i)[4])), (long double) cos(wrap_angle(sigma_points.col(i)[4]));
        // u_accel_temp = rotation_mat * u_accel;
        // u << u_accel_temp[0], u_accel_temp[1], imu.angular_velocity.z;
        sigma_points.col(i) = (Mat) F * sigma_points.col(i) + u_mat * u;
      }

      // Compute predicted mean and covariance
      X.setZero();
      for (i = 0; i < N_sigma; i++) {
        X = (Mat) X + weights[i] * sigma_points.col(i);
      }
      std::cout << "X = " << X << std::endl;
      P.setZero();
      for(i = 0; i < N_sigma; i++) {
        P += weights[i] * (sigma_points.col(i) - X) * (sigma_points.col(i) - X).transpose();
      }
      P += Q;

      // write to external file for debugging
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
      ROS_INFO("X KEUBAHHHH PREDICT");
      std::cout << "X = " << setprecision(12) << X << std::endl;
    // }

    X(4,0) = wrap_angle(X(4,0));
    yaw_odom = wrap_angle(X(4,0) + pi/2);

    golfi_msg.stamp = ros::Time::now();
    golfi_msg.x = X(0,0);
    golfi_msg.y = X(1,0);
    golfi_msg.vx = X(2,0);
    golfi_msg.vy = X(3,0);
    golfi_msg.vx_gnss = vx_gnss;
    golfi_msg.vy_gnss = vy_gnss;
    golfi_msg.v_gnss = v_gnss;
    golfi_msg.v_tach = vel;
    golfi_msg.yaw_est = yaw_odom;
    golfi_msg.yaw_imu = yaw;
    golfi_msg.yaw_dydx = yaw_gnss;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.stamp  = current_time;
    odom_trans.transform.translation.x = X(0,0) - x_init;
    odom_trans.transform.translation.y = X(1,0) - y_init;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_odom);

    // odom.header.seq = 0;
    odom.header.frame_id  = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = X(0,0) - x_init;
    odom.pose.pose.position.y = X(1,0) - y_init;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_odom);
    odom.twist.twist.linear.x = X(2,0);
    odom.twist.twist.linear.y = X(3,0);
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = u[2];

    u_mat << pow(dt, 2) / 2., 0., 0.,
            0., pow(dt, 2) / 2., 0.,
            dt, 0., 0.,
            0, dt, 0.,
            0, 0., dt;
    rotation_mat << (long double) cos(wrap_angle(yaw + pi/2)), (long double) -sin(wrap_angle(yaw + pi/2)),
                    (long double) sin(wrap_angle(yaw + pi/2)), (long double) cos(wrap_angle(yaw + pi/2));
    u_accel << imu.linear_acceleration.x, -imu.linear_acceleration.y;
    u_accel = rotation_mat * u_accel;
    u << u_accel[0], u_accel[1], -imu.angular_velocity.z;
   
   
  }
    ukf_pub.publish(golfi_msg);
    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);
    ros::spinOnce();
    loop_rate.sleep();
    last_time = current_time;

  }
  myfile.close();
  return 0;
}
