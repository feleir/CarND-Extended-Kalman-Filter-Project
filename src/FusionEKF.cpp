#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

 // Initialize covariance matrix.
 ekf_.P_ = MatrixXd(4, 4);
 ekf_.P_ << 1, 0, 0 , 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  // Initialize H matrix for laser measurements.
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "EKF initialization: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "EKF: RADAR measurement" << endl;
      // Range measurement
      double rho = measurement_pack.raw_measurements_[0];
      // Bearing
  	  double phi = measurement_pack.raw_measurements_[1];
      // Velocity
  	  double rho_dot = measurement_pack.raw_measurements_[2];
  	  // Coordinates convertion from polar to cartesian
  	  double x = max(rho * cos(phi), 0.000001);
  	  double y = max(rho * sin(phi), 0.0001);
      double vx = rho_dot * cos(phi);
  	  double vy = rho_dot * sin(phi);
      // Initialize state based on x,y, vx, vy
      ekf_.x_ << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "EKF: LASER measurement" << endl;
      // Measurement coordinates are cartesian.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Initialize timestamp value in seconds.
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Calculate timestamp difference between the previous and actual.
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update state transition matrix.
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // Update the process noise covariance matrix.
  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
  float dt_3_2 = dt_3/2;
	float dt_4 = dt_3 * dt;
  float dt_4_4 = dt_4/4;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4_4* noise_ax, 0, dt_3_2 * noise_ax, 0,
              0, dt_4_4* noise_ay, 0, dt_3_2 * noise_ay,
              dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;

  // Kalman filter predict.
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Update state and covariance matrix depending on the sensor type.
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    cout << "EKF: Radar update" << endl;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    cout << "EKF: Laser update" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;  
    ekf_.Update(measurement_pack.raw_measurements_); 
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}