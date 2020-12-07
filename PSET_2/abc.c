estimate = initial_state
for measurement in all_data_to_be_filtered
	prediction = estimate + (change_rate_of_state * time_step)

	residual = measurement - prediction
	change_rate_of_state = change_rate_of_state + h * (residual) / time_step
	estimate = prediction + g * residual


acc_est(1,:) = acc_init
gyro_est(1,:) = gyro_init

k = 2;
for i in acc:
	acc_pred = acc_est(k-1,:) + (change_rate*time_step)
	change_rate = change_rate

	residual = i - acc_pred
	change_rate = change_rate + h * (residual) / time_step
	acc_est(k,:) = acc_pred + g * residual
	k = k+1
end

k = 2;
for i in acc:
	gyro_pred = gyro_est(k-1,:) + (change_rate*time_step)
	change_rate = change_rate

	residual = i - gyro_pred
	change_rate = change_rate + h * (residual) / time_step
	gyro_est(k,:) = gyro_pred + g * residual
	k = k+1
end


eulerd (acc_est, gyro_est)



//8.3------------------------------------------------------
/*
*  tiny_ekf_struct.h
*/

typedef struct {
	int number_of_state_values
	int number_of_observables

	state_vector

	prediction_error_covariance
	process_noise_covariance
	measurement_error_covariance

	Kalman gain

	transpose of measurement Jacobian
    transpose of process Jacobian
    post-prediction, pre-update
    
    output of user defined state-transition function
    output of user defined measurement function

    temporary_storage
}ekf_t;

/*
* TinyEKF.h
*/
#include <stdio.h>
#include <stdlib.h>
#include "tiny_ekf_struct.h"

initial ekf_init and ekf_step function

//define A header-only class for the Extended Kalman Filter. 

class TinyEKF {
	private ekf_t ekf;
	protected
		the current_state
		initializes a TinyEKF object
		TinyEKF(){
			ekf_init(ekf, Nsta, Mobs)
			current_state
		}
		Deallocates memory for a TinyEKF object.

		set the parameters for model function
			@param get output of state-transition function
			@param get the number of state values and jacobian of state-transition function
			@param get output of observation function
			@param get number of measurement and jacobian of observation function

		setP function sets the specified value of the prediction error covariance
		setQ function sets the specified value of the process noise covariance
		setR function sets the specified value of the observation noise covariance

	public
		getX function returns the state element at a given index
		setX function sets the state element at a given index
		boolean setp function to perform one step of the prediction and pre-update
			assign value into the model function parameters
			return true if parameter of observation vector assign successfully
			return false on failure caused by non-positive-definite matrix.
}


/* 
* IMU_GPS_Fusion.ino
*/


#define Nsta 9 (accelerometer, gyroscope, GPS position)
#define Mobs 9 (9 measurements: 3 acc, 3 gyro, 3 gps_position)

#include <TinyEKF.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include (any necessary libiaries)

class Fuser : public TinyEKF
	public:
		Fuser(){
			set all process noise and measurement noise to be .0001
		}

	protected:
		void model(){
			process model fx[0 to 8] = x[0 to 8]
			process model Jacobian is identity matrix
			F[0][0] to F[8][8] = 1


			accelerometer measurement from previous state
			hx[0-2] = accelerometer previous state
			gyroscope measurement from previous state
			hx[3-5] = gyroscope previous state
			gps_position measurement from previous state
			hx[6-8] = gps position measurement from previous state

			Jacobian of measurement function
			assign all jacobian of measurement model to be 1
			H[0][0] to H[8][0] = 1;       
		}

void setup(){
	begin all
	start reading from sensors, acc, gyro, and gps position
}

void loop(){

	get reading from accelerometer. gyroscope and gps position
	double z[9] = {acc, gyro, gps}
	ekf.step(z)

	Report measured and predicte/fused values

}

functions to get reading from accelerometer, gyroscope, and gps_position