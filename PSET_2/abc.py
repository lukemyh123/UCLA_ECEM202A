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


eulerd (fuse(acc_est, gyro_est))

