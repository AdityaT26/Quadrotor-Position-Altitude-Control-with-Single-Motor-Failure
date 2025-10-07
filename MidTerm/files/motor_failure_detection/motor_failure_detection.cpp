#include <torch/script.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <poll.h>
#include <errno.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/event.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_odometry.h>

const int total_motors = 4;
const int minimum_iteration_threshold = 10;
const int maximum_iteration_threshold = 20;

extern "C" __EXPORT int motor_failure_detection_main(int argc, char *argv[]);

int motor_failure_detection_main(int argc, char *argv[]) {
	PX4_INFO("STARTING MOTOR FAILURE DETECTION");

	torch::jit::script::Module module;
	try {
		module = torch::jit::load("/home/ideaForge/PX4-Autopilot/src/modules/motor_failure_detection/model.pt");
	} catch (const std::exception& e) {
		std::cerr << "error loading the model\n";
		return -1;
	}
	std::cout << "Model loaded successfully.\n";

	struct event_s failed_motor;
	memset(&failed_motor, 0, sizeof(failed_motor));
	orb_advert_t failed_motor_publisher = orb_advertise(ORB_ID(event), &failed_motor);

	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	int odometry_sub_fd = orb_subscribe(ORB_ID(vehicle_odometry));
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = odometry_sub_fd,   .events = POLLIN },
	};
	std::vector<float> input_data(10);
	std::vector<torch::jit::IValue> inputs;
	at::Tensor output;
	std::vector<int> failed_number;
	int max_number_repeats, current_repeats, iterations = 0;
	uint8_t failed_motor_number = 0;
	while (true) {
		px4_poll(fds, 2, 1000);
		if (fds[0].revents & POLLIN) {
			struct sensor_combined_s raw;
			orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
			input_data[4] = (float) raw.gyro_rad[0];
			input_data[5] = (float) raw.gyro_rad[1];
			input_data[6] = (float) raw.gyro_rad[2];
			input_data[7] = (float) raw.accelerometer_m_s2[0];
			input_data[8] = (float) raw.accelerometer_m_s2[1];
			input_data[9] = (float) raw.accelerometer_m_s2[2];
		}
		if (fds[1].revents & POLLIN) {
			struct vehicle_odometry_s odometry_data;
			orb_copy(ORB_ID(vehicle_odometry), odometry_sub_fd, &odometry_data);
			input_data[0] = (float) odometry_data.q[0];
			input_data[1] = (float) odometry_data.q[1];
			input_data[2] = (float) odometry_data.q[2];
			input_data[3] = (float) odometry_data.q[3];
		}
		at::Tensor input_tensor = torch::tensor(input_data).view({1, 1, 10});
		inputs.push_back(input_tensor);
		output = module.forward(inputs).toTensor();
		at::Tensor thresholded_output = (output > 0.8).to(torch::kFloat);
		for(int motor_number = 0; motor_number < total_motors && iterations < maximum_iteration_threshold; motor_number++)
			if(thresholded_output[0][motor_number].item<int>() == 1) {
				failed_number.push_back(motor_number);
				iterations++;
		}
		if(iterations > minimum_iteration_threshold) {
			failed_motor_number = 0;
			max_number_repeats = 0;
			for(int motor_number = 0; motor_number < total_motors; motor_number++) {
				if(count(failed_number.begin(), failed_number.end(), motor_number) > max_number_repeats) {
					max_number_repeats = current_repeats;
					failed_motor_number = (uint8_t) motor_number;
				}
			}
			failed_motor_number = 4 - failed_motor_number;
		}
		failed_motor.arguments[0] = failed_motor_number;
		orb_publish(ORB_ID(event), failed_motor_publisher, &failed_motor);
		inputs.clear();
	}
	PX4_INFO("EXITING MOTOR FAILURE DETECTION");
	return 0;
}
