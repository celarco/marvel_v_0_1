#include <radio.h> 
int radio::calc_roll(int roll_cmd) {
	int range = roll.max - roll.min;
	float coeff = range / 200;
	int result = roll.trim + coeff * roll_cmd * roll.rev;
	return result;
}

int radio::calc_pitch(int pitch_cmd) {
	int range = pitch.max - pitch.min;
	float coeff = range / 200;
	int result = pitch.trim + coeff * pitch_cmd * pitch.rev;
	return result;
}

int radio::calc_yaw(int yaw_cmd) {
	int range = yaw.max - yaw.min;
	float coeff = range / 200;
	int result = yaw.trim + coeff * yaw_cmd * yaw.rev;
	return result;
}

int radio::calc_throttle(int throttle_cmd) {
	int range = throttle.max - throttle.min;
	float coeff = range / 200;
	int result = throttle.trim + coeff * throttle_cmd * throttle.rev;
	return result;
}

int radio::calc_mode(int mode_cmd) {
	int range = mode.max - mode.min;
	float coeff = range / 200;
	int result = mode.trim + coeff * mode_cmd * mode.rev;
	return result;
}