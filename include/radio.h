struct rc_channel {
	int min, trim, max, rev, dz;
};
class radio {
public:
	rc_channel roll, pitch, yaw, throttle, mode;	
	int calc_roll(int roll_cmd);
	int calc_pitch(int pitch_cmd);
	int calc_yaw(int yaw_cmd);
	int calc_throttle(int throttle_cmd);
	int calc_mode(int mode_cmd);
	
};
