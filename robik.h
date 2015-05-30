#ifndef RobikTypes_h
#define RobikTypes_h

struct estimated_pos_t {
	ros::Time orig_time;
	uint32_t time_to_complete; //[milliseconds]
	uint32_t orig_pos;
	uint32_t target_pos;
	boolean corrected;
};

#endif
