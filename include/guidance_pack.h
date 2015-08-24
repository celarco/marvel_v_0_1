#ifndef GUIDANCE_PACK_HEADER
#define GUIDANCE_PACK_HEADER

enum vertical_mode {
    VERTICAL_HOLD,
    VERTICAL_LOCK,
    VERTICAL_CLIMB
};

enum horizontal_mode {
    HORIZONTAL_HOLD,
    HORIZONTAL_LOCK,
    HORIZONTAL_VELOCITY
};

enum heading_mode {
    HEADING_HOLD,
    HEADING_LOCK,
    HEADING_RATE
};

enum lock_param {
	WINDOW_DETECTOR = 1,
	FLOWER,
	MARKER,
	ROPE
};
#endif
