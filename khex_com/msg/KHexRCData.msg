Header header

# position of the right stick
uint32 right_fb # forward-backward pos.
uint32 right_rl # right-left pos.

# position of the left stick
uint32 left_fb # forward-backward pos.
uint32 left_rl # right-left pos.

uint32[8] misc # KHex broadcasts through 12 channels.
			   # misc[] carries the remaining other
			   # 8 channels

