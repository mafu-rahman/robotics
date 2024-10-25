.PHONY: cmd1 cmd2 cmd3 all

cmd1:
	@colcon build --packages-select cpmr_ch12

all: cmd1
	@source install/setup.zsh && ros2 launch cpmr_ch12 launch.py
