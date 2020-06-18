all: git_submodule ColugoBroker

ColugoBroker: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	g++ -g -Wall -I mavlink/include/mavlink/v2.0 mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o ColugoBroker -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o ColugoBroker
