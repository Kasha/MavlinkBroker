VPATH = src 

all: git_submodule ColugoBroker

ColugoBroker: mavlink_control.cpp src/serial_port.cpp src/udp_port.cpp src/colugo_broker.cpp src/CC/ECC.cpp autopilot_interface.cpp
	g++ -g -Wall -I include -I include/CC -I mavlink/include/mavlink/v2.0 -I mavlink/include/mavlink/v2.0/ardupilotmega mavlink_control.cpp  src/serial_port.cpp src/udp_port.cpp src/colugo_broker.cpp src/CC/ECC.cpp autopilot_interface.cpp -o ColugoBroker -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o ColugoBroker
