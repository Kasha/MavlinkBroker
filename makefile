VPATH = src 

all: git_submodule ColugoBroker

ColugoBroker: mavlink_control.cpp src/colugo_broker.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp #src/ECC/ECC.cpp
	g++ -g -Wall -I include -I include/ECC -I mavlink/include/mavlink/v2.0 -I mavlink/include/mavlink/v2.0/ardupilotmega mavlink_control.cpp src/colugo_broker.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o ColugoBroker -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o ColugoBroker
