all:simulation.cpp
	g++ simulation.cpp -lws2_32 -o simulation
clean:
	rm -f simulation
