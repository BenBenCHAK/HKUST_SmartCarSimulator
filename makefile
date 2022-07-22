all:
	g++ -I lib/sdl/src/include -L lib/sdl/src/lib -o main main.cpp -lws2_32 -lmingw32 -lSDL2main -lSDL2

# all:
# 	g++ -I lib/sdl/src/include -L lib/sdl/src/lib -o main main.cpp -lmingw32 -lSDL2main -lSDL2

# Below is for original nogui compilation
# all:simulation.cpp
# 	g++ simulation.cpp -lws2_32 -o simulation
# clean:
# 	rm -f simulation
