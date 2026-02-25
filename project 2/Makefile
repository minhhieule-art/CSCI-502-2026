CXX=g++
CXXFLAGS=-std=c++17 -O2 -Wall -Isrc
LDFLAGS=-lGL -lGLU -lglut

SRCS=src/main.cpp src/I2CBus.cpp src/MinIMU9.cpp src/MadgwickFilter.cpp
OBJS=$(SRCS:.cpp=.o)

imu_glut: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

clean:
	rm -f $(OBJS) imu_glut
