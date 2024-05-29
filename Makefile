# Define the C++ compiler to use
CXX ?= g++

# Define any compile-time flags
CXXFLAGS = -std=c++20 -O2 -Wall -Wextra -Wconversion -Wshadow -Wpedantic -g

# Define the linker flags
LDFLAGS = -li2c -lwiringPi

# Define the target executable
TARGET = Example

# Define the source files
SRCS = Example.cpp MPU6050.cpp filters.cpp button.cpp rgbled.cpp sequence.cpp # detection.cpp queue.cpp logging.cpp gpio.cpp

# Define the object files
OBJS = $(SRCS:.cpp=.o)

# Default rule
all: $(TARGET)

# Link the object files into the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Compile the source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up the project directory
clean:
	$(RM) $(TARGET) $(OBJS)