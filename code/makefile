# Compiler
CXX = g++

# ── ROS2 configuration ────────────────────────────────────────────────────
# Override ROS_DISTRO or ROS2_ROOT if your installation is non-standard.
ROS_DISTRO ?= humble
ROS2_ROOT  ?= /opt/ros/$(ROS_DISTRO)

ROS2_INCLUDES = \
    -I$(ROS2_ROOT)/include \
    -I$(ROS2_ROOT)/include/rclcpp \
    -I$(ROS2_ROOT)/include/rcl \
    -I$(ROS2_ROOT)/include/rcutils \
    -I$(ROS2_ROOT)/include/rcl_interfaces \
    -I$(ROS2_ROOT)/include/rmw \
    -I$(ROS2_ROOT)/include/nav_msgs \
    -I$(ROS2_ROOT)/include/geometry_msgs \
    -I$(ROS2_ROOT)/include/std_msgs \
    -I$(ROS2_ROOT)/include/builtin_interfaces \
    -I$(ROS2_ROOT)/include/rosidl_runtime_cpp \
    -I$(ROS2_ROOT)/include/rosidl_typesupport_interface

ROS2_LIBS = \
    -L$(ROS2_ROOT)/lib \
    -lrclcpp -lrcl -lrcutils -lrmw \
    -lnav_msgs__rosidl_typesupport_cpp \
    -lgeometry_msgs__rosidl_typesupport_cpp \
    -lstd_msgs__rosidl_typesupport_cpp \
    -lrosidl_typesupport_cpp \
    -Wl,-rpath,$(ROS2_ROOT)/lib

# ── Compiler flags ────────────────────────────────────────────────────────
CXXFLAGS = -std=c++17 -Wall \
           -Isrc/Headers \
           -Isrc/Libs \
           -Isrc/Libs/eigen-5.0.0 \
           $(ROS2_INCLUDES)

# Directories
SRC_DIR = src/Code
OBJ_DIR = build

# Find all source files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)

# Convert to object files
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# Output binary
TARGET = complilation

# Default rule
all: $(TARGET)

# Link
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(ROS2_LIBS)

# Compile
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

# Rebuild
rebuild: clean all