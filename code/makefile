# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++17 -Wall -Isrc/Headers -Isrc/Libs -Isrc/Libs/eigen-5.0.0

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
	$(CXX) $(OBJS) -o $(TARGET)

# Compile
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

# Rebuild
rebuild: clean all