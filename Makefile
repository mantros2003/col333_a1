SRC_DIR = src
BUILD_DIR = build

CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2

EXEC = $(BUILD_DIR)/main
CHECKER_EXEC = $(BUILD_DIR)/format_checker

# --- Source Files ---
_SRCS = main.cpp io_handler.cpp solver.cpp cost_function.cpp lp_solver.cpp expand.cpp
_CHECKER_SRCS = format_checker.cpp

# Prepend the source directory path to each source file name.
SRCS = $(patsubst %,$(SRC_DIR)/%,$(_SRCS))
CHECKER_SRCS = $(patsubst %,$(SRC_DIR)/%,$(_CHECKER_SRCS))

# --- Object Files ---
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))
CHECKER_OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(CHECKER_SRCS))

# Explicitly define the path for the shared object file for clarity.
IO_HANDLER_OBJ = $(BUILD_DIR)/io_handler.o

# Default target: build the main executable.
all: $(EXEC)

# Rule to build the checker executable.
checker: $(CHECKER_EXEC)

# --- Linking Rules ---
$(EXEC): $(OBJS) | $(BUILD_DIR)
	@echo "Linking executable: $@"
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

# Rule to link the checker program.
# It depends on its own object file, the shared io_handler.o, and the build directory.
$(CHECKER_EXEC): $(CHECKER_OBJS) $(IO_HANDLER_OBJ) | $(BUILD_DIR)
	@echo "Linking executable: $@"
	$(CXX) $(CXXFLAGS) -o $@ $^

# --- Compilation Rule ---
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp $(SRC_DIR)/structures.h | $(BUILD_DIR)
	@echo "Compiling: $< -> $@"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# --- Directory Creation ---
$(BUILD_DIR):
	@echo "Creating directory: $@"
	@mkdir -p $@

# Clean up the build directory.
clean:
	@echo "Cleaning build directory..."
	@rm -rf $(BUILD_DIR)

# Declare targets that are not files.
.PHONY: all clean checker