#!/bin/bash

# Directory containing the input files.
DATA_DIR="data"
# The main solver program to run (now located in the build directory).
SOLVER_EXEC="./build/main"
# Directory where generated output files will be stored.
OUTPUT_DIR="${DATA_DIR}"

# Check if the data directory exists
if [ ! -d "$DATA_DIR" ]; then
    echo "Error: Directory '$DATA_DIR' not found."
    echo "Please ensure you have a 'data/' directory with your input files."
    exit 1
fi

# Check if the main executable exists and is executable
if [ ! -x "$SOLVER_EXEC" ]; then
    echo "Error: '$SOLVER_EXEC' not found or is not executable."
    echo "Please compile the project first by running 'make'."
    exit 1
fi

# Create the output directory if it doesn't already exist.
mkdir -p "$OUTPUT_DIR"
echo "Storing generated files in the './${OUTPUT_DIR}/' directory."
echo ""

# Find all files matching 'input*.txt' in the data directory
for input_file in ${DATA_DIR}/input*.txt; do
    # Extract the base name (e.g., "input1") from the full path
    base_name=$(basename "$input_file" .txt)

    # Extract the numeric index (e.g., "1") from the base name
    i=${base_name#input}

    # Construct the path for the corresponding output file in the new directory
    output_file="${OUTPUT_DIR}/mout${i}.txt"

    echo "========================================"
    echo "         RUNNING SOLVER ON TEST CASE $i"
    echo "========================================"
    echo "Input:  '$input_file'"
    echo "Output: '$output_file'"
    echo "----------------------------------------"

    # Use the 'time' command to measure the execution duration of the solver.
    time $SOLVER_EXEC "$input_file" "$output_file"

    # Check the exit code of the last command.
    if [ $? -eq 0 ]; then
        echo "Solver finished successfully."
    else
        echo "ERROR: Solver exited with a non-zero status code."
    fi
    echo "" # Add a blank line for readability
done

echo "========================================"
echo "          All runs finished."
echo "========================================"