#!/bin/bash

# Directory containing the input, user output, and expected output files.
DATA_DIR="data"
# The format checker program to run (now located in the build directory).
CHECKER_EXEC="./build/format_checker"

# Check if the data directory exists
if [ ! -d "$DATA_DIR" ]; then
    echo "Error: Directory '$DATA_DIR' not found."
    exit 1
fi

# Check if the checker executable exists and is executable
if [ ! -x "$CHECKER_EXEC" ]; then
    echo "Error: '$CHECKER_EXEC' not found or is not executable."
    exit 1
fi

# Find all files matching 'input*.txt' in the data directory
for input_file in ${DATA_DIR}/input*.txt; do
    # Extract the base name (e.g., "input1") from the full path
    base_name=$(basename "$input_file" .txt)

    # Extract the numeric index (e.g., "1") from the base name
    i=${base_name#input}

    # Construct the paths for the corresponding output files
    user_output_file="${DATA_DIR}/mout${i}.txt"
    expected_output_file="${DATA_DIR}/out${i}.txt"

    echo "========================================"
    echo "         RUNNING CHECKER ON TEST CASE $i"
    echo "========================================"
    echo "Input:  '$input_file'"
    echo "User Output: '$user_output_file'"
    echo "----------------------------------------"

    # Check that both required output files exist for the current test case
    if [ ! -f "$user_output_file" ] || [ ! -f "$expected_output_file" ]; then
        echo "Warning: Skipping test case $i because an output file is missing."
        [ ! -f "$user_output_file" ] && echo "  Missing: '$user_output_file'"
        [ ! -f "$expected_output_file" ] && echo "  Missing: '$expected_output_file'"
        echo ""
        continue # Skip to the next iteration of the loop
    fi

    # 1. Run the format checker and show the score
    echo "--- SCORE ---"
    # The grep command will only print the line containing "FINAL SCORE"
    $CHECKER_EXEC "$input_file" "$user_output_file" | grep "FINAL SCORE"
    $CHECKER_EXEC "$input_file" "$expected_output_file" | grep "FINAL SCORE"
    echo ""

done

echo "========================================"
echo "          All checks finished."
echo "========================================"