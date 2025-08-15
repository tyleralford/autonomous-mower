#!/bin/bash

# This script recursively copies all files in the current directory and its subdirectories,
# appends the .xml extension to the new copied files,
# and does not copy the script file itself.

# Get the script's own name to prevent it from copying itself.
script_path="./$(basename "$0")"

# Use 'find' to locate all files (-type f) starting from the current directory (.).
find . -type f -not -path "$script_path" | while IFS= read -r file; do
    # For each file found, copy it to a new file with the .xml extension.
    cp -- "$file" "$file.xml"
    echo "Copied $file to $file.xml"
done

echo "Script finished."
