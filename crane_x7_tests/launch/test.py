import subprocess

# Command to run in the background
command = "ign topic -t /world/shapes/dynamic_pose/info -e"

# Start the subprocess and redirect stdout and stderr to pipes
process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

# Initialize flags to track whether the target parts have been found
found_pose = False
found_name = False

# Initialize the variable to store the output
output_call = ""

# Initialize a counter for the word "pose"
pose_count = 0

while True:
    # Read a line from the stdout (non-blocking)
    output_line = process.stdout.readline()

    # Check if the process has finished
    if process.poll() is not None and not output_line:
        break

    if output_line:
        # Append the output_line to the variable
        output_call += output_line

        # Count the number of times "pose" appears in the output_line
        pose_count += output_line.count("pose")

        # Check if 'pose {' is in the output_line
        if 'pose {' in output_line:
            found_pose = True
        
        # Check if 'name: "crane_x7_gripper_base_link"' is in the output_line
        if 'name: "crane_x7_gripper_base_link"' in output_line:
            found_name = True

        # Print the output_line
        print(output_line.strip())

        # Check if both parts have been found
        if found_pose and found_name:
            print("hey")
            break

# Close the subprocess
process.kill()

pose_count = pose_count - 1 
# Print the number of times "pose" appears in the entire output
# print("Number of times 'pose' appears:", pose_count)