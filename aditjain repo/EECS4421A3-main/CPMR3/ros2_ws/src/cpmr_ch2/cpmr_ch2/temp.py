# import subprocess
# print("HELLO WORLD")
# command = "echo \" Here We Go \" "

# result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

# command = "source install/setup.bash" 
# result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
# command = "ros2 param set /drive_to_goal newGoal \"3.0 & 1.0\"" 
# result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
# # Print any errors
# print("Errors:", result.stderr)

# # Print the return code
# print("Return code:", result.returncode)

# print("Output:", result.stdout)
# x = input("PRESS TO CONINTUE")
# Open a file in read mode


import time 
import subprocess
with open('waypoints.txt', 'r') as file:
    # Read the file line by line
    for line in file:
        output = line.strip()
        print(output)
        i = output[1:-1].split(',')

        print("\n\n\nWORKING ON MOVEMENT \n\n\n")
        command = f"ros2 param set /drive_to_goal newGoal \"{i[0]} & {i[1]}\"" 
        result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print("Errors:", result.stderr)
        # Print the return code
        print("Return code:", result.returncode)
        print("Output:", result.stdout)
        time.sleep(2)


# with open('waypoints.txt', 'r') as file:
#     # Read the file line by line
#     for line in file:
#         output = line.strip()
#         print(output)
#         numbers = output[1:-1].split(',')
#           # strip() removes the newline character at the end of each line

# # Given string
# string = "(28,81)"

# # Remove parentheses and split the string at the comma
# numbers = string[1:-1].split(',')

# # Convert the strings to integers
# number1 = int(numbers[0])
# number2 = int(numbers[1])

# # Print the result
# print("Number 1:", number1)
# print("Number 2:", number2)
