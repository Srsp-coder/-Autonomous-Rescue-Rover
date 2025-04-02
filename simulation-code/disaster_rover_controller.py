from controller import Robot

# Initialize the robot
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Initialize motors
motors = []
motor_names = ['front left wheel', 'front right wheel', 'back left wheel', 'back right wheel']
for name in motor_names:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))  # Set position to infinity for velocity control
    motor.setVelocity(0.0)  # Initialize velocity to 0
    motors.append(motor)

# Initialize the sonar sensors
sonars = []
sonar_names = ['so0', 'so1', 'so2', 'so3', 'so4', 'so5', 'so6', 'so7']
for name in sonar_names:
    try:
        sonar = robot.getDevice(name)
        sonar.enable(timestep)
        sonars.append(sonar)
        print(f"Enabled sonar: {name}")
    except Exception as e:
        print(f"Could not enable sonar {name}: {e}")

# Constants
MAX_SPEED = 6.28  # Maximum wheel speed (rad/s)
THRESHOLD = 901  # Threshold value for obstacle detection
SAFE_THRESHOLD = 900  # Safe threshold to stop turning

# Function to set motor speeds
def set_speeds(left_speed, right_speed):
    motors[0].setVelocity(left_speed)  # front left
    motors[2].setVelocity(left_speed)  # back left
    motors[1].setVelocity(right_speed)  # front right
    motors[3].setVelocity(right_speed)  # back right

# Main control loop
print("Starting the automated Pioneer 3-AT controller")

turning = False
turning_direction = None

while robot.step(timestep) != -1:
    # Read sonar values
    sonar_values = [sonar.getValue() for sonar in sonars]
    print("Sonar values:", [round(val, 2) for val in sonar_values])
    
    # Check for obstacles (values > THRESHOLD)
    obstacle_detected = False
    obstacle_direction = None
    
    for i, value in enumerate(sonar_values):
        if value > THRESHOLD:
            obstacle_detected = True
            print(f"Obstacle detected by sensor {i} with value {value:.2f}")
            
            # Determine direction based on which sensor detected the obstacle
            if i in [0, 1]:  # Left side sensors
                obstacle_direction = "left"
            elif i in [2, 3]:  # Front-left sensors
                obstacle_direction = "front-left"
            elif i in [4, 5]:  # Front-right sensors
                obstacle_direction = "front-right"
            elif i in [6, 7]:  # Right side sensors
                obstacle_direction = "right"
            break  # React to the first detected obstacle
    
    # Handle obstacle avoidance
    if obstacle_detected or turning:
        if not turning:
            # Start turning
            turning = True
            if obstacle_direction in ["left", "front-left"]:
                turning_direction = "right"
                print(f"Avoiding obstacle on the {obstacle_direction}, turning right")
            else:
                turning_direction = "left"
                print(f"Avoiding obstacle on the {obstacle_direction}, turning left")
        
        # Continue turning
        if turning_direction == "right":
            set_speeds(MAX_SPEED, -MAX_SPEED/2)
        else:
            set_speeds(-MAX_SPEED/2, MAX_SPEED)
        
        # Check if we can stop turning
        if all(value < SAFE_THRESHOLD for value in sonar_values):
            turning = False
            turning_direction = None
            print("Path clear, resuming forward movement")
    else:
        # No obstacle detected, move forward
        set_speeds(MAX_SPEED, MAX_SPEED)
        print("Moving forward")
