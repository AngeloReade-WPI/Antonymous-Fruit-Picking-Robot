    
# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       sacca                                                        #
# 	Created:      2/9/2025, 3:20:16 PM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

########## ========== INITIALIZATION ========== ##########

# Library imports
from vex import *

import math

# Robot States

ROBOT_IDLE = 0

ROBOT_TELEOP = 1

ROBOT_FOLLOW_FRUIT = 2

ROBOT_FOC = 3

ROBOT_APPROACHING = 4

ROBOT_COLLECTING = 5

ROBOT_FACE_DIRECTION = 6

ROBOT_FOLLOW_HEADING = 7

ROBOT_FOLLOW_LINE = 8

ROBOT_DEPOSITING = 9

ROBOT_SEARCHING = 10

robotState = ROBOT_IDLE

closed = False

elevator_position = "Neither"

# Hardware Declarations

brain = Brain()

controller = Controller()

Drivetrain_RPM = 100

BR_Motor = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
FR_Motor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
BL_Motor = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
FL_Motor = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
claw_motor = Motor(Ports.PORT2, GearSetting.RATIO_36_1, False)
elevator_motor = Motor(Ports.PORT1, GearSetting.RATIO_36_1, False)
basket_motor = Motor(Ports.PORT6, GearSetting.RATIO_18_1,False )

right_linefollow = Line(brain.three_wire_port.a)
left_linefollow = Line(brain.three_wire_port.b)

right_ultrasonic = Sonar(brain.three_wire_port.c)
left_ultrasonic = Sonar(brain.three_wire_port.e)

lower_limitswitch = Limit(brain.three_wire_port.g)
upper_limitswitch = Limit(brain.three_wire_port.h)

claw_motor.reset_position()
elevator_motor.reset_position()
basket_motor.reset_position()

# Vision Signatures

Vision5__ORANGE_FRUIT = Signature (1, 2957, 5417, 4188, -2599, -2261, -2430, 2.5, 0)
Vision5__GREEN_FRUIT = Signature (2, -6053, -5501, -5777, -3213, -2651, -2932, 2.5, 0)
Vision5__YELLOW_FRUIT = Signature (3, 1405, 1675, 1540, -3415, -3141, -3278, 3, 0)
Vision5 = Vision (Ports.PORT5, 40, Vision5__ORANGE_FRUIT, Vision5__GREEN_FRUIT, Vision5__YELLOW_FRUIT)

# Initialized Variables 

min_height = 50

screen_x = 315

screen_y = 211

x_tol = 12

distance_tol = 15

z_tol = 36

desired_heading = 0

current_heading = 0

face_heading = 0

fruits_picked = 0

step = 1 # Traverse Field, Update Y, Update X, Collect, Return 

queue_counter = 2

# Global variable to track timer start time
timer_start = None

timer_duration = 20

deposit_complete = False

collection_complete = False

up_flag = False

# Navigation Waypoints

right_side = 110

left_side = 1100

right_ultrasonic_distance = 0

left_ultrasonic_distance = 0

prev_right_dist = 0

prev_left_dist = 0

return_x_dist = 110

return_y_dist = 350

lemon_deposit_dist = 600

lime_deposit_dist = 900

orange_deposit_dist = 1350

initial_wall_distance = 110

line_wall_dist = 110

A5x = 450

A5y = 1100

B2x = 450

B2y = 1200

C2x = 450

C2y = 2250

# IMU Calibration

imu = Inertial(Ports.PORT15)

imu.calibrate()

while imu.is_calibrating():
    wait(2000)

print(" ========== INITIALIZATION COMPLETE ========== ")


########## ========== ROBOT FUNCTIONS ========== ##########

def update_ultrasonic():

    global prev_left_dist, prev_right_dist, right_ultrasonic_distance, left_ultrasonic_distance

    right_ultrasonic_distance = right_ultrasonic.distance(MM)
    left_ultrasonic_distance = left_ultrasonic.distance(MM)

    if abs(right_ultrasonic_distance) > 5000: 

        right_ultrasonic_distance = prev_right_dist
    else: 
        prev_right_dist = right_ultrasonic_distance
    
    if abs(left_ultrasonic_distance)  > 5000: 

        left_ultrasonic_distance = prev_left_dist
    else: 
        prev_left_dist = left_ultrasonic_distance

def update_queue():

    global queue_counter, next_x_pos, next_y_pos, next_pos, deposit_dist, initial_wall_distance, desired_color

    if queue_counter == 0: 

        next_pos = "A5"

        next_x_pos = A5x
        next_y_pos = A5y

        deposit_dist = lemon_deposit_dist

        desired_color = Vision5__YELLOW_FRUIT

        initial_wall_distance = right_side

    elif queue_counter == 1: 

        next_pos = "B2"

        next_x_pos = B2x
        next_y_pos = B2y

        deposit_dist = lime_deposit_dist

        desired_color = Vision5__GREEN_FRUIT

        initial_wall_distance = right_side
    
    elif queue_counter == 2: 

        next_pos = "C2"

        next_x_pos = C2x
        next_y_pos = C2y

        deposit_dist = orange_deposit_dist

        desired_color = Vision5__ORANGE_FRUIT

        initial_wall_distance = right_side

def update_heading():

    global desired_heading,face_heading,step,queue_counter

    if next_pos == "A5":

        if step == 1: 

            desired_heading = 270

            face_heading = 0

        if step == 2: 

            desired_heading = 0

            face_heading = 90

        if step == 3: 

            desired_heading = 270

            face_heading = 180

        if step == 4:

            desired_heading = 90

            face_heading = 180

        if step == 5: 

            desired_heading = 90

            face_heading = 180

        if step == 6: 

            desired_heading = 180

            face_heading = 270
        
        if step == 7: 

            desired_heading = 270

            face_heading = 180

        if step == 8: 

            desired_heading = 270

            face_heading = 180

        if step == 9: 

            desired_heading = 90

            face_heading = 0
        
        if step == 12: 

            desired_heading = 90

            face_heading = 180
    
    if next_pos == "B2":

        if step == 1: 

            desired_heading = 270

            face_heading = 0

        if step == 2: 

            desired_heading = 0

            face_heading = 90

        if step == 3: 

            desired_heading = 270

            face_heading = 0

        if step == 4:

            desired_heading = 90

            face_heading = 180

        if step == 5: 

            desired_heading = 90

            face_heading = 0

        if step == 6: 

            desired_heading = 180

            face_heading = 270
        
        if step == 7: 

            desired_heading = 270

            face_heading = 180

        if step == 8: 

            desired_heading = 270

            face_heading = 180

        if step == 9: 

            desired_heading = 90

            face_heading = 0
        
        if step == 12: 

            desired_heading = 90

            face_heading = 0

    if next_pos == "C2":

        if step == 1: 

            desired_heading = 270

            face_heading = 0

        if step == 2: 

            desired_heading = 0

            face_heading = 90

        if step == 3: 

            desired_heading = 270

            face_heading = 0

        if step == 4:

            desired_heading = 90

            face_heading = 0

        if step == 5: 

            desired_heading = 90

            face_heading = 0

        if step == 6: 

            desired_heading = 180

            face_heading = 270
        
        if step == 7: 

            desired_heading = 270

            face_heading = 180

        if step == 8: 

            desired_heading = 270

            face_heading = 180

        if step == 9: 

            desired_heading = 90

            face_heading = 0
        
        if step == 12: 

            desired_heading = 90

            face_heading = 180
        
########## ========== STATE TIMER ========== ##########

def handleLineTimer():

    global robotState

    if robotState == ROBOT_IDLE:

        BR_Motor.stop()
        FR_Motor.stop()
        BL_Motor.stop()
        FL_Motor.stop()
        claw_motor.stop()
        elevator_motor.stop()

    if robotState == ROBOT_FOC:

        i = controller.axis1.position()  # Forward / Backward

        j = controller.axis2.position()  # Strafe Left / Strafe Right 

        r = controller.axis4.position() # Rotation (Turn)

        theta = imu.heading()*math.pi/180

        # xVelocity =  i * math.cos(theta) + j * math.sin(theta)
        # yVelocity = i * math.sin(theta) - j * math.cos(theta)
        xVelocity = j * math.cos(theta) + i * math.sin(theta)
        yVelocity = -j * math.sin(theta) + i * math.cos(theta)

        BR_Velocity = -yVelocity - xVelocity + r
        FR_Velocity = yVelocity - xVelocity + r
        BL_Velocity = -yVelocity + xVelocity + r
        FL_Velocity = yVelocity + xVelocity + r

        Max_Velocity = Drivetrain_RPM

        
        if abs(BR_Velocity) > Max_Velocity: 
            Max_Velocity = abs(BR_Velocity)

        if abs(FR_Velocity) > Max_Velocity: 
            Max_Velocity = abs(FR_Velocity)

        if abs(BL_Velocity) > Max_Velocity: 
            Max_Velocity = abs(BL_Velocity)

        if abs(FL_Velocity) > Max_Velocity: 
            Max_Velocity = abs(FL_Velocity)

        if Max_Velocity == 0:
            print("Error: Maximum Velocity Cannot be Zero!")

        elif Max_Velocity > Drivetrain_RPM: 

            BL_Velocity = Drivetrain_RPM * BL_Velocity / Max_Velocity
            FR_Velocity = Drivetrain_RPM * FR_Velocity / Max_Velocity 
            FL_Velocity = Drivetrain_RPM * FL_Velocity / Max_Velocity
            BR_Velocity = Drivetrain_RPM * BR_Velocity / Max_Velocity

        BL_Motor.spin(FORWARD, BL_Velocity)
        BR_Motor.spin(FORWARD, BR_Velocity)
        FR_Motor.spin(FORWARD, FR_Velocity)
        FL_Motor.spin(FORWARD, FL_Velocity)

    if robotState == ROBOT_TELEOP:

        yVelocity = controller.axis1.position()  # Strafe (Left/Right)

        xVelocity = controller.axis2.position()  # Forward/Backward

        r = controller.axis4.position() # Rotation (Turn)

        BR_Velocity = -yVelocity - xVelocity + r
        FR_Velocity = yVelocity - xVelocity + r
        BL_Velocity = -yVelocity + xVelocity + r
        FL_Velocity = yVelocity + xVelocity + r

        Max_Velocity = Drivetrain_RPM

        if abs(BR_Velocity) > Max_Velocity: 
            Max_Velocity = abs(BR_Velocity)

        if abs(FR_Velocity) > Max_Velocity: 
            Max_Velocity = abs(FR_Velocity)

        if abs(BL_Velocity) > Max_Velocity: 
            Max_Velocity = abs(BL_Velocity)

        if abs(FL_Velocity) > Max_Velocity: 
            Max_Velocity = abs(FL_Velocity)

        if Max_Velocity == 0:
            print("Error: Maximum Velocity Cannot be Zero!")

        elif Max_Velocity > Drivetrain_RPM: 

            BL_Velocity = Drivetrain_RPM * BL_Velocity / Max_Velocity
            FR_Velocity = Drivetrain_RPM * FR_Velocity / Max_Velocity 
            FL_Velocity = Drivetrain_RPM * FL_Velocity / Max_Velocity
            BR_Velocity = Drivetrain_RPM * BR_Velocity / Max_Velocity

        BL_Motor.spin(FORWARD, BL_Velocity)
        BR_Motor.spin(FORWARD, BR_Velocity)
        FR_Motor.spin(FORWARD, FR_Velocity)
        FL_Motor.spin(FORWARD, FL_Velocity)

    if robotState == ROBOT_APPROACHING:

        global desired_color

        Vision5.take_snapshot(desired_color)

        largest_object = Vision5.largest_object()

        if largest_object is not None:  # Ensure object is detected

            print("obj detected")

            Vision5.take_snapshot(desired_color)
            largest_object = Vision5.largest_object()

            # print("Object Found!")

            if largest_object.height > min_height:

                global error_x, distance_error, z_error

                Vision5.take_snapshot(desired_color)
                largest_object = Vision5.largest_object()

                cx = largest_object.centerX  # Object center X
                cy = largest_object.centerY # Object center Y
                obj_width = largest_object.width  # Object width
                obj_height = largest_object.height  # Object height

                # Target values
                target_x = (screen_x / 2)  
                target_y = (screen_y / 2) 
                target_size = 150  # Desired object width when "close enough"
                strafe_threshold = 50  # How far off-center before strafing instead of turning

                # **Determine movement efforts
                K_x = 0.5 # Rotation scaling
                K_y = 0.4  # Forward movement scaling
                K_s = 0.7  # Strafing scaling
                K_e = 0.4 # Elevator scaling

                # Calculate movement based on object position
                error_x = cx - target_x  # How far left/right the object is
                distance_error = target_size - obj_width  # How far away the object is
                z_error = cy - target_y

                forward_effort = K_y * distance_error  # Move forward/backward
                turn_effort = 0 # Default turn effort
                strafe_effort = 0  # Default strafe effort

                elevator_effort = K_e * z_error

                # **Strafe if object is slightly off-center, turn if it's far off-center**
                if abs(error_x) < strafe_threshold:
                    strafe_effort = K_s * error_x  # Strafe to center object
                    turn_effort = 0
                else:
                    turn_effort = K_x * error_x  # Rotate to center object
                    strafe_effort = 0

                xVelocity = strafe_effort
                yVelocity = forward_effort

                # Apply movement to motors (without rotation affecting strafing)
                BR_Velocity = -xVelocity - yVelocity + turn_effort
                FR_Velocity = xVelocity - yVelocity + turn_effort
                BL_Velocity = -xVelocity + yVelocity + turn_effort
                FL_Velocity = xVelocity + yVelocity + turn_effort

                # Normalize velocity if necessary
                Max_Velocity = max(abs(BR_Velocity), abs(FR_Velocity), abs(BL_Velocity), abs(FL_Velocity), Drivetrain_RPM)
                
                if Max_Velocity > Drivetrain_RPM:
                    BR_Velocity = (BR_Velocity / Max_Velocity) * Drivetrain_RPM
                    FR_Velocity = (FR_Velocity / Max_Velocity) * Drivetrain_RPM
                    BL_Velocity = (BL_Velocity / Max_Velocity) * Drivetrain_RPM
                    FL_Velocity = (FL_Velocity / Max_Velocity) * Drivetrain_RPM 
                    # Set motor speeds

                BL_Motor.spin(FORWARD, BL_Velocity)
                BR_Motor.spin(FORWARD, BR_Velocity)
                FR_Motor.spin(FORWARD, FR_Velocity)
                FL_Motor.spin(FORWARD, FL_Velocity)

                if upper_limitswitch.pressing() == 1: 

                    if elevator_effort < 0: 

                        elevator_effort = 0

                    elif elevator_effort >= 0:

                        elevator_effort = elevator_effort 

                    else: 
                        pass
                
                if lower_limitswitch.pressing() == 1:

                    if elevator_effort > 0: 

                        elevator_effort = 0

                    elif elevator_effort <= 0:

                        elevator_effort = elevator_effort 

                    else: 
                        pass

                elevator_motor.spin(FORWARD, elevator_effort)
                    
                if target_size < obj_width: 
                
                    print("Object Within Reach")   

                if checkApproachComplete(): 
                    
                    handleApproachComplete()
            else: 

                BL_Motor.stop()
                BR_Motor.stop()
                FR_Motor.stop()
                FL_Motor.stop()
                elevator_motor.stop()
                print("no object")
    
    if robotState == ROBOT_COLLECTING:

        global collection_complete 

        claw_motor.spin_to_position(200, DEGREES, wait = False)

        wait(750)

        elevator_motor.stop()

        print("Claw Closed")

        # Backup Command

        BL_Motor.spin_for(REVERSE, 500, DEGREES, wait = False)
        BR_Motor.spin_for(FORWARD, 500, DEGREES, wait = False)
        FL_Motor.spin_for(REVERSE, 500, DEGREES, wait = False)
        FR_Motor.spin_for(FORWARD, 500, DEGREES, wait = True)

        print("Backup Complete")

        claw_motor.spin_to_position(5, DEGREES, True)

        collection_complete = True

        if checkCollectionComplete():
            handleCollectionComplete()
    
    if robotState == ROBOT_FACE_DIRECTION:

        global face_heading, current_heading, turning_error

        current_heading = imu.heading()

        turning_error = face_heading - current_heading

        Kr = 0.95
        
        turn_effort_imu = turning_error * Kr
        
        BR_Velocity = turn_effort_imu
        FR_Velocity = turn_effort_imu
        BL_Velocity = turn_effort_imu
        FL_Velocity = turn_effort_imu

        BL_Motor.spin (FORWARD,BL_Velocity)
        BR_Motor.spin (FORWARD,BR_Velocity)
        FL_Motor.spin (FORWARD,FL_Velocity)
        FR_Motor.spin (FORWARD,FR_Velocity)

        if checkTurnComplete():
            handleTurnComplete()

    if robotState == ROBOT_FOLLOW_HEADING:

        global desired_heading

        current_heading = imu.heading()

        error = desired_heading - current_heading

        Kp = 0.00

        base_effort = 20

        if desired_heading == 0: 
            x0 = 0
            y0 = base_effort
        elif desired_heading == 90: 
            x0 = base_effort
            y0 = 0
        elif desired_heading == 180: 
            x0 = 0
            y0 = -base_effort
        elif desired_heading == 270:
            x0 = -base_effort
            y0 = 0
        else: 
            print("Desired heading must be a cardinal (0, 90, 180, 270)")

        theta = math.radians(current_heading) # convert to radians

        xVelocity = x0 * math.cos(theta) - y0 * math.sin(theta)
        yVelocity = y0 * math.cos(theta) + x0 * math.sin(theta)

        turn_effort = Kp * error
        
        BR_Velocity = - xVelocity - yVelocity + turn_effort
        FR_Velocity = xVelocity - yVelocity + turn_effort
        BL_Velocity = - xVelocity + yVelocity + turn_effort
        FL_Velocity = xVelocity + yVelocity + turn_effort

        # Normalize velocity if necessary
        Max_Velocity = max(abs(BR_Velocity), abs(FR_Velocity), abs(BL_Velocity), abs(FL_Velocity), Drivetrain_RPM)
                
        if Max_Velocity > Drivetrain_RPM:
            BR_Velocity = (BR_Velocity / Max_Velocity) * Drivetrain_RPM
            FR_Velocity = (FR_Velocity / Max_Velocity) * Drivetrain_RPM
            BL_Velocity = (BL_Velocity / Max_Velocity) * Drivetrain_RPM
            FL_Velocity = (FL_Velocity / Max_Velocity) * Drivetrain_RPM 
            # Set motor speeds

        BL_Motor.spin(FORWARD, BL_Velocity)
        BR_Motor.spin(FORWARD, BR_Velocity)
        FR_Motor.spin(FORWARD, FR_Velocity)
        FL_Motor.spin(FORWARD, FL_Velocity)

        if checkXUpdated():
            handleXUpdated()

    if robotState == ROBOT_FOLLOW_LINE:

        right_reflectivity = right_linefollow.reflectivity()
        left_reflectivity = left_linefollow.reflectivity()

        error = left_reflectivity - right_reflectivity 

        Kp = 0.4

        line_follow_effort = Kp * error

        base_effort = -50

        current_heading = imu.heading()

        error = face_heading - current_heading

        Kr = 0.05
        
        turn_effort_imu = error * Kr
        
        xVelocity = base_effort
        yVelocity = line_follow_effort + turn_effort_imu
        
        BR_Velocity = -xVelocity + yVelocity
        FR_Velocity = xVelocity + yVelocity 
        BL_Velocity = -xVelocity + yVelocity
        FL_Velocity = xVelocity + yVelocity

        # Normalize velocity if necessary
        Max_Velocity = max(abs(BR_Velocity), abs(FR_Velocity), abs(BL_Velocity), abs(FL_Velocity), Drivetrain_RPM)
                
        if Max_Velocity > Drivetrain_RPM:
            BR_Velocity = (BR_Velocity / Max_Velocity) * Drivetrain_RPM
            FR_Velocity = (FR_Velocity / Max_Velocity) * Drivetrain_RPM
            BL_Velocity = (BL_Velocity / Max_Velocity) * Drivetrain_RPM
            FL_Velocity = (FL_Velocity / Max_Velocity) * Drivetrain_RPM 
            # Set motor speeds

        BL_Motor.spin(FORWARD, BL_Velocity)
        BR_Motor.spin(FORWARD, BR_Velocity)
        FR_Motor.spin(FORWARD, FR_Velocity)
        FL_Motor.spin(FORWARD, FL_Velocity)

        # Checkers & Handlers

        if checkYUpdated():
            handleYUpdated()

    if robotState == ROBOT_DEPOSITING:

        BR_Motor.stop()
        FR_Motor.stop()
        BL_Motor.stop()
        FL_Motor.stop()
        
        global fruits_picked, deposit_complete
             
        if upper_limitswitch.pressing() == 1:

            elevator_motor.stop()

            print("Elevator Raised")

        if upper_limitswitch.pressing() == 0:
                    
            elevator_motor.spin(FORWARD, -50, RPM)

        if upper_limitswitch.pressing() == 1:
            
            if fruits_picked >= 0:

                print("Basket Raising")

                basket_motor.spin_to_position(-135, DEGREES, 75, RPM, wait = True)

                wait(3, SECONDS)

                basket_motor.spin_to_position(-1,DEGREES, 100, RPM, wait = True )

                print("Basket Raised")
            
                fruits_picked = 0

                deposit_complete = True

        if checkDepositComplete(): 
            handleDepositComplete()

    if robotState == ROBOT_SEARCHING:

        global elevator_position, up_flag
        
        BR_Motor.stop()
        FR_Motor.stop()
        BL_Motor.stop()
        FL_Motor.stop()

        if lower_limitswitch.pressing() == 1: 

            elevator_position = "Down"
        
        if upper_limitswitch.pressing() == 1: 

            elevator_position = "Up"

        if upper_limitswitch.pressing() == 0:

            if lower_limitswitch.pressing() == 0: 
                
                elevator_position = "Neither" 

        if elevator_position == "Neither": 

            if up_flag == False:
            
                elevator_motor.spin(FORWARD, -30, RPM)

            if up_flag == True:

                elevator_motor.spin(REVERSE, -30, RPM) 

        if elevator_position == "Up":

            elevator_motor.spin(REVERSE, -30, RPM) 

            up_flag = True

        if elevator_position == "Down": 

            elevator_motor.stop()

            if checkFruitMissed():
                handleFruitMissed()

            print("Search Complete") 

            up_flag = False

        if checkFruitFound():
            handleFruitFound()

        print("elevator position: ", elevator_position)
        print("flag condition", up_flag)

    lineTimer.event(handleLineTimer, 50)

lineTimer = Timer()

lineTimer.event(handleLineTimer, 50) 

########## ========== AUTO CHECKERS ========== ##########

def start_timer(duration):

    global timer_start, timer_duration

    timer_start = brain.timer.time()  # Record the current time

    timer_duration = duration

def timer_expired():

    if timer_start is None:

        return False  # Timer hasn't started

    return (brain.timer.time() - timer_start) >= timer_duration

def checkApproachComplete():

    global timer_duration, timer_start

    if abs(error_x) is not None: 

        if x_tol >= abs(error_x) or lower_limitswitch.pressing() == 1:

            if distance_tol >= abs(distance_error):

                if z_tol >= abs(z_error) or lower_limitswitch.pressing() == 1: 

                    return True
    
   # Check if approach timer has expired

    start_timer(20)

    if timer_start is not None:

        if (brain.timer.time() - timer_start) >= timer_duration:

            return True  
    return False

def checkCollectionComplete():

    global collection_complete
    
    if collection_complete == True:

        collection_complete = False

        return True
    
    return False

def checkDepositComplete():

    global deposit_complete

    if deposit_complete == True: 

        return True
    
    return False

def checkTurnComplete(): 

    global turning_error

    turn_tol = 2.5

    if robotState == ROBOT_FACE_DIRECTION: 

        if abs(turning_error) <= turn_tol: 

            return True

    return False

def checkFruitFound():

    Vision5.take_snapshot(desired_color)

    largest_object = Vision5.largest_object()

    if largest_object is not None:  # Ensure object is detected

        print("obj detected")

        Vision5.take_snapshot(desired_color)
        largest_object = Vision5.largest_object()

        # print("Object Found!")

        if largest_object.height >= min_height:

            return True
        
        elif largest_object.height < min_height: 
    
            return False

    return False

def checkFruitMissed():

    Vision5.take_snapshot(desired_color)

    largest_object = Vision5.largest_object()

    if largest_object is not None:  # Ensure object is detected

        print("obj detected")

        Vision5.take_snapshot(desired_color)
        largest_object = Vision5.largest_object()

        # print("Object Found!")

        if largest_object.height >= min_height:

            handleFruitFound()
        
        elif largest_object.height < min_height: 
    
            return True
        
    elif largest_object is None: 

        return True

    return False

def checkXUpdated():

    global initial_wall_distance, next_x_pos, line_wall_dist, step, deposit_dist, right_ultrasonic_distance, left_ultrasonic_distance

    if step == 1:
        
        # right_error = right_ultrasonic.distance(MM) - initial_wall_distance
        # left_error = left_ultrasonic.distance(MM) - initial_wall_distance

        # print(left_ultrasonic.distance(MM))
        # print("Left Error:", left_error)
        if right_ultrasonic_distance >= initial_wall_distance:

            return True

    if step == 3:
        
        if next_pos == "A1" or next_pos == "A2" or next_pos == "A3" or next_pos == "B1" or next_pos == "C1" or next_pos == "A5" or next_pos == "B5" or next_pos == "C5":
            if left_ultrasonic_distance >= next_x_pos:
                print(left_ultrasonic_distance)
                return True
        else: 
            if right_ultrasonic_distance >= next_x_pos: 
                print(right_ultrasonic_distance)
                return True
        
    if step == 5 or step == 12:
        
        # right_error = right_ultrasonic.distance(MM) - line_wall_dist
        # left_error = left_ultrasonic.distance(MM) - line_wall_dist

        if next_pos == "A1" or next_pos == "A2" or next_pos == "B1" or next_pos == "C1" or next_pos == "A5" or next_pos == "B5" or next_pos == "C5":
            if left_ultrasonic_distance <= line_wall_dist:
                print(left_ultrasonic_distance)
                return True
        else: 
            if right_ultrasonic_distance <= line_wall_dist: 
                print(right_ultrasonic_distance)
                return True

    if step == 9:
        
        # right_error = right_ultrasonic.distance(MM) - line_wall_dist
        # left_error = left_ultrasonic.distance(MM) - line_wall_dist

        if next_pos == "A1" or next_pos == "A2" or next_pos == "B1" or next_pos == "C1" or next_pos == "A5" or next_pos == "B2" or next_pos == "B5" or next_pos == "C5" or next_pos == "C2":
            if left_ultrasonic_distance <= line_wall_dist:

                return True
        else: 
            if right_ultrasonic_distance <= line_wall_dist: 

                return True

    if step == 7:
        
        # right_error = right_ultrasonic.distance(MM) - deposit_dist
        # left_error = left_ultrasonic.distance(MM) - deposit_dist
        if next_pos == "A1" or next_pos == "A4" or next_pos == "B1" or next_pos == "B3" or next_pos == "C1" or next_pos == "C3":
            if left_ultrasonic_distance <= deposit_dist:

                print(left_ultrasonic_distance)

                return True
        else: 
            
            if left_ultrasonic_distance >= deposit_dist: 

                print(left_ultrasonic_distance)

                return True
        
    return False

def checkYUpdated():

    global initial_wall_distance, next_y_pos, line_wall_dist, step, deposit_dist

    if step == 2:
        
        # right_error = right_ultrasonic.distance(MM) - next_y_pos
        # left_error = left_ultrasonic.distance(MM) - next_y_pos

        if right_ultrasonic_distance >= next_y_pos:

            print("next y pos,", next_y_pos)
            print("Ultrasonic", right_ultrasonic_distance)

            return True
    
    if step == 6:
        
        if left_ultrasonic_distance <= return_y_dist:

            return True
        
    return False

def checkMoleHillReached():
    return False

########## ========== AUTO HANDLERS ========== ##########

def handleApproachComplete():

    global robotState

    robotState = ROBOT_COLLECTING

    print("APPROACH => COLLECT, 1")
    print("Step is: ", step)

def handleDepositComplete():

    global deposit_complete, robotState, step

    step += 1
    
    deposit_complete = False

    if lower_limitswitch.pressing() is not 1: 

        elevator_motor.spin(REVERSE, -50, RPM)

    elif lower_limitswitch.pressing == 1: 

        elevator_motor.stop

    robotState = ROBOT_FOLLOW_HEADING

    print("DEPOSIT => FOLLOW HEADING, 2")
    print("Step is: ", step)

def handleCollectionComplete():
    
    global step, robotState

    step += 1

    robotState = ROBOT_FACE_DIRECTION
    print("COLLECTION => FACE DIRECTION, 3")
    print("Step is: ", step)

def handleMoleHillReached():
    pass

def handleTurnComplete():

    global robotState
    
    if robotState == ROBOT_FACE_DIRECTION:
        
        if step == 2 or step == 6: 
            
            robotState = ROBOT_FOLLOW_LINE

            print("FACE DIRECTION => FOLLOW LINE, 4")
            print("Step is: ", step)
        
        if step == 3 or step == 5 or step == 7 or step == 9 or step == 12: 

            robotState = ROBOT_FOLLOW_HEADING 

            print("FACE DIRECTION => FOLLOW HEADING, 5")
            print("Step is: ", step)

def handleFruitFound():

    global robotState, step

    if robotState == ROBOT_SEARCHING: 

        robotState = ROBOT_APPROACHING

        print("SEARCHING => APPROACHING, 6")
        print("Step is: ", step)
    else:
        print("Handle Fruit Found Entrance State Error")
        print("Step is: ", step)

def handleFruitMissed():
    
    global robotState, step

    if robotState == ROBOT_SEARCHING:

        step = 12

        robotState = ROBOT_FACE_DIRECTION

        print("APPROACHING => FACE DIRECTION, 7")
        print("Step is: ", step)
    else: 
        print("Handle Fruit Found Entrance State Error")
        print("Step is: ", step)

def handleXUpdated():

    global step, robotState

    step += 1

    if step == 4:

        elevator_motor.spin_to_position(-400, DEGREES)

        robotState = ROBOT_SEARCHING

        print("FOLLOW HEADING => SEARCHING, 8")
        print("Step is: ", step)

    elif step == 8: 

        robotState = ROBOT_DEPOSITING
        print("Step is: ", step)
    
    else: 

        robotState = ROBOT_FACE_DIRECTION

        print("FOLLOW HEADING => FACING DIRECTION, 9")
        print("Step is: ", step)

def handleYUpdated():

    global step, robotState 

    step += 1

    robotState = ROBOT_FACE_DIRECTION

    print("FOLLOW HEADING => FACING DIRECTION, 10")
    print("Step is: ", step)

########## ========== TELEOP BINDINGS ========== ##########

def handleAButton():

    global robotState

    robotState = ROBOT_IDLE

    print("ROBOT IDLE")
    print("Step is: ", step)

def handleBButton():

    global robotState

    robotState = ROBOT_APPROACHING

    print("ROBOT APPROACHING")
    print("Step is: ", step)

def handleXButton():

    global robotState 
    
    robotState = ROBOT_FOLLOW_HEADING

    print("Robot State is: ", robotState)
    print("Step is: ", step)

def handleYButton():

    global robotState

    robotState = ROBOT_TELEOP

    print("ROBOT TELEOP")
    print("Step is: ", step)

def handleL1Button():

    elevator_motor.spin(FORWARD, 25)

def handleL2Button():

    elevator_motor.spin(REVERSE, 25)

def handleR1Button():

    global robotState

    robotState = ROBOT_FACE_DIRECTION

    print("left: ", left_linefollow.reflectivity())
    print("right: ", right_linefollow.reflectivity())

def handleR2Button():

    global robotState, error, desired_heading

    print("====================")
    print("\n")
    print("Right Ultrasonic Distance: ", right_ultrasonic_distance)
    print("Left Ultrasonic Distance: ", left_ultrasonic_distance)
    print("\n")
    print("====================")

    robotState = ROBOT_APPROACHING

########## ========== TELEOP HANDLERS ========== ##########

controller.buttonX.pressed(handleXButton)
controller.buttonA.pressed(handleAButton)
controller.buttonB.pressed(handleBButton)
controller.buttonY.pressed(handleYButton)
controller.buttonL1.pressed(handleL1Button)
controller.buttonL2.pressed(handleL2Button)
controller.buttonR1.pressed(handleR1Button)
controller.buttonR2.pressed(handleR2Button)

########## ========== MAIN LOOP ========== ##########

while True:

    if step == 10: 

        robotState = ROBOT_FOLLOW_HEADING

        queue_counter += 1

        step = 1
    
    if step == 13:

        step = 2

        queue_counter += 1

    update_queue()

    update_heading()

    update_ultrasonic()


