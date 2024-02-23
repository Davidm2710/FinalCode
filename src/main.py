# ---------------------------------------------------------------------------- #
#                                                                              #
#   Module:       main.py                                                      #
#   Author:       Vanessa                                                      #
#   Created:      1/31/2024, 10:11:33 AM                                       #
#   Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()
#defining motors
left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
arm_motor = Motor(Ports.PORT8, GearSetting.RATIO_36_1,True)
Wheel_Diameter = 4
CIRCUMFERENCE = math.pi * Wheel_Diameter  
RowDistance1 = 0
RowDistance2 = 44
RowDistance3 = 84
ColumnDistance1 = 11
ColumnDistance2 = 33
ColumnDistance3 = 55
LowBranchHeight = 200
MidBranchHeight = 300
HighBranchHeight = 400

imu = Inertial(Ports.PORT5)
FrontUltrasonic = Sonar(brain.three_wire_port.a)
DistanceToWall = FrontUltrasonic.distance(MM)
TopUltrasonic = Sonar(brain.three_wire_port.c)
DistanceToBranch = TopUltrasonic.distance(MM)
SideUltrasonic = Sonar(brain.three_wire_port.g)
SideDistance = SideUltrasonic.distance(MM)

## define the colors; we'll use the default sensitivity of 3.0
## you don't have to retrain the camera to use different sensitivities; you can
##just change the value here

#camera constants
Camera_Sensitivity = 2.5
Camera_ResolutionX = 316
Camera_ResolutionY = 212
ORANGE_FRUIT = Signature (3, 6641, 7821, 7231, -2221, -1859, -2040, Camera_Sensitivity, 0)
LIME = Signature (1, -6097, -5119, -5608, -3305, -2567, -2936, Camera_Sensitivity, 0)
LEMON = Signature (2, 2685, 3133, 2909, -3517, -3161, -3339, Camera_Sensitivity, 0)
GRAPEFRUIT = Signature(4, 5137, 5535, 5336, 1175, 1683, 1429, Camera_Sensitivity, 0)
## define the camera on port 3; the library says the colors are optional 
VisionB = Vision (Ports.PORT3, 40, ORANGE_FRUIT, LIME, LEMON, GRAPEFRUIT)
VisionF = Vision (Ports.PORT4, 40, ORANGE_FRUIT, LIME, LEMON)
Fruit = [ORANGE_FRUIT, LIME, LEMON]
Held_Fruit_Type = 'No Fruit'
Held_Fruit_Num = 0
Max_Fruit = 4
Trees_Checked = 0
Bucket1 = 0
Bucket2 = 0
Bucket3 = 0

ROBOT_IDLE = 0
ROBOT_DRIVING_ORCHARD = 1
ROBOT_LOOT_TREE = 2
ROBOT_GRAB_POSITIONING = 3
ROBOT_GRAB_CIRCLE = 4
ROBOT_SEARCHING = 5
ROBOT_SEARCH_SPECIFIC = 6
ROBOT_CENTERING = 7
ROBOT_DRIVING_FRUIT = 8
ROBOT_GRAB_FRUIT = 9
ROBOT_DRIVING_BACK = 10
ROBOT_SEARCH_BUCKET = 11
ROBOT_B_CENTERING = 12
ROBOT_DRIVING_BUCKET = 13
ROBOT_DUMP_FRUIT = 14
Robot_State = 0
SOFT = 1
imu.calibrate()
while imu.is_calibrating():sleep(50, MSEC)
imu.reset_heading()
def go_inches_straight(distance, speed, targetHeading):
    global CIRCUMFERENCE
    left_motor.set_position(0,TURNS)
    right_motor.set_position(0,TURNS)
    while left_motor.position(TURNS) < distance*5/CIRCUMFERENCE and right_motor.position(TURNS) < distance*5/CIRCUMFERENCE:
        CurrentHeading = imu.heading()
        if targetHeading == 0:
            if CurrentHeading > 180:
                CurrentHeading -= 360 
        errorHeading = targetHeading - CurrentHeading
        effortHeading = errorHeading * 1
        left_motor.spin(FORWARD, speed *  5 + effortHeading) # wait = blocking 

        right_motor.spin(FORWARD, speed * 5 - effortHeading)
    
def handleButtonPress():
    global Robot_State
    if(Robot_State == ROBOT_IDLE):
        print("DRIVING TO TREE")
        Robot_State = ROBOT_DRIVING_ORCHARD
    else:
        Robot_State = ROBOT_IDLE
        print("STOPPPPPP!!!!")

button5 = Bumper(brain.three_wire_port.f)
button5.pressed(handleButtonPress)
Robot_State = ROBOT_IDLE

SOFT = 1
imu.calibrate()
while imu.is_calibrating():sleep(50, MSEC)
imu.reset_heading()

# Rotate until in target, with shortest path
def rotate(target_angle):
    while True:
        current_heading = imu.heading()
        angle_difference = target_angle - current_heading

        # Adjust for circular nature of headings
        if angle_difference > 180:
            angle_difference -= 360
        elif angle_difference < -180:
            angle_difference += 360

        # Check if within the soft limit
        if -SOFT <= angle_difference <= SOFT:
            break
          # Stop rotating if within tolerance

        # Determine rotation direction
        if angle_difference > 0:
            # Rotate clockwise
            left_motor.spin(FORWARD)
            right_motor.spin(REVERSE)
        else:
            # Rotate counter-clockwise
            left_motor.spin(REVERSE)
            right_motor.spin(FORWARD)
          # For debugging
        sleep(50)  # Small delay for responsiveness

    # Stop motors once the target is reached
    left_motor.stop()
    right_motor.stop()

def DriveToWall(TargetDistance):
    global DistanceToWall
    DistanceToWall = FrontUltrasonic.distance(MM)
    while DistanceToWall > TargetDistance + 50:
        left_motor.spin(FORWARD,40 * 5)
        right_motor.spin(FORWARD,40 * 5)
        DistanceToWall = FrontUltrasonic.distance(MM)
        while DistanceToWall > TargetDistance:
            left_motor.spin(FORWARD,20 * 5)
            right_motor.spin(FORWARD,20 * 5)
            DistanceToWall = FrontUltrasonic.distance(MM)

    left_motor.stop()
    right_motor.stop()

def identify_third_bucket_fruit(firstBucket_fruit, secondBucket_fruit):
    # Define the three possible fruits
    global Fruit

    # Remove the known fruits from the list

    # Remove the known fruits from the list
    remaining_fruits = [fruit for fruit in Fruit if fruit != firstBucket_fruit and fruit != secondBucket_fruit]

    # The remaining fruit is the fruit of the third bucket
    return remaining_fruits[0]

def handleObjects():
    global Held_Fruit_Type
    global Robot_State
    VisionB.take_snapshot(Fruit)
    if Held_Fruit_Type == "No Fruit":
        if Fruit == ORANGE_FRUIT:
            print("Orange detected")
            Held_Fruit_Type = ORANGE_FRUIT
            Robot_State = ROBOT_CENTERING
        elif Fruit == LIME:
            print("lime detected")
            Held_Fruit_Type = LIME
            Robot_State = ROBOT_CENTERING
        elif Fruit == LEMON:
            print("lemon detected")
            Held_Fruit_Type = LEMON
            Robot_State = ROBOT_CENTERING
        else:
            print("YOU'RE CRAZY THERE IS NO FRUIT")
    elif Held_Fruit_Type == ORANGE_FRUIT:
        if Fruit == ORANGE_FRUIT:
            print("Another Orange detected")
            Held_Fruit_Type = ORANGE_FRUIT
            Robot_State = ROBOT_CENTERING  
        else:
            Robot_State = ROBOT_SEARCH_SPECIFIC
    elif Held_Fruit_Type == LIME:
        if Fruit == LIME:
            print("Another Lime detected")
            Held_Fruit_Type = LIME
            Robot_State = ROBOT_CENTERING  
        else:
            Robot_State = ROBOT_SEARCH_SPECIFIC
    elif Held_Fruit_Type == LEMON:
        if Fruit == LEMON:
            print("Another Lemon detected")
            Held_Fruit_Type = LEMON
            Robot_State = ROBOT_CENTERING  
    else:
        Robot_State = ROBOT_SEARCH_SPECIFIC


ROTATION_SWITCH = 115
SPEED = 80
IN_SPEED = SPEED / 4
OUT_C_DEG = 14400
IN_C_DEG = 3600
S = 40
T = 330
D = 2610

H1 = 250

H2 = 350


def position(Reverse):
    SideDistance = SideUltrasonic.distance(MM)
    if (Reverse):
        right_motor.spin_for(REVERSE, D, DEGREES, S, RPM, False)
        left_motor.spin_for(REVERSE, D, DEGREES, S, RPM, True)

    else:
        while SideDistance > 130:
            SideDistance = SideUltrasonic.distance(MM)
            print("SIDE DISTANCE IS",SideDistance)
            right_motor.spin(FORWARD, 20 * 5)
            left_motor.spin(FORWARD, 20 * 5)
        left_motor.stop()
        right_motor.stop()
    wait(1000)

def sensorHook():

    start = left_motor.position() 

    DistanceToBranch = TopUltrasonic.distance(MM)

    print(DistanceToBranch)

    while not (50 < DistanceToBranch <= 400):

        left_motor.spin(FORWARD, IN_SPEED)

        right_motor.spin(FORWARD, SPEED)

        DistanceToBranch = TopUltrasonic.distance(MM)

    DistanceToBranch = TopUltrasonic.distance(MM)    


    if (DistanceToBranch <= H1): 

        arm_motor.spin_to_position(160*5,DEGREES,30) 

        print("low", DistanceToBranch)

    elif(H1 < DistanceToBranch <= H2): 

        arm_motor.spin_to_position(123*5,DEGREES,30)

        print("mid", DistanceToBranch)

    elif(H2 <= DistanceToBranch < 500): 

        arm_motor.spin_to_position(95*5,DEGREES,30)

        print("high", DistanceToBranch)

    else:

        print("too high", DistanceToBranch)

    for items in Fruit:
                    print(items)
                    Snap2 = VisionB.take_snapshot(items)
                    print(Snap2)
                    if Snap2:
                        Held_Fruit_Type = items
                    wait(100)
    print("HOLDING ", Held_Fruit_Type)

    new = left_motor.position()
    wait(500)

    while not (new - 5 <= start <= new + 5):

        new = left_motor.position()

        left_motor.spin(REVERSE, IN_SPEED)

        right_motor.spin(REVERSE, SPEED)

    left_motor.stop()

    right_motor.stop()

def tree():

    arm_position = arm_motor.position()

    if (arm_position / 5 <= ROTATION_SWITCH):
        print("ARM POSTION IS",arm_motor.position()/5)
        print("going backwards")
        left_motor.spin_for(REVERSE, IN_C_DEG, DEGREES, IN_SPEED, RPM, False)

        right_motor.spin_for(REVERSE, OUT_C_DEG, DEGREES, SPEED, RPM, True)

    else:
        print("ARM POSTION IS",arm_motor.position()/5)
        print("going Forwards")
        left_motor.spin_for(FORWARD, IN_C_DEG, DEGREES, IN_SPEED, RPM, False)

        right_motor.spin_for(FORWARD, OUT_C_DEG, DEGREES, SPEED, RPM, True)


'''
def handleBucketSearch():
    global Held_Fruit_Type
    global Robot_State
    for items in Fruit:
        objects = VisionB.take_snapshot(items)
        G_Fruit = VisionB. take_snapshot(GRAPEFRUIT)
        if objects and G_Fruit:
            if Held_Fruit_Type == ORANGE_FRUIT:
                if objects == ORANGE_FRUIT and G_Fruit:
                    print("Orange Bucket Detected")
                    Robot_State = ROBOT_B_CENTERING
            elif Held_Fruit_Type == LEMON:
                if objects == LEMON and G_Fruit:
                    print("Lemon Bucket Detected")
                    Robot_State = ROBOT_B_CENTERING
            if Held_Fruit_Type == LIME:
                if objects == LIME and G_Fruit:
                    print("Orange Bucket Detected")
                    Robot_State = ROBOT_B_CENTERING
'''   

## start in the idle Robot_State
while True: 
    
    while Robot_State == ROBOT_IDLE:
        print("idle!!!")
        left_motor.stop()
        right_motor.stop()
        arm_motor.spin_to_position(160 * 5,DEGREES,30)

    while Robot_State == ROBOT_DRIVING_ORCHARD:
        print("driving orchard!!!")
        arm_motor.spin_to_position(155 * 5 ,DEGREES,10)
        if Trees_Checked < 3:
            #drive to row 1
            IN_ROW = 1
            go_inches_straight(RowDistance1,30,0)
            #turn to 270 degrees
            rotate(270)
            if Trees_Checked == 0:
                #drive to tree 1
                go_inches_straight(ColumnDistance1,30,270)
            elif Trees_Checked == 1:
                #drive to tree 2
                go_inches_straight(ColumnDistance2,30,270)
            elif Trees_Checked == 2:
                #drive to tree 3
                go_inches_straight(ColumnDistance3,30,270)
        elif Trees_Checked < 6 and Trees_Checked > 2:
            #drive to row 2 
            go_inches_straight(RowDistance2,30,0)
            IN_ROW = 2
            #turn 90 degrees
            rotate(270)
            if Trees_Checked == 3:
                #drive to tree 4
                go_inches_straight(ColumnDistance1,30,270)
            elif Trees_Checked == 4:
                #drive to tree 5
                go_inches_straight(ColumnDistance2,30, 270)
            elif Trees_Checked == 5:
                #drive to tree 6
                go_inches_straight(ColumnDistance3,30,270)
        elif Trees_Checked < 9:
            #drive to row 3 
            go_inches_straight(RowDistance3,30,0)
            IN_ROW = 3
            #turn 90 degrees
            rotate(270)
            if Trees_Checked == 6:
                #drive to tree 7
                go_inches_straight(ColumnDistance1,30,270)
            elif Trees_Checked == 7:
                #drive to tree 8
                go_inches_straight(ColumnDistance2,30,270)
            elif Trees_Checked == 8:
                #drive to tree 9
                go_inches_straight(ColumnDistance3,30,270)
        else:
            Robot_State = ROBOT_IDLE
            print("YAY YOU GOT ALL THE FRUIT")
        rotate(0)
        #DONT FORGET TO CHANGE BACK THIS STATE IT SHOULD BE ROBOT_SEARCHING
        Trees_Checked += 1
        print("Trees Checked is" , Trees_Checked)
        wait(1000)
        Robot_State = ROBOT_LOOT_TREE

    while Robot_State == ROBOT_LOOT_TREE:
        print("going to tree!!!")
        arm_motor.spin_to_position(160 * 5, DEGREES, 30)
        #go to the side of the tree
        position(False)
        wait(1000)
        #figure out hook positioning by driving forward and back until sense height
        sensorHook()
        #drive around the tree collecting fruit
        tree()
        #drive back from side of tree
        position(True)
        Robot_State = ROBOT_DRIVING_BACK


    '''
    while Robot_State == ROBOT_GRAB_POSITIONING:
        print("grabbing fruit!!!!")
        DistanceToBranch = TopUltrasonic.distance(MM)
        if DistanceToBranch < LowBranchHeight:
            arm_motor.spin_to_position(153)
        elif DistanceToBranch < MidBranchHeight:
            arm_motor.spin_to_position(123)
        elif DistanceToBranch < HighBranchHeight:
            arm_motor.spin_to_position(106)

        else:
            left_motor.spin_for(FORWARD, .5, TURNS , 20*5)
            right_motor.spin_for(FORWARD, .5, TURNS , 20*5)
    
    while Robot_State == ROBOT_SEARCHING:
        left_motor.spin(FORWARD, 30)
        right_motor.spin(REVERSE, 30)
        for items in Fruit:
            objects = VisionB.take_snapshot(items)
            if objects and Robot_State == ROBOT_SEARCHING:
                handleObjects()      



    while Robot_State == ROBOT_SEARCH_SPECIFIC:
        left_motor.spin(FORWARD, 30)
        right_motor.spin(REVERSE, 30)
        objects = VisionB.take_snapshot(Held_Fruit_Type)
        if objects and Robot_State == ROBOT_SEARCH_SPECIFIC:
            handleObjects() 
    '''

    offset = 15
    Center_Camera = Camera_ResolutionX / 2
    '''
    while Robot_State == ROBOT_CENTERING:
        objects = VisionB.take_snapshot(Held_Fruit_Type)
        if VisionB.largest_object().centerX > Center_Camera + offset: #half of the camera resolution
            right_motor.spin(REVERSE,30)
            left_motor.spin(FORWARD, 30)
        elif VisionB.largest_object().centerX < Center_Camera - offset: #half of the camera resolution
            right_motor.spin(FORWARD,30)
            left_motor.spin(REVERSE, 30)
        else:
            print("ROBOT CENTERED")
            right_motor.stop()
            left_motor.stop()

            Robot_State = ROBOT_DRIVING_FRUIT
        wait(50)


    while Robot_State == ROBOT_DRIVING_FRUIT:
        FRUIT_TARGET_HEIGHT = 205
        FRUIT_TARGET_SIDE = Center_Camera
        objects = VisionB.take_snapshot(Held_Fruit_Type)
        if VisionB.largest_object().height < FRUIT_TARGET_HEIGHT: #change to desired number to change distance away
            errorForward = FRUIT_TARGET_HEIGHT - VisionB.largest_object().height
            effortForward = errorForward * 2
            errorSIDE = VisionB.largest_object().centerX - FRUIT_TARGET_SIDE
            effortSIDE = errorSIDE * 1
            left_motor.spin(FORWARD, effortForward + effortSIDE)
            right_motor.spin(FORWARD, effortForward + effortSIDE)
            sleep(20)
        else:
            print("ARRIVED AT FRUIT")
            left_motor.stop()
            right_motor.stop()
            Robot_State = ROBOT_GRAB_FRUIT


    while Robot_State == ROBOT_GRAB_FRUIT:
        Initial_Torque = arm_motor.torque()    
        Torque_Diff = .25 #change when have final torque measurements
        while Initial_Torque < Initial_Torque + Torque_Diff :
            arm_motor.spin(REVERSE,15)
        print("TOUCHED FRUIT YAY")
        left_motor.spin_for(REVERSE, 10, TURNS, 100)
        right_motor.spin_for(REVERSE, 10, TURNS, 100)
        Held_Fruit_Num += 1
        wait(2000)
        arm_motor.spin_to_position(150, DEGREES, 30)
        if Held_Fruit_Num < Max_Fruit:
            Robot_State = ROBOT_SEARCH_SPECIFIC
        else:
            Robot_State = ROBOT_SEARCH_BUCKET
    '''

    while Robot_State == ROBOT_DRIVING_BACK:
        print("driving back!!!")

        if Trees_Checked == 1 or 4 or 7:
            rotate(90)
        else:
            rotate(270)
        wait(500)
        #drive until hit wall
        DriveToWall(150)
        rotate(180)
        arm_motor.spin_to_position(140*5,DEGREES,30)
        if IN_ROW == 3:
            go_inches_straight(RowDistance3 - 5,30, 180)
        elif IN_ROW == 2:
            go_inches_straight(RowDistance2 - 5,30, 180)
        IN_ROW = 0
        rotate(180)
        Robot_State = ROBOT_DRIVING_BUCKET
            
    while Robot_State == ROBOT_DRIVING_BUCKET:
        arm_motor.spin_to_position(140*5,DEGREES,30)
        print("WE LOOKING FOR BUCKETS")
        if Bucket1 == 0 and Bucket2 == 0 and Bucket3 == 0:
            rotate(180)
            if Trees_Checked == 1 or 4 or 7:
                for items in Fruit:
                    print(items)
                    Snap1 = VisionB.take_snapshot(items)
                    print(Snap1)
                    if Snap1:
                        Bucket1 = items
                    wait(1000)

                print("BUCKET 1 IS", Bucket1)
                rotate(270)
                go_inches_straight(ColumnDistance2,30,270)
                rotate(180)
                for items in Fruit:
                    print(items)
                    Snap2 = VisionB.take_snapshot(items)
                    print(Snap2)
                    if Snap2:
                        Bucket2 = items
                    wait(1000)
                print("BUCKET 2 IS", Bucket2)
                Bucket3 = identify_third_bucket_fruit(Bucket1, Bucket2) 
                print("BUCKET 3 IS", Bucket3)
            else:
                for items in Fruit:
                    print(items)
                    Snap3 = VisionB.take_snapshot(items)
                    print(Snap3)
                    if Snap3:
                        Bucket3 = items
                    wait(1000)
                    print("BUCKET 3 IS", Bucket3)
                rotate(90)
                go_inches_straight(ColumnDistance2,30,90)
                rotate(180)
                for items in Fruit:
                    print(items)
                    Snap2 = VisionB.take_snapshot(items)
                    print(Snap2)
                    if Snap2:
                        Bucket2 = items
                    wait(1000)
                print("BUCKET 2 IS", Bucket2)
                Bucket1 =  identify_third_bucket_fruit(Bucket3, Bucket2)
            if Bucket1 == Held_Fruit_Type:
                rotate(90)
                DriveToWall(150)
            elif Bucket3 == Held_Fruit_Type:
                rotate(270)
                DriveToWall(150)

        else:
            if Trees_Checked == 1 or 4 or 7:
                if Bucket3 == Held_Fruit_Type:
                    rotate(270)
                    DriveToWall(150)
                elif Bucket2 == Held_Fruit_Type:
                    rotate(270)
                    go_inches_straight(ColumnDistance2,30,270)
                
            else:
                if Bucket1 == Held_Fruit_Type:
                    rotate(90)
                    DriveToWall(150)
                elif Bucket2 == Held_Fruit_Type:
                    rotate(90)
                    go_inches_straight(ColumnDistance2,30,90)
        rotate(170)
        Robot_State = ROBOT_SEARCH_BUCKET
                


        
                


    while Robot_State == ROBOT_SEARCH_BUCKET: 
        print("searching for bucket!!!") 
        left_motor.spin(FORWARD, 30)
        right_motor.spin(REVERSE, 30)
        for items in Fruit:
            HeldObject = VisionB.take_snapshot(Held_Fruit_Type)
            G_Fruit = VisionB. take_snapshot(GRAPEFRUIT)
            if HeldObject and G_Fruit and Robot_State == ROBOT_SEARCH_BUCKET:
                print(Held_Fruit_Type,"BUCKET FOUND")
                Robot_State = ROBOT_B_CENTERING


    while Robot_State == ROBOT_B_CENTERING:
        print("centering robot!!!")
        HeldObject = VisionB.take_snapshot(Held_Fruit_Type)
        if VisionB.largest_object().centerX > Center_Camera + offset: #half of the camera resolution
            right_motor.spin(REVERSE,30)
            left_motor.spin(FORWARD, 30)
        elif VisionB.largest_object().centerX < Center_Camera - offset: #half of the camera resolution
            right_motor.spin(FORWARD,30)
            left_motor.spin(REVERSE, 30)
        else:
            print("ROBOT CENTERED")
            right_motor.stop()
            left_motor.stop()
            Robot_State = ROBOT_DUMP_FRUIT
        wait(500)

# Old drive to bucket code
        """FRUIT_TARGET_HEIGHT = 180
        FRUIT_TARGET_SIDE = Center_Camera
        objects = VisionB.take_snapshot(Held_Fruit_Type)
        if VisionB.largest_object().height < FRUIT_TARGET_HEIGHT: #change to desired number to change distance away
            errorForward = FRUIT_TARGET_HEIGHT - VisionB.largest_object().height
            effortForward = errorForward * 2
            errorSIDE = VisionB.largest_object().centerX - FRUIT_TARGET_SIDE
            effortSIDE = errorSIDE * 1
            left_motor.spin(FORWARD, effortForward + effortSIDE)
            right_motor.spin(FORWARD, effortForward + effortSIDE)
            sleep(20)
        else:
            print("ARRIVED AT BUCKET")
            left_motor.stop()
            right_motor.stop()
            Robot_State = ROBOT_DUMP_FRUIT
"""
    while Robot_State == ROBOT_DUMP_FRUIT:
        print("dumping fruit!!!")
        #spin 180 degrees
        DriveToWall(80)
        rotate(0)
        wait(1000)
        arm_motor.spin_to_position(0 * 5, DEGREES, 20)
        print("FRUIT DUMPED")
        Held_Fruit_Num = 0
        Held_Fruit_Type = "No Fruit"
        arm_motor.spin_to_position(140 *5,DEGREES,30)
        #turn 90 degrees clockwise
        go_inches_straight(-5,30,180)
        rotate(90)
        wait(500)
        #drive  until hit wall
        DriveToWall(150)
        #turn 90 degrees counterclockwise
        rotate(0)
        Robot_State = ROBOT_DRIVING_ORCHARD