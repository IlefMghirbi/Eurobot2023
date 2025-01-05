#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from enum import Enum
import copy
import smach_ros
from time import sleep
import time
import math
import pigpio
import subprocess
import DStarLite
from DStarLite import ox , oy , ROBOT_Radius
from PATH_PLANNER_Bilel import optimizer
#from PathOptimizer import Path_Optimizer
import numpy as np
from sensor_msgs.msg import LaserScan
   
#from StateMachMec import *
from Strategy_SMach import *
from MasterConfig import *
import ScreenConfig as ScreenConfig

import drivers
import RPi.GPIO as GPIO
import time
from encoder import Encoder
import serial
from sys import version_info

##################################### DECLARE VARIABLES HERE ####################################

################### NAVIGATION VARIABLES ##################

GPIO.cleanup()

ROS_Minimal_Sleep_Time = 0.1
MASTER_First_Time_Publishing_Targets                = 1
MASTER_FIRST_TIME_RECIEVING_COORDIANTES             = 1 
PATH_PLANNED_bool                                   = 0
MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool           = 0
MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec           = 0
MASTER_SUDDEN_STOP_FRONT_RANGE_m                    = 0.7
MASTER_SUDDEN_STOP_AJNEB_RANGE_m                    = 0.35
MASTER_SUDDEN_STOP_BACK_RANGE_m                     = 0.1
MASTER_SUDDEN_STOP_RANGE_m                          = MASTER_SUDDEN_STOP_FRONT_RANGE_m
MASTER_SUDDEN_STOP_REDUCED_FRONT_RANGE_m            = 0.4
MASTER_SUDDEN_STOP_REDUCED_AJNEB_RANGE_m            = 0.25
MASTER_SUDDEN_STOP_REDUCED_BACK_RANGE_m             = 0.1
MASTER_MAXIMUM_VELOCITY                             = 0.8         
MASTER_SUDDEN_STOP_REDUCED_RANG_TIME                = 4
Closest_Obstacle_Range                              = 0
Closest_Obstacle_Range_OLD                          = 0 
CONVERSION_FROM_METRE_TO_PATH_PLANNER               = 20
CONVERSION_FROM_PATH_PLANNER_TO_MM                  = 50 # 1000mm / 20

# MASTER Orders and NAVIGATOR Status

MASTER_Navigator_MessageRecieved            = 0
MASTER_Navigator_Stop_Flag                  = 1
MASTER_Navigator_Indicator_WaitForOrders    = 2
MASTER_Navigator_Can_GO_Forward             = 3
MASTER_Navigator_Can_GO_Backward            = 4
MASTER_Navigator_Still_IDLE                 = 5
MASTER_Navigator_No_Rotate_After            = 1000 
MASTER_PermissionTo_Navigator = []
TargetStack_Indicator = 1
STRATEGY_CAN_PUBLISH_INDICATORS = 1 


# MASTER EXECUTE TASK INDICATOR

MASTER_WONT_Execute_Task          = 0
MASTER_WILL_Execute_Task          = 1
MASTER_WILL_Parallel_Execute_Task = 2
MASTER_PATH_PLANNER_TARGET        = 3  


# object detection boolean flag
Lidar_ObstacleDetected_b = False


# TARGETS Variables and Arrays
Targets_Publisher_Timeout_Start = 0 
Nav_XYTargets_Numbers = 4
Nav_CurrentTargetNumber_u8 = 0
MASTER_CurrentTargets = []
MASTER_TARGET_INDICATOR_PILE   = 1
MASTER_TARGET_INDICATOR_PLATE  = 2 
MASTER_TARGET_INDICATOR_INTERM = 0   
MASTER_TARGET_INDICATOR_BackFull = 99
"""MASTER_ALL_XYTHETA_Targets_BLUE_arr = []
MASTER_ALL_XYTHETA_Targets_GREEN_arr = [ 775.0  , 225.0 , #gateau 775.0 225.0
                                         1125.0  , 725.0 , #gateau 1125.0  725.0
                                         525.0  , 1000.0, #pt interm
                                         300.0 , 1750.0   #plateau 
                                        ]
MASTER_ALL_TASK_INDICATORS_GREEN_arr = [MASTER_TARGET_INDICATOR_PILE   , 
                                        MASTER_TARGET_INDICATOR_PILE   ,
                                        MASTER_TARGET_INDICATOR_INTERM ,
                                        MASTER_TARGET_INDICATOR_PLATE ,
                                        MASTER_TARGET_INDICATOR_BackFull
                                        ]"""


MASTER_PathPlannerTargets=[]
# Navigator Feeedback variables : 
Nav_CurrentX_mm_d = 0.0
Nav_CurrentY_mm_d = 0.0
Nav_OldX_mm_d = 0.0
Nav_OldY_mm_d = 0.0
Nav_CurrentAngle_deg_d = 0.0
Velocity_Delay_Counter = 0
Nav_CurrentVelocity_d = 0.0  
MASTER_TargetsArray = []
MASTER_FlagsArray = []
MASTER_TargetIncr = 0

MASTER_IsPathPlanned = False

# TARGET GENERATOR:
CakeX=200 #Strategie
CakeY=200 #Strategie

NextCenterX=0 #Target Center X
NextCenterY=0 #Target Center Y

DISTDOORMIDPOINT=85 #Constant value
DISTCENTERMIDPOINT=143 #Constant value


class DoorToFill (enumerate) :
    Right=1
    Left=2
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ RASPBERRY CONFIGURATIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#    STEPPER     : 

PULSE=21  # Pompe GPIO Pin
DIR = 20  # Direction GPIO Pin
STEP = 18 # Step GPIO Pin
UPPER_LIMIT_SWITCH = 21 # fin de course Robot1 GPIO Pin
LOWER_LIMIT_SWITCH = 26 # fin de course Robot1 GPIO PIN
CW = 1
CCW = 0
SPR = 200 # 360/1.8

RAYON_POULIE_MM = 6.4
NUM_STEPS_REV = SPR*16
UPPER_INIT_POS= 221.5
Current_StepperPos_Mm = 0

#       SCREEN        : 

currentScreen = "none"
currentCursorIndex = 0
MyColor=""
PlateConfig=""
MyStrategy=""
Confirmation = 0
FirstLoop=0
CurrentScore = 0

GPIO.setmode(GPIO.BCM)
prevTime=0
BUTTON_PIN=14

FRONT_INFRAROUGE_UPPER = 0
FRONT_INFRAROUGE_MID =   0
FRONT_INFRAROUGE_LOWER = 0
BACK_INFRAROUGE_UPPER =  0
BACK_INFRAROUGE_MID   =  0
BACK_INFRAROUGE_LOWER =  0

GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


JACK_PIN = 24
GPIO.setup(JACK_PIN,GPIO.IN,GPIO.PUD_UP)
GPIO.add_event_detect(JACK_PIN,GPIO.RISING,bouncetime=200)
PY2 = version_info[0] == 2   #Running Python 2.x?

ServoBot     ={"ServoBasket":[3,"ServoD2",6000],"ServoBasketDoorRight":[4,"ServoS8",10000],
		"ServoBasketDoorLeft":[2,"ServoS9",1500],"StealServoFrontRight":[5,"ServoS11",2600],
		"StealServoFrontLeft":[6,"ServoS6",9300], "StealServoBackRight":[0,"ServoS4",2400],
		"StealServoBackLeft":[1,"ServoS3",10000]}  #frontleft 9600 , frontright 2300                                                           
ServoCharacs ={"ServoR0":[1432,10432,42.86],"ServoR1":[1432,10532,43.32],"ServoS0":[1600,10100,43.36],"ServoS1":[1000,10850,46.46],"ServoS2":[1450,10550,43.32],"ServoS3":[1900,10550,43.82],"ServoS4":[1450,11000,44.8],"ServoS5":[1450,10500,44.6],"ServoS6":[1450,10500,44.6],"ServoS7":[1450,10500,44.6],"ServoS8":[1450,10500,45.7],"ServoS9":[1450,10500,44.6],"ServoS10":[1450,10500,43.7],"ServoS11":[1450,10500,45.7],"ServoS12":[1450,10500,45.7],"ServoS13":[1450,10500,45.7],"ServoS14":[1450,10500,44.6],"ServoS15":[1450,10500,45.7],"ServoS16":[1550,10500,43.2],"ServoD2":[1300,10700,43.7]}

LineInExecution2=1

TIME_TO_COMPLETE = 0
TIME_TO_DEPOSE = 0.5

Prev_Scissor_LevelRight=0
Prev_Scissor_LevelLeft=0

FrontRes= ReservoirRobot2(Color.vide_en,Color.vide_en,Color.vide_en,0,ReservoirRobot2Side.FrontReservoir_en)
BackRes = ReservoirRobot2(Color.vide_en,Color.vide_en,Color.vide_en,0,ReservoirRobot2Side.BackReservoir_en)

#     INFRAROUGE   # 
RES_DOWN_RIGHT_INFRAROUGE_UPPER = 22
RES_DOWN_RIGHT_INFRAROUGE_MID   =  23
RES_DOWN_RIGHT_INFRAROUGE_LOWER = 24
RES_DOWN_LEFT_INFRAROUGE_UPPER  =  25
RES_DOWN_LEFT_INFRAROUGE_MID    =  26
RES_DOWN_LEFT_INFRAROUGE_LOWER  =  21

################################ STRATEGY VARIABLES #######################

# SEARCH REQUEST : 

MASTER_SearchRequest = 1

MASTER_FirstCommunicateStrategy = True

MASTER_ContainerRight = 1
MASTER_ContainerLeft = 2 

DoorIndicator = MASTER_ContainerLeft



############################# STATE CLASSES START HERE ##################################


'''
* Brief * : This class will be responsible of establishing communication for the first time with Navigation Node
* OUTPUT * : The state will be looping on itself until navigator sends back the trash value and then goes to state preparation

Navigator state is a T = 0 Idle, establishing communication, it will go to GOING TO RESPOND ==> READY and finally got back to Idle so we can use Idle state in robot status callback AGAIN !  
'''


class Idle(smach.State):  #TODO: INTEGRATE COMMUNICATION LORA

    global MASTER_CurrentMachineState_en
    global MASTER_CurrentMasterState_en
    global MASTER_CurrentNavigatorState_en
    global ServoBot
    global RisingEdge_variable
    global MASTER_FlagsArray

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Idle', 'Go to: Preparation']
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en 
        global MASTER_CurrentNavigatorState_en
        global MASTER_First_Time_Publishing_Targets
        global Targets_Publisher_Timeout_Start

        
        print (MASTER_CurrentNavigatorState_en)
        MASTER_FlagsArray = [100]
        if (    MASTER_First_Time_Publishing_Targets          ) :
                       
            vMASTER_Orders_Publisher(MASTER_FlagsArray)
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en
            print("published orders init")
            sleep(ROS_Minimal_Sleep_Time + 0.2)
            vMASTER_Orders_Publisher(MASTER_FlagsArray)
            sleep(ROS_Minimal_Sleep_Time + 0.2)
            Targets_Publisher_Timeout_Start = time.time()
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en
            MASTER_First_Time_Publishing_Targets = 0
           
            servo=Controller() 
            for key,value in ServoBot.items():
                servo.setTarget(value[0],value[2])
            

            Stepper_vInit()
            Stepper_vTurn(Stepper2Height.BasketHeight_en.value)
            
            

        # Check Timeout and republish if timeout 
        
        if ( MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en): 
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Preparation_en
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorIdle_en
            MASTER_First_Time_Publishing_Targets = 1
            return 'Go to: Preparation'
        elif (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en):
            print("going to respond") 
            if( ( time.time() - Targets_Publisher_Timeout_Start ) > 1  ):
                Targets_Publisher_Timeout_Start = time.time()
                vMASTER_Orders_Publisher(MASTER_FlagsArray) 
                sleep(ROS_Minimal_Sleep_Time + 0.1)
                print("published orders")
            sleep(ROS_Minimal_Sleep_Time + 0.3 )     
            return 'Go to: Idle'

'''
* BRIEF * This state will be responsible of ROBOTS PREPARATION SUCH AS POSITIONNING and Getting configurations
'''
class Preparation(smach.State):  # TODO: needs lcd code + MECHANISMS INITIALIZATION FUNCTION
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Preparation',
                                             'Go to: WaitForSignal'])

    def execute(self, userdata):
        global MyColor
        global Targets_Publisher_Timeout_Start
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global My_plates
        global Opponent_Plates
        global PLATE
        global Confirmation
        global FirstLoop
        global Nav_CurrentX_mm_d , Nav_CurrentY_mm_d , Nav_CurrentAngle_deg_d 
        global MASTER_FlagsArray
        global currentScreen
        ConfigArray = Int8MultiArray()


        # STATES:  DANS L'ORDRE KIMA FEL ECRAN
        # STATE GET COLOR
        # STATE GET STRATEGY
        # STATE GET LOCALIZATION
        # STATE GET PLATEAU
        # ACQUIRE ALL THE VARIABLES INSIDE THE STATE MACHINE AND THEN MOVE TO IF COLOR ....

        if FirstLoop==0:
            ScreenConfig.welcomeInit()
            print('welcomeInit')
            start_config=""
            
        #GPIO.add_event_detect(13,GPIO.RISING,callback=buttonClicked,bouncetime=200)
            GPIO.add_event_detect(BUTTON_PIN,GPIO.RISING,callback=ScreenConfig.buttonClicked,bouncetime=200)
            e1 = Encoder(15, 23, ScreenConfig.valueChanged)
            FirstLoop=1

        sleep(0.5)
        if (Confirmation == 1):
            #GPIO.cleanup()
            if (MyColor == 'BLUE'):
                Nav_CurrentX_mm_d = 1100.0
                Nav_CurrentY_mm_d = 2000.0 - 240.0
                Nav_CurrentAngle_deg_d = 180.0 
                if (MyStrategy == 'Hard'):
                    ConfigArray.data = [1,1]
                elif (MyStrategy == 'Medium'):
                    ConfigArray.data = [1,2]
                elif (MyStrategy == 'Easy'):
                    ConfigArray.data = [1,3]
            elif (MyColor == 'GREEN'):
                Nav_CurrentX_mm_d = 900.0 + 285.0
                Nav_CurrentY_mm_d = 258.0
                Nav_CurrentAngle_deg_d = 180.0 
                if(MyStrategy == 'Hard'):
                    ConfigArray.data = [2,1]
                elif (MyStrategy == 'Medium'):
                    ConfigArray.data = [2,2]
                elif (MyStrategy == 'Easy'):
                    ConfigArray.data = [2,3]
            # PUBLISH TO STRATEGY        
            ScreenConfigurations.publish(ConfigArray)
            Initial_XY = [Nav_CurrentX_mm_d , Nav_CurrentY_mm_d , Nav_CurrentAngle_deg_d ]
            MASTER_FlagsArray =[MASTER_Navigator_Still_IDLE]  
            if (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorIdle_en) : 
                vMASTER_Orders_Publisher(MASTER_FlagsArray , Initial_XY )
                MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en
                Targets_Publisher_Timeout_Start = time.time()
            elif (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en) : 
                if (Targets_Publisher_Timeout_Start - time.time() > 1 ) : 
                    vMASTER_Orders_Publisher(MASTER_FlagsArray , Initial_XY )
                    sleep(ROS_Minimal_Sleep_Time + 0.1)
                    Targets_Publisher_Timeout_Start = time.time()

        if (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en) :
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_WaitForSignal_en
            return  'Go to: WaitForSignal'

        # get color, plate where the robot is placed, strategy, and localization data (disposition du robot pour l'auto-calibration)
        # confirm communication with stm, lora and camera
        # launch localisation process
        else:
            return 'Go to: Preparation'


class WaitForSignal(smach.State):

    global RisingEdge_variable

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: MoveOn', 'Go to: WaitForSignal'],
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global MASTER_SearchRequest 

        #rospy.Subscriber("JackSignal", String, JACKcallback)
        #sleep(0.01)
        print ("wait for signal ",MASTER_CurrentMasterState_en)
        if (MASTER_CurrentMasterState_en == MASTER_MasterStates_ten.MASTER_Start_en):
            # sleep(1)
            SearchRequest.publish(Int8(MASTER_SearchRequest))
            sleep(1)
            MASTER_SearchRequest = 2
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
            MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
            sleep(5)
            return 'Go to: MoveOn'  
        else:
            return 'Go to: WaitForSignal'

class RobotBobDoor(enumerate):
    Front=0
    Back=1

class MoveOn(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: WaitToReach', 'Go to: MoveOn']
                             )

    def execute(self, userdata):

        # NAVIGATOR 
        global MASTER_First_Time_Publishing_Targets
        global Targets_Publisher_Timeout_Start

        # GLOBAL VARIABLES

        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en

        global MASTER_SearchRequest
        #global MASTER_SituationsEnded #TODO: get it from strategy 
        global MASTER_CurrentTargets
        global MASTER_TargetsArray
        global MASTER_FlagsArray
        global MASTER_FirstCommunicateStrategy
        global XY_Request
        global TargetStack_X , TargetStack_Y, TargetStack_Indicator
        global Nav_CurrentX_mm_d,Nav_CurrentY_mm_d, Nav_CurrentAngle_deg_d
        global MASTER_PathPlannerTargets
        global MASTER_TargetsArray , MASTER_TargetIncr
        global MyColor
        global BackRes, FrontRes
        global CurrentTaskBob
        global STRATEGY_CAN_PUBLISH_INDICATORS
        DoorTobeEmptied=0
        CollectCherries = 0

        # TargetStack_Indicator = 3


        # LOCAL VARIABLES
        

        if (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en): 
            
            if ( len(MASTER_TargetsArray) > 0 ) : 
                print(MASTER_TargetsArray)
                if ( MASTER_TargetsArray [3]  ==  MASTER_WONT_Execute_Task ) :
                    pass 
            else :
                """print ("Nav_CurrentTargetNumber_u8  ", Nav_CurrentTargetNumber_u8)
                TargetStack_Indicator = MASTER_ALL_TASK_INDICATORS_GREEN_arr[Nav_CurrentTargetNumber_u8]
                TargetStack_X = MASTER_ALL_XYTHETA_Targets_GREEN_arr [ 2 * Nav_CurrentTargetNumber_u8 + 0 ]
                TargetStack_Y = MASTER_ALL_XYTHETA_Targets_GREEN_arr [ 2 * Nav_CurrentTargetNumber_u8 + 1 ]"""
                if  (( TargetStack_Indicator == 404 + 1   ) or ( TargetStack_Indicator == 99 + 1 )) :
                    STRATEGY_CAN_PUBLISH_INDICATORS = 1 
                XY_Request.publish(True)
                sleep(0.15)
                if ( TargetStack_Indicator == 0 ): #PT ITERM
                    XY_GeneratedTarget = [TargetStack_X, TargetStack_Y]
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget
                    MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward]
                    MASTER_CurrentTargets = MASTER_TargetsArray
                    
                    SearchRequest.publish(Int8(MASTER_SearchRequest)) # REQUESTING THE NEXT target
                    sleep(0.005)
            
                elif( TargetStack_Indicator == 1 ): #PILE
                    print('INDICATOR == 1')

                    DoorTobeEmptied += 1
                    if FrontRes.Couche3 == Color.vide_en:
                        print("FrontRes.Couche3")
                        Door = RobotBobDoor.Front
                        CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseFrontArms_Unknown_en
                        MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward]
                    elif BackRes.Couche3 == Color.vide_en:
                        print("BackRes.Couche3")
                        Door = RobotBobDoor.Back
                        CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en
                        MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Backward]
                    print("MASTER_FlagsArray ",MASTER_FlagsArray)
                    XY_GeneratedTarget = CenterGeneratorRobotBob(Nav_CurrentX_mm_d,Nav_CurrentY_mm_d,TargetStack_X, TargetStack_Y,Door)
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget
                    
                    MASTER_CurrentTargets = MASTER_TargetsArray
                    print('TARGETS ARE:', MASTER_CurrentTargets)
                    print('FLAGS ARE :', MASTER_FlagsArray)
                    SearchRequest.publish(Int8(MASTER_SearchRequest)) # REQUESTING THE NEXT target
                    sleep(0.005)
                    
                elif (TargetStack_Indicator==2): #plate
                    print('INDICATOR == 2')

                    XY_GeneratedTarget = [TargetStack_X,TargetStack_Y]
                    print ('TARGETS ARE: [', TargetStack_X,',' , TargetStack_Y ,']' )
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget
                    MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward]
                    CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployFrontArms_en
                    if (BackRes.Couche3 == Color.vide_en):
                        TargetStack_Indicator = 404 # EL KODEMI M3EBI W TILENI FERAGH
                    else:
                        TargetStack_Indicator = 99 # EL KODEMI M3EBI W TILENI M3EBI
                    print("TargetStack_Indicator ",TargetStack_Indicator)
                    STRATEGY_CAN_PUBLISH_INDICATORS = 0 
                        
                elif (TargetStack_Indicator == 404):
                    print (" dkhalt lel target stack 404 ")
                    reculer = PlateCenterGenerator(Nav_CurrentX_mm_d,Nav_CurrentY_mm_d,Nav_CurrentAngle_deg_d)
                    reculer.append(MASTER_Navigator_No_Rotate_After)
                    reculer.append(MASTER_WONT_Execute_Task)
                    MASTER_TargetsArray = reculer 
                    MASTER_FlagsArray.append(MASTER_Navigator_Can_GO_Backward)
                    SearchRequest.publish(Int8(MASTER_SearchRequest))
                    TargetStack_Indicator = 404 + 1 
                    sleep(0.05) 
                elif (TargetStack_Indicator == 99):
                    print (" dkhalt lel target stack 99 ")
                    reculer = PlateCenterGenerator(Nav_CurrentX_mm_d,Nav_CurrentY_mm_d,Nav_CurrentAngle_deg_d)
                    reculer.append(Nav_CurrentAngle_deg_d+180)
                    reculer.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray.extend(reculer)
                    MASTER_FlagsArray.append(MASTER_Navigator_Can_GO_Backward)
                    CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployBackArms_en

                    TargetStack_Indicator = 99 + 1 
                    SearchRequest.publish(Int8(MASTER_SearchRequest))
                    sleep(0.05)
                                                                     
                elif (TargetStack_Indicator == 3): #cerises 
                    print ( "TargetStack_Indicator == 3 ")

                    CollectCherries += 1
                    
                    #NB: TARGETS HERE ARE POINT LTELI MCH KODEM DISTRIBUTEUR
                    if (TargetStack_Y == 450):
                        #APPEND HERE Avancer(WILL) W Reculer
                        XY_GeneratedTarget =  [TargetStack_X, TargetStack_Y]
                        XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                        XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                        MASTER_TargetsArray = XY_GeneratedTarget

                        Avancer = [TargetStack_X , TargetStack_Y - 350]#just a test value
                        Avancer.append(-90)
                        Avancer.append(MASTER_WILL_Execute_Task)
                        MASTER_TargetsArray.extend(Avancer)

                        Reculer = [TargetStack_X, TargetStack_Y]
                        Reculer.append(MASTER_Navigator_No_Rotate_After) #rotate 180 degré
                        Reculer.append(MASTER_WONT_Execute_Task)
                        MASTER_TargetsArray.extend(Reculer)
                        
                        MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Backward]

                        if CollectCherries == 1 :
                            CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesFirst_en
                        else : CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesSecond_en

                        SearchRequest.publish(Int8(MASTER_SearchRequest))
                        sleep(0.05)
                        # KAMMAL L APPENDS 
                    
                    elif (TargetStack_Y == 1550):
                        print ('TARGETS ARE ', TargetStack_X , TargetStack_Y )
                        XY_GeneratedTarget = [ TargetStack_X, TargetStack_Y ]
                        XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                        XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                        MASTER_TargetsArray = XY_GeneratedTarget
                        Avancer = [TargetStack_X , TargetStack_Y + 350] #just a test value
                        Avancer.append(MASTER_Navigator_No_Rotate_After)
                        Avancer.append(MASTER_WILL_Execute_Task)
                        MASTER_TargetsArray.extend(Avancer)

                        Reculer = [TargetStack_X, TargetStack_Y]
                        Reculer.append(MASTER_Navigator_No_Rotate_After) #rotate 180 degré
                        Reculer.append(MASTER_WONT_Execute_Task)
                        MASTER_TargetsArray.extend(Reculer)
                        MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Backward]

                        if CollectCherries == 1 :
                            CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesFirst_en
                        else : CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesSecond_en

                        SearchRequest.publish(Int8(MASTER_SearchRequest))
                        sleep(0.05)
                                      
                elif (TargetStack_Indicator == 4): #PANIER
                    print ( " TargetStack_Indicator == 4 ")
                    XY_GeneratedTarget = [TargetStack_X,TargetStack_Y]                                     
                    sleep(0.005)
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget
                    MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward]
                    MASTER_CurrentTargets = MASTER_TargetsArray
                    if MyColor == 'BLUE':
                        CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_PlaceCherriesRight_en
                    elif MyColor == 'GREEN':
                        CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_PlaceCherriesLeft_en
                    SearchRequest.publish(Int8(MASTER_SearchRequest))
                    sleep(0.05)

                elif (TargetStack_Indicator == 5): #STEALING #TODO  TAAASSKKK
                
                    DoorTobeEmptied += 1
                    if BackRes.Couche3 == Color.vide_en:
                        Door = RobotBobDoor.Back
                        CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en
                    elif FrontRes.Couche3 == Color.vide_en:
                        Door = RobotBobDoor.Front
                        CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseFrontArms_Unknown_en
                    XY_GeneratedTarget = CenterGeneratorRobotBob(Nav_CurrentX_mm_d,Nav_CurrentY_mm_d,TargetStack_X, TargetStack_Y,Door)
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget
                    MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward]
                    MASTER_CurrentTargets = MASTER_TargetsArray
                    print('TARGETS ARE:', MASTER_CurrentTargets)
                    print('FLAGS ARE :', MASTER_FlagsArray)
                    SearchRequest.publish(Int8(MASTER_SearchRequest)) # REQUESTING THE NEXT target
                    sleep(0.005)
            print (" MASTER_TargetsArray " , MASTER_TargetsArray)
            vMASTER_Orders_Publisher(MASTER_FlagsArray,MASTER_TargetsArray)
            sleep(0.005)
            if (MASTER_First_Time_Publishing_Targets):
                print(" Published targets ")
                Targets_Publisher_Timeout_Start = time.time()
                MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en
                MASTER_First_Time_Publishing_Targets = 0
                # Check Timeout and republish if timeout
        print (" MASTER_CurrentNavigatorState_en ",MASTER_CurrentNavigatorState_en )
        if (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en):
            if ((time.time() - Targets_Publisher_Timeout_Start) > 1):
                Targets_Publisher_Timeout_Start = time.time()
                vMASTER_Orders_Publisher(MASTER_FlagsArray,MASTER_TargetsArray)
                sleep(ROS_Minimal_Sleep_Time)
            sleep(0.5)
            return 'Go to: MoveOn'

        elif (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorBusy_en):
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_WaitToReach_en
            MASTER_First_Time_Publishing_Targets = 1
            
            print("targets ,", MASTER_TargetsArray)
            print("flags ,", MASTER_FlagsArray)


            return 'Go to: WaitToReach'
            
class WaitToReach(smach.State):  # starts moving here

    global Lidar_ObstacleDetected_b

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: SuddenStop', 'Go to: DestinationReached', 'Go to: WaitToReach'],
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec
        global MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool

        if ( MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool ) : 
            if (time.time() - MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool > 2.5 ) : 
                MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool = False
        
        if MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorConfirmedStop_en :
            vMASTER_Orders_Publisher([MASTER_Navigator_Stop_Flag])
            sleep(ROS_Minimal_Sleep_Time + 0.1)
            return 'Go to: SuddenStop'
        elif MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_SuddenStop_en :
            return 'Go to: SuddenStop' 

        elif Lidar_ObstacleDetected_b:
            vMASTER_Orders_Publisher([MASTER_Navigator_Stop_Flag])
            sleep(ROS_Minimal_Sleep_Time + 0.1)
            # communicate with navigator starts here
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_SuddenStop_en
            return 'Go to: SuddenStop'            
        elif (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorReachedDest_en):
            # if there are no obstacles, once the master receives smothing from the nav it switches to dest reached
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_DestinationReached_en
            return 'Go to: DestinationReached'
        else:
            #print(MASTER_CurrentNavigatorState_en.value)
            sleep(ROS_Minimal_Sleep_Time + 0.1)
            return 'Go to: WaitToReach'

class DestinationReached(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: ExecuteTask', 'Go to: MoveOn', 'Go to: Parallel_MoveOn' ],
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global MASTER_CurrentTargets
        global Nav_CurrentTargetNumber_u8

        MASTER_TargetsArray.pop(0)
        MASTER_TargetsArray.pop(0)
        MASTER_TargetsArray.pop(0)
        MASTER_TargetsArray.pop(0)
        MASTER_FlagsArray.pop(0)
        if ( not MASTER_CurrentTargets[3] == MASTER_PATH_PLANNER_TARGET ) : 
            Nav_CurrentTargetNumber_u8 = Nav_CurrentTargetNumber_u8 + 1     

        if (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_DestinationReached_en):  # TRUE

            if (MASTER_CurrentTargets[3] == MASTER_WILL_Execute_Task):
                MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ExecuteTask_en
                MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en
                MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
                return 'Go to: ExecuteTask'

            else:
                MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
                MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
                return 'Go to: MoveOn'


class ExecuteTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: waitToFinishTask'])

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global MASTER_LegendEnded
        global CurrentTask_Blue_en, CurrentTask_Green_en

        if (MASTER_CurrentMasterState_en == MASTER_MasterStates_ten.MASTER_MasterReady_en):  # TRUE
            sleep(0.05)

            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_WaitToFinishTask_en
            MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterBusy_en
        return 'Go to: waitToFinishTask'

class waitToFinishTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: waitToFinishTask', 'Go to: MoveOn',
                                             'Go to: Parallel_MoveOn'])

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global CurrentTaskBob
        global MyColor

        global score
        global CurrentTask_en
        # execute task
        # outcome depends on the execution state machine
        if (MASTER_CurrentMasterState_en == MASTER_MasterStates_ten.MASTER_MasterBusy_en):
            ExecuteTaskBob()

        if (MASTER_CurrentMasterState_en == MASTER_MasterStates_ten.MASTER_MasterReady_en ):
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
            return 'Go to: MoveOn'
        elif (MASTER_CurrentMasterState_en == MASTER_MasterStates_ten.MASTER_ParallelMasterReady_en):
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ParallelMoveOn_en            
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorBusy_en
            return 'Go to: Parallel_MoveOn'
        else:
            return 'Go to: waitToFinishTask'

class Parallel_MoveOn (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Parallel_MoveOn', 'Go to: Parallel_WaitToReach'],
                             )

    def execute(self, userdata):
        # NAVIGATOR 
              # NAVIGATOR 
        global MASTER_First_Time_Publishing_Targets
        global Targets_Publisher_Timeout_Start

        # GLOBAL VARIABLES

        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en

        global MASTER_SearchRequest
        #global MASTER_SituationsEnded #TODO: get it from strategy 
        global MASTER_CurrentTargets
        global MASTER_TargetsArray
        global MASTER_FlagsArray
        global MASTER_FirstCommunicateStrategy
        global XY_Request
        global TargetStack_X , TargetStack_Y, TargetStack_Indicator
        global Nav_CurrentX_mm_d,Nav_CurrentY_mm_d
        global MASTER_PathPlannerTargets
        global MASTER_TargetsArray , MASTER_TargetIncr
        global MASTER_FlagsArray
        global MyColor
        global BackRes, FrontRes
        global CurrentTaskBob
        DoorTobeEmptied=0
        CollectCherries = 0

        # LOCAL VARIABLES 

        if (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorReady_en): 
            
            if ( len(MASTER_TargetsArray) > 0 ) : 
                print(MASTER_TargetsArray)
                if ( MASTER_TargetsArray [3]  ==  MASTER_WONT_Execute_Task ) :
                    pass 

            elif(TargetStack_Indicator==1): #PILE
                XY_Request.publish(True)  
                sleep(0.05)
                DoorTobeEmptied += 1
                if FrontRes.Couche3 == Color.vide_en:
                    Door = RobotBobDoor.Front
                    CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseFrontArms_Unknown_en
                elif BackRes.Couche3 == Color.vide_en:
                    Door = RobotBobDoor.Back
                    CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en
                
                print ('TARGETS ARE: [', TargetStack_X,',' , TargetStack_Y ,']' )
                XY_GeneratedTarget = CenterGeneratorRobotBob(Nav_CurrentX_mm_d,Nav_CurrentY_mm_d,TargetStack_X, TargetStack_Y,Door)
                XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                MASTER_TargetsArray = XY_GeneratedTarget
                MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward]
                MASTER_CurrentTargets = MASTER_TargetsArray
                SearchRequest.publish(Int8(MASTER_SearchRequest)) # REQUESTING THE NEXT target
                sleep(0.005)
                    
            elif (TargetStack_Indicator==2): #plate
                XY_Request.publish(True)  
                sleep(0.05)
                
                if (DoorTobeEmptied == 2):
                    CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployFrontArms_en
                    DoorTobeEmptied -= 1
                    XY_GeneratedTarget = [TargetStack_X,TargetStack_Y]
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget

                    if (0<TargetStack_Y<1000):
                        XY_GeneratedTarget.extend(TargetStack_X, TargetStack_Y+180)
                        XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                        XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                        MASTER_TargetsArray = XY_GeneratedTarget
                    else: 
                        XY_GeneratedTarget.extend(TargetStack_X, TargetStack_Y-180)
                        XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                        XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                        MASTER_TargetsArray = XY_GeneratedTarget
                    MASTER_CurrentTargets = MASTER_TargetsArray
                    MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Backward]

                elif (DoorTobeEmptied == 1):
                    CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployBackArms_en
                    DoorTobeEmptied -= 1
                    XY_GeneratedTarget = [TargetStack_X,TargetStack_Y]
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget
                    if (0<TargetStack_Y<1000):
                        XY_GeneratedTarget.extend(TargetStack_X, TargetStack_Y+180)
                        XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                        XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                        MASTER_TargetsArray = XY_GeneratedTarget
                    else: 
                        XY_GeneratedTarget.extend(TargetStack_X, TargetStack_Y-180)
                        XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                        XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                        MASTER_TargetsArray = XY_GeneratedTarget
                    MASTER_CurrentTargets = MASTER_TargetsArray
                    MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Backward]
                SearchRequest.publish(Int8(MASTER_SearchRequest))
                sleep(0.05)

            elif (TargetStack_Indicator == 3): #cerises 
                XY_Request.publish(True)  
                sleep(0.05)
                CollectCherries += 1 
    
                    #NB: TARGETS HERE ARE POINT LTELI MCH KODEM DISTRIBUTEUR
                if (TargetStack_Y == 450):
                    #APPEND HERE Avancer(WILL) W Reculer
                    XY_GeneratedTarget =  TargetStack_X, TargetStack_Y
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget
                    Avancer = TargetStack_X , TargetStack_Y - 35 #just a test value
                    Avancer.append(MASTER_Navigator_No_Rotate_After)
                    Avancer.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray.extend(Avancer)

                    Reculer = TargetStack_X, TargetStack_Y
                    Reculer.append(180) #rotate 180 degré
                    Reculer.append(MASTER_WONT_Execute_Task)
                    MASTER_TargetsArray.extend(Reculer)
                    MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Backward]

                    if CollectCherries == 1 :
                        CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesFirst_en
                    else : CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesSecond_en

                    SearchRequest.publish(Int8(MASTER_SearchRequest))
                    sleep(0.05)
                    # KAMMAL L APPENDS 
                    pass
                elif (TargetStack_Y == 1550):
                    XY_GeneratedTarget =  TargetStack_X, TargetStack_Y
                    XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                    XY_GeneratedTarget.append(MASTER_WONT_Execute_Task)
                    MASTER_TargetsArray = XY_GeneratedTarget
                    Avancer = TargetStack_X , TargetStack_Y + 35 #just a test value
                    Avancer.append(MASTER_Navigator_No_Rotate_After)
                    Avancer.append(MASTER_WILL_Execute_Task)
                    MASTER_TargetsArray.extend(Avancer)

                    Reculer = TargetStack_X, TargetStack_Y
                    Reculer.append(180) #rotate 180 degré
                    Reculer.append(MASTER_WONT_Execute_Task)
                    MASTER_TargetsArray.extend(Reculer)
                    MASTER_FlagsArray = [MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Forward, MASTER_Navigator_Can_GO_Backward]

                    if CollectCherries == 1 :
                        CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesFirst_en
                    else : CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesSecond_en

                    SearchRequest.publish(Int8(MASTER_SearchRequest))
                    sleep(0.05)
                                      
            elif (TargetStack_Indicator == 4): #PANIER
                XY_Request.publish(True)  
                sleep(0.05)
                XY_GeneratedTarget = [TargetStack_X,TargetStack_Y]                                      
                sleep(0.005)
                XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                MASTER_TargetsArray = XY_GeneratedTarget
                MASTER_CurrentTargets = MASTER_TargetsArray
                if MyColor == 'BLUE':
                    CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_PlaceCherriesRight_en
                elif MyColor == 'GREEN':
                    CurrentTaskBob = MASTER2_MicroTasks_ten.MASTER2_MicroTask_PlaceCherriesLeft_en
                SearchRequest.publish(Int8(MASTER_SearchRequest))
                sleep(0.05)

            elif (TargetStack_Indicator == 5): #STEALING #TODO
                XY_Request.publish(True)  
                sleep(0.05)
                XY_GeneratedTarget = [TargetStack_X,TargetStack_Y]                                      
                sleep(0.005)
                XY_GeneratedTarget.append(MASTER_Navigator_No_Rotate_After)
                XY_GeneratedTarget.append(MASTER_WILL_Execute_Task)
                MASTER_TargetsArray = XY_GeneratedTarget
                MASTER_CurrentTargets = MASTER_TargetsArray
                SearchRequest.publish(Int8(MASTER_SearchRequest))
                sleep(0.05)

            vMASTER_Orders_Publisher(MASTER_FlagsArray,MASTER_TargetsArray)
            sleep(0.005)
            if (MASTER_First_Time_Publishing_Targets):
                Targets_Publisher_Timeout_Start = time.time()
                MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en
                MASTER_First_Time_Publishing_Targets = 0
                    # Check Timeout and republish if timeout

        if (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en):
            if ((time.time() - Targets_Publisher_Timeout_Start) > 1):
                Targets_Publisher_Timeout_Start = time.time()
                vMASTER_Orders_Publisher(MASTER_FlagsArray,MASTER_TargetsArray)
                sleep(ROS_Minimal_Sleep_Time)
            sleep(0.5)
            return 'Go to: Parallel_MoveOn'

        elif (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorBusy_en):
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ParallelWaitToReach_en
            MASTER_First_Time_Publishing_Targets = 1
            
            print("targets 0,", MASTER_TargetsArray)
            return 'Go to: Parallel_WaitToReach'

class Parallel_WaitToReach (smach.State):  #involves task execution
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Parallel_WaitToReach',
                                              'Go to: Parallel_DestinationReached',
                                              'Go to: Parallel_SuddenStop']
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec
        global MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool

        if ( MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool ) : 
            if ( ( time.time() - MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec ) > MASTER_SUDDEN_STOP_REDUCED_RANG_TIME ) : 
                MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool       = 0
                MASTER_SUDDEN_STOP_RANGE_m

        if Lidar_ObstacleDetected_b:
            vMASTER_Orders_Publisher([MASTER_Navigator_Stop_Flag])
            sleep(ROS_Minimal_Sleep_Time + 0.1)
            # communicate with navigator starts here
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ParallelSuddenStop_en
            return 'Go to: Parallel_SuddenStop'
        
        if MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorConfirmedStop_en :
            vMASTER_Orders_Publisher([MASTER_Navigator_Stop_Flag])
            sleep(ROS_Minimal_Sleep_Time + 0.1)
            return 'Go to: Parallel_SuddenStop'
        elif MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_SuddenStop_en :
            return 'Go to: Parallel_SuddenStop' 

        if (MASTER_CurrentMasterState_en == MASTER_MasterStates_ten.MASTER_ParallelMasterBusy_en):
            ExecuteTaskBob()

        elif (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorReachedDest_en):
            # if there are no obstacles, once the master receives smothing from the nav it switches to dest reached
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ParallelDestReached_en
            sleep(0.5)
            return 'Go to: Parallel_DestinationReached'
        else:
            #print(MASTER_CurrentNavigatorState_en.value)
            sleep(1)
            return 'Go to: Parallel_WaitToReach'

class Parallel_DestinationReached(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: ExecuteTask', 'Go to: Parallel_DestinationReached','Go to: MoveOn'],
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global MASTER_CurrentTargets
        global MASTER_TargetsArray
        
        MASTER_TargetsArray.pop()
        MASTER_TargetsArray.pop()
        MASTER_TargetsArray.pop()
        MASTER_TargetsArray.pop()

        if ( MASTER_CurrentMasterState_en == MASTER_MasterStates_ten.MASTER_ParallelMasterBusy_en ) :
            if  (MASTER_CurrentTargets [3] == MASTER_WONT_Execute_Task) :
                MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ParallelMoveOn_en
                MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorReady_en
                return 'Go to: Parallel_MoveOn'
            else : 
                ExecuteTaskBob()
                return 'Go to: Parallel_DestinationReached'

        elif (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorReachedDest_en ):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorReady_en
            return 'Go to: Parallel_DestinationReached'
        
        elif (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorReady_en) :   
            if  (MASTER_CurrentTargets [3] == MASTER_WONT_Execute_Task) :
                MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
                return 'Go to: MoveOn'
        
            if   (MASTER_CurrentTargets [3] == MASTER_WILL_Execute_Task ) :
                MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ExecuteTask_en
                return 'Go to: ExecuteTask'
        

class SuddenStop (smach.State):


    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: confirmed stop', 'Go to: sudden stop'],
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global Lidar_ObstacleDetected_b

        # communicated with navigator under if obstacle detected in wait to reach and waits for callback (conf stop)
        if (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorConfirmedStop_en):
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ConfirmedStop_en
            return 'Go to: confirmed stop'

        else:
            sleep(ROS_Minimal_Sleep_Time + 0.2)
            return 'Go to: sudden stop'

class ConfirmedStop (smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: MoveOn', 'Go to: ConfirmedStop'],
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en
        global PATH_PLANNED_bool
        global Nav_CurrentX_mm_d
        global Nav_CurrentY_mm_d
        global MASTER_CurrentTargets
        global Nav_CurrentTargetNumber_u8
        global MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec
        global MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool
        global MASTER_SUDDEN_STOP_RANGE_m
        global Closest_Obstacle_Range
        global RobotAdvr_Scanning_CLONE
        global ClosestObstacle_X_Map_OLD
        global ClosestObstacle_Y_Map_OLD
        global MASTER_TargetsArray
        global MASTER_SUDDEN_STOP_REDUCED_FRONT_RANGE_m
        X_GLOBAL_TARGET_BEFORE_PATH_Planner = 0 
        Y_GLOBAL_TARGET_BEFORE_PATH_Planner = 0
        Obstacle_X = []
        Obstacle_Y = []
        PATH_PLANNER_X_Y_TARGETS_LoL = []
        PATH_PLANNER_X_Y_TARGETS_arr = []
        print ("X obstacle  ",ClosestObstacle_X_Map_OLD )
        print ("Y Obstacle  ",ClosestObstacle_Y_Map_OLD)


        if not (Lidar_ObstacleDetected_b):
            print("lidar_bool ", Lidar_ObstacleDetected_b)
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
            PATH_PLANNED_bool = 0
            return 'Go to: MoveOn'

        else:
            print(Nav_CurrentX_mm_d)
            print(Nav_CurrentY_mm_d)
            print(MASTER_CurrentTargets [ 0 ])
            print(MASTER_CurrentTargets [ 1 ])
            print(ClosestObstacle_X_Map_OLD)
            print(ClosestObstacle_Y_Map_OLD)
            if ( optimizer(Nav_CurrentX_mm_d / CONVERSION_FROM_PATH_PLANNER_TO_MM ,
                         Nav_CurrentY_mm_d / CONVERSION_FROM_PATH_PLANNER_TO_MM ,
                         MASTER_CurrentTargets [ 0 ] / CONVERSION_FROM_PATH_PLANNER_TO_MM , 
                         MASTER_CurrentTargets [ 1 ] / CONVERSION_FROM_PATH_PLANNER_TO_MM , 
                         ClosestObstacle_X_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ,
                         ClosestObstacle_Y_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ,
                         ROBOT_Radius, 0.15 * 20) ) :
                print (" Optimizeeer before path planner ")
                MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec = time.time()
                MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool = 1
                MASTER_SUDDEN_STOP_TEMPORARY_RANGE_m        = Closest_Obstacle_Range - 0.1
                MASTER_SUDDEN_STOP_RANGE_m                  = MASTER_SUDDEN_STOP_TEMPORARY_RANGE_m
                MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
                MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
                return 'Go to: MoveOn'
            X_GLOBAL_TARGET_BEFORE_PATH_Planner = MASTER_TargetsArray [ 0 ]
            Y_GLOBAL_TARGET_BEFORE_PATH_Planner = MASTER_TargetsArray [ 1 ]

            Obstacle_X ,Obstacle_Y  = DStarLite.addRectangle (int (ClosestObstacle_X_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ) - 0.15 * 20 , int (ClosestObstacle_X_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ) + 0.15 * 20 ,  int (ClosestObstacle_Y_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ) - 0.15 * 20 , int (ClosestObstacle_Y_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ) + 0.15 * 20  )
            for x , y in zip(Obstacle_X,Obstacle_Y):
                ox.append(x)
                oy.append(y)
            print ("Nav_CurrentX_mm_d ",Nav_CurrentX_mm_d)
            PATH_PLANNED_bool , PATH_PLANNER_X_Y_TARGETS_LoL = DStarLite.main(int (Nav_CurrentX_mm_d * 0.02) ,int ( Nav_CurrentY_mm_d * 0.02 ),int(X_GLOBAL_TARGET_BEFORE_PATH_Planner * 0.02) , int ( Y_GLOBAL_TARGET_BEFORE_PATH_Planner * 0.02 ) , int (ClosestObstacle_X_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ) , int (ClosestObstacle_Y_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ) ) # dont forget the * 20 factor of map dimensions
            
            if PATH_PLANNED_bool :
                print('PATH_PLANNER_X_Y_TARGETS_LoL ' , PATH_PLANNER_X_Y_TARGETS_LoL)
                PATH_PLANNER_X_Y_TARGETS_LoL.pop(0)
                print ( [[int (ClosestObstacle_X_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER) ,
                          int (ClosestObstacle_Y_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER )]] )
                PATH_PLANNER_X_Y_TARGETS_LoL = DStarLite.Path_Optimizer(PATH_PLANNER_X_Y_TARGETS_LoL ,[[int (ClosestObstacle_X_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER) , int (ClosestObstacle_Y_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER )]] )
                print('PATH_PLANNER_X_Y_TARGETS_LoL ' , PATH_PLANNER_X_Y_TARGETS_LoL)
                print (" PATH PLANNED " , PATH_PLANNED_bool) 
                for Target in PATH_PLANNER_X_Y_TARGETS_LoL : 
                    Target[0] = Target[0] * CONVERSION_FROM_PATH_PLANNER_TO_MM
                    Target[1] = Target[1] * CONVERSION_FROM_PATH_PLANNER_TO_MM 
                    PATH_PLANNER_X_Y_TARGETS_arr.extend(Target)
                    PATH_PLANNER_X_Y_TARGETS_arr.extend([MASTER_Navigator_No_Rotate_After , MASTER_PATH_PLANNER_TARGET ])

                print ("after optim ",PATH_PLANNER_X_Y_TARGETS_arr )
                PATH_PLANNER_X_Y_TARGETS_arr.extend(MASTER_TargetsArray)
                MASTER_TargetsArray = PATH_PLANNER_X_Y_TARGETS_arr

                print ("After Path Planner " ,MASTER_TargetsArray)
                        
                MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec = time.time()
                MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool = 1
                MASTER_SUDDEN_STOP_REDUCED_FRONT_RANGE_m  = Closest_Obstacle_Range - 0.2
                    
                MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
                MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
                ox.clear()
                oy.clear()
                return 'Go to: MoveOn'
            else :
                oy.clear()
                ox.clear()
                if (optimizer
                    (Nav_CurrentX_mm_d / CONVERSION_FROM_PATH_PLANNER_TO_MM,
                    Nav_CurrentY_mm_d / CONVERSION_FROM_PATH_PLANNER_TO_MM ,
                    ( X_GLOBAL_TARGET_BEFORE_PATH_Planner / CONVERSION_FROM_PATH_PLANNER_TO_MM ),
                    (Y_GLOBAL_TARGET_BEFORE_PATH_Planner / CONVERSION_FROM_PATH_PLANNER_TO_MM ) ,
                    (ClosestObstacle_X_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ),
                    (ClosestObstacle_Y_Map_OLD * CONVERSION_FROM_METRE_TO_PATH_PLANNER ) ,
                    ROBOT_Radius, 0.15 * 20)) :
                        MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec = time.time()
                        MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool = True
                        MASTER_SUDDEN_STOP_TEMPORARY_RANGE_m        = Closest_Obstacle_Range - 0.1
                        MASTER_SUDDEN_STOP_RANGE_m                  = MASTER_SUDDEN_STOP_TEMPORARY_RANGE_m

                        MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
                        MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
                        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en

                        return 'Go to: MoveOn'


                sleep(ROS_Minimal_Sleep_Time + 0.3)
                return 'Go to: ConfirmedStop'

class Parallel_SuddenStop (smach.State):
    global Lidar_ObstacleDetected_b
    Lidar_ObstacleDetected_b = False

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Parallel_ConfirmedStop', 'Go to: Parallel_SuddenStop'],
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en

        # communicated with navigator under if obstacle detected in wait to reach and waits for callback (conf stop)
        if (MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorParallelConfirmedStop_en):
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ConfirmedStop_en
            return 'Go to: Parallel_ConfirmedStop'

        else:
            return 'Go to: Parallel_SuddenStop'

class Parallel_ConfirmedStop (smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Parallel_MoveOn', 'Go to: Parallel_ConfirmedStop'],
                             )

    def execute(self, userdata):
        global MASTER_CurrentMachineState_en
        global MASTER_CurrentMasterState_en
        global MASTER_CurrentNavigatorState_en

        if not (Lidar_ObstacleDetected_b):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorReady_en
            MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_ParallelMoveOn_en
            return 'Go to: Parallel_MoveOn'

        if (MASTER_CurrentMasterState_en == MASTER_MasterStates_ten.MASTER_ParallelMasterBusy_en):
            ExecuteTaskBob()

            return 'Go to: Parallel_ConfirmedStop'
        else:
            return 'Go to: Parallel_ConfirmedStop'

###################################### GENERAL FUNCTIONS START HERE ###################################
################################     CALLBACK FUNCTIONS      ###################################

def MyColor_Callback (msg):
    global MyColor
    MyColor= msg.data

def MyStrategy_Callback(msg):
    global MyStrategy
    MyStrategy= msg.data

def Confirmation_Callback(msg):
    global Confirmation
    if (msg.data):
        Confirmation = 1


def Location_state_CALLBACK(msg):
    global Nav_CurrentAngle_deg_d
    global Nav_CurrentX_mm_d
    global Nav_CurrentY_mm_d

    global Velocity_Delay_Counter
    global Nav_CurrentVelocity_d

    '''print ("Nav_CurrentX_mm_d ",Nav_CurrentX_mm_d)
    print ("Nav_CurrentY_mm_d ",Nav_CurrentY_mm_d)
    print ("Nav_CurrentAngle_deg_d", Nav_CurrentAngle_deg_d)'''

    Nav_CurrentX_mm_d = msg.pose.pose.position.x
    Nav_CurrentY_mm_d = msg.pose.pose.position.y
    Nav_CurrentAngle_deg_d = msg.pose.pose.orientation.z
    Nav_CurrentVelocity_d =msg.twist.twist.linear.x
    Nav_CurrentVelocity_d = Nav_CurrentVelocity_d / 1000  

    print ("Nav_CurrentX_mm_d ",Nav_CurrentX_mm_d)
    print ("Nav_CurrentY_mm_d ",Nav_CurrentY_mm_d)
    print ("Nav_CurrentAngle_deg_d", Nav_CurrentAngle_deg_d)
    print ("Nav_CurrentVelocity_d ",Nav_CurrentVelocity_d)

def NextTarget_Callback (msg): # GETS (X,Y) OF THE TRAGET STACK
    global TargetStack_X , TargetStack_Y
    global TargetStack_Indicator
    global STRATEGY_CAN_PUBLISH_INDICATORS
    if STRATEGY_CAN_PUBLISH_INDICATORS :
        TargetStack_X , TargetStack_Y =  msg.data[0], msg.data[1]
        TargetStack_Indicator = msg.data[2]  # EXPLANATION: 
    # USED AS A COLOR INDICATOR WHEN I AM WORKING ON LEGEND
    # USED AS A PLATE/PILE INDICATOR WHEN I AM WORKING ON NON LEGEND:  1 == PILE XY , 2 == PLATE 

  ################################## NAVIGATION FUNCTIONS ############################################
    
def vMASTER_Orders_Publisher(FlagArray,target=[0,0,0]):
    global MASTER_ALL_XYTHETA_Targets_arr
    global Nav_CurrentTargetNumber_u8
    global MASTER_Orders_Publisher
    global MASTER_CurrentTargets
    global MASTER_FlagsArray
    raw_data = []
    MASTER_CurrentTargets = target[0:4]
    flag = FlagArray[0]
    data_to_be_sent = Float32MultiArray()
    target= target[:3]
    raw_data.append(flag)
    raw_data.extend(target)
    #print('published orders ' , MASTER_CurrentTargets)
    data_to_be_sent.data = raw_data
    MASTER_Orders_Publisher.publish(data_to_be_sent)

'''
* Brief * : This function is executed when STM publish an update in /Robot_Status

'''
def Robot_Status_MessageRecieved_CALLBACK(msg):
    global MASTER_PermissionTo_Navigator
    global Nav_CurrentTargetNumber_u8
    global Nav_XYTargets_Numbers
    global MASTER_CurrentNavigatorState_en
    global MASTER_CurrentMasterState_en
    global MASTER_CurrentMachineState_en
    global MASTER_ALL_XYTHETA_Targets_arr
    global Lidar_ObstacleDetected_b
    global MASTER_WILL_Execute_Task
    global MASTER_WONT_Execute_Task
    global MASTER_WILL_Parallel_Execute_Task

# ****** WILL BE USED TO ESTABLISH COMMUNICATION AT IDLE STATE **********
    if (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_Idle_en):
        # Will be used in the transition from Idle to Preparation
        MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en

# ***** WILL BE USED TO CONFIRM RECEPTION OF INITIAL X Y AND THETA IN PREPARATION STATE
    elif (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_Preparation_en):
        MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en

# ****** WILL BE USED TO GO THROUGH X Y TARGETS and go to either Next targets or execute Task or // Execute Task !

    elif (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_WaitToReach_en):
        if (msg.data == MASTER_Navigator_Indicator_WaitForOrders):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReachedDest_en

    elif (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_ParallelWaitToReach_en):
        if (msg.data == MASTER_Navigator_Indicator_WaitForOrders): # 2 
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorReachedDest_en
        elif( msg.data == MASTER_Navigator_MessageRecieved): # 0
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorBusy_en
        

# ***** WILL BE USED TO CONFIRM STOP

    elif (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_SuddenStop_en) or \
    (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_WaitToReach_en) or \
     (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_ParallelWaitToReach_en)     :
        if (msg.data == MASTER_Navigator_Stop_Flag):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorConfirmedStop_en

# Will be used to Confirm being busy MOveOn ==> Wait to reach
    elif (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en):
        if (msg.data == MASTER_Navigator_MessageRecieved):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorBusy_en

    elif (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_ParallelMoveOn_en):
        if (msg.data == MASTER_Navigator_MessageRecieved):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_ParallelNavigatorBusy_en

def CenterGeneratorRobotBob(CurrentX,CurrentY,CakeX,CakeY,DoorToFill):
    
    CenterToBackDist = 100
    CenterToFrontDist = 110
    
    DirectionAngle=math.atan2(CakeY-CurrentY,CakeX-CurrentX)
    if DoorToFill==RobotBobDoor.Back:
        TargetX=CakeX-math.cos(DirectionAngle)*CenterToBackDist
        TargetY=CakeY-math.sin(DirectionAngle)*CenterToBackDist
    elif DoorToFill==RobotBobDoor.Front:
        TargetX=CakeX-math.cos(DirectionAngle)*CenterToFrontDist
        TargetY=CakeY-math.sin(DirectionAngle)*CenterToFrontDist
    
    return [round(TargetX,1),round(TargetY,1)]

def Backward (x,y):
    if (x<=1500 and y<= 1000):
        return [500,500]
    elif (x<=1500 and y>= 1000):
        return [500,1500]
    elif (x>=1500 and y<= 1000):
        return [2500,500]
    elif (x>=1500 and y>= 1000):
        return [2500,1500]

def PlateCenterGenerator(CurrentX,CurrentY,CurrentAngle):
    SecurityDist=160
    TargetX= CurrentX-math.cos(math.radians(CurrentAngle))*SecurityDist
    TargetY= CurrentY-math.sin(math.radians(CurrentAngle))*SecurityDist
    return [round(TargetX),round(TargetY)]

######################################## LIDAR FUNCTIONS ##############################################

def indice (T):
    min= T[0]
    ind=0
    if (len(T)==1 ):
        return ( 0 , 0 )
    else:

        for i in range (1,len(T)-1):
            if (T[i]<min):
                min=T[i]
                ind=i
        return(ind , min)
    
def distance (x1,x2,y1,y2) :
    return math.sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)))  

def OBSTACLE_STATIQUE_bool ( X_New , X_Old , Y_New , Y_Old) : 
    return ( distance (X_New , X_Old , Y_New , Y_Old ) < 0.05 )

def OBSTACLE_MOVING_AWAY_bool (Distance_NEW , DISTANCE_OLD) : 
    return (Distance_NEW > DISTANCE_OLD )

def Scan_Gateau(scan_data):

    global Nav_CurrentAngle_deg_d 
    global Nav_CurrentX_mm_d 
    global Nav_CurrentY_mm_d
    global MASTER_Orders_Publisher
    global MASTER_CurrentNavigatorState_en
    global Lidar_ObstacleDetected_b
    global MASTER_CurrentMachineState_en
    global MASTER_CurrentNavigatorState_en
    global RobotAdvr_Scanning_CLONE
    global MASTER_SUDDEN_STOP_RANGE_m
    global Closest_Obstacle_Range
    global Closest_Obstacle_Range_OLD
    global Closest_Obstacle_Angle
    global MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool , MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec 
    global ClosestObstacle_X_Map_OLD
    global ClosestObstacle_Y_Map_OLD
    global Nav_CurrentVelocity_d
    


    ClosestObstacle_X_Map = 0 
    ClosestObstacle_Y_Map = 0
    i=0
    angle = 0.0
    RobotAdvr_Scanning = []
    RobotAdvr_XY_MapRepere=[]
    RobotAdvr_XY_LidarRepere=[]
    DistAdvr=[]
    index = 0 
    N= len (scan_data.ranges)
    Lidar_ObstacleDetected_b = False
    Closest_Obstacle_Range , index  = min_range_index (scan_data.ranges , len(scan_data.ranges))
    Closest_Obstacle_Range_360 = ((Closest_Obstacle_Range + 360 ) % 360) 
    Closest_Obstacle_Angle = (math.degrees( index * scan_data.angle_increment)-5)
    ClosestObstacle_X_LIDAR = Closest_Obstacle_Range*math.cos(Closest_Obstacle_Angle*math.pi/180)
    ClosestObstacle_Y_LIDAR = Closest_Obstacle_Range*math.sin(Closest_Obstacle_Angle*math.pi/180)
    ClosestObstacle_X_Map = (ClosestObstacle_X_LIDAR*math.cos(Nav_CurrentAngle_deg_d*math.pi/180)) + \
    (-ClosestObstacle_Y_LIDAR*math.sin(Nav_CurrentAngle_deg_d * math.pi/180)) + (Nav_CurrentX_mm_d / 1000)
        
    ClosestObstacle_Y_Map = ClosestObstacle_X_LIDAR*math.sin(Nav_CurrentAngle_deg_d*math.pi/180) + ClosestObstacle_Y_LIDAR*math.cos(
    Nav_CurrentAngle_deg_d*math.pi/180) + (Nav_CurrentY_mm_d / 1000)
    if ( not MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool ):

        #if ( not ( OBSTACLE_MOVING_AWAY_bool(Closest_Obstacle_Range , Closest_Obstacle_Range_OLD) ) ) :
        if ( ( - 45) < Closest_Obstacle_Range_360 <  ( + 45) ) :
            MASTER_SUDDEN_STOP_RANGE_m =  MASTER_SUDDEN_STOP_FRONT_RANGE_m 
            print("front")
        elif( ( - 135) < Closest_Obstacle_Range_360 < ( - 45 )  ) or \
            ( ( + 45) < Closest_Obstacle_Range_360 <  ( + 135)  ):
            MASTER_SUDDEN_STOP_RANGE_m = MASTER_SUDDEN_STOP_AJNEB_RANGE_m 
            print("ajneb")
        else : 
             MASTER_SUDDEN_STOP_RANGE_m =  MASTER_SUDDEN_STOP_BACK_RANGE_m 
             print("BACK")
        print (" MASTER_SUDDEN_STOP_RANGE_m ", MASTER_SUDDEN_STOP_RANGE_m)
        
        if   ( Closest_Obstacle_Range < MASTER_SUDDEN_STOP_RANGE_m )  :
                        
            if ( (0.0 < ClosestObstacle_X_Map < 3.0) and (0.0 < ClosestObstacle_Y_Map < 2.0 ) ) :
                Lidar_ObstacleDetected_b = True

                MASTER_PermissionTo_Navigator = [MASTER_Navigator_Stop_Flag]
                if ((MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_ParallelWaitToReach_en) or
                    (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_WaitToReach_en)):
                    ClosestObstacle_X_Map_OLD  = ClosestObstacle_X_Map
                    ClosestObstacle_Y_Map_OLD  = ClosestObstacle_Y_Map
                    Closest_Obstacle_Range_OLD = Closest_Obstacle_Range
                    vMASTER_Orders_Publisher(MASTER_PermissionTo_Navigator)
                    sleep(ROS_Minimal_Sleep_Time + 0.2)
                    #MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en
                    rospy.loginfo("Stop order")
                    MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en

    # Basically I will try to make my algorithm robust in what concerns looping over sudden stop and move On 
    # SUDDEN STOP RANGE WILL BE REDUCED FOR A CERTAIN TIME
    # THE RANGE IS MODIFIED IN THE CONFIRMED STOP STATE
    #   
    elif ( MASTER_SUDDEN_STOP_REDUCED_ACTIVATED_bool ) :
        ClosestObstacle_X_LIDAR = Closest_Obstacle_Range*math.cos(Closest_Obstacle_Angle*math.pi/180)
        ClosestObstacle_Y_LIDAR = Closest_Obstacle_Range*math.sin(Closest_Obstacle_Angle*math.pi/180)
        ClosestObstacle_X_Map = (ClosestObstacle_X_LIDAR*math.cos(Nav_CurrentAngle_deg_d*math.pi/180)) + \
        (-ClosestObstacle_Y_LIDAR*math.sin(Nav_CurrentAngle_deg_d * math.pi/180)) + (Nav_CurrentX_mm_d / 1000)        
        ClosestObstacle_Y_Map = (ClosestObstacle_X_LIDAR*math.sin(Nav_CurrentAngle_deg_d*math.pi/180)) + \
        ( ClosestObstacle_Y_LIDAR*math.cos(Nav_CurrentAngle_deg_d * math.pi/180)) + (Nav_CurrentY_mm_d / 1000)

        if (not ( OBSTACLE_MOVING_AWAY_bool(Closest_Obstacle_Range , Closest_Obstacle_Range_OLD) or
                  OBSTACLE_STATIQUE_bool (ClosestObstacle_X_Map , ClosestObstacle_Y_Map , ClosestObstacle_X_Map_OLD ,ClosestObstacle_Y_Map_OLD) ) ) :
            if (Closest_Obstacle_Range_360 in [Nav_CurrentAngle_deg_d - 45 , Nav_CurrentAngle_deg_d + 45]) :
                MASTER_SUDDEN_STOP_RANGE_m =  MASTER_SUDDEN_STOP_REDUCED_FRONT_RANGE_m
            elif(Closest_Obstacle_Range_360 in [Nav_CurrentAngle_deg_d - 45 , Nav_CurrentAngle_deg_d - 135]) or \
                (Closest_Obstacle_Range_360 in [Nav_CurrentAngle_deg_d + 45 , Nav_CurrentAngle_deg_d + 135]):
                MASTER_SUDDEN_STOP_RANGE_m =  MASTER_SUDDEN_STOP_REDUCED_AJNEB_RANGE_m
            else : 
                 MASTER_SUDDEN_STOP_RANGE_m =  MASTER_SUDDEN_STOP_REDUCED_BACK_RANGE_m 
            if   ( Closest_Obstacle_Range < MASTER_SUDDEN_STOP_RANGE_m ) :        

                if ( (0.0 < ClosestObstacle_X_Map < 3.0) and (0.0 < ClosestObstacle_Y_Map < 2.0 ) ) :
                    Lidar_ObstacleDetected_b = True
                    
                    MASTER_PermissionTo_Navigator = [MASTER_Navigator_Stop_Flag]                                                            
                    if ((MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_ParallelWaitToReach_en) or
                        (MASTER_CurrentMachineState_en == MASTER_SMachineStates_ten.MASTER_SMachine_WaitToReach_en)):
                        vMASTER_Orders_Publisher(MASTER_PermissionTo_Navigator)
                        sleep(ROS_Minimal_Sleep_Time + 0.15)
                        Closest_Obstacle_Range_OLD = Closest_Obstacle_Range
                        print('X Obstacle  ' , ClosestObstacle_X_Map)
                        print('Y Obstacle  ' , ClosestObstacle_Y_Map)

                        ClosestObstacle_X_Map_OLD  = ClosestObstacle_X_Map
                        ClosestObstacle_Y_Map_OLD  = ClosestObstacle_Y_Map
                        MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorGoingToRespond_en
                        rospy.loginfo("Stop order")
                        MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_SuddenStop_en
        elif (OBSTACLE_MOVING_AWAY_bool(Closest_Obstacle_Range ,Closest_Obstacle_Range_OLD ) or 
              OBSTACLE_STATIQUE_bool(ClosestObstacle_X_Map , ClosestObstacle_Y_Map , ClosestObstacle_X_Map_OLD ,ClosestObstacle_Y_Map_OLD)) :
                MASTER_SUDDEN_STOP_REDUCE_RANGE_TIMER_sec =  time.time()

    
    
    Closest_Obstacle_Range_OLD = Closest_Obstacle_Range
    
    for i in  range(len (scan_data.ranges)) :#N+1!!!!!!!!!!!!!!! TODO 
        if scan_data.ranges[i] == 0.0 :
            pass
        else:    
         angle=(math.degrees( i * scan_data.angle_increment)-5)

         RobotAdvr_XY_LidarRepere=[scan_data.ranges[i]*math.cos(angle*math.pi/180) , scan_data.ranges[i]*math.sin(angle*math.pi/180)]
         

         RobotAdvr_XY_MapRepere =[(RobotAdvr_XY_LidarRepere[0]*math.cos(Nav_CurrentAngle_deg_d*math.pi/180)) + (-RobotAdvr_XY_LidarRepere[1]*math.sin(Nav_CurrentAngle_deg_d*math.pi/180))+ ( Nav_CurrentX_mm_d / 1000 )
                                     ,RobotAdvr_XY_LidarRepere[0]*math.sin(Nav_CurrentAngle_deg_d*math.pi/180)  + RobotAdvr_XY_LidarRepere[1]*math.cos(Nav_CurrentAngle_deg_d*math.pi/180)+ ( Nav_CurrentY_mm_d / 1000 ) ]
         EuclidienneDistance =  distance(RobotAdvr_XY_MapRepere[0],Nav_CurrentX_mm_d/1000,RobotAdvr_XY_MapRepere[1],Nav_CurrentY_mm_d/1000)
         if (0< RobotAdvr_XY_MapRepere[0] < 3) and ( 0< RobotAdvr_XY_MapRepere[1] <2 ) : # and (EuclidienneDistance < 0.5):
                RobotAdvr_Scanning.append([round(RobotAdvr_XY_MapRepere[0],4),
                                           round(RobotAdvr_XY_MapRepere[1],4)])
                DistAdvr.append(round(EuclidienneDistance,3))
            
    DistAdvr_Copy = copy.deepcopy(DistAdvr)
    RobotAdvr_Scanning_CLONE =copy.deepcopy(RobotAdvr_Scanning)
    count =0
    while (len(DistAdvr_Copy) > 0):
        
        count=count+1
        j,Trash_Value=indice(DistAdvr_Copy)
       
        k=j
        while ((j< (len(DistAdvr_Copy)-1))and((math.fabs(DistAdvr_Copy[j]-DistAdvr_Copy[j+1]))<0.1)) :
            
            j=j+1

        while ((k >0 )and (math.fabs((DistAdvr_Copy[k]-DistAdvr_Copy[k-1]))<0.1) ):
            #DistAdvr_Copy.pop(k)
            k=k-1  
        for z in range(k,j+1,1) :
            #print(k)
            DistAdvr_Copy.pop(k)
            
            
def min_range_index(ranges, count):
    i = 0
    index_min = 0
    global min
    while ranges[i] == 0.0:
        i = i+1

    min = ranges[i]
    index_min = i

    for j in range(i+1, count-2):
        if (ranges[j] < min and ranges[j] != 0.0):
            min = ranges[j]
            index_min = j

    return (min, index_min)

def my_round(float):
    if ((float-int(float)) > 0.5):
        return (int(float) + 1)
    else:
        return int(float)

######################################### MECHANISMS FUNCTIONS START HERE #####################################
class Controller:
   
    def __init__(self,ttyStr='/dev/MAESTROCAMERA',device=0x0c):
        self.usb = serial.Serial(ttyStr)
        self.PololuCmd = chr(0xaa) + chr(device)
        self.Targets = [0] * 24
        self.Mins = [0] * 24
        self.Maxs = [0] * 24
        
    def close(self):
        self.usb.close()

    def sendCmd(self, cmd):
        cmdStr = self.PololuCmd + cmd
        if PY2:
            self.usb.write(cmdStr)
        else:
            self.usb.write(bytes(cmdStr,'latin-1'))

    def setRange(self, chan, min, max):
        global ServoBot
        global ServoCharacs
        self.Mins[chan] = min
        self.Maxs[chan] = max

    def getMin(self, chan):
        return self.Mins[chan]

    def getMax(self, chan):
        return self.Maxs[chan]
        
    def angle_Deg_QuartMicroSec(self,angleDeg,slope,offset):
        angleMicroSec=int(slope*angleDeg+offset)
        return angleMicroSec
                  
    def setTarget(self, chan,targetPos):
        target=targetPos
        if self.Mins[chan] > 0 and target < self.Mins[chan]:
            target = self.Mins[chan]
        if self.Maxs[chan] > 0 and target > self.Maxs[chan]:
            target = self.Maxs[chan]   
        lsb = target & 0x7f #7 bits for least significant byte
        msb = (target >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
        self.Targets[chan] = target
        
    def setSpeed(self, chan, speed):
        lsb = speed & 0x7f #7 bits for least significant byte
        msb = (speed >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    def setAccel(self, chan, accel):
        lsb = accel & 0x7f #7 bits for least significant byte
        msb = (accel >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    def getPosition(self, chan):
        cmd = chr(0x10) + chr(chan)
        self.sendCmd(cmd)
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb

    def isMoving(self, chan):
        if self.Targets[chan] > 0:
            if self.getPosition(chan) != self.Targets[chan]:
                return True
        return False
    
    def getMovingState(self):
        cmd = chr(0x13)
        self.sendCmd(cmd)
        if self.usb.read() == chr(0):
            return False
        else:
            return True
            
    def runScriptSub(self, subNumber):
        cmd = chr(0x27) + chr(subNumber)
        self.sendCmd(cmd)

    def stopScript(self):
        cmd = chr(0x24)

    def ServosInit(self):
        global ServoBot
        global ServoCharacs
        for Key , Value in ServoBot.items():
            ServCaracs=ServoCharacs.get(Value[1])
            self.setRange(Value[0],ServCaracs[0],ServCaracs[1])
            self.setSpeed(Value[0],200)
            self.setAccel(Value[0],100)


    def MoveServo(self,ServoToMove,angle,wait,sens=1):
        global ServoBot
        global ServoCharacs
        ServoMoveCharacs=ServoCharacs.get(ServoToMove[1])                
        ServMv_TargetPos=self.angle_Deg_QuartMicroSec(sens*angle.value,ServoMoveCharacs[2],ServoToMove[2])
        self.setTarget(ServoToMove[0],ServMv_TargetPos)
        if wait=="wait":
            while self.isMoving(ServoToMove[0]):
                pass
        elif wait=="notwait":
            pass #TODO EXIT !!!!!!!!!
                           
    def CherryOnTop(self,ServoWheel,ServoPress,CherryNb):
        global ServoBot
        global ServoCharacs
        #ServoWhlCharacs=ServoCharacs.get(ServoWheel[1])
        #ServoPrsCharacs=ServoCharacs.get(ServoPress[1])
        ServWhl_TargetAng=27*(5-CherryNb)
        self.MoveServo(ServoWheel,ServWhl_TargetAng,"wait")
        self.MoveServo(ServoPress,90,"wait")
        self.MoveServo(ServoPress,0,"wait")
        
    def ScissorLift(self,ServoScissor,Height,wait):
        global ServoBot
        global ServoCharacs
        if (Height.value<228):
             ServSc_TargetPos=int(9655-(11.2*Height.value))
        else:
             ServSc_TargetPos=int(16035-(40*Height.value))
        self.setTarget(ServoScissor[0],ServSc_TargetPos)    
        if wait=="wait":
            while self.isMoving(ServoScissor[0]):
                pass
        elif wait=="notwait":
            pass  

def Convert_intMmToSteps(MmDistnace):    
    return((int)( (NUM_STEPS_REV*MmDistnace)/(2*math.pi*RAYON_POULIE_MM)))

def Stepper_vTurn(Target_Pos_Mm):
    global Current_StepperPos_Mm
    pi=pigpio.pi()
    pi.set_mode(DIR,pigpio.OUTPUT)
    pi.set_mode(STEP,pigpio.OUTPUT)
    DistanceToTravel = Target_Pos_Mm-Current_StepperPos_Mm 
    Target_Pos_Steps =Convert_intMmToSteps(DistanceToTravel)

         
    if (Target_Pos_Steps<0):
       pi.write(DIR,CCW) 
    else :
       pi.write(DIR,CW)

    

    
    Target_Pos_StepsAcc=int(Target_Pos_Steps/10)
    accel_sleep=abs(Target_Pos_StepsAcc/10000)
    pi.set_PWM_dutycycle(STEP, 128)  # PWM 1/2 On 1/2 Off
    pi.set_PWM_frequency(STEP, 10000)  # 10000 pulses per second   
    time.sleep(accel_sleep)

    Target_Pos_StepsMax=int(Target_Pos_Steps*0.9)
    max_sleep=abs(Target_Pos_StepsMax/20000)
    pi.set_PWM_frequency(STEP,20000)  # 20000 pulses per second
    pi.set_PWM_dutycycle(STEP, 128) 
    time.sleep(max_sleep)
    pi.set_PWM_dutycycle(STEP, 0)
    
    if (pi.read(UPPER_LIMIT_SWITCH)== False):       
       Current_StepperPos_Mm = UPPER_INIT_POS
    else :
       Current_StepperPos_Mm = Target_Pos_Mm
    
    pi.stop()


def Stepper_vInit():
    global Current_StepperPos_Mm

    pi=pigpio.pi()
    pi.set_mode(UPPER_LIMIT_SWITCH,pigpio.INPUT)
    pi.set_pull_up_down(UPPER_LIMIT_SWITCH, pigpio.PUD_DOWN)
    while (pi.read(UPPER_LIMIT_SWITCH)):
      pi.write(DIR,CW) 
      pi.set_PWM_dutycycle(STEP, 128)  # PWM 1/2 On 1/2 Off
      pi.set_PWM_frequency(STEP, 7000)  # 7000 pulses per second 
    pi.set_PWM_dutycycle(STEP,0)
    Current_StepperPos_Mm = UPPER_INIT_POS
    time.sleep(0.5)


def CherryAcquirDepos (StepperHeight , BasketServoLevel):
    servo =Controller()
    global ServoBot
    global ServoCharacs
    if(StepperHeight == Stepper2Height.BasketHeight_en):
        Stepper_vTurn(StepperHeight.value)
        if (BasketServoLevel == BasketServoLevels.IdleSide_en):
            servo.MoveServo(ServoBot.get("ServoBasket"),BasketServoLevel,"notwait")
        elif (BasketServoLevel == BasketServoLevels.RightSide_en):
            servo.MoveServo(ServoBot.get("ServoBasket"),BasketServoLevel,"notwait")
            servo.MoveServo(ServoBot.get("ServoBasketDoorRight"),DoorBasketServos.OpenLevel_en,"notwait",-1)
            time.sleep(TIME_TO_COMPLETE)
            servo.MoveServo(ServoBot.get("ServoBasketDoorRight"),DoorBasketServos.ClosedLevel_en,"notwait")
            servo.MoveServo(ServoBot.get("ServoBasket"),BasketServoLevels.IdleSide_en,"notwait")
        elif (BasketServoLevel == BasketServoLevels.LeftSide_en):
            servo.MoveServo(ServoBot.get("ServoBasket"),BasketServoLevel,"notwait")
            servo.MoveServo(ServoBot.get("ServoBasketDoorLeft"),DoorBasketServos.OpenLevel_en,"notwait")
            time.sleep(TIME_TO_COMPLETE)
            servo.MoveServo(ServoBot.get("ServoBasketDoorLeft"),DoorBasketServos.ClosedLevel_en,"notwait")
            servo.MoveServo(ServoBot.get("ServoBasket"),BasketServoLevels.IdleSide_en,"notwait")
    elif(StepperHeight == Stepper2Height.CherrySupport_en):
        servo.MoveServo(ServoBot.get("ServoBasket"),BasketServoLevels.IdleSide_en,"notwait")
        Stepper_vTurn(StepperHeight.value)


def StealingDoors(Reservoir , DoorLevel , StackColor):
    servo =Controller()
    global ServoBot
    global ServoCharacs

    if (Reservoir.Side == ReservoirRobot2Side.FrontReservoir_en ):
        if(DoorLevel == StealingDoorsServos.FrontOpened_en):
            servo.MoveServo(ServoBot.get("StealServoFrontRight"),DoorLevel,"notwait")
            servo.MoveServo(ServoBot.get("StealServoFrontLeft"),DoorLevel,"notwait",-1)
            Reservoir.Couche1=Color.vide_en
            Reservoir.Couche1=Color.vide_en
            Reservoir.Couche1=Color.vide_en
            Reservoir.Cerise =0
        elif(DoorLevel == StealingDoorsServos.FrontClosed_en) :
            servo.MoveServo(ServoBot.get("StealServoFrontRight"),DoorLevel,"notwait")
            servo.MoveServo(ServoBot.get("StealServoFrontLeft"),DoorLevel,"notwait",-1)
            Reservoir.Couche1 = StackColor
            Reservoir.Couche2 = StackColor
            Reservoir.Couche3 = StackColor
                #Reservoir.Cerise = CAMERA !!!!!!!!!!!!!!
            
        else :
            Master_TaskState = 0
    elif (Reservoir.Side == ReservoirRobot2Side.BackReservoir_en):
        if(DoorLevel == StealingDoorsServos.BackOpened_en):
            servo.MoveServo(ServoBot.get("StealServoBackRight"),DoorLevel,"notwait")
            servo.MoveServo(ServoBot.get("StealServoBackLeft"),DoorLevel,"notwait",-1)
            Reservoir.Couche1=Color.vide_en
            Reservoir.Couche1=Color.vide_en
            Reservoir.Couche1=Color.vide_en
            Reservoir.Cerise =0
        elif(DoorLevel == StealingDoorsServos.BackClosed_en) :
            servo.MoveServo(ServoBot.get("StealServoBackRight"),DoorLevel,"notwait")
            servo.MoveServo(ServoBot.get("StealServoBackLeft"),DoorLevel,"notwait",-1)            
            Reservoir.Couche1 = StackColor
            Reservoir.Couche2 = StackColor
            Reservoir.Couche3 = StackColor
                #Reservoir.Cerise = CAMERA !!!!!!!!!!!!!!

        else :
            Master_TaskState = 0
   

def ExecuteTaskBob():

    global CurrentTaskBob
    global LineInExecution2
    global MASTER_CurrentMasterState_en

    
    if CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseFrontArms_Unknown_en:
        print('EXECUTING FIRST TASK')    
        StealingDoors(FrontRes,StealingDoorsServos.FrontClosed_en,Color.Unknown_en)
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en
            
    #elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployBackArms_en:  
    #    StealingDoors(BackRes,StealingDoorsServos.BackOpened_en,Color.Unknown_en)
    #    CurrentTaskBob=MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en

    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en:    
        StealingDoors(BackRes,StealingDoorsServos.BackClosed_en,Color.Unknown_en)
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en       
    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployFrontArms_en:
        StealingDoors(FrontRes,StealingDoorsServos.FrontOpened_en,Color.Unknown_en)
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en

    #elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseFrontArms_Unknown_en:    
    #    StealingDoors(FrontRes,StealingDoorsServos.FrontClosed_en,Color.Unknown_en)
    #    CurrentTaskBob=MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployBackArms_en

    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployBackArms_en:  
        StealingDoors(BackRes,StealingDoorsServos.BackOpened_en,Color.Unknown_en)
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en

    #elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en:    
    #    StealingDoors(BackRes,StealingDoorsServos.BackClosed_en,Color.Unknown_en)  
    #    CurrentTaskBob=MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherries_en 

    elif CurrentTaskBob ==MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesFirst_en :
        if LineInExecution2==1:
            CherryAcquirDepos (Stepper2Height.CherrySupport_en ,BasketServoLevels.IdleSide_en)
            LineInExecution2+=1
            time.sleep(0.5) #TODO test and modify
        if LineInExecution2==2:
            CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.IdleSide_en)
            LineInExecution2=1
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en

    #elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_IdleCollector_en :
    #    CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.IdleSide_en)
    #    CurrentTaskBob=MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherries_en 

    elif CurrentTaskBob ==MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesSecond_en :
        if LineInExecution2==1:
            CherryAcquirDepos (Stepper2Height.CherrySupport_en ,BasketServoLevels.IdleSide_en)
            time.sleep(0.5) #TODO test and modify
            LineInExecution+=1
        if LineInExecution2==2:
            CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.IdleSide_en)
            LineInExecution2=1
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en

    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_PlaceCherriesRight_en :
        if LineInExecution2==1:
            Stepper_vTurn(Stepper2Height.BasketHeight_en.value-10)
            LineInExecution2+=1
        if LineInExecution2==2:
            CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.RightSide_en)
            LineInExecution2=1
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en

    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_PlaceCherriesLeft_en :
        if LineInExecution2==1:
            Stepper_vTurn(Stepper2Height.BasketHeight_en.value-10)
            LineInExecution2+=1
        if LineInExecution2==2:
            CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.LeftSide_en)
            LineInExecution2=1
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_MasterReady_en

def ExecuteTaskBobFutur():

    global CurrentTaskBob
    global LineInExecution2
    global MASTER_c

    
    if CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseFrontArms_Unknown_en:
        print('EXECUTING FIRST TASK')    
        StealingDoors(FrontRes,StealingDoorsServos.FrontClosed_en,Color.Unknown_en)
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Ready_en
            
    #elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployBackArms_en:  
    #    StealingDoors(BackRes,StealingDoorsServos.BackOpened_en,Color.Unknown_en)
    #    CurrentTaskBob=MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en

    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en:    
        StealingDoors(BackRes,StealingDoorsServos.BackClosed_en,Color.Unknown_en)
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Ready_en       
    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployFrontArms_en:
        StealingDoors(FrontRes,StealingDoorsServos.FrontOpened_en,Color.Unknown_en)
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Ready_en

    #elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseFrontArms_Unknown_en:    
    #    StealingDoors(FrontRes,StealingDoorsServos.FrontClosed_en,Color.Unknown_en)
    #    CurrentTaskBob=MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployBackArms_en

    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_DeployBackArms_en:  
        StealingDoors(BackRes,StealingDoorsServos.BackOpened_en,Color.Unknown_en)
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Ready_en

    #elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_CloseBackArms_Unknown_en:    
    #    StealingDoors(BackRes,StealingDoorsServos.BackClosed_en,Color.Unknown_en)  
    #    CurrentTaskBob=MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherries_en 

    elif CurrentTaskBob ==MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesFirst_en :
        if LineInExecution2==1:
            CherryAcquirDepos (Stepper2Height.CherrySupport_en ,BasketServoLevels.IdleSide_en)
            LineInExecution2+=1
            time.sleep(0.5) #TODO test and modify
        if LineInExecution2==2:
            CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.IdleSide_en)
            LineInExecution2=1
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Ready_en

    #elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_IdleCollector_en :
    #    CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.IdleSide_en)
    #    CurrentTaskBob=MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherries_en 

    elif CurrentTaskBob ==MASTER2_MicroTasks_ten.MASTER2_MicroTask_CollectCherriesSecond_en :
        if LineInExecution2==1:
            CherryAcquirDepos (Stepper2Height.CherrySupport_en ,BasketServoLevels.IdleSide_en)
            time.sleep(0.5) #TODO test and modify
            LineInExecution+=1
        if LineInExecution2==2:
            CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.IdleSide_en)
            LineInExecution2=1
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Ready_en

    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_PlaceCherriesRight_en :
        if LineInExecution2==1:
            Stepper_vTurn(Stepper2Height.BasketHeight_en.value-10)
            LineInExecution2+=1
        if LineInExecution2==2:
            CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.RightSide_en)
            LineInExecution2=1
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Ready_en

    elif CurrentTaskBob == MASTER2_MicroTasks_ten.MASTER2_MicroTask_PlaceCherriesLeft_en :
        if LineInExecution2==1:
            Stepper_vTurn(Stepper2Height.BasketHeight_en.value-10)
            LineInExecution2+=1
        if LineInExecution2==2:
            CherryAcquirDepos (Stepper2Height.BasketHeight_en ,BasketServoLevels.LeftSide_en)
            LineInExecution2=1
        MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Ready_en

############################################### MAIN STARTS HERE ######################################
def main():

    # initialize global variables
    global RisingEdge_variable
    RisingEdge_variable = 0

    # CURRENT STATES
    global MASTER_CurrentMachineState_en
    global MASTER_CurrentMasterState_en
    global MASTER_CurrentNavigatorState_en

    global CurrentTask_Blue_en
    global CurrentTask_Green_en

    MASTER_CurrentMachineState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Idle_en
    MASTER_CurrentMasterState_en = MASTER_MasterStates_ten.MASTER_Start_en #put back to stop
    MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorIdle_en
    # TODO put back to idle

    # table of plates coordinates
    global TargetStack_X , TargetStack_Y
    TargetStack_X , TargetStack_Y = 0 , 0

    servo= Controller()

    # NODE INITIALIZATION : 
    rospy.init_node('HafnaouiSMach', anonymous=True)

    # STEPPER INITIALIZATION : 
    #subprocess.run(['sudo', 'pigpiod', '-s1'])
    #Stepper_vInit()

    ############  STRATEGY PUBLISHERS  ##################
    global ScreenConfigurations
    ScreenConfigurations = rospy.Publisher("ScreenConfigurations", Int8MultiArray, queue_size=10)

    global SearchRequest
    SearchRequest = rospy.Publisher("SearchRequest", Int8, queue_size=10)

    global XY_Request
    XY_Request = rospy.Publisher("XY_Request", Bool, queue_size=10)

    ############  NAVIGATION PUBLISHERS  ##################
    global MASTER_Orders_Publisher
    MASTER_Orders_Publisher=rospy.Publisher("MASTER_Orders",Float32MultiArray, queue_size=10)
    
    
    ############ SUBSCRIBERS HERE : 
    rospy.Subscriber("Robot_Status", Int8, Robot_Status_MessageRecieved_CALLBACK)
    rospy.Subscriber("scan",LaserScan,Scan_Gateau)
    rospy.Subscriber("Location_state",Odometry,Location_state_CALLBACK)
    rospy.Subscriber("NextTarget",Int32MultiArray, NextTarget_Callback )
    
    rospy.Subscriber ("MyColor", String, MyColor_Callback)
    rospy.Subscriber ("MyStrategy", String, MyStrategy_Callback)
    rospy.Subscriber ("Confirmation", Bool, Confirmation_Callback)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['NavigatorReached'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Idle', Idle(),
                               transitions={'Go to: Idle': 'Idle',
                                            'Go to: Preparation': 'Preparation'})

        smach.StateMachine.add('Preparation', Preparation(),
                               transitions={'Go to: Preparation': 'Preparation',
                                            'Go to: WaitForSignal': 'WaitForSignal'})

        smach.StateMachine.add('WaitForSignal', WaitForSignal(),
                               transitions={'Go to: MoveOn': 'MoveOn',
                                            'Go to: WaitForSignal': 'WaitForSignal'})

        smach.StateMachine.add('MoveOn', MoveOn(),
                               transitions={'Go to: WaitToReach': 'WaitToReach',
                                            'Go to: MoveOn': 'MoveOn'})

        smach.StateMachine.add('WaitToReach', WaitToReach(),
                               transitions={'Go to: SuddenStop': 'SuddenStop',
                                            'Go to: DestinationReached': 'DestinationReached',
                                            'Go to: WaitToReach': 'WaitToReach'})

        smach.StateMachine.add('SuddenStop', SuddenStop(),
                               transitions={'Go to: confirmed stop': 'ConfirmedStop',
                                            'Go to: sudden stop': 'SuddenStop'})

        smach.StateMachine.add('ConfirmedStop', ConfirmedStop(),
                               transitions={'Go to: ConfirmedStop': 'ConfirmedStop',
                                            'Go to: MoveOn': 'MoveOn'})
        smach.StateMachine.add('DestinationReached', DestinationReached(),
                               transitions={'Go to: MoveOn': 'MoveOn',
                                            'Go to: ExecuteTask': 'ExecuteTask',
                                            'Go to: Parallel_MoveOn':'Parallel_MoveOn'})
        smach.StateMachine.add('ExecuteTask', ExecuteTask(),
                               transitions={'Go to: waitToFinishTask': 'waitToFinishTask',
                                            })
        smach.StateMachine.add('waitToFinishTask', waitToFinishTask(),
                               transitions={'Go to: waitToFinishTask': 'waitToFinishTask' ,
                                'Go to: MoveOn' : 'MoveOn',
                                'Go to: Parallel_MoveOn':'Parallel_MoveOn'})
        smach.StateMachine.add('Parallel_MoveOn', Parallel_MoveOn(),
                               transitions={'Go to: Parallel_MoveOn': 'Parallel_MoveOn',
                                            'Go to: Parallel_WaitToReach':'Parallel_WaitToReach'
                                            })
        smach.StateMachine.add('Parallel_WaitToReach', Parallel_WaitToReach(),
                               transitions={'Go to: Parallel_WaitToReach': 'Parallel_WaitToReach',
                                              'Go to: Parallel_DestinationReached' : 'Parallel_DestinationReached',
                                              'Go to: Parallel_SuddenStop': 'Parallel_SuddenStop'
                                            })

        smach.StateMachine.add('Parallel_DestinationReached', Parallel_DestinationReached(),
                               transitions={'Go to: ExecuteTask' : 'ExecuteTask',
                                'Go to: Parallel_DestinationReached' : 'Parallel_DestinationReached',
                                'Go to: MoveOn' : 'MoveOn'
                                            })

        smach.StateMachine.add('Parallel_SuddenStop', Parallel_SuddenStop(),
                               transitions={'Go to: Parallel_ConfirmedStop': 'Parallel_ConfirmedStop',
                                'Go to: Parallel_SuddenStop': 'Parallel_SuddenStop'})

        smach.StateMachine.add('Parallel_ConfirmedStop', Parallel_ConfirmedStop(),
                               transitions={'Go to: Parallel_MoveOn': 'Parallel_MoveOn',
                                'Go to: Parallel_ConfirmedStop': 'Parallel_ConfirmedStop'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/Hafnaoui_Jr')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    
    main()
