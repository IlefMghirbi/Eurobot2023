#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray

from time import sleep
from enum import Enum



""" YOU'LL FIND HERE THE COMMON CLASSES THAT ARE USED BY MORE THAN 1 NODE ;) """
##################################### TYPEDEF ENUMS HERE ###############################
class MASTER_SMachineStates_ten (Enum): # The state that I am executing 

    MASTER_SMachine_Idle_en = 1
    MASTER_SMachine_Preparation_en = 2
    MASTER_SMachine_WaitForSignal_en = 3
    MASTER_SMachine_MoveOn_en = 4
    MASTER_SMachine_WaitToReach_en = 5
    MASTER_SMachine_DestinationReached_en = 6
    MASTER_SMachine_ExecuteTask_en = 7
    MASTER_SMachine_WaitToFinishTask_en = 8

    MASTER_SMachine_SuddenStop_en = 9 
    MASTER_SMachine_ConfirmedStop_en = 10

    MASTER_SMachine_ParallelSuddenStop_en = 14 
    MASTER_SMachine_ParallelConfirmedStop_en = 15

    MASTER_SMachine_ParallelMoveOn_en = 11
    MASTER_SMachine_ParallelWaitToReach_en = 12
    MASTER_SMachine_ParallelDestReached_en = 13


class MASTER_MasterStates_ten (Enum):

    MASTER_Stop_en = 1
    MASTER_Start_en = 2
    MASTER_MasterReady_en = 3
    MASTER_MasterBusy_en = 4
    MASTER_ParallelMasterReady_en = 5
    MASTER_ParallelMasterReadyToMove_en = 6
    MASTER_ParallelMasterReadyToExecute_e = 7
    MASTER_ParallelMasterBusy_en = 8
    MASTER_Preparing_en = 9

class MASTER_NavigatorStates_ten (Enum):

    MASTER_NavigatorIdle_en = 1
    MASTER_NavigatorReady_en = 2
    MASTER_NavigatorBusy_en = 3
    MASTER_NavigatorGoingToRespond_en = 4
    MASTER_NavigatorReachedDest_en = 5


    MASTER_ParallelNavigatorReady_en = 6
    MASTER_ParallelNavigatorBusy_en = 7
    MASTER_ParallelNavigatorGoingToRespond_en = 8
    MASTER_ParallelNavigatorReachedDest_en = 9

    MASTER_NavigatorConfirmedStop_en = 10
    MASTER_NavigatorParallelConfirmedStop_en = 11

class MASTER_NavigatorOrders_ten (Enum):

    Navigator_SuddenStop_en = 1
    Navigator_MoveOn_en = 2

class MASTER_MicroTasks_ten (Enum):

    MASTER_MicroTask_Idle_en=0

    MASTER_MicroTask_CollectPinkLeft_en = 23
    MASTER_MicroTask_CollectPinkRight_en = 24
    MASTER_MicroTask_DeployLeftArms_en = 1
    MASTER_MicroTask_DeployRightArms_en = 2

    MASTER_MicroTask_CloseRightArms_en  = 3
    MASTER_MicroTask_CloseLeftArms_en   = 4
    MASTER_MicroTask_CloseBothArms_en   = 5
    
    
    MASTER_MicroTask_GrabPinkRight_en = 6 # CLOSE ARMS --> OPEN SECURITY --> ACTIVATE MONTE CHARGE THEN LIFT
    MASTER_MicroTask_LiftAndPutCherryRight_en = 7 #CONTINUE LIFTING AND PLACE CHERRY (parallel task)
    MASTER_MicroTask_GrabPinkLeft_en = 8 
    MASTER_MicroTask_LiftAndPutCherryLeft_en= 9
    
    MASTER_MicroTask_GrabYellowRight_en = 10 # CLOSE ARMS --> OPEN SECURITY --> ACTIVATE MONTE CHARGE THEN LIFT
    MASTER_MicroTask_LiftYellowRight_en = 11 #CONTINUE LIFTING UNTIL DONE (parallel task)
    MASTER_MicroTask_GrabYellowLeft_en = 12 
    MASTER_MicroTask_LiftYellowLeft_en = 13

    MASTER_MicroTask_GrabBrown_en = 14 # JUST CLOSING THE ARMS TO GRAB THE BROWN STACK
    MASTER_MicroTask_SortLegend_en = 15 #TRI GATEAU LEGENDAIRE
    MASTER_MicroTask_SortNonLegend1Stack_en = 16 #TRI GATEAU NON LEGENDAIRE USING 1 STACK
    MASTER_MicroTask_SortNonLegend2Stacks_en =17 #TRI GATEAU NON LEGENDAIRE USING 2 STACKS
  
    MASTER_MicroTask_PlaceTwo_en = 18 #TO PLACE 2 LEGEND CAKES AT THE SAME TIME 
    MASTER_MicroTask_PlaceOneLeft_en = 19 #TO PLACE 1 LEGEND CAKE
    MASTER_MicroTask_PlaceOneRight_en = 20 
    MASTER_MicroTask_PlaceOneLegendAfterTwo_en=21

    MASTER_MicroTask_GrabUnknown_en = 24
    
    MASTER_MicroTask_PutCherryRightSide = 22
    MASTER_MicroTask_PutCherryLeftSide = 23

   
class MASTER2_MicroTasks_ten (Enum):

    MASTER2_MicroTask_DeployFrontArms_en = 1
    MASTER2_MicroTask_DeployBackArms_en = 2

    MASTER2_MicroTask_CloseFrontArms_Unknown_en = 3
    MASTER2_MicroTask_CloseFrontArms_Brown_en =4
    MASTER2_MicroTask_CloseBackArms_Unknown_en = 5
    MASTER2_MicroTask_CloseBackArms_Brown_en = 6

    MASTER2_MicroTask_CollectCherriesSecond_en = 7 #lmarra thenia pour éviter boucle mtaa execute task

    MASTER2_MicroTask_PlaceCherriesRight_en = 8
    MASTER2_MicroTask_PlaceCherriesLeft_en = 9

    MASTER2_MicroTask_IdleCollector_en = 10

    MASTER2_MicroTask_CollectCherriesFirst_en=11
    MASTER2_MicroTask_PrepIdle = 12

class Color(Enum):
    vide_en=0
    Marron_en =1
    Jaune_en=2
    Rose_en=3
    Unknown_en=99

class Reservoir :
    def __init__(self,Couche1,Couche2,Couche3,Cerise,ServoPos,StepperHeight):
        self.Couche1=Couche1
        self.Couche2=Couche2
        self.Couche3=Couche3
        self.Cerise=Cerise
        self.ServoPos=ServoPos
        self.StepperHeight=StepperHeight

#enum for controlling the possible heights that the stpper would make 
class StepperHeight(Enum):
    MARRON_NIVEAU_HAUT_en=60
    STATION_ASS_1_en=0
    ROSE_JAUNE_NIVEAU_HAUT_en=200
    STEPPER_SEC_MARGE_en=0
"""
class ContentState(Enum):
    StolenRight=0
    StolenLeft=1
    LegalRight=2
    LegalLeft=3
"""
#enum for controlling the scissorlift mecanism (it conatains the value to command the scissor lift servos)
class ScissorLift(Enum):
    FloorLevelCouche3_en=150
    FloorLevelCouche2_en=100
    FloorLevelCouche1_en=50
    Res3LevelCouche3_en=80
    Res3LevelCouche2_en=100
    Res3LevelCouche1_en=70

    CherryCouche1Level_en=50
    CherryCouche2Level_en=100
    CherryCouche3Level_en=150
    SecLevel_en = 0

    ServoRight_en= 1
    ServoLeft_en = 0

    AboveLeftDoor=90
    AboveRightDoor=60




#enum for controlling Doors servos in order to collect stacks or to put cakes 
class ServoDoors(Enum):
    ClosedRight_en = 50
    DoorSecRight_en = 60
    OpenRight_en = 80
    ServoDoorRight_en=1
    
    ClosedLeft_en = 30
    Semi_ClosedLeft_en = 40
    OpenLeft_en = 90
    DoorSecLeft_en=70
    ServoDoorleft_en=50

#enum to know which action to make after controlling doors servos (testing over this enum allow to update the reservoir objects) 
class AfterDoorRole (Enum):
    Other_en=0
    PlateR_en = 1
    Transfert_en = 2
    
#enum to know with cherry mecanism to activate right or left side 
class CherrySide (Enum):
    CherrySideRight_en =1
    CherrySideLeft_en  =0 

#enum contaning the possible angle commands for the arm servos
class ServoBrasPos (Enum):
    SERVO_POS_DROIT_en=-100
    SERVO_POS_GAUCHE_en=100
    SERVO_POS_IDLE_en=0
   
class Stepper2Height(Enum):
    BasketHeight_en =    26+49 #39
    CherrySupport_en  = 223.5

class BasketServoLevels(Enum):
    RightSide_en = 20
    LeftSide_en  = 20
    IdleSide_en  = 0

class DoorBasketServos (Enum) :
    OpenLevel_en  = 100
    ClosedLevel_en= 0

class StealingDoorsServos (Enum):
    FrontOpened_en = 0
    FrontClosed_en = 88
    BackOpened_en  = 0
    BackClosed_en  = 88
class ReservoirRobot2Side(Enum):
    FrontReservoir_en = 1
    BackReservoir_en  = 0
    
class ReservoirRobot2 :
    def __init__(self,Couche1,Couche2,Couche3,Cerise,Side):
        self.Couche1=Couche1
        self.Couche2=Couche2
        self.Couche3=Couche3
        self.Cerise=Cerise
        self.Side=Side
        
