import rospy
import smach

from time import sleep
import time
import math
import pigpio

import drivers
import RPi.GPIO as GPIO
import time
from encoder import Encoder
import serial
from sys import version_info
from std_msgs.msg import Int8
from std_msgs.msg import String
from std_msgs.msg import Bool

currentScreen = "none"
currentCursorIndex = 0
CurrentScore = 0

GPIO.setmode(GPIO.BCM)
prevTime=0
BUTTON_PIN=14
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

######################################### screen configurations #####################""
def buttonClicked(self):
    print("i got clicked")
    GPIO.remove_event_detect(BUTTON_PIN)
    global MyColor
    global prevTime
    global PlateConfig
    global MyStrategy
    global Confirmation
    global currentScreen

    print(time.time()-prevTime)
    if time.time()-prevTime <1:
        print("returned")
        return 0
    prevTime=time.time()

    confirmation_pub = rospy.Publisher("confirmation",Bool, queue_size=10)
    if currentScreen == "mainScreen":
        if currentCursorIndex==1:
            teamColorInit()
        elif currentCursorIndex==2:
            startingZoneInit()
        elif currentCursorIndex==3:
            strategyInit()
        elif currentCursorIndex==4:
            Localize()
        elif currentCursorIndex==5:
            Confirmation = True
            confirmation_pub.publish(Confirmation)
            scoreInit(0)
            sleep(1)

    elif currentScreen == "teamColor":
        MyColor_pub = rospy.Publisher("MyColor",String, queue_size=10)
        if currentCursorIndex == 2:
            MyColor = "BLUE"
            MyColor_pub.publish(MyColor)
        else:
            MyColor = "GREEN"
            MyColor_pub.publish(MyColor)

        print("str of color going to master is :",MyColor)
        mainScreen()

    elif currentScreen == "strategy":
        MyStrategy_pub = rospy.Publisher("MyStrategy",String, queue_size=10)
        if currentCursorIndex == 2:
            MyStrategy = "Hard"
            MyStrategy_pub.publish(MyStrategy)
        elif currentCursorIndex == 3 :
            MyStrategy = "Medium"
            MyStrategy_pub.publish(MyStrategy)
        else :
            MyStrategy ="Easy"
            MyStrategy_pub.publish(MyStrategy) 
        print("str of match condition going to master is :",MyStrategy)       
        mainScreen()

    elif currentScreen=="startingZone":
        if MyColor =="BLUE":
           if currentCursorIndex == 2 :
                PlateConfig = "Plate1"
           elif currentCursorIndex == 3 :
                PlateConfig ="Plate5"
           elif currentCursorIndex == 4 :
                PlateConfig ="Plate9"
           elif currentCursorIndex == 5 :
                PlateConfig ="Plate5"
           elif currentCursorIndex == 6 :
                PlateConfig ="Plate7"               

        if MyColor =="GREEN":
           if currentCursorIndex == 2 :
                PlateConfig = "Plate2"
           elif currentCursorIndex == 3 :
                PlateConfig ="Plate6"
           elif currentCursorIndex == 4 :
                PlateConfig ="Plate10"
           elif currentCursorIndex == 5 :
                PlateConfig ="Plate4"
           elif currentCursorIndex == 6 :
                PlateConfig ="Plate8"          
                   
        
        mainScreen()
        print("str of PLate going to master is :",PlateConfig)
    GPIO.add_event_detect(BUTTON_PIN,GPIO.RISING,callback=buttonClicked,bouncetime=200)    

def valueChanged(value, direction):
    global currentScreen
    #print("* New value: {}, Direction: {}".format(value, direction))
    if currentScreen=="score":
        return 0
    print("current sensor index" + str(currentCursorIndex))
    # Right (+1)
    if direction == 'R':
        if currentScreen == "mainScreen":
            if currentCursorIndex != 5:
                setCursor(display, currentCursorIndex+1)

        elif currentScreen == "teamColor":
            if currentCursorIndex == 2:
                setCursor(display, 3)

        elif currentScreen == "startingZone":
            if currentCursorIndex != 7:
                setCursor(display, currentCursorIndex+1)

        elif currentScreen == "strategy":
            if currentCursorIndex != 4:
                setCursor(display, currentCursorIndex+1)


    # Left (-1)
    else:
        if currentScreen == "mainScreen":
            if currentCursorIndex != 1:
                setCursor(display, currentCursorIndex-1)
        elif currentScreen == "teamColor":
            if currentCursorIndex == 3:
                setCursor(display, 2)

        elif currentScreen=="startingZone":
            if currentCursorIndex != 1:
                setCursor(display, currentCursorIndex-1)

        elif currentScreen == "strategy":
            if currentCursorIndex != 2:
                setCursor(display, currentCursorIndex -1)


def showString(display, text='', num_line=1, num_cols=20):
        """
        Parameters: (driver, string to print, number of line to print, number of columns of your display)
        Return: This function send to display your scrolling string.
        """
        if len(text) > num_cols:
            display.lcd_display_string(text[:num_cols], num_line)
            """sleep(1)
            for i in range(len(text) - num_cols + 1):
                text_to_print = text[i:i+num_cols]
                display.lcd_display_string(text_to_print, num_line)
                sleep(0.2)
            sleep(1)"""
        else:
            display.lcd_display_string(text, num_line)

display = drivers.Lcd()
BlueStartingZoneStringArray=["  Plate1  |  Plate3",
                         "  Plate5  |  Plate7",
                         "  Plate9  |     "  ]

GreenStartingZoneStringArray=["  Plate2  |  Plate4",
                         "  Plate6 |  Plate8",
                         "  Plate10  |    "  ]


mainScreenStringArray= ["  Color   |  Plate ",
                        "  Strat   |  Locali",
                        "  Confirm |        ",
                        "  Lora  X -  VSN  X"]

def setCursor(display,num_line):
    global currentScreen
    global currentCursorIndex
    global MyColor



    if currentScreen=="mainScreen":
        # Clear old cursor
        showString(display, mainScreenStringArray[0], 1)
        showString(display, mainScreenStringArray[1], 2)
        showString(display, mainScreenStringArray[2], 3)

        # Add new cursor
        print("chnhot cursor fel" + str(num_line))

        if num_line in [1,3,5]:
            if num_line==1:
                display.lcd_display_string(">",1)
            elif num_line==3:
                display.lcd_display_string(">",2)
            else:
                display.lcd_display_string(">",3)
        elif num_line in [2,4]:
            if num_line == 2:
                tempMainScreenString= mainScreenStringArray[0]
                tempMainScreenString=tempMainScreenString[:11] + ">" + tempMainScreenString[12:]
                display.lcd_display_string(tempMainScreenString  ,1)
            else:
                tempMainScreenString= mainScreenStringArray[1]
                tempMainScreenString=tempMainScreenString[:11] + ">" + tempMainScreenString[12:]
                display.lcd_display_string(tempMainScreenString  ,2)




    elif currentScreen=="teamColor":
        if num_line==2:
            showString(display,"  GREEN", 3)
        else:
            showString(display,"  BLUE", 2)
        display.lcd_display_string(">",num_line)

    elif currentScreen=="strategy":
        if num_line==2:
            showString(display,"  Medium", 3)
            showString(display,"  Easy", 4)
        elif num_line == 3:
            showString(display,"  Hard", 2)    
            showString(display,"  Easy", 4)
        elif num_line==4:
            showString(display,"  Medium", 3)
            showString(display,"  Hard", 2)    
        display.lcd_display_string(">",num_line)

    elif currentScreen=="startingZone":
        # Clear old cursor
        if MyColor == "BLUE" :
            if currentCursorIndex in [2,5]:
                display.lcd_display_string(BlueStartingZoneStringArray[0]  ,2)
            elif  currentCursorIndex in [3,6]:
                display.lcd_display_string(BlueStartingZoneStringArray[1]  ,3)
            elif  currentCursorIndex in [4,7]:
                display.lcd_display_string(BlueStartingZoneStringArray[2]  ,4)
            if num_line in [2,3,4]:
               display.lcd_display_string(">",num_line)
            elif num_line in [5,6,7]:
                tempStartingZoneString= BlueStartingZoneStringArray[num_line-5]
                tempStartingZoneString=tempStartingZoneString[:11] + ">" + tempStartingZoneString[12:]
                display.lcd_display_string(tempStartingZoneString  ,num_line-3)    

        elif MyColor == "GREEN" :
            if currentCursorIndex in [2,5]:
                display.lcd_display_string(GreenStartingZoneStringArray[0]  ,2)
            elif  currentCursorIndex in [3,6]:
                display.lcd_display_string(GreenStartingZoneStringArray[1]  ,3)
            elif  currentCursorIndex in [4,7]:
                display.lcd_display_string(GreenStartingZoneStringArray[2]  ,4)
            if num_line in [2,3,4]:
                display.lcd_display_string(">",num_line)
            elif num_line in [5,6,7]:
                tempStartingZoneString= GreenStartingZoneStringArray[num_line-5]
                tempStartingZoneString=tempStartingZoneString[:11] + ">" + tempStartingZoneString[12:]
                display.lcd_display_string(tempStartingZoneString  ,num_line-3)            
        # Add new cursor
        

    currentCursorIndex = num_line

def welcomeInit():
    global currentScreen
    global currentCursorIndex

    display.lcd_clear()
    text1="    IEEE RAS INSAT"
    currentScreen="welcome"

    display.lcd_display_string("   IEEE RAS INSAT", 1)
    time.sleep(0.2)
    display.lcd_display_string("       EUROBOT", 2)
    time.sleep(0.2)
    display.lcd_display_string("        2023", 3)
    time.sleep(0.8)
    mainScreen()

def mainScreen():
    global currentScreen
    global currentCursorIndex

    currentScreen="mainScreen"
    display.lcd_clear()
    showString(display, mainScreenStringArray[0], 1)
    showString(display, mainScreenStringArray[1], 2)
    showString(display, mainScreenStringArray[2], 3)
    showString(display, mainScreenStringArray[3], 4)
    setCursor(display,1)

def teamColorInit():
    global currentScreen
    global currentCursorIndex
    global MyColor

    display.lcd_clear()
    currentScreen="teamColor"
    showString(display,"-----TEAM COLOR-----", 1)
    showString(display,"  BLUE", 2)
    showString(display,"  GREEN", 3)
    setCursor(display,2)
def startingZoneInit():
    global currentScreen
    global currentCursorIndex
    global MyColor

    display.lcd_clear()
    currentScreen="startingZone"
    showString(display, "-----START ZONE-----", 1)
    if MyColor =="BLUE":
        showString(display, BlueStartingZoneStringArray[0], 2)
        showString(display, BlueStartingZoneStringArray[1], 3)
        showString(display, BlueStartingZoneStringArray[2], 4)
    if MyColor =="GREEN":
        showString(display, GreenStartingZoneStringArray[0], 2)
        showString(display, GreenStartingZoneStringArray[1], 3)
        showString(display, GreenStartingZoneStringArray[2], 4)    
    setCursor(display,2)

def strategyInit():
    global currentScreen
    global currentCursorIndex

    display.lcd_clear()
    currentScreen="strategy"
    showString(display,"------STRATEGY------", 1)
    showString(display,"  Hard", 2)
    showString(display,"  Medium", 3)
    showString(display,"  Easy", 4)
    setCursor(display,2)

def scoreInit(value):
    display.lcd_clear()
    global currentScreen
    currentScreen="score"
    showString(display, "--------SCORE-------", 1)
    showString(display, "--------------------", 4)
    showString(display, "|      "+str(value)+"     |", 2)
    showString(display, "|                  |", 3)

def Localize():
    pass


if __name__ == '__main__':
    MyColor_pub = rospy.Publisher("MyColor",String, queue_size=10)