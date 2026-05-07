
#-----------------------------------------------------------------------------
# 
#   Omar Fernandez-Contreras 
#   xacademy 
#   Linear-act-test.py // FileName
#   demo and testing the linear act movement while demonstrating max/min height, 
#   soft max leveling, defualt level, live servo montior pulse width 
# 
#-----------------------------------------------------------------------------



#import keyboard
import pi_servo_hat
import time
import os

# init the PI servo pHat
servo = pi_servo_hat.PiServoHat() 
servo.restart()

# Channel #, edit to what channel being used
CHANNEL = 0

def set_pos(percent):
    degrees = percent * 180 /100
    servo.move_servo_position(CHANNEL,degrees,180)
    print(f"Position: {percent}% ({percent * 0.5:.1f}mm)")
    
#Test
set_pos(0)      # fully retracted
time.sleep(1)
set_pos(50)     # Mid stroke ~25mm
time.sleep(1)
set_pos(100)    # Fully extended 50mm
time.sleep(1)
set_pos(0)



# print("Use Up/Down arrows to change the value. Press 'esc' to quit.")
#value = 10

# while True:
#     if keyboard.is_pressed('up'):
#         value += 1
#         printf(f"Value: {value}", end = "\r")
#         time.sleep(0.1) #small delay prevent flying away
#     elif keyboard.is_pressed('down'):
#         value -= 1
#         print(f"Value: {value}", end="\r")
#         time.sleep(0.1)

#     elif keyboard.is_pressed('esc'):
#         print("\nExiting...")
#         break

