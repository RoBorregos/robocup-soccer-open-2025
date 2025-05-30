#Untitled - By: Emil - Wed Mar 19 2025

import sensor
import time
import utime
import pyb
import math
import image
from pyb import UART

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)

uart = UART(3, 115200, timeout_char=0)
uart.init(115200, bits=8, parity=None, stop=1)
threshold = (35, 88, 90, 30, -52, 112) #(0, 0, 0,0, 15, 127)#(6, 98, 13, 127, 14, 78) # #(49, 84, 16, 39, 6, 100)#9, 93, 26, 92, -34, 78)(43, 100, 14, 39, -53, 76) (16, 100, 12, 127, -11, 127) (0, 100, -128, 127, -128, 127)
threshold_2 = (0, 35, 33, -10, -122, -14) #Azul
threshold_1 = (50, 73, -55, 48, 8, 28) #(4, 94, 29, 46, 4, 45) (99, 25, 20, -83, 127, 40) Amarillo esta bien para 0,0,0

# The reference point for angle and distance are set in the folloxing coordinates

# The resolution of the cam is 320x240

# X and Y coordinates for a 50° of Dribbler cam

X_CENTER = 222 #162 #168 #157 #182
Y_CENTER = 152 #172 #177 #180 #175 #166

# X and Y coordinates for a 60° of Dribbler cam

#X_CENTER = 195 #182
#Y_CENTER = 169 #175 #166

def initialize_open():
    led = pyb.LED(1)  # 1 = Rojo, 2 = Verde, 3 = Azul
    led.on()
    time.sleep(1)  # Mantener el LED encendido por 1 segundo
    led.off()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    print("Final Gain Ceiling:", sensor.get_gain_db())#Checar el valor de gain para H7
    print("Final Exposure:", sensor.get_exposure_us()) #Checar el valor de exposicion H7
    sensor.skip_frames(time=3000)
    sensor.set_auto_gain(False)
    sensor.set_gainceiling(16)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=25000)
    #Sin luz natural usar exposure_us=35000
    sensor.set_brightness(1)
    sensor.set_hmirror(True)
    sensor.set_vflip(False)
    sensor.set_transpose(True)
    sensor.set_contrast(-5)
    sensor.set_saturation(-6)


def find_ball(img):
    blob_ball = img.find_blobs([threshold], area_threshold=30, merge=True) #Checar los valores de area y pixeles
    for blob in blob_ball:
        img.draw_rectangle(blob.rect(), color = (0, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color = (0, 0, 0))

    return blob_ball

def find_goal(img):
    blob_goal = img.find_blobs([threshold_1], pixels_threshold=200, area_threshold=800, merge=True) #Checar los valores de area y pixeles
    for blob in blob_goal:
        img.draw_rectangle(blob.rect(), color = (0, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color = (0, 0, 0))

    return blob_goal

def find_goal_opp(img):
    blob_goal_opp = img.find_blobs([threshold_2], pixels_threshold=200, area_threshold=800, merge=True) #Checar los valores de area y pixeles
    for blob in blob_goal_opp:
        img.draw_rectangle(blob.rect(), color = (0, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color = (0, 0, 0))

    return blob_goal_opp

def distance_ball(blob):
    relative_distx = blob.cx() - X_CENTER
    relative_disty = blob.cy() - Y_CENTER

    vector_dist = math.sqrt(relative_distx**2 + relative_disty**2)
    distance =vector_dist
    #print("Distance: %d" % distance)

    return distance

def distance_goal(blob):
    relative_distx = blob.cx() - X_CENTER
    relative_disty = blob.cy() - Y_CENTER
    #print("Relative Distx: %d" % relative_distx)
    #print("Relative Disty: %d" % relative_disty)

    vector_dist = math.sqrt(relative_distx**2 + relative_disty**2)
    distance = vector_dist
    #print("Distance: %d" % distance)

    return distance

def angle(blob):
    relative_distx = blob.cx() - X_CENTER
    relative_disty = blob.cy() - Y_CENTER
    #print("Relative Distx: %d" % relative_distx)
    #print("Relative Disty: %d" % relative_disty)

    angle = math.atan2(relative_disty, relative_distx)
    angle_degree = math.degrees(angle)
    if angle_degree < 0:
        angle_degree = 360 + angle_degree
    return angle_degree


def main():
    initialize_open()
    global distance_b, distance_g, distance_gop, angle_ball, angle_goal, angle_gop
    clock = time.clock()
    distance_b = 0
    distance_g = 0
    angle_ball = 0
    angle_goal = 0
    distance_gop = 0
    angle_gop = 0
    area = 0

    while True:
        clock.tick()
        img = sensor.snapshot()

        X_CENTER = 222 #162 #168 #157 #182
        Y_CENTER = 152 #172 #177 #180 #175 #166

        img.draw_cross(X_CENTER, Y_CENTER, color=(255, 255, 255))

        blob_ball = find_ball(img)
        #blob_goal = find_goal(img)
        #blob_goal_opp = find_goal_opp(img)

        if blob_ball:
            for blob in blob_ball:
                distance_b = distance_ball(blob)
                angle_ball = -(angle(blob) - 180)
                area = blob.area()
                print("Area: ", area)
                if area < 510:
                    distance_b = 0
                    angle_ball = 0
                if angle_ball < 5 and angle_ball > -7:
                    angle_ball = 0
                elif distance_b < 60:
                    angle_ball = 0
                #print("Distance Ball: %d" % distance_b)
                #print("Angle Ball: %d" % angle_ball)

        elif not blob_ball:
            distance_b = 0
            angle_ball = 0



        data = "{:.1f} {:.1f}\n".format(distance_b, angle_ball)
        print("Sending: ", data)
        uart.write(data)
        pyb.delay(50)


if __name__ == "__main__":
    main()