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
threshold = (45, 74, 71, 27, -92, 77)#(50, 68, 85, 13, -96, 76) (54, 66, 50, 76, 68, 42) #(59, 23, 12, 62, -4, 22) #(48, 7, 9, 62, -4, 22)
#Azul
threshold_own = (35, 61, 37, -4, -28, -11) #(16, 56, 18, -69, -47, -8) #(35, 61, 37, -4, -28, -11) #(127, 127, 33, -10, -120, -6) #(0, 35, 33, -10, -120, -6) #Azul
#Amarillo
threshold_goal = (54, 100, -30, 14, 19, 112) #(75, 100, -30, 14, 19, 112) #(44, 94, 0, -8, 8, 124) #(31, 87, -115, 77, 13, 124) #(44, 94, 0, -8, 8, 124)#(68, 100, 16, -10, -20, 116) #(44, 94, 0, -8, 8, 124)#(41, 99, -52, 127, 15, 127) #(68, 100, 16, -10, -20, 116) #(41, 99, -52, 127, 15, 127) (99, 25, 20, -83, 127, 40) Amarillo esta bien para 0,0,0

# The reference point for angle and distance are set in the folloxing coordinates


X_CENTER = 136 #72 #99 #104 #111
Y_CENTER = 134 #174 #156 #163 #140


def initialize_open():
    led = pyb.LED(3)  # 1 = Rojo, 2 = Verde, 3 = Azul
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
    blob_ball = img.find_blobs([threshold], area_threshold=20, merge=True) #Checar los valores de area y pixeles
    for blob in blob_ball:
        img.draw_rectangle(blob.rect(), color = (255, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color = (255, 0, 0))

    return blob_ball

def find_goal(img):
    blob_goal = img.find_blobs([threshold_goal], pixels_threshold=100, area_threshold=200, merge=True) #Checar los valores de area y pixeles
    for blob in blob_goal:
        img.draw_rectangle(blob.rect(), color = (0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color = (0, 255, 0))

    return blob_goal

def find_goal_opp(img):
    blob_goal_opp = img.find_blobs([threshold_own], pixels_threshold=100, area_threshold=200, merge=True) #Checar los valores de area y pixeles
    for blob in blob_goal_opp:
        img.draw_rectangle(blob.rect(), color = (0, 0, 255))
        img.draw_cross(blob.cx(), blob.cy(), color = (0, 0, 255))

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

def calculate_cathetus(distance_b, distance_goal, angle_ball, angle_goal):
    # Calcular el ángulo entre los dos vectores
    adjusted_angle = angle_goal - angle_ball

    # Normalizar el ángulo al rango [-180, 180)
    adjusted_angle = abs((adjusted_angle + 180) % 360 - 180)
    print("Adjusted catetus angle: ", adjusted_angle)
    if adjusted_angle > -10 and adjusted_angle < 10:
        cathethus = distance_b + distance_goal

    # Convertir a radianes
    angle_rad = math.radians(adjusted_angle)

    # Aplicar ley de cosenos (c² = a² + b² - 2ab*cos(θ))
    cathethus = math.sqrt(distance_b**2 + distance_gop**2 - 2 * distance_b * distance_gop * math.cos(angle_rad))

    return cathethus

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
    cathethus = 0


    while True:
        clock.tick()
        img = sensor.snapshot()


        X_CENTER = 136  #83 #72 #99 #104 #111
        Y_CENTER = 134  #174 #156 #163 #140

        img.draw_cross(X_CENTER, Y_CENTER, color=(255, 255, 255))

        blob_ball = find_ball(img)
        blob_goal = find_goal(img)
        blob_goal_opp = find_goal_opp(img)

        if blob_ball:
            for blob in blob_ball:
                distance_b = distance_ball(blob)
                angle_ball = -(angle(blob) - 180)
                #print("Distance Ball: %d" % distance_b)
                #print("Angle Ball: %d" % angle_ball)
                if angle_ball < 5 and angle_ball > -5:
                    angle_ball = 0
                if angle_ball > 135 or angle_ball < -135:
                    distance_b = distance_b + 10;

        if blob_goal:
            for blob in blob_goal:
                distance_g = distance_goal(blob)
                angle_goal = -(angle(blob) - 180)
                #print("Distance Goal: %d" % distance_g)
                #print("Angle Goal: %d" % angle_goal)
                if angle_goal < 10 and angle_goal > -10:
                    angle_goal = 0
                #if angle_goal > 135 or angle_goal < -135:
                    #distance_g = distance_g - 30;

        elif not blob_goal:
            distance_g = 0
            angle_goal = 0

        if blob_goal_opp:
            for blob in blob_goal_opp:
                distance_gop = distance_goal(blob)
                angle_gop = -(angle(blob) - 180)
                #print("Distance Opposite Goal: %d" % distance_gop)
                #print("Angle Opposite Goal: %d" % angle_goal)
                if angle_gop < 5 and angle_gop > -5:
                    angle_gop = 0
                if angle_gop > 135 or angle_gop < -135:
                    distance_gop = distance_gop - 30;
                if distance_gop < 10:
                    distance_gop = 10


        elif not blob_goal_opp:
            distance_gop = 0
            angle_gop = 0

        cathethus = calculate_cathetus(distance_b, distance_g, angle_ball, angle_gop)
        #print("Cateto: ", cathethus)

        data = "{} {} {} {} {} {}\n".format(distance_b, angle_ball, distance_g, angle_goal, distance_gop, angle_gop)
        print("Sending: ", data)
        print(cathethus)
        uart.write("{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}\n".format(distance_b, angle_ball, distance_g, angle_goal, distance_gop, angle_gop))
        pyb.delay(50)

if __name__ == "__main__":
    main()
