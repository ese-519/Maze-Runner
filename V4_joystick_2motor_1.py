# -*- coding: utf-8 -*
#stty -F /dev/ttyACM0 115200 ? 9600
#ls /dev/tty*
import time
import RPi.GPIO as GPIO
import cv2

##GPIO.cleanup()
def init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7, GPIO.OUT)#direction for the 1st motor
    GPIO.setup(11, GPIO.OUT)#steps for the 1st motor
    GPIO.setup(13, GPIO.OUT)#direction for the 2nd motor
    GPIO.setup(15, GPIO.OUT)#steps for the 2nd motor

    GPIO.setup(29, GPIO.OUT)#direction for the 3rd motor
    GPIO.setup(31, GPIO.OUT)#steps for the 3rd motor
    GPIO.setup(35, GPIO.OUT)#direction for the 4th motor
    GPIO.setup(37, GPIO.OUT)#steps for the 4th motor

def juegemode():
    try:
        while True:
            f = open('/dev/ttyACM0','r')#read serial port
            if f != 0:
                a = f.readline()
                #print(a)
                if a == '1\n':#my flag
                    H = int(f.readline())#horizontial value
                    V = int(f.readline())#vertical value
                    #print(H,V)
                    if H > 50:
                        GPIO.output(7, GPIO.HIGH)# + direction
                        GPIO.output(13, GPIO.LOW)# - direction
                        
                        GPIO.output(11, GPIO.HIGH)
                        GPIO.output(15, GPIO.HIGH)
                        time.sleep(0.01)#frequency
                        GPIO.output(15, GPIO.LOW)
                        GPIO.output(11, GPIO.LOW)
                        
                    if H < -50:
                        GPIO.output(7, GPIO.LOW)# - direction
                        GPIO.output(13, GPIO.HIGH)# + direction
                        
                        GPIO.output(11, GPIO.HIGH)
                        GPIO.output(15, GPIO.HIGH)
                        time.sleep(0.01)#frequency
                        GPIO.output(15, GPIO.LOW)
                        GPIO.output(11, GPIO.LOW)
                        
                    if V > 50:
                        GPIO.output(29, GPIO.HIGH)# + direction
                        GPIO.output(35, GPIO.LOW)# - direction
                        
                        GPIO.output(31, GPIO.HIGH)
                        GPIO.output(37, GPIO.HIGH)
                        time.sleep(0.01)#frequency
                        GPIO.output(37, GPIO.LOW)
                        GPIO.output(31, GPIO.LOW)
                        
                    if V < -50:
                        GPIO.output(29, GPIO.LOW)# - direction
                        GPIO.output(35, GPIO.HIGH)# + direction
                        
                        GPIO.output(31, GPIO.HIGH)
                        GPIO.output(37, GPIO.HIGH)
                        time.sleep(0.01)#frequency
                        GPIO.output(37, GPIO.LOW)
                        GPIO.output(31, GPIO.LOW)
                        
                    else:
                        pass#stand still
                else:
                    pass
        if cv2.waitKey(1)==27:#to be tested
            return 0
    except KeyboardInterrupt:
        print('error')
 
if __name__ == '__main__':
    cost = time.time()
    GPIO.cleanup()
    GPIO.setwarnings(False)
    init()
    juegemode()
    GPIO.cleanup()#clean all the status
    print("Total time cost:",time.time()-cost)

