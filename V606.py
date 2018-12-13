from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2
import colorList
import time
import picamera
import picamera.array
from picamera import PiCamera
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO

maze_width = 16#15#64
maze_height = 16#15#64
ball_color = 'red'#
extra_color = 'black'#
bg_color = 'green'#
bg2_color = 'blue'#
filename='real.jpg'#newmaze.png#maze1.jpg
startx = 1
starty = 1
bx = 0
by = 0
endx = maze_width - 1#minus 1
endy = maze_height - 1
ans = 40
f1 = 0

def get_rid_of_color(frame):
    print('Analysing color')
    img = frame
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)#HSV空间
    mask=cv2.inRange(hsv,color_dict[bg_color][0],color_dict[bg_color][1])#cheng色
    mask2=cv2.inRange(hsv,color_dict[ball_color][0],color_dict[ball_color][1])
    mask3=cv2.inRange(hsv,color_dict[extra_color][0],color_dict[extra_color][1])
    mask4=cv2.inRange(hsv,color_dict[bg2_color][0],color_dict[bg2_color][1])
    mask = mask2 + mask3 + mask4 #color of the path, ball and extra color
    #cv2.imshow('img',cv2.resize(mask, (380 ,380)))
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    cv2.bitwise_not(mask,mask)
    return mask
    #red=cv2.bitwise_and(img,img,mask=red_mask)#对原图像处理
    #res=red
    #cv2.imshow('img',red_mask)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    #cv2.imshow('name',cv2.resize(maze, (380 ,380)))

def mazeMatrix(maze):
    res=cv2.resize(maze,(maze_width,maze_height),interpolation=cv2.INTER_CUBIC)
    
    for i in range(0,maze_width):
        for j in range(0,maze_height):
            if res[i][j] > 150:
                res[i][j] = 1
            else:
                res[i][j] = 0
    return res

def get_ball_color(frame):
    print('Locating ball')
    img = frame
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)#HSV空间
    mask2=cv2.inRange(hsv,color_dict[ball_color][0],color_dict[ball_color][1])
    #mask3=cv2.inRange(hsv,color_dict[extra_color][0],color_dict[extra_color][1])
    mask = mask2 #color of the ball and extra color
    cv2.bitwise_not(mask,mask)
    
    #cv2.imshow('img',cv2.resize(mask, (380 ,380)))
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    return mask

def ballMatrix(ball):
    res=cv2.resize(ball,(maze_width,maze_height),interpolation=cv2.INTER_CUBIC)
    
    for i in range(0,maze_width-1):
        for j in range(0,maze_height-1):
            if (res[i][j] == 0 and res[i+1][j] == 0):#&(res[i][j] == 0 and res[i+1][j] == 0):
                return i,j
            else:
                pass
    return False
    

#BFS algorithm
class node:
    def __init__(self, x=0, y=0, t=0):
        self.x = x
        self.y = y
        self.t = t #t表示走到这个格子用的步数
 
class father:
    def __init__(self, x=0, y=0, cz=[]):
        self.x = x #当前格子的父节点坐标
        self.y = y
        self.cz = cz #由什么操作到达的这个格子
 

xx =[1,0,0,-1]                          #下、右、左、上
yy =[0,1,-1,0]

 
def bfs(mmap,s,q,lj,vis,ssx,ssy):
    print('Calculating path')
    startx = ssx
    starty = ssy
    if mmap[startx][starty] == 1 or mmap[endx][endy] == 1:
        print('wall')
        print(res)
        startx += 1
        return 0,lj
        
        return False
    n = maze_width
    m = maze_height
    s.x = startx
    s.y = starty
    s.t = 0

    q.append(s)
    lj[s.x][s.y].x = 0
    lj[s.x][s.y].y = 0
    lj[s.x][s.y].cz = 0
    vis[s.x][s.y] = True    #标为已经访问过
    #print("vis={}".format(vis))
    while q:
        now = q[0]
        q.pop(0)
        for i in range(0, 4):
            new = node()
            new.x = now.x + xx[i]
            new.y = now.y + yy[i]
            new.t = now.t + 1
            #print("i={} new.x={} new.y={} now.x={} now.y={}".format(i, new.x, new.y, now.x, now.y))
            #print("new.x ={} new.y={} n={} m={} vis[new.x][new.y]={} mmap[new.x][new.y]={}".format(new.x, new.y, n, m, vis[new.x][new.y], mmap[new.x][new.y]))
            if new.x < 0 or new.y < 0 or new.x >= n or new.y >= m or vis[new.x][new.y] == True or mmap[new.x][new.y] == 1:  # 下标越界或者访问过或者是障碍物
                continue
 
            q.append(new)
            lj[new.x][new.y].x = now.x
            lj[new.x][new.y].y = now.y
            if i == 0:
                lj[new.x][new.y].cz = 'D'
            elif i == 1:
                lj[new.x][new.y].cz = 'R'
            elif i == 2:
                lj[new.x][new.y].cz = 'L'
            elif i == 3:
                lj[new.x][new.y].cz = 'U'
            vis[new.x][new.y] = True
            #print("value={} ({},{}) {}\n".format(mmap[new.x][new.y], new.x, new.y, lj[new.x][new.y].cz))
            #print("=============================================================")
            if new.x == endx and new.y == endy:
                return new.t,lj                        #到达终点
    return False

def dfs(x, y,lj,ans_lj,ssx,ssy):
    startx = ssx
    starty = ssy
    if x == startx and y == starty: return
    else: dfs(lj[x][y].x,lj[x][y].y,lj,ans_lj,startx,starty)
    #print(lj[x][y].cz)
    #print(lj[x][y].x,lj[x][y].y)
    ans_lj.append(lj[x][y].cz)

def moveMotors(ans_lj):
    count = 1
    for i in range(0,len(ans_lj)):#←↑→↓
        if ans_lj[i] == 'U':
            if i+1 == len(ans_lj):
                print('↑ ',count,' times')
                move(count,7,GPIO.HIGH,11,13,GPIO.LOW,15)
                return
                break
            if ans_lj[i+1] == ans_lj[i]:
                count += 1
            else:
                print('↑ ',count,' times')#Move up
                move(count,7,GPIO.HIGH,11,13,GPIO.LOW,15)
                return
                count = 1
        if ans_lj[i] == 'D':
            if i+1 == len(ans_lj):
                print('↓ ',count,' times')
                move(count,7,GPIO.LOW,11,13,GPIO.HIGH,15)
                return
                break
            if ans_lj[i+1] == ans_lj[i]:
                count += 1
            else:
                print('↓ ',count,' times')#Move down
                move(count,7,GPIO.LOW,11,13,GPIO.HIGH,15)
                return
                count = 1
        if ans_lj[i] == 'L':
            if i+1 == len(ans_lj):
                print('← ',count,' times')
                move(count,35,GPIO.HIGH,37,29,GPIO.LOW,31)
                return
                break
            if ans_lj[i+1] == ans_lj[i]:
                count += 1
            else:
                print('← ',count,' times')#Move left
                move(count,35,GPIO.HIGH,37,29,GPIO.LOW,31)
                return
                count = 1
        if ans_lj[i] == 'R':
            if i+1 == len(ans_lj):
                move(count,35,GPIO.LOW,37,29,GPIO.HIGH,31)
                return
                print('→ ',count,' times')
                break
            if ans_lj[i+1] == ans_lj[i]:
                count += 1
            else:
                print('→ ',count,' times')#Move right
                move(count,35,GPIO.LOW,37,29,GPIO.HIGH,31)
                return
                count = 1
               
def move(count,control1,dir1,step1,control2,dir2,step2):
    global bx,by
    if count == 1 and control1==35 and dir1==GPIO.LOW:
        f1 =1
        by +=1
        return
    if count == 1 and control1==7 and dir1==GPIO.HIGH:
        f1 =1
        bx -=1
        return
    GPIO.output(control1,dir1)
    GPIO.output(control2,dir2)
    print('working on ',count,control1,dir1,step1,control2,dir2,step2)
    #if count < 2:
    #    return
    for i in range(0,116):#count*
        GPIO.output(step1, GPIO.HIGH)
        GPIO.output(step2, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(step1, GPIO.LOW)
        GPIO.output(step2, GPIO.LOW)
    time.sleep(2)
    GPIO.output(control1,dir2)
    GPIO.output(control2,dir1)
    
    for i in range(0,116):#count*
        GPIO.output(step1, GPIO.HIGH)
        GPIO.output(step2, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(step1, GPIO.LOW)
        GPIO.output(step2, GPIO.LOW)
    time.sleep(2)

def initGPIO():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7, GPIO.OUT)#dir1   
    GPIO.setup(11, GPIO.OUT)#steps for the 1st motor
    GPIO.setup(13, GPIO.OUT)#dir2
    GPIO.setup(15, GPIO.OUT)#steps for the 2nd motor
    
    GPIO.setup(29, GPIO.OUT)#dir1   
    GPIO.setup(31, GPIO.OUT)#steps for the 1st motor
    GPIO.setup(35, GPIO.OUT)#dir2
    GPIO.setup(37, GPIO.OUT)#steps for the 2nd motor


def camera():
    image, contours, hier = cv2.findContours(maze, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in range(0,len(contours)):
        x1, y1, w1, h1 = cv2.boundingRect(contours[c])
        #if (w1 > 450 & h1 >100) & ( w1 < 1900):#19201080
        #if h1 > 2400 & w1 > 1000 :#46083456
        if h1 > 100 & w1 > 100:
            x = x1
            y = y1
            w = w1
            h = h1
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    cv2.imshow('image',cv2.resize(image, (720 ,560)))#cv2.resize(frame, (380 ,380))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def usecamera():
    with picamera.PiCamera() as camera:
        camera.resolution = (540,540)
        camera.framerate = 30
        #camera.saturation = 80
        #camera.brightness = 40
        camera.iso = 100
        #camera.sharpness = 50
    
        print("start preview direct from GPU")
        camera.start_preview() # the start_preview() function   
        time.sleep(1)
        camera.capture('hahaha.jpg', use_video_port = False)
        print("end preview")
        camera.close()

def findlj():
    global f1,bx,by,ans
    usecamera()
    filename='hahaha.jpg'
    frame = cv2.imread(filename,1)
    
    ball = get_ball_color(frame)#get ball
    ball_maze = mazeMatrix(ball)

    try:
        bx,by = ballMatrix(ball_maze)#find ball coord
        print('The coord of ball is',bx,by)
    except:
        #startx = bx - 1
        print('can not find')
        time.sleep(1)
        startx = bx
        starty = by
    #startx = bx + 1
    startx = bx + 1#看命，是否有1
    starty = by

    if f1 == 1:
        startx += 1
        print('+++++++++++++++++')
        f1 = 0
    
    vis = []
    for i in range(0, maze_width):
        vis += [[]]
        for j in range(0, maze_height):
            vis[i] += [False]
 
    lj = []
    for i in range(0, maze_width):
        lj += [[]]
        for j in range(0, maze_height):
            lj[i] += [father()]
 
    q = []
    s = node() 
    ans_lj = []
    ans,lj = bfs(res,s,q,lj,vis,startx,starty)
    if ans == False:
        print("No way to get out of this maze")
        #dfs(endx, endy,lj,ans_lj,startx,starty)###
        #print(ans_lj)
    else:
        print('There are',ans,'steps to get out')
        dfs(endx, endy,lj,ans_lj,startx,starty)
        return ans_lj
        #print("迷宫行走方式{}".format(ans_lj))

if __name__ == '__main__':
    cost = time.time()

    initGPIO()
    usecamera()
    filename='hahaha.jpg'
    
    color_dict = colorList.getColorList() 
    frame = cv2.imread(filename,1)
    maze = get_rid_of_color(frame)#create maze graph
    #camera()
    
    res = mazeMatrix(maze)#resize maze
    res[0][0]=0
    res[0][1]=0
    res[1][0]=0
    res[1][1]=0
    res[14][14]=0
    res[14][15]=0
    res[15][14]=0
    res[15][15]=0

    while (endx - bx)>2 or (endy - by) >2:# and ans > 6:
        print('v')
        ans_lj = findlj()
        moveMotors(ans_lj)

    
    print("Total time cost:",time.time()-cost)
    #cv2.imshow('name',cv2.resize(maze, (380 ,380)))
    #cv2.imshow('name',cv2.resize(ball, (380 ,380)))
#自整定且跳过球报错
