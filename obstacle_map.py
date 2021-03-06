# -*- coding: utf-8 -*-
import numpy as np
import cv2
from math import *

rc = 0
def findLine(pt1,pt2):
  m = (pt2[1]-pt1[1])/(pt2[0]-pt1[0])
  c = pt1[1]-m*pt1[0]
  return m,c 
  
def obstacle_circle(world):
  crow= 50 
  ccol= 225 
  rad= 25+rc
  for row in range(200):
    for col in range(300):
      if (row- crow)**2+(col-ccol)**2- rad**2<= 0:
        world[row][col]=(255,0,0)

def obstacle_ellipse(world):
  center= [100, 150]
  a= 40 +rc                                                                     #major axis
  b= 20 +rc                                                                     #minor axis
  for col in range(300):
    for row in range(200):
      if ((row-center[0])**2/(b**2))+((col-center[1])**2/(a**2))-1<=0:
        world[row][col]=(255,0,0)

def obstacle_rhombus(world):
   p1=(160,225)
   p2=(175,250)
   p3=(190,225)
   p4=(175,200)
   m1,c1= findLine(p1,p2)
   #print(m1,c1)
   m2,c2= findLine(p2,p3)
   #print(m2,c2)
   m3,c3= findLine(p3,p4)
   #print(m3,c3)
   m4,c4= findLine(p4,p1)
   #print(m4,c4)
   for row in range(200):
     for col in range(300):
      if col-m1*row -c1<=0 and col-m2*row-c2<=0 and col-m3*row - c3>=0 and col-m4*row - c4>=0:
        world[row][col]=(255,0,0)

def obstacle_rectangle(world):
  p1=(170,95)
  p2=(170-(75*sin(radians(30))), 95-(75*cos(radians(30))))
  p3=(170-75*sin(radians(30))-10*sin(radians(60)), 95-75*cos(radians(30))+10*cos(radians(60)))
  p4=(170-10*sin(radians(60)), 95+10*cos(radians(60)))
  m1,c1= findLine(p1,p2)
  #print(m1,c1)
  m2,c2= findLine(p2,p3)
  #print(m2,c2)
  m3,c3= findLine(p3,p4) 
  #print(m3,c3)
  m4,c4= findLine(p4,p1) 
  #print(m4,c4)
  for row in range(200):
     for col in range(300):
      if col-m1*row -c1>=0 and col-m2*row-c2>=0 and col-m3*row - c3<=0 and col-m4*row - c4<=0:
        world[row][col]=(255,0,0)

def obstacle_polygon(world):
  p1=(15,25)
  p2=(15,75)
  p3=(50,100)
  p4=(80,75)
  p5=(50,50)
  p6=(80,20)
  # m1,c1= findLine(p1,p2)
  # print(m1,c1)
  m2,c2= findLine(p2,p3)
  #print(m2,c2)
  m3,c3= findLine(p3,p4) 
  #print(m3,c3)
  m4,c4= findLine(p4,p5) 
  #print(m4,c4)
  m5,c5= findLine(p5,p6)
  #print(m5,c5)
  m6,c6= findLine(p6,p1)
  #print(m6,c6)
  for row in range(200):
     for col in range(300):
       if row-15>=0 and col-m2*row-c2<=0 and col-m3*row - c3<=0 and col-m6*row-c6>=0 and (col-m4*row - c4>=0 or col-m5*row-c5<=0):
       #if row-15>=0:
         world[row][col]=(255,0,0)


if __name__=='__main__':
    world= 255*np.ones((200,300,3))
    rc=0
    obstacle_circle(world)
    obstacle_ellipse(world)
    obstacle_rhombus(world)
    obstacle_rectangle(world)
    obstacle_polygon(world)
    cv2.imwrite('./map.jpg', world)
    cv2.imshow('map', world)
    cv2.waitKey(0);cv2.destroyAllWindows()
