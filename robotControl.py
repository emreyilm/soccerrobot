#! usr/bin/env python
# -*- coding: utf-8 -*-

from arduinoComm import *
from eye import *
import random
import numpy as np
import sys
from time import time, sleep
import math
import serial

class robotController():
  def __init__(self):
    self.camera=1 # camera 1--> with white band
    self.goal_choice=1 #for green goal=1, for the blue one=2
    self.neg_theta_corr=1.11 # we need to make calibration to rotate wheels desired conditions
    self.pos_theta_corr=0.98
    # the arduino communication is initiliazed
    self.arduino = CommunicationDriver(stabValues=[0,0,0,0],macAddr='98:D3:34:90:6A:95')
    # camera initialized
    self.imageProcess = Eye(camera=self.camera)
    if self.camera==1:
      self.x_max=561.0 # maximum and minimum allowable points to play game after cropping
      self.y_max=442.0
      if self.goal_choice==1:  # give which goal position will be used
        self.position_goal=[561.0,221.0] # define the position of the goal by consdiering the cropped image
      else:
        self.position_goal=[0.0,221.0]
    else:
      self.x_max=567.0
      self.y_max=450.0
      if self.goal_choice==1:
        self.position_goal=[567.0,225.0]
      else:
        self.position_goal=[0.0,225.0]
  
  def GameLoop(self): # main game controller
    print "Identifying robots and deciding on the game.."
    self.stop=True
    while self.stop:# check which robots exits, which game will be played
      robot1=self.imageProcess.get_robot1_centr()
      print robot1
      robot2=self.imageProcess.get_robot2_centr()
      print robot2
      self.The_First_Game()
      
  # Game Loop Methods
  def The_First_Game(self):
    self.ball_exists=False
    print "Waiting for the Ball Stop"
    while not self.ball_exists: # check whether the ball is in the field,
      ball=self.imageProcess.get_ball_centr() # if it is the case, start the game
      print ball
      if ball!=[0,0]:
        self.ball_exists=True

    print "Ball is in the field, it is time to move on"
    while self.ball_exists: # if the ball disappears during the game, it should be a goal :D
      self.get_position_ball()
      self.old_x=self.position_ball[0]
      self.old_y=self.position_ball[1]
      error_x=10
      error_y=10
      while error_x>0.5 or error_y>0.5: # chech that the ball is still and we can shoot
        self.new_x=self.position_ball[0]
        self.new_y=self.position_ball[1]
        error_x=abs(self.old_x-self.new_x)
        error_y=abs(self.old_y-self.new_y)
        self.old_x=self.new_x
        self.old_y=self.new_y
        print "Waiting for the ball to be stabilized "
      print "Now,ball is Stabilized"

      self.get_position_ball()
      self.get_position_robot()
      self.get_ball_goal_distance()
      self.get_position_shoot()
      
      # if there is no possibility to shoot the ball then strike the ball in order to move towards allowable region by random shooting
      if 44.0 < self.position_shoot[0] < self.x_max-44.0:
        self.out_of_range_x=False
      else:
        self.out_of_range_x=True
      if 44.0 < self.position_shoot[1] < self.y_max-44.0:#44 is taken as pixels defines not allowable region
        self.out_of_range_y=False
      else:
        self.out_of_range_y=True

      # activate the random shoot method 
      if (self.out_of_range_x or self.out_of_range_y) == True:
        self.get_random_shoot()

      else: # that means ball is in a region that robot can make a goal 
        
        self.get_ball_goal_theta()
        self.get_robot_shoot_theta()
        self.intersection_theta=abs(self.get_theta_with_three_position(self.position_goal,self.position_shoot,self.position_robot))
        if self.intersection_theta<21.8: # define this theta as 21.8 degrees which defines whether robot is in between ball and goal
          self.get_three_move()
        else:
          self.get_two_move()

      ball=self.imageProcess.get_ball_centr()
      if ball==[0,0]: #ball is out of the plaground which means a goal is made.
        self.ball_exists=False
        print " NE GOLDU AMA !"
        self.rotate(720)

  def The_Second_Game(self): 
    self.ball_exists=False
    print "Waiting for the ball.."
    while not self.ball_exists: # is the ball on the playground ?
      ball=self.imageProcess.get_ball_centr() # if yes, start!
      print ball
      if ball!=[0,0]:
        self.ball_exists=True

    print "Ball is in the field, run run run"
    while self.ball_exists: # in the case of ball exist on the playground 
      self.get_position_ball()
      self.old_x=self.position_ball[0]
      self.old_y=self.position_ball[1]
      error_x=10
      error_y=10
      while error_x>0.5 or error_y>0.5: # whether the ball is in shootable area
        self.new_x=self.position_ball[0]
        self.new_y=self.position_ball[1]
        error_x=abs(self.old_x-self.new_x)
        error_y=abs(self.old_y-self.new_y)
        self.old_x=self.new_x
        self.old_y=self.new_y
        print "Wait the ball until it stops"
      print "Now, ball is Stoped"

      self.get_position_ball()
      self.get_position_robot()
      self.get_ball_goal_distance()
      self.get_position_shoot()
      
      # contour is defined in this region, in the case of shoot position 
      # close to the boundaries of the playground, it is tried to make 
      # the ball out of that region
      if 44.0 < self.position_shoot[0] < self.x_max-44.0:
        self.out_of_range_x=False
      else:
        self.out_of_range_x=True
      if 44.0 < self.position_shoot[1] < self.y_max-44.0:
        self.out_of_range_y=False
      else:
        self.out_of_range_y=True

      # activate the random shoot method 
      if (self.out_of_range_x or self.out_of_range_y) == True:
        self.get_random_shoot2()

      else: 
        self.get_ball_goal_theta()
        self.get_robot_shoot_theta()
        self.get_two_move2()

      ball=self.imageProcess.get_ball_centr()
      if ball==[0,0]: # when it is reached to the goal position
        self.ball_exists=False
        print "GOL OLDU !"
        self.rotate(720)

  def get_random_shoot(self):
    # used to get the ball out of the unsafe region
    print "Random Shoot"
    self.get_position_ball()
    if self.position_ball[0]<130.0 or self.get_position_ball[0]>430.0:
      pass
    else:
      if self.position_ball[0]<280.0:
        self.position_shoot=[80,position_goal[1]]
        self.get_first_theta()
        print "First Rotation Theta:"
        print self.first_theta
        self.rotate(self.first_theta)
        self.get_feedback()
        
        self.get_position_robot()
        self.get_position_ball()
        self.get_shoot_robot_distance()
        self.get_front_position(self.position_shoot)
        self.drive(self.shoot_robot_distance-2,150)
        self.get_feedback()

        self.get_position_ball()
        self.position_shoot=self.position_ball
        self.get_first_theta()
        print "Second Rotation Theta:"
        print self.first_theta
        self.rotate(self.first_theta)
        self.get_feedback()
        
        self.get_position_robot()
        self.get_position_ball()
        self.get_shoot_robot_distance()
        self.get_front_position(self.position_shoot)
        self.drive(self.shoot_robot_distance-4,150)
        self.get_feedback()

      else:
        self.position_shoot=[self.x_max-80.0,position_goal[1]]
        self.get_first_theta()
        print "First Rotation Theta: "
        print self.first_theta
        self.rotate(self.first_theta)
        self.get_feedback()
        
        self.get_position_robot()
        self.get_position_ball()
        self.get_shoot_robot_distance()
        self.get_front_position(self.position_shoot)
        self.drive(self.shoot_robot_distance-2,150)
        self.get_feedback()

        self.get_position_ball()
        self.position_shoot=self.position_ball
        self.get_first_theta()
        print "First Rotation Theta: "
        print self.first_theta
        self.rotate(self.first_theta)
        self.get_feedback()
        
        self.get_position_robot()
        self.get_position_ball()
        self.get_shoot_robot_distance()
        self.get_front_position(self.position_shoot)
        self.drive((self.shoot_robot_distance-4),150)
        self.get_feedback()

  def get_random_shoot2(self):
    pass


  def get_two_move2(self):
    print " * TWO MOVEMENT CASE2 * "
    self.get_first_theta()
    print " First Rotation Theta: "
    print self.first_theta
    self.rotate(self.first_theta)
    self.get_feedback()

    self.get_position_robot2()
    self.get_position_robot()
    dist1=self.get_distance(self.position_robot,self.position_ball)
    dist2=self.get_distance(self.position_robot2,self.position_ball)
    if dist1<dist2:
      self.get_position_robot()
      self.get_position_ball()
      self.get_shoot_robot_distance()
      self.get_front_position(self.position_shoot)
      self.drive((self.shoot_robot_distance-2),150)
      self.get_feedback()

      self.get_position_robot()
      self.get_second_theta()
      print "Second Rotation Theta: "
      print self.second_theta
      self.rotate(self.second_theta)
      self.get_feedback()

      self.get_position_robot()
      self.get_ball_robot_distance()
      self.get_front_position(self.position_ball)
      self.drive(self.ball_robot_distance,180)
      self.get_feedback()
    else:
      pass

  def get_two_move(self): #that means ball is between robot and goal post
    print " * TWO MOVEMENT CASE * "
    self.get_first_theta()
    print "First Rotation Theta: "
    print self.first_theta
    self.rotate(self.first_theta)
    self.get_feedback()
    
    self.get_position_robot()
    self.get_position_ball()
    self.get_shoot_robot_distance()
    self.get_front_position(self.position_shoot)
    self.drive((self.shoot_robot_distance-2),150)
    self.get_feedback()

    # repeat the step
    self.get_first_theta()
    print "First Rotation Theta: "
    print self.first_theta
    self.rotate(self.first_theta)
    self.get_feedback()
    
    self.get_position_robot()
    self.get_position_ball()
    self.get_shoot_robot_distance()
    self.get_front_position(self.position_shoot)
    self.drive((self.shoot_robot_distance-2),150)
    self.get_feedback()

    self.get_position_robot()
    self.get_second_theta()
    print "Second Rotation Theta: "
    print self.second_theta
    self.rotate(self.second_theta)
    self.get_feedback()

    self.get_position_robot()
    self.get_ball_robot_distance()
    self.get_front_position(self.position_ball)
    self.drive(self.ball_robot_distance,180)
    self.get_feedback()

    self.get_position_robot()
    self.get_second_theta()
    print "Second Rotation Theta: "
    print self.second_theta
    self.rotate(self.second_theta)
    self.get_feedback()

    self.get_position_robot()
    self.get_ball_robot_distance()
    self.get_front_position(self.position_ball)
    self.drive(self.ball_robot_distance,180)
    self.get_feedback() 

  def get_three_move(self): #that means robot is between ball and goal post
    print " * THREE MOVEMENT CASE * "
    self.get_preshoot_position()
    print self.preshoot_position
    self.get_preshoot_theta()
    print self.preshoot_theta
    self.rotate(self.preshoot_theta)
    self.get_feedback()
    self.get_robot_preshoot_distance()
    self.get_front_position(self.position_ball)
    self.drive(self.robot_preshoot_distance,180)
    self.get_feedback()
    self.get_position_robot()
    self.get_two_move()

  def drive(self,distance,speed):
    if self.camera==1:
      necessary_distance=distance/4.417  # 4.417 is the coefficient used for pixel to cm conversion
    elif self.camera==2:
      necessary_distance=distance/4.417
    self.arduino.drive(val=[necessary_distance,speed],direction=self.front)

  def rotate(self,theta):
    if theta<0:
      self.arduino.rotate(theta*self.neg_theta_corr)
    else:
      self.arduino.rotate(theta*self.pos_theta_corr)

  def get_feedback(self):
    self.feedback=True
    print "Feedback is taken"
    while self.feedback:
      print "Received: "
      input=self.arduino.arduFace.server_sock.recv(1)
      print input
      sleep(1)
      if input=='F':
        self.feedback=False

  def get_distance(self,position1,position2):
    distance = math.sqrt(pow(position1[0]-position2[0],2)+pow(position1[1]-position2[1],2))
    return distance
  
  def get_ball_goal_distance(self):
    self.ball_goal_distance=self.get_distance(self.position_ball,self.position_goal)
    print "Ball to Goal Distance: "
    print self.ball_goal_distance

  def get_robot_preshoot_distance(self):
    self.robot_preshoot_distance=self.get_distance(self.position_robot,self.preshoot_position)

  def get_ball_goal_theta(self):
    self.ball_goal_theta=self.get_theta(self.position_ball,self.position_goal)

  def get_robot_shoot_theta(self):
    self.robot_shoot_theta=self.get_theta(self.position_robot,self.position_shoot)
    print "Robot Shoot theta: "
    print self.robot_shoot_theta

  def get_robot_ball_theta(self):
    self.robot_ball_theta=self.get_theta(self.position_robot,self.position_ball)

  def get_ball_robot_distance(self):
    self.ball_robot_distance=self.get_distance(self.position_ball,self.position_robot)

  def get_shoot_robot_distance(self):
    self.shoot_robot_distance=self.get_distance(self.position_shoot,self.position_robot)
    print "Shoot to Robot Distance:"
    print self.shoot_robot_distance

  def get_position_shoot(self):
    shoot_x=(((self.position_ball[0]-self.position_goal[0])*(100+self.ball_goal_distance))/self.ball_goal_distance)+self.position_goal[0]
    shoot_y=(((self.position_ball[1]-self.position_goal[1])*(100+self.ball_goal_distance))/self.ball_goal_distance)+self.position_goal[1]
    self.position_shoot=[shoot_x,shoot_y]
    print "Shoot Position: "
    print self.position_shoot

  def get_preshoot_position(self):
    self.get_ball_goal_theta()
    if self.position_shoot[1]>225.0:
      preshoot_x=self.position_shoot[0]+100*math.sqrt(2)*math.sin(math.degrees(45-self.ball_goal_theta))
      preshoot_y=self.position_shoot[1]+100*math.sqrt(2)*math.sin(math.degrees(45+self.ball_goal_theta))
    else:
      preshoot_x=self.position_shoot[0]+100*math.sqrt(2)*math.sin(math.degrees(90+self.ball_goal_theta))
      preshoot_y=self.position_shoot[1]+100*math.sqrt(2)*math.sin(math.degrees(self.ball_goal_theta))
    self.preshoot_position=[preshoot_x,preshoot_y]

  def get_robot_preshoot_theta(self):
    self.robot_preshoot_theta=self.get_theta(self.position_robot,self.preshoot_position)

  def get_position_ball(self):
    i=1
    while i<10:
      self.position_ball=self.imageProcess.get_ball_centr()
      i=i+1
    print "Ball Position: "
    print self.position_ball

  def get_robot_theta(self):
    a=self.robot_centrs[1]
    b=self.robot_centrs[0]
    self.robot_theta=self.get_theta(a,b)
    print "Robot theta: "
    print self.robot_theta

  def get_position_robot(self):
    i=1
    while i<10:
      self.robot_centrs=self.imageProcess.get_robot1_centr()
      i=i+1
    print "Centers of Robot: "
    print self.robot_centrs
    centroid_x=(self.robot_centrs[0][0]+self.robot_centrs[1][0])/2.0
    centroid_y=(self.robot_centrs[0][1]+self.robot_centrs[1][1])/2.0
    self.position_robot=[centroid_x,centroid_y]
    print "Robot Position:"
    print self.position_robot

  def get_position_robot2(self):
    i=1
    while i<10:
      self.robot_centrs2=self.imageProcess.get_robot2_centr()
      i=i+1
    print "Centers of Other Robot: "
    print self.robot_centrs2
    centroid_x=(self.robot_centrs2[0][0]+self.robot_centrs2[1][0])/2.0
    centroid_y=(self.robot_centrs2[0][1]+self.robot_centrs2[1][1])/2.0
    self.position_robot2=[centroid_x,centroid_y]
    print "Position of Other Robot: "
    print self.position_robot2
  
  def get_preshoot_theta(self):
    self.get_position_robot()
    self.get_robot_theta()
    self.get_robot_preshoot_theta()
    self.preshoot_theta=self.robot_preshoot_theta-self.robot_theta

  def get_first_theta(self):
    self.get_position_robot()
    self.get_robot_theta()
    self.get_robot_shoot_theta()
    self.first_theta=self.robot_shoot_theta-self.robot_theta

  def get_second_theta(self):
    self.get_position_robot()
    self.get_robot_theta()
    self.get_robot_ball_theta()
    self.second_theta=self.robot_ball_theta-self.robot_theta

  def get_front_position(self,target_position):
    self.get_position_robot()
    distance_front=self.get_distance(target_position,self.robot_centrs[0])
    distance_back=self.get_distance(target_position,self.robot_centrs[1])
    if distance_front<distance_back:
      self.front=True
      print "Movement vector of the robot defined from yellow to blue"
    else:
      self.front=False
      print "Movement vector of the robot defined from blue to yellow"
      
  def get_theta_with_three_position(self,pos1,pos2,pos3):
    ang1=self.get_theta(pos2,pos3)
    ang2=self.get_theta(pos2,pos1)
    ang3=ang1-ang2
    print "Theta of Three Positions: "
    print ang3
    return ang3

  def get_theta(self,first_pos,second_pos):
    theta=(180/math.pi)*math.atan2(second_pos[1]-first_pos[1],second_pos[0]-first_pos[0])
    if theta<0.0:
      theta=theta+360
    return theta

