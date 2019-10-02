# !/usr/bin/env python

import serial
import sys
import math
import bluetooth as BT
from collections import deque
from copy import deepcopy
#.encode(encoding='UTF-8')

# Arduino Serial Interface Handling
class BluetoothInterfaceHandler:

  def __init__(self, macAddr='98:D3:34:90:6A:95'):
    self.server_sock = BT.BluetoothSocket(BT.RFCOMM)
    self.macAddr = macAddr
    self.connected = False
    
    try:
      self.server_sock.connect((macAddr, 1))
      self.connected = True
      print("Direct connection")
    except:
      self.server_sock.close()
      self.server_sock.recv(1024)
      print("Robot is closed")
      self.server_sock.connect((macAddr, 1))
      print("Robot is connected")
      self.connected = True


  def send(self, data):
    temp = []
    for element in data:
      temp.append(element)
    print temp
    messageToArduino=""
    for item in temp:
      messageToArduino=messageToArduino+str(item)
      messageToArduino=messageToArduino+','
    messageToArduino=messageToArduino
    print messageToArduino
    self.server_sock.send(messageToArduino)

  def recon(self):
    self.arduPort = serial.Serial(self.portName, self.baudRate, timeout=0.1)


# Arduino Serial Data Packaging Handling
class SerialDataPacker:
  names = {}  # Dict for names to data indexes
  initVals = []  # List of middle uSec values
  curVals = []  # List of current list of values
  data = []

  def __init__(self, initVals, defaultNumBytes=2):

    self.defaultNumBytes = defaultNumBytes
    self.initVals = initVals
    self.num = len(self.initVals)
    # Set the current values to be the middle values
    self.curVals = deepcopy(self.initVals)
    # Data packaging
    self.drive_definers = {
      'BEGIN': 'FBX',  
      'END':  'FBY',  
    }
    self.rotate_definers = {
      'BEGIN': 'RX',  
      'END':  'RY',   
    }

  def setDriveVals(self,vals,direction):
    self.data=[]
    if direction==True:
      self.data.append(self.drive_definers['BEGIN'])
      self.data.append(vals[1]*0.9225)
      # 0.875 is the correction factor for left motor
      self.data.append(0)
      self.data.append(vals[1])
      self.data.append(0)
      self.data.append(vals[0])
      self.data.append(self.drive_definers['END'])
    else:
      self.data.append(self.drive_definers['BEGIN'])
      self.data.append(0)
      self.data.append(vals[1]*0.9525)
      # 0.88 is the correction factor for left motor
      self.data.append(0)
      self.data.append(vals[1])
      self.data.append(vals[0])
      self.data.append(self.drive_definers['END'])  


  def setRotateVals(self, vals):
    self.data=[]
    self.data.append(self.rotate_definers['BEGIN'])
    self.data.append(vals)
    self.data.append(0)
    self.data.append(0)
    self.data.append(0)
    self.data.append(0)
    self.data.append(self.rotate_definers['END'])


class CommunicationDriver:
  def __init__(self, stabValues=None, macAddr=None, dataPacker=None):

    if stabValues == None:
      self.stabValues = [0, 0, 0, 0, 0]
    else:
      self.stabValues = stabValues

    if macAddr == None:
      self.arduFace = BluetoothInterfaceHandler()
    else:
      self.arduFace = BluetoothInterfaceHandler(macAddr=macAddr)

    if dataPacker == None:
      self.dataPacker = SerialDataPacker(self.stabValues)
    else:
      self.dataPacker = dataPacker()

  def stop(self):
    self.dataPacker.setDriveVals(self.dataPacker.initVals)
    self.arduFace.send(self.dataPacker.data)

  def drive(self, val=None,direction=True):
    if val != None:
      self.dataPacker.setDriveVals(val,direction)
    self.arduFace.send(self.dataPacker.data)

  def rotate(self,angle=None):
    if angle != None:
      self.dataPacker.setRotateVals(angle) # corrected in the robotControl 
    self.arduFace.send(self.dataPacker.data)

rcvBuffer = deque()
