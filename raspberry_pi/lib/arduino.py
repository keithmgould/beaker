import serial
import sys
import time

class Arduino:
  def __init__(self):
    self.serial = serial.Serial( port='/dev/serial0', baudrate = 115200,
                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS, timeout=1)

  def __constrain(self, val, min_val, max_val):
    return min(max_val, max(min_val, val))

  def accelerateMotorPower(self, acc):
    acc = self.__constrain(acc, -0.5, 0.5);
    self.__writeMessage("A" + str(acc))

  def updateMotorPower(self, newPower):
    newPower = self.__constrain(newPower, -10, 10);
    self.__writeMessage("W" + str(newPower))

  def writeMessage(self, message):
    self.__writeMessage(message)

  def writeMessageAndWait(self, message):
    self.__writeMessage(message)

    response = self.__waitForArduinoMessage()
    if response == False:
      print("errored on message receive.")
      return False
    return response

  # theta, thetaDot, phi, phiDot, outerDt, targerRadPerSec
  def getState(self):
    self.__writeMessage("S") # S for "state" on Arduino
    message = self.__waitForArduinoMessage()
    if message == False:
      print("errored on getState.")
      return False
    lst = message.split(",")
    return [float(i) for i in lst]

  def __waitForArduinoMessage(self):
    waiting = True
    message = ""
    while waiting or char != '!':
      waiting = False
      char = self.serial.read()
      try:
        char = char.decode("utf-8")
      except:
        return False # we got someting undecodable from Arduino
      message += char
    return message[1:-1] # removes expected and !

  def __writeMessage(self, message):
    message = message + "!"
    self.serial.write(message.encode())

