import serial
import sys
import time

class Arduino:
  def __init__(self):
    self.serial = serial.Serial( port='/dev/serial0', baudrate = 115200,
                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS, timeout=1)

  def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

  # in Arduino land, we use actual rads/sec for power.
  # so we convert here from +/- 1 to +/- 10
  def updateMotorPower(self, newPower):
    newPower = newPower * 10.0;
    newPower = this.constrain(newPower, -10, 10);
    self.__writeMessage("W" + str(newPower))

  def stopMotors(self):
    self.updateMotorPower(0)

  def resetRobot(self):
    self.__writeMessage("R")
    message = self.__waitForArduinoMessage("A")
    if message == False:
      print("errored on resetRobot.")
      return False
    return True

  # theta, thetaDot, xPos, phiDot
  def getObservation(self):
    self.__writeMessage("S") # S for "state" on Arduino
    message = self.__waitForArduinoMessage("A")
    if message == False:
      print("errored on getObservation.")
      return False
    lst = message.split(",")
    return [float(i) for i in lst]

  def __waitForArduinoMessage(self, expected):
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
    if message[0] == expected:
      return message[1:-1] # removes expected and !
    else:
      print("Expected message type {}. Found: {}".format(expected, message))
      return False

  def __writeMessage(self, message):
    message = message + "!"
    self.serial.write(message.encode())
