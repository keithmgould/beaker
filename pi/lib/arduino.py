import serial
import sys
import time

class Arduino:
  def __init__(self):
    self.serial = serial.Serial( port='/dev/serial0', baudrate = 115200,
                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS, timeout=1)

  def updateMotorPower(self, newPower):
    self.__writeMessage("M" + str(newPower))

  def stopMotors(self):
    self.updateMotorPower(0)

  def resetPhi(self):
    self.__writeMessage("R")

  def reset_robot(self):
    self.__writeMessage("L")
    message = self.__waitForArduinoMessage("W")
    if message == "t":
      return True
    else:
      print("winch problem. Returned: {}".format(message))
      raise "Problem with winch!"

  def give_robot_slack(self):
    self.__writeMessage("S")

  # xPos, xVel, Theta, ThetaDot
  def getObservation(self):
    while True:
      try:
        self.__writeMessage("O")
        message = self.__waitForArduinoMessage("S")
        break
      except:
        print("errored on getObservation. Trying again")
    lst = message.split(",")
    return [float(i) for i in lst]

  def __waitForArduinoMessage(self, expected):
    waiting = True
    message = ""
    while waiting or char != '!':
      waiting = False
      char = self.serial.read()
      char = char.decode("utf-8")
      message += char
    if message[0] == expected:
      return message[1:-1] # removes expected and !
    else:
      print("Expected message type {}. Found: {}".format(expected, message))
      raise "^^ Expected Message But found something else"

  def __writeMessage(self, message):
    message = message + "!"
    self.serial.write(message.encode())

