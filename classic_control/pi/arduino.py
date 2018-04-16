import serial
import sys
import time

class Arduino:
  def __init__(self):
    self.serial = serial.Serial( port='/dev/serial0', baudrate = 115200,
                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS, timeout=1)

  def playInitSong(self):
    self.__writeMessage("S")

  def zeroes(self):
    self.updatePIDValues(0,0,0)

  def updatePIDValues(self, p, i, d):
    self.__writeMessage("U" + str(p) + " " + str(i) + " " + str(d))

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

