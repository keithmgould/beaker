import serial
import sys
import time

class Arduino:
  def __init__(self):
    self.serial = serial.Serial( port='/dev/serial0', baudrate = 115200,
                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS, timeout=1)

  def writeMessage(self, message):
    self.__writeMessage(message)

  def writeMessageAndWait(self, message):
    self.__writeMessage(message)

    response = self.__waitForArduinoMessage()
    if response == False:
      print("errored on message receive.")
      return False
    print(response)

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

