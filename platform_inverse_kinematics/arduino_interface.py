from serial import Serial
import time
from StewartPlatform import StewartPlatform
from MotionPath import MotionPath
import numpy as np
import threading
message=''
class arduino_interface():
    def __init__(self,port,baud_rate):
        """
       create a serial arduino interface
       :param port: serial port that the arduino is connected to
       :param baud_rate: connection speed
       """
        self.ser=Serial(port,baud_rate)
        time.sleep(1)
        thread = threading.Thread(target=self.non_blocking_read)
        thread.start()
        self.last_serial_string=None

    def non_blocking_read(self):
        """
       listener that will parse strings the arduino sends over serial and write them to self.last_serial_string
       """
        while(True):
            if(self.ser.in_waiting>0):
                data_str = self.ser.readline().decode().strip('\r\n')
                self.last_serial_string=data_str# read the bytes and convert from binary array to ASCII
                print(data_str) # print the incoming string without putting a new-line ('\n') automatically after every print()


    def poll_until_message(self,message):
        """
       block until a specific message is revieved from the arduino
       :param message: string to wait for
       """

        while(self.last_serial_string!=message):
            time.sleep(0.001)

    def reset_serial_buff(self):
        """
       clear the serial buffer
       """
        self.last_serial_string=None

    def send_string(self,message):
        """
       send a string to the arduino
       :param message: string to send
       """
        print("Sending {}".format(message))
        message+="\r\n"
        self.ser.write(message.encode())




