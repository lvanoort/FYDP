from Tkinter import *
import threading
import serial
import time

class SerialThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.quit_serial = False;
        self._address = 40
        self._data1   = 1
        self._data2   = 1

    def write_message(self):
        addHigh = (self._address & 0xFF00) >> 8
        addLow = (self._address & 0xFF) 
        dat1High = (self._data1 & 0xFF00) >> 8
        dat1Low = (self._data1 & 0xFF) 
        dat2High = (self._data2 & 0xFF00) >> 8
        dat2Low = (self._data2 & 0xFF) 
        checksum = self._address + self._data1 + self._data2
        checksumHigh = (checksum & 0xFF00) >> 8
        checksumLow = (checksum & 0xFF) 
        message = [0xFF,0xFF,addHigh,addLow,dat1High,dat1Low,dat2High,dat2Low,checksumHigh,checksumLow]
        self.ser.write(bytearray(message))        

    def run(self):
        print "openning serial port"
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.02)
        print "opened"
        
        time.sleep(3)
        
        self.write_message()
        
        
        
        
        while not self.quit_serial:
          line = self.ser.readline();
          if line:
            print line
          
          time.sleep(0.02)
          self.write_message()
          #ser.write(bytearray([1]))
          
        self.ser.close()





class Application(Frame):
    def send_register(self):
        _address = self.address.get()
        _data1   = self.data1.get()
        _data2   = self.data2.get()
        if not (isinstance(3, int) and isinstance(3, int) and isinstance(3, int)):
          print "bad arguments"
          self.serial._address = 0
          self.serial._data1   = 0
          self.serial._data2   = 0
        else:
          print "Sending: " + str(_data1) + " " + str(_data2) + " to address " + str(_address)
          self.serial._address = int(_address)
          self.serial._data1   = int(_data1)
          self.serial._data2   = int(_data2)
        
    def stop_app(self):
        self.serial.quit_serial = True
        self.quit

    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] =  self.stop_app
        self.QUIT.pack({"side": "top"})

        self.send = Button(self)
        self.send["text"] = "Send"
        self.send["command"] = self.send_register
        self.send.pack({"side": "top"})

        self.stop = Button(self)
        self.stop["text"] = "Stop"
        #self.stop["command"] = self.send_register #TODO: some stop command
        self.stop.pack({"side": "top"})

        self.address_label = Label(self)
        self.address_label["text"] = "Address"
        self.address_label.pack({"side": "top"})
        
        self.address = Entry(self)
        self.address.pack({"side": "top"})

        self.data1_label = Label(self)
        self.data1_label["text"] = "data1"
        self.data1_label.pack({"side": "top"})
        
        self.data1 = Entry(self)
        self.data1.pack({"side": "top"})

        self.data2_label = Label(self)
        self.data2_label["text"] = "data2"
        self.data2_label.pack({"side": "top"})
        
        self.data2 = Entry(self)
        self.data2.pack({"side": "top"})

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()
        self.serial = SerialThread()
        self.serial.start()
        

root = Tk()
root.title("Register Tester")
app = Application(master=root)
app.mainloop()
app.serial.join()
root.destroy()
