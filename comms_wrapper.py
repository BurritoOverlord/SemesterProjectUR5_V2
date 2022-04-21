import serial
import serial.tools.list_ports
from threading import Thread
import time
from pynput import keyboard
import copy

########
### Arduino communication
########

class Arduino: 
    def __init__(self, descriptiveDeviceName, portName, baudrate): 
        # About the device
        self.descriptiveDeviceName = descriptiveDeviceName
        self.portName = portName
        self.baudrate = baudrate
        
        # Communication
        self._rawReceivedMessage = None
        self.receivedMessages = {}
        self.messageNames = None
        self.arduino = None
        self.handshakeStatus = False
        self.connectionStatus = False
        self._echo_python_msg = None
        self.newMsgRecieved = False

        # Threading
        self.__thread = None

    # Private methods
    def _serial_readline(self):
        while 1:
            try:
                self._rawReceivedMessage = self.arduino.readline().decode('utf-8')[:-2]
            except:
                pass
            time.sleep(0.001)


    def _startReadingThread(self):
        self.__thread = Thread(target=self._serial_readline)
        self.__thread.daemon = True
        self.__thread.start()


    def _serial_write(self, msg):
        if self.arduino is not None:
            self.arduino.write(bytes(msg, 'utf-8'))


    def _add_new_message_name(self, msgName):
        if msgName in self.receivedMessages:
            print("!! Message name ", msgName, " already exists so it cannot be added. !!")
        else:
            self.receivedMessages[msgName] = None
            self.messageNames = list(self.receivedMessages.keys())


    def _connect_to_arduino(self):
        # Connect to the arduino device
        try: 
            self.arduino = serial.Serial(port=self.portName, baudrate=self.baudrate)

            # toggle dtr to reset the arduino
            self.arduino.dtr = True
            self.arduino.dtr = False

            self.connectionStatus = True
        
            print("Successfully connected to " + self.descriptiveDeviceName)
            return True
        except:
            print("!! Cannot connect to " + self.descriptiveDeviceName + " !!")
            return False
        

    def _disect_and_save_message(self, msg):
        receivedMessageTemp = copy.deepcopy(self.receivedMessages)

        if msg[-2] != ":":
            return False

        msg = msg[1:-2]
        splitMsg = msg.split(":")
        for singleMsg in splitMsg:
            if len(singleMsg.split(";")) == 2:
                msgName = singleMsg.split(";")[0]
                msgPayload = singleMsg.split(";")[1]


                if msgName == "echo_python":
                        self._echo_python_msg = msgPayload
                else:
                    if msgName not in self.messageNames:
                        return False
                    
                    receivedMessageTemp[msgName] = msgPayload

            else:
                return False
        
        if receivedMessageTemp == self.receivedMessages: 
            self.newMsgRecieved = False
        else: 
            self.newMsgRecieved = True
        
        self.receivedMessages = receivedMessageTemp
        return True

        
    # Public methods
    def define_message_names(self, msgNames):
        if type(msgNames) == list:
            for msg in msgNames:
                self._add_new_message_name(msg) 
        else: 
            print("Argument should be of type 'list'")


    def connect_and_handshake(self): 
        # Connect to the arduino device

        if self._connect_to_arduino():
            pass
        else:
            return False

        # Start the reading thread
        self._startReadingThread()
        
        # Wait for a bit for the arduino to initialise nicely
        time.sleep(0.5)

        # Conduct the handshake process
        timeoutTimer = time.time()
        handshakeTimeoutSec = 5

        self.arduino.reset_input_buffer()
        self.arduino.reset_output_buffer()


        while time.time() - timeoutTimer < handshakeTimeoutSec:
            self._serial_write("handshake1\n")
            time.sleep(0.1)

            if self._rawReceivedMessage == "handshake2":
                self.handshakeStatus = True
                break

        if self.handshakeStatus: 
            print("Successfull handshake with " + self.descriptiveDeviceName)
            while 1: 
                self.receive_message()
                if self._echo_python_msg == "NO_PYTHON_MESSAGE":
                    break
        else:
            print("!! Handshake failed with " + self.descriptiveDeviceName + " !!")

        return self.handshakeStatus


    def disconnect_arduino(self):
        self.arduino.close()


    def send_message(self, msg):
        self._serial_write( msg + "\n")


    def receive_message(self, printOutput = False, verbose = False):
        if not self.handshakeStatus:
            print("!! Handshake not completed !!")
            return False
        else:
            isMessageValid = True
            msg = self._rawReceivedMessage
            # sanity check 1: check if ends of the message are < and >
            try:
                if msg[0] == "<" and msg[-1] == ">":
                    pass
                else:
                    isMessageValid = False
            except:
                isMessageValid = False
            if isMessageValid:
                isMessageValid = self._disect_and_save_message(msg)
            else:
                pass
            
            if printOutput: 
                if verbose: 
                    print("----------------------")
                    print("Raw message received on python side: ", self._rawReceivedMessage)
                    print("Messege received from the arduino: ", self.receivedMessages)
                    print("Python message stored on", self.descriptiveDeviceName, ": ", self._echo_python_msg, "\n")

                else:
                    print("Messege received from the arduino: ", self.receivedMessages)

            time.sleep(0.000001)
            return True


    def current_status(self):
        status = {
            "Device name" : self.descriptiveDeviceName, 
            "Baudrate: ": self.baudrate,
            "Portname: ": self.portName, 
            "Connection: ": self.connectionStatus, 
            "Handshake: ": self.handshakeStatus, 
            "Message names: ": self.messageNames
            }

        return status
    

    def debug(self, verbose = False):
        if verbose:   
            self.receive_message(printOutput=True, verbose = True)

            print("Current status of this device:")
            print(self.current_status())
            
        else:
            print("----------------------")
            self.receive_message(printOutput=True)
            print("Python message stored on", self.descriptiveDeviceName, ": ", self._echo_python_msg, "\n")
            

        for key, value in self.receivedMessages.items():
            if value is None: 
                print("Check if message name: ", key, " agrees with the arduino side")


########
### Key commands
########

class Key():
    def __init__(self):                
        self.keyPressLatching = None
        self._keyReleaseLatching = None
        self.keyPress = None
        self._start_keyboard_listener()

    def _on_press(self, key):
        try:
            self.keyPressLatching = key.char
            self.keyPress = key.char
            
        except AttributeError:
            self.keyPressLatching = key
            self.keyPress = key
            

    def _on_release(self, key):
        try:
            self._keyReleaseLatching = key.char

            if self._keyReleaseLatching == self.keyPress: 
                self.keyPress = None

        except AttributeError:
            self._keyReleaseLatching = key

            if self._keyReleaseLatching == self.keyPress: 
                self.keyPress = None


    def _start_keyboard_listener(self):  
        listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        listener.start()
        print("keyboard listener started")