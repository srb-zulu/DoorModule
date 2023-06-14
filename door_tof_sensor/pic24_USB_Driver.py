
#from builtins import input

import time
import serial
import numpy as np 
import math       # import math for math.pi constant
import subprocess # for executing system calls
from threading import Thread

#Configure the Jetson to use the GPIO libary. These pins are used to drive LED indicators
import RPi.GPIO as GPIO
LED1_IND = 7

#Use Board pin naming convention (Pins identify by their 40 pin header number)
#GPIO.cleanup()
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup (LED1_IND,GPIO.OUT)  #Set pin 7 (GPIO 9) as output  
#GPIO.setup(LED1_IND, GPIO.OUT, initial=GPIO.LOW)

ID = ''

print("")
print("")
print("Start of PIC24 Interface Application")
print("------------------------------------")
print("")

#Try to connect to the serial port (USB)
try:
    print("STEP 1: Searching for PIC24 USB/Serial Port.")
    #Configure serial port. Make it global
    #serial_port = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5)    
    serial_port = serial.Serial(port="/dev/ttyUSB0",baudrate=921600,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5)
    print('USB Connection acquired')
    print("")
    # Wait a second to let the port initialize
    time.sleep(1)
    hardware_connected = True
except:
    hardware_connected = False
    print ("ERROR: USB Communication not present.")
    exit()  #Exit if not USB COM Present
    
FIRMWARE_VERSION_REQUIRED = "1.0.x" # Make sure the top 2 of 3 numbers match

missedPcktCtr = 0

class CircularBuffer(object):

    def __init__(self, max_size=10):
        """Initialize the CircularBuffer with a max_size if set, otherwise
        max_size will elementsdefault to 10"""
        self.buffer = [None] * max_size
        self.head = 0
        self.tail = 0
        self.max_size = max_size

    def __str__(self):
        """Return a formatted string representation of this CircularBuffer."""
        items = ['{!r}'.format(item) for item in self.buffer]
        return '[' + ', '.join(items) + ']'

    def size(self):
        """Return the size of the CircularBuffer
        Runtime: O(1) Space: O(1)"""
        if self.tail >= self.head:
            return self.tail - self.head
        return self.max_size - self.head - self.tail

    def is_empty(self):
        """Return True if the head of the CircularBuffer is equal to the tail,
        otherwise return False
        Runtime: O(1) Space: O(1)"""
        return self.tail == self.head

    def is_full(self):
        """Return True if the tail of the CircularBuffer is one before the head,
        otherwise return False
        Runtime: O(1) Space: O(1)"""
        return self.tail == (self.head-1) % self.max_size

    def enqueue(self, item):
        """Insert an item at the back of the CircularBuffer
        Runtime: O(1) Space: O(1)"""
        if self.is_full():
            raise OverflowError(
                "CircularBuffer is full, unable to enqueue item")
        self.buffer[self.tail] = item
        self.tail = (self.tail + 1) % self.max_size

    def front(self):
        """Return the item at the front of the CircularBuffer
        Runtime: O(1) Space: O(1)"""
        return self.buffer[self.head]

    def dequeue(self):
        """Return the item at the front of the Circular Buffer and remove it
        Runtime: O(1) Space: O(1)"""
        if self.is_empty():
            raise IndexError("CircularBuffer is empty, unable to dequeue")
        item = self.buffer[self.head]
        self.buffer[self.head] = None
        self.head = (self.head + 1) % self.max_size
        return item

class RingBuffer:
    """ class that implements a not-yet-full buffer (Circular Buffer)
        This buffer is used with the serial port interface class
    """

    def __init__(self, size_max):
        self.max = size_max
        self.data = []

    class __Full:
        """ class that implements a full buffer """

        def append(self, x):
            """ Append an element overwriting the oldest one. """
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max

        def get(self):
            """ return list of elements in correct order """
            return self.data[self.cur:]+self.data[:self.cur]

    def append(self, x):
        """append an element at the end of the buffer"""
        self.data.append(x)
        if len(self.data) == self.max:
            self.cur = 0
            # Permanently change self's class from non-full to full
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data
        
class Enumeration(object):
    def __init__(self, names):  # or *names, with no .split()
        number = 0
        for line, name in enumerate(names.split('\n')):
            if name.find(",") >= 0:
                # strip out the spaces
                while(name.find(" ") != -1):
                    name = name[:name.find(" ")] + name[(name.find(" ") + 1):]

                # strip out the commas
                while(name.find(",") != -1):
                    name = name[:name.find(",")] + name[(name.find(",") + 1):]

                # if the value was specified
                if(name.find("=") != -1):
                    number = int(float(name[(name.find("=") + 1):]))
                    name = name[:name.find("=")]

                # optionally print to confirm that it's working correctly
                #print ("%40s has a value of %d" % (name, number))

                setattr(self, name, number)
                number = number + 1

class Rover_HW_Driver(object):
    WHEEL_BASE_WIDTH         = 117  # distance (mm) from left wheel to right wheel. This works with the initial GPG3 prototype. Will need to be adjusted.
    WHEEL_DIAMETER           = 66.5 # wheel diameter (mm)
    WHEEL_BASE_CIRCUMFERENCE = WHEEL_BASE_WIDTH * math.pi # The circumference of the circle the wheels will trace while turning (mm)
    WHEEL_CIRCUMFERENCE      = WHEEL_DIAMETER   * math.pi # The circumference of the wheels (mm)

    MOTOR_GEAR_RATIO           = 120 # Motor gear ratio # 220 for Nicole's prototype
    ENCODER_TICKS_PER_ROTATION = 6   # Encoder ticks per motor rotation (number of magnet positions) # 16 for early prototypes
    MOTOR_TICKS_PER_DEGREE = ((MOTOR_GEAR_RATIO * ENCODER_TICKS_PER_ROTATION) / 360.0) # encoder ticks per output shaft rotation degree
    
    print('In PIC24 HW Driver Class')
    print(' ')

    USB_MESSAGE_TYPE = Enumeration("""
        NONE,
        GET_NAME,
        GET_ROVER_ORIENTATION_STATUS,
        GET_FIRMWARE_VERSION,
        GET_ANALOG_VALUES,
        GET_RC_RADIO_CHANNELS,
        GET_POT_VOLTAGE,
        GET_BAT_VOLTAGE,
        GET_CORNER_SERVO_MOTORS,
        SET_SERVO_MOTORS,
        SET_LEFT_MOTOR_PWM,
        SET_RIGHT_MOTOR_PWM,
        GET_LEFT_MOTOR_PWM,
        GET_RIGHT_MOTOR_PWM,
        SET_MOTOR_POSITION,
        SET_MOTOR_POSITION_KP,
        SET_MOTOR_POSITION_KD,
        SET_MOTOR_DPS,
        SET_MOTOR_LIMITS,
        ROVER_CONTROL_COMMAND,
        MOVE_ROVER_FWD,
        MOVE_ROVER_BKWD,
        MOVE_ROVER_LEFT,
        MOVE_ROVER_RIGHT,
        GET_HEART_BEAT,
        OFFSET_MOTOR_ENCODER,
        GET_ROVER_ODOMETRY_VALUES,
        GET_MOTOR_ENCODER_LEFT,
        GET_MOTOR_ENCODER_RIGHT,
        GET_MOTOR_STATUS_LEFT,
        GET_MOTOR_STATUS_RIGHT,
        GET_IMU_VALUES,
    """)
    
    SERVO_1 = 0x01
    SERVO_2 = 0x02

    MOTOR_LEFT  = 0x01
    MOTOR_RIGHT = 0x02
    MOTOR_FLOAT = -128

    def __init__(self, addr = 0x0A, detect = False):
        """
        Do any necessary configuration and detect the PIC24ROVER Board
        """
        print('Initializing PIC24 Device.')        

        missedPcktCtr = 0
        #Create a ring buffer of size (100)
        self.UART_ReceiveBuffer = RingBuffer(100)   
        self.UART_rx = CircularBuffer(100)
        #This ring buffer is global for the instance of this class.
        #Globals in the namespace are accessible to all objects. 
        print('Created PIC24 Received Buffer Object.\n\r')    
            
        self.init_digital_io()  #Initialize the digital IO
        
        connectCtr = 0
        self.imu_ctr = 0
        self.err_accel_x = 0

        self.RxData = [None] * 100  

        #Try to connect to the serial port (USB)    
        self.PIC24_Address = addr
        if detect == True:
            try:
                print('Sending Board Name Request to PIC24\n\r')

                time.sleep(0.01)

                #Get the PIC24 Board name to authenticate
                board = self.get_board()
                if board == 'PIC24INTER':
                    print('Found the PIC24 Driver Board')
                    print('')
                    GPIO.output(LED1_IND, GPIO.HIGH)  #Set LED1 'on' to indicate successful PIC24 connection
                else:                 
                    print('Did not found the PIC24 Driver Board')
                    #Get the PIC24 Board name to authenticate
                    board = self.get_board()   

                    #add retry counter
                    connectCtr = connectCtr + 1
                    print(connectCtr)

                #Imu_Values = self.get_imu_values()    
            except IOError:
                GPIO.output(LED1_IND, GPIO.LOW)       #Set LED1 'off' to indicate un-successful PIC24 connection  
                GPIO.cleanup()
                raise IOError("PIC24 Driver Board not Connected. USB COM Error.")

        print('Instantiated HW_Driver class.')
    
    def set_heartbeat_led_on(self):
        """
        This turns on the heart beat led
        """
        GPIO.output(LED1_IND, GPIO.HIGH)  #Set LED1 'on' to indicate successful PIC24 connection

    def set_heartbeat_led_off(self):
        """
        This turns on the heart beat led
        """
        GPIO.output(LED1_IND, GPIO.LOW)  #Set LED1 'on' to indicate successful PIC24 connection        
        
    def init_digital_io(self):
        """
        This method initilizes the digital IO pins
        """
        #GPIO.cleanup()
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup (LED1_IND,GPIO.OUT)  #Set pin 7 (GPIO 9) as output  
        GPIO.setup(LED1_IND, GPIO.OUT, initial=GPIO.LOW)
        print('Initialized Digial IO Pins')

    def cleanup_digital_io(self):
        """
        This method cleans up all digital IOs before exiting
        """
        #GPIO.output(LED1_IND, GPIO.LOW)

        print("In cleanup_digital_io method.")
        GPIO.cleanup()       

    def usb_transfer_array(self, data_out):
        """
        Conduct a USB/UART transaction
        Keyword arguments:
        data_out -- a list of bytes to send. The length of the list will determine how many bytes are transferred.
        Returns a list of the bytes read.
        """
        
        #Transmit data packet to PIC24 via USB/UART interface
        serial_port.write(data_out)
        #print('Transmitted: ', data_out)
        
        '''
        This function is use to read data from the serial port (UART)
        '''        
        PacketLength = 80 #83
        PacketData = [None] * 100
        BoardID = []
        
        PacketSize = 0
        strPacketLength = ''

        PacketID = 0
        strPacketID = ''  
        strData = ''      
        Data = []

        BufferLength = 0
        PacketReady = False
        FoundHeader = False
        index = 0
        
        Listlength = 0
        Received_Data = [None] * 100
        RawData = [None] * 100
        global missedPcktCtr

        #dataArry = []

        #b = ['24','24','24']  #Sequence of charaters to look for ($$$)
        b = 36  #Sequence of charaters to look for ($$$)
        indexes = [36, 36, 36]
        
        Timeout_ctr = 0  #Use a time out timer in case PIC24 doesn't reply or USB/COM port error

        PacketReady = False

        try:
            while PacketReady == False:

                # Decode the bytes sent back from the PIC24
                if serial_port.inWaiting() > 1:
                    
                    Timeout_ctr = 0
                    Received_Data = serial_port.read(size = 100)
                    #print("Received USB Data: ", Received_Data)

                    if Received_Data[0] == None:
                        print('Received none')
                        BufferLength = 0
                        #break
                    else:                                         
                        self.UART_ReceiveBuffer.append(Received_Data[0])   #Collect data into a ring (circular) buffer       
                        #BufferLength = len(self.UART_ReceiveBuffer.get())                        
                        BufferLength = len(Received_Data)

                        #self.UART_rx.enqueue(Received_Data[0])
                        #print("Buffer: ", self.UART_rx)
                        #print("Buffer Data: ", self.UART_ReceiveBuffer.get())
                    
                    #Wait for at least 50 bytes before retrieving the packet
                    if BufferLength >= PacketLength:   #50 to make sure we receive at least one entire packet.
                        #RawData = self.UART_ReceiveBuffer.get()
                        RawData = Received_Data

                        print ('Raw Data: ', RawData)
                        print("")
                        print('List Length:', BufferLength)

                        index = 0
                        #Look for the $$$ sequence indicating the start of the packet
                        for i in range(BufferLength):
                            if b == RawData[i] and RawData[i+1] == 36 and RawData[i+2] == 36: 
                                indexes.append(i)                                
                                index = i   
                                #print("-------------------")
                                #print('Index Value:', index)
                                #print("-------------------")                                
                                #print('Indexes:', indexes)                                
                                if index > 53:    #62
                                    print("Index too high.")
                                    FoundHeader = False
                                    #self.UART_ReceiveBuffer.data.clear()   #---> Python 3 only
                                    break
                                elif index < 52:   #32        
                                    print("")
                                    print('Found Header Sequence!')
                                    print("")
                                    FoundHeader = True
                                    break      #index found break for loop         
                                else:
                                    print("Header not found.")
                                    FoundHeader = False
                                    #self.UART_ReceiveBuffer.data.clear()   #---> Python 3 only
                                    break                                    

                        if FoundHeader == True:                        
                            
                            try:    
                                strPacketLength = chr(RawData[index+3]) + chr(RawData[index+4]) + chr(RawData[index+5])
                                PacketSize = int(strPacketLength)                               
                                #print("Packet Size: ", PacketSize) 

                                strPacketID = chr(RawData[index+6]) + chr(RawData[index+7]) + chr(RawData[index+8])
                                PacketID = int(strPacketID) 
                                #print("Packet ID: ", PacketID) 

                                #Retrieve the entire packet using the packet size variable
                                for i in range(PacketSize):  
                                    Data.append(RawData[(index+9)+i])    

                                #Retrieve the entire packet using the packet size variable
                                #PacketData = RawData[(index + 9) : (PacketSize + 9)] 
                                                                    
                                print('Packet Data: ', Data) 
                                Length = len(Data)
                                print("Packet Length: ", Length)
                                PacketReady = True        #Packet decoded, break while loop
                                FoundHeader = False
                            except Exception as exception_error:
                                print("Error: " + str(exception_error))

                        else:
                            #Header sequence not found. Return empty list 
                            print('Packet header not found.')                            
                            PacketReady = True
                            missedPcktCtr += 1
                            print("Missed Packets: ", missedPcktCtr)                            
                            #PacketData.clear()

                # time.sleep(0.001)  # 1 ms counter
                # Timeout_ctr = Timeout_ctr + 1
                # print('Ctr: ', Timeout_ctr)

                # if Timeout_ctr >= 10:
                #     print('Serial Port Timed Out. No reply from the PIC24')
                #     #Flush input and output buffers before exiting
                    
                #     serial_port.reset_input_buffer()
                #     serial_port.reset_output_buffer()
                #     #serial_port.close()
                #     break  

            #Flush input and output buffers before exiting
            serial_port.reset_input_buffer()
            serial_port.reset_output_buffer()
            
            #print("Received Serial Data")
            #Clear the ring buffer. data is the list/array that collects the values
            #del self.UART_ReceiveBuffer.data[:]        
            #self.UART_ReceiveBuffer.data.clear()   #---> Python 3 only
            
            print("Missed Packets: ", missedPcktCtr)   

            if PacketReady:               
                #print("")
                #print(self.UART_ReceiveBuffer.get())
                PacketReady = False
                #return PacketData
                return Data                
            else:
                serial_port.reset_input_buffer()
                serial_port.reset_output_buffer()
                print("No Valid Data to return.")                         
                PacketReady = False                
                return ""

        except KeyboardInterrupt:
            serial_port.close()
            self.cleanup_digital_io()
            print("Exiting Program")

        except Exception as exception_error:
            print("Receiving Error (USB). Exiting Program")
            print("Error: " + str(exception_error))

    def get_board(self):
        """
        Get the PIC24 board name
        """
        try:
            BoardID = [""]

            outArray = [0x24, 0x24, 0x24, self.PIC24_Address, self.USB_MESSAGE_TYPE.GET_NAME,\
                        0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x23, 0x23 ]
            print('Transmitting:', outArray)
            
            reply = self.usb_transfer_array(outArray)
            print('Receiving Board ID:', reply)

            #Convert string to ASCII character
            BoardID = [chr(x) for x in reply]
            print('Board ID: ', BoardID)

            #Board ID Byte starts at byte[0]. After Board ID Byte[0]   
            if(BoardID[0] == 'P'):   #'P'
                name = ""
                #Concatenate the string value
                for c in range(0, 10):   #Board ID is 10 characters long
                    if BoardID[c] != 0:
                        name += BoardID[c]
                    else:
                        break
                return name  #Return the name of the board
        
        except ValueError:
            print('Not a number')
            return ""
        except Exception as exception_error:
            print("Error in get_board()")
            print("Error: " + str(exception_error))
            
    def get_servo_motors(self):
        """
        Get the servo motor positions
        """
        Servo_1_Pos = 0  
        Servo_2_Pos = 0

        try:
            outArray = [0x24, 0x24, 0x24, self.PIC24_Address, self.USB_MESSAGE_TYPE.GET_CORNER_SERVO_MOTORS,\
                        0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x23, 0x23 ]
            #print('Transmitting:', outArray)
            
            reply = self.usb_transfer_array(outArray)
            print('Receiving:', reply)
            print('Type:', type(reply[0]))
           
            if(len(reply) != 0):
                print('Received Servo Motor Position Data: ', reply)
                print("")
                print("")  

                Byte0 = bin(reply[0]) 
                print("-----------------------------")
                print("Byte 0: ", Byte0)
                print("-----------------------------")
                Byte1 = bin(reply[1])             
                print("Byte 1: ", Byte1)
                print("-----------------------------")
                Pos1 = (reply[0] << 8) | reply[1]
                Servo_1_Pos = int(Pos1)  #Load the new value
                print("Servo 1 Position = ", Servo_1_Pos)

                Byte2 = bin(reply[2]) 
                print("-----------------------------")
                print("Byte 2 ", Byte2)
                print("-----------------------------")
                Byte3 = bin(reply[3])             
                print("Byte 3: ", Byte3)
                print("-----------------------------")
                Pos2 = (reply[2] << 8) | reply[3]
                Servo_2_Pos = int(Pos2)  #Load the new value
                print("Servo 2 Position = ", Servo_2_Pos)
                                 
                Byte4 = hex(reply[4])
                if Byte4 != "0x32":
                    print("UART Rx Error.")                
                Byte5 = hex(reply[5])
                if Byte5 != "0x33":
                    print("UART Rx Error.")                
                Byte6 = hex(reply[6])
                if Byte6 != "0x34":
                    print("UART Rx Error.")                  
                Byte7 = hex(reply[7])
                if Byte7 != "0x35":
                    print("UART Rx Error.")                  
                Byte8 = hex(reply[8])
                if Byte8 != "0x36":
                    print("UART Rx Error.")                  

                return [Servo_1_Pos, Servo_2_Pos]

        except ValueError:
            print('Not a number')
            return ""
        except Exception as exception_error:
            print("Error in get_servo_motors()")
            print("Error: " + str(exception_error))

    def get_rc_channels(self):
        """
        Read the 8 (2-byte) RC Channel values
        """
        RC_CH1 = 0
        RC_CH2 = 0
        RC_CH3 = 0
        RC_CH4 = 0
        RC_CH5 = 0
        RC_CH6 = 0
        RC_CH7 = 0
        RC_CH8 = 0
        
        outArray = [0x24, 0x24, 0x24, self.PIC24_Address, self.USB_MESSAGE_TYPE.GET_RC_RADIO_CHANNELS,\
                    0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x23, 0x23 ]
        print('Transmitting:', outArray)
        reply = self.usb_transfer_array(outArray)
        print('Reply:', reply)
        
        #Convert the 10-bit ADC hex number to decimal and then to voltage
        RC_CH1 = (hex(ord(reply[1]))[2:]) + (hex(ord(reply[2]))[2:])
        RC_CH2 = (hex(ord(reply[3]))[2:]) + (hex(ord(reply[4]))[2:])
        RC_CH3 = (hex(ord(reply[5]))[2:]) + (hex(ord(reply[6]))[2:])
        RC_CH4 = (hex(ord(reply[7]))[2:]) + (hex(ord(reply[8]))[2:])
        RC_CH5 = (hex(ord(reply[9]))[2:]) + (hex(ord(reply[10]))[2:])
        RC_CH6 = (hex(ord(reply[11]))[2:]) + (hex(ord(reply[12]))[2:])
        RC_CH7 = (hex(ord(reply[13]))[2:]) + (hex(ord(reply[14]))[2:])
        RC_CH8 = (hex(ord(reply[15]))[2:]) + (hex(ord(reply[16]))[2:])
        
        #raise IOError("No SPI response")
        return [RC_CH1, RC_CH2, RC_CH3, RC_CH4, RC_CH5, RC_CH6, RC_CH7, RC_CH8]

    def get_imu_values(self):
        """
        Read the imu values
        """
        Gyro_x = 0
        Gyro_y = 0
        Gyro_z = 0
        
        Accel_x = 0
        Accel_y = 0
        Accel_z = 0

        Pitch = 0
        Roll = 0
        
        x = 0
        y = 0
        ListLength = 0

        IMU_Data = [""]

        try:
            outArray = [0x24, 0x24, 0x24, self.PIC24_Address, self.USB_MESSAGE_TYPE.GET_IMU_VALUES,\
                        0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x23, 0x23 ]
            print("")
            print("")

            print('Transmitting IMU Request:', outArray)
            reply = self.usb_transfer_array(outArray)
            
            if(len(reply) != 0):
                print('Received IMU Data: ', reply)
                print("")
                print("")

                print("--------------------------------------------------")
                #Convert the 10-bit ADC hex number to decimal and then to voltage
                Temp1 = (chr(reply[0])) + (chr(reply[1])) + (chr(reply[2])) + (chr(reply[3])) + (chr(reply[4])) + (chr(reply[5]))      
                Accel_x = round((float(Temp1)), 6)              
                print("Accel x: ", Accel_x)
                
                Temp2 = (chr(reply[6])) + (chr(reply[7])) + (chr(reply[8])) + (chr(reply[9])) + (chr(reply[10])) + (chr(reply[11]))
                Accel_y = round((float(Temp2)), 6)
                print("Accel y: ", Accel_y)
                
                Temp3 = (chr(reply[12])) + (chr(reply[13])) + (chr(reply[14])) + (chr(reply[15])) + (chr(reply[16])) + (chr(reply[17]))
                Accel_z = round((float(Temp3)), 6)
                print("Accel z: ", Accel_z)    

                #Calculate roll
                Roll = math.atan2(Accel_y, Accel_z); 
                #Calculate pitch
                Pitch = math.atan(Accel_x/(math.sqrt(Accel_y*Accel_y + Accel_z*Accel_z)));
                
                #Covert radians to degrees
                Roll  *= (180.0 / math.pi);
                Pitch *= (180.0 / math.pi);     

                Roll = round(Roll, 6)    
                Pitch = round(Pitch, 6)  
                
                print("--------------------------------------------------")
                print("Roll: ", round(Roll, 4))       
                print("Pitch: ", round(Pitch, 4))       
                print("--------------------------------------------------")

            ###############################################################################################################################################
            ###################################                         RETRIEVE GYRO DATA              ###################################################
            ###############################################################################################################################################
                Temp1 = (chr(reply[18])) + (chr(reply[19])) + (chr(reply[20])) + (chr(reply[21])) + (chr(reply[22])) + (chr(reply[23]))    
                #print("Gyro x: ", Temp1)
                Gyro_x = round((float(Temp1)), 6)            
                print("Gyro x: ", Gyro_x)
                
                Temp2 = (chr(reply[24])) + (chr(reply[25])) + (chr(reply[26])) + (chr(reply[27])) + (chr(reply[28])) + (chr(reply[29]))
                #print("Gyro y: ", Temp2)
                Gyro_y = round((float(Temp2)), 6)
                print("Gyro y: ", Gyro_y)
                
                Temp3 = (chr(reply[30])) + (chr(reply[31])) + (chr(reply[32])) + (chr(reply[33])) + (chr(reply[34])) #+ (chr(reply[35]))
                #print("Gyro z: ", Temp3)
                Gyro_z = round((float(Temp3)), 6)
                print("Gyro z: ", Gyro_z)    

                print("--------------------------------------------------")
                print("")

                # print('Servo Motor Position Data')
                # print("-------------------------")

                # Temp1 = hex(reply[36])
                # Temp2 = hex(reply[37])
                # Temp3 = Temp1[2:] + Temp2[2:]

                # print("--------------")
                # print("Temp1: ", Temp1)
                # print("--------------")
                # print("Temp2: ", Temp2)
                # print("--------------")
                # print("Temp3: ", int(Temp3, 16))
                # print("--------------")

            else:
               print("IMU Value not valid.")  

        except ValueError:
            print('Not a number')
            return ""
        except IOError:
            print('USB Connection closed.')
            #Try to reconnect with the PIC24 board via USB.
            if serial_port.is_open() == False:
                print('PIC24 USB Connection closed.')
            else:
                print('Unknown Communication Error.')

            return ""            
        except Exception as exception_error:
            print("Error in get_imu_values()")
            print("Error: " + str(exception_error))


        finally:
            #print('Returning Values')
            #return [Accel_x, Accel_y, Accel_z, Roll, Pitch, Gyro_x, Gyro_y, Gyro_z]
            return [Accel_x, Accel_y, Accel_z, Gyro_x, Gyro_y, Gyro_z]

    def get_heart_beat(self):
        """
        Send a periodic message to the PIC24 board to unsure that is connected
        """        
        HeartBeatMessage = ''   #Declare and clear a string variable
        
        try:
            outArray = [0x24, 0x24, 0x24, self.PIC24_Address, self.USB_MESSAGE_TYPE.GET_HEART_BEAT,\
                        0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x23, 0x23 ]

            reply = self.usb_read_16(outArray)
            #print('Reply:', reply)
            
            #Retrieve the message
            HeartBeatMessage = (reply[1]) + (reply[2]) + (reply[3])              
            #print(HeartBeatMessage)        
                        
        except ValueError:
            #print('Not a number')
            return ""
        except:
            #print('Unknown Error')
            return ""
        finally:
            #print('Returning Values')
            #This finally block always executes
            return HeartBeatMessage
        
    def get_version_hardware(self):
        """
        Read the hardware version
        Returns touple:
        hardware version, error
        """
        version = self.usb_read_32(self.USB_MESSAGE_TYPE.GET_HARDWARE_VERSION)
        return ("%d.x.x" % (version / 1000000))

    def get_version_firmware(self):
        """
        Read the firmware version
        Returns touple:
        firmware version, error
        """
        version = self.usb_read_32(self.USB_MESSAGE_TYPE.GET_FIRMWARE_VERSION)
        return ("%d.%d.%d" % ((version / 1000000), ((version / 1000) % 1000), (version % 1000)))

    def get_id(self):
        """
        Read the 128-bit GoPiGo3 hardware serial number
        Returns touple:
        serial number as 32 char HEX formatted string, error
        """
        outArray = [self.PIC24_Address, self.USB_MESSAGE_TYPE.GET_ID,\
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        reply = self.usb_transfer_array(outArray)
        if(reply[3] == 0xA5):
            return ("%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X" % \
            (reply[4], reply[5], reply[6], reply[7], reply[8], reply[9], reply[10], reply[11], \
             reply[12], reply[13], reply[14], reply[15], reply[16], reply[17], reply[18], reply[19]))
        raise IOError("No SPI response")
        return "00000000000000000000000000000000"

    def set_led(self, led, red, green = 0, blue = 0):
        """
        Set an LED
        Keyword arguments:
        led -- The LED(s). LED_LEFT_EYE, LED_RIGHT_EYE, LED_LEFT_BLINKER, LED_RIGHT_BLINKER, and/or LED_WIFI.
        red -- The LED's Red color component (0-255)
        green -- The LED's Green color component (0-255)
        blue -- The LED's Blue color component (0-255)
        """

        outArray = [self.PIC24_Address, self.USB_MESSAGE_TYPE.SET_LED, led, red, green, blue]
        reply = self.usb_transfer_array(outArray)

    def get_pot_voltage(self):
        """
        Get the PIC24 potentiometer voltage
        Returns touple:
        """
        pot_voltage = 0
        outArray = [0x24, 0x24, 0x24, self.PIC24_Address, self.USB_MESSAGE_TYPE.GET_POT_VOLTAGE,\
                    0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x23, 0x23 ]
        
        reply = self.usb_read_16(outArray)
        print('Received:', reply)        
        
        #Convert the 10-bit ADC hex number to decimal and then to voltage
        pot_voltage = reply[1] + reply[2]
        pot_voltage = (int(pot_voltage, 16)*3.3)/1023
        pot_voltage = round(pot_voltage, 2)
        print('Pot Voltage: {0}{1}'.format(pot_voltage,'V'))
        
        return (pot_voltage)
        
    def get_voltage_battery(self):
        """
        Get the battery voltage. Use this request as a heart beat check. Illuminate an indicator
        LED. 
        Returns float:
        battery voltage, error
        """
        batt_voltage = 0
        outArray = [0x24, 0x24, 0x24, self.PIC24_Address, self.USB_MESSAGE_TYPE.GET_BAT_VOLTAGE,\
                    0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x23, 0x23 ]
        
        reply = self.usb_read_16(outArray)
        #print('Received:', reply)
        try:
            #Convert the 10-bit ADC hex number to decimal and then to voltage
            num1 = hex(ord(reply[1]))[2:]
            num2 = hex(ord(reply[2]))[2:]
            #print(num1)
            #print(num2)

            batt_voltage = num1 + num2
            #print('Battery Voltage:', batt_voltage)
            batt_voltage = (int(batt_voltage, 16)*3.3)/1023
            batt_voltage = round(batt_voltage*4.6, 2)
            #print('Battery Voltage: {0}{1}'.format( batt_voltage,'V' ))
 
        except IndexError:
            #print('Not a number')
            return ""
        except:
            #print('Unknown Error')
            return ""
        finally:
            #print('Returning Values')
            return (batt_voltage)

    def set_servo(self, servo, us):
        """
        Set a servo position in microseconds
        Keyword arguments:
        servo -- The servo(s). SERVO_1 and/or SERVO_2.
        us -- The pulse width in microseconds (0-16666)
        """
        outArray = [self.PIC24_Address, self.USB_MESSAGE_TYPE.SET_SERVO, servo,\
                    ((us >> 8) & 0xFF), (us & 0xFF)]
        reply = self.usb_transfer_array(outArray)

    def set_motor_power(self, port, power):
        """
        Set the motor power in percent
        Keyword arguments:
        port -- The motor port(s). MOTOR_LEFT and/or MOTOR_RIGHT.
        power -- The PWM power from -100 to 100, or MOTOR_FLOAT for float.
        """
        if(power > 127):
            power = 127
        if(power < -128):
            power = -128
        outArray = [self.PIC24_Address, self.USB_MESSAGE_TYPE.SET_MOTOR_PWM, port, int(power)]
        self.usb_transfer_array(outArray)

    def set_motor_position(self, port, position):
        """
        Set the motor target position in degrees
        Keyword arguments:
        port -- The motor port(s). MOTOR_LEFT and/or MOTOR_RIGHT.
        position -- The target position
        """
        position_raw = int(position * self.MOTOR_TICKS_PER_DEGREE)
        outArray = [self.PIC24_Address, self.USB_MESSAGE_TYPE.SET_MOTOR_POSITION, port,\
                    ((position_raw >> 24) & 0xFF), ((position_raw >> 16) & 0xFF),\
                    ((position_raw >> 8) & 0xFF), (position_raw & 0xFF)]
        reply = self.usb_transfer_array(outArray)

    def set_motor_dps(self, port, dps):
        """
        Set the motor target speed in degrees per second
        Keyword arguments:
        port -- The motor port(s). MOTOR_LEFT and/or MOTOR_RIGHT.
        dps -- The target speed in degrees per second
        """
        dps = int(dps * self.MOTOR_TICKS_PER_DEGREE)
        outArray = [self.PIC24_Address, self.USB_MESSAGE_TYPE.SET_MOTOR_DPS, int(port),\
                    ((dps >> 8) & 0xFF), (dps & 0xFF)]
        self.usb_transfer_array(outArray)

    def set_motor_limits(self, port, power = 0, dps = 0):
        """
        Set the motor speed limit
        Keyword arguments:
        port -- The motor port(s). MOTOR_LEFT and/or MOTOR_RIGHT.
        power -- The power limit in percent (0 to 100), with 0 being no limit (100)
        dps -- The speed limit in degrees per second, with 0 being no limit
        """
        dps = int(dps * self.MOTOR_TICKS_PER_DEGREE)
        outArray = [self.PIC24_Address, self.USB_MESSAGE_TYPE.SET_MOTOR_LIMITS, int(port), int(power),\
                    ((dps >> 8) & 0xFF), (dps & 0xFF)]
        self.usb_transfer_array(outArray)

    def get_motor_status(self, port):
        """
        Read a motor status
        Keyword arguments:
        port -- The motor port (one at a time). MOTOR_LEFT or MOTOR_RIGHT.
        Returns a list:
            flags -- 8-bits of bit-flags that indicate motor status:
                bit 0 -- LOW_VOLTAGE_FLOAT - The motors are automatically disabled because the battery voltage is too low
                bit 1 -- OVERLOADED - The motors aren't close to the target (applies to position control and dps speed control).
            power -- the raw PWM power in percent (-100 to 100)
            encoder -- The encoder position
            dps -- The current speed in Degrees Per Second
        """
        if port == self.MOTOR_LEFT:
            message_type = self.USB_MESSAGE_TYPE.GET_MOTOR_STATUS_LEFT
        elif port == self.MOTOR_RIGHT:
            message_type = self.USB_MESSAGE_TYPE.GET_MOTOR_STATUS_RIGHT
        else:
            raise IOError("get_motor_status error. Must be one motor port at a time. MOTOR_LEFT or MOTOR_RIGHT.")
            return

        outArray = [self.PIC24_Address, message_type, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        reply = self.usb_transfer_array(outArray)
        if(reply[3] == 0xA5):
            power = int(reply[5])
            if power & 0x80:
                power = power - 0x100

            encoder = int((reply[6] << 24) | (reply[7] << 16) | (reply[8] << 8) | reply[9])
            if encoder & 0x80000000:
                encoder = int(encoder - 0x100000000)

            dps = int((reply[10] << 8) | reply[11])
            if dps & 0x8000:
                dps = dps - 0x10000

            return [reply[4], power, int(encoder / self.MOTOR_TICKS_PER_DEGREE), int(dps / self.MOTOR_TICKS_PER_DEGREE)]
        raise IOError("No SPI response")
        return

    def get_motor_encoder(self, port):
        """
        Read a motor encoder in degrees
        Keyword arguments:
        port -- The motor port (one at a time). MOTOR_LEFT or MOTOR_RIGHT.
        Returns the encoder position in degrees
        """
        if port == self.MOTOR_LEFT:
            message_type = self.USB_MESSAGE_TYPE.GET_MOTOR_ENCODER_LEFT
        elif port == self.MOTOR_RIGHT:
            message_type = self.USB_MESSAGE_TYPE.GET_MOTOR_ENCODER_RIGHT
        else:
            raise IOError("Port(s) unsupported. Must be one at a time.")
            return 0

        encoder = self.usb_read_32(message_type)
        if encoder & 0x80000000:
            encoder = int(encoder - 0x100000000)
        return int(encoder / self.MOTOR_TICKS_PER_DEGREE)

    def offset_motor_encoder(self, port, offset):
        """
        Offset a motor encoder
        Keyword arguments:
        port -- The motor port(s). MOTOR_LEFT and/or MOTOR_RIGHT.
        offset -- The encoder offset
        Zero the encoder by offsetting it by the current position
        """
        offset = int(offset * self.MOTOR_TICKS_PER_DEGREE)
        outArray = [self.PIC24_Address, self.USB_MESSAGE_TYPE.OFFSET_MOTOR_ENCODER, int(port),\
                    ((offset >> 24) & 0xFF), ((offset >> 16) & 0xFF), ((offset >> 8) & 0xFF), (offset & 0xFF)]
        self.usb_transfer_array(outArray)

    def reset_motor_encoder(self, port):
        """
        Reset a motor encoder to 0
        Keyword arguments:
        port -- The motor port(s). MOTOR_LEFT and/or MOTOR_RIGHT.
        """
        if port & self.MOTOR_LEFT:
            self.offset_motor_encoder(self.MOTOR_LEFT, self.get_motor_encoder(self.MOTOR_LEFT))

        if port & self.MOTOR_RIGHT:
            self.offset_motor_encoder(self.MOTOR_RIGHT, self.get_motor_encoder(self.MOTOR_RIGHT))

    def reset_all(self):
        """
        Reset the GoPiGo3.
        """
        # reset all sensors
        self.set_grove_type(self.GROVE_1 + self.GROVE_2, self.GROVE_TYPE.CUSTOM)
        self.set_grove_mode(self.GROVE_1 + self.GROVE_2, self.GROVE_INPUT_DIGITAL)

        # Turn off the motors
        self.set_motor_power(self.MOTOR_LEFT + self.MOTOR_RIGHT, self.MOTOR_FLOAT)

        # Reset the motor limits
        self.set_motor_limits(self.MOTOR_LEFT + self.MOTOR_RIGHT, 0, 0)

        # Turn off the servos
        self.set_servo(self.SERVO_1 + self.SERVO_2, 0)

        # Turn off the LEDs
        self.set_led(self.LED_EYE_LEFT + self.LED_EYE_RIGHT + self.LED_BLINKER_LEFT + self.LED_BLINKER_RIGHT, 0, 0, 0)       
        
def main():

    try:
        while True:
        
            print('In Hardware class main loop.')

            #Rover1 = Rover_HW_Driver() 
            #Value1 = Rover1.get_voltage_battery() 
            #print(Value1)

            time.sleep(5)
            #exit()        

    except KeyboardInterrupt:
        time.sleep(0.5)
        print("Exiting Program")

    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

if __name__ == '__main__':
    main()    
    