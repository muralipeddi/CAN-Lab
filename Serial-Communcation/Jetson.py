import serial
import time

uart = serial.Serial(port='/dev/ttyACM0',
                     baudrate=115200,
                     bytesize = serial.EIGHTBITS, 
                     parity = serial.PARITY_NONE,
                     stopbits = serial.STOPBITS_ONE,
                     timeout=1, 
                     xonxoff = False, 
                     rtscts = False, 
                     dsrdtr = False, 
                     writeTimeout = 2)

if uart.is_open:
   print("UART Established")

try:
    while True:
        message="Hi"
        uart.write(message.encode())
        print(f"sent:{message}")
     #time.sleep(1)
        #data = uart.readline()   
        #if data:
            #print(data)
        time.sleep(1)
    
except KeyboardInterrupt:
    print("Communication stopped")

finally:
    uart.close()
    print("UART closed")


