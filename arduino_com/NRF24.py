import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev 

GPIO.setmode(GPIO.BCM)

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17) # CE0, CSN

radio.setPayloadSize(32)
radio.setChannel(0x76)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)

radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openReadingPipe(1, pipes[1])
radio.printDetails()
# radio.startListening()

message = "1" # example message (number 1)
while len(message) < 32:
   message.append(0)

while True:
   start = time.time()
   radio.write(message)
   while not radio.available(0):
      time.sleep(1/100)
      if time.time() - start > 2:
         print('timed out')
         break
      
   receivedMessage = []
   radio.read(receivedMessage, radio.getDynamicPayloadSize())
   print("received: {}".format(receivedMessage))

   print("translating message to unicode")

   string = ""
   for c in receivedMessage:
      if (c >= 32 and c <= 126):
         string += chr(c)
   print("message decodes to: {}".format(string))

   radio.stopListening()
   time.sleep(1)