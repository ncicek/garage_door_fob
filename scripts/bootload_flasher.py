#flash msp430 over nrf24
#this script  >serial>  arduino  >spi>  nrf  >wireless>  nrf  >spi>  msp430
import serial
import time
from intelhex import IntelHex

def get_byte():
	return (int.from_bytes(ser.read(),byteorder='big'))

start_addr = 0xe200
end_addr = 0xffff-1

ih = IntelHex()                     # create empty object
ih.fromfile('hex.hex',format='hex')               # load from hex

hex_dict = ih.todict()

hex_lenght = ih.maxaddr()
#print (ih.minaddr())

hex_list = [0]*(hex_lenght +1)
for key, value in hex_dict.items():
	hex_list[key] = value

hex_list = hex_list[start_addr:end_addr]

#print (hex_list[0])
#print (hex(len(hex_list)))
ser = serial.Serial('COM4', 115200, timeout = 1)
while (get_byte() != 123):
	continue

ser.write(b'3\r\n')
#ser.write(10)
#while (1):
#	print(get_byte())
#print (ser.read())
for byte in hex_list:
	ser.write(byte)

