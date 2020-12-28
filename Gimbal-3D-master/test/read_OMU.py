import os
import smbus
import time

bus = smbus.SMBus(1)
address = 0x68

def pr():
    cnt = 10
    for i in range(cnt):
        x = bus.read_word_data(address, i)
        print x

q = raw_input()

while q is not 'q':
    pr()
    q = raw_input()
