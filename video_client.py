import socket

HOST = '192.168.0.108' #'163.18.57.213'
PORT = 20
ADDR = (HOST,PORT)
BUFSIZE = 4096
videofile = "original.avi"

bytes = open(videofile).read()

print len(bytes)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

client.send(bytes)

client.close()
