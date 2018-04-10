import socket
import numpy as np
import cv2

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

HOST = '192.168.0.108'#'163.18.57.213'
PORT = 20
ADDR = (HOST,PORT)
BUFSIZE = 4096

serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

serv.bind(ADDR)
serv.listen(5)

print 'listening ...'

recive_flag = True

while (recive_flag):
  conn, addr = serv.accept()
  print 'client connected ... ', addr
  myfile = open('log/testfile.avi', 'w')

  while True:
    data = conn.recv(BUFSIZE)
    if not data: break
    myfile.write(data)
    print 'writing file ....'

  myfile.close()
  print 'finished writing file'
  conn.close()
  print 'client disconnected'

  recive_flag = False

camera = cv2.VideoCapture('log/testfile.avi')

fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('recivied_sobel_video.avi',fourcc, 7.0, (640,480))

cnt_write = 0
keep_write = True

while (keep_write) :
  ret, frame = camera.read()
  #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  sobel = cv2.Sobel(frame,cv2.CV_8U,1,1,ksize=5)
  out.write(sobel)
  cnt_write = cnt_write + 1
  if(cnt_write>30):
    keep_write = False

  
out.release()
camera.release()
cv2.destroyAllWindows()

  
