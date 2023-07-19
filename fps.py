import cv2
import time
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)
cam = cv2.VideoCapture('/dev/video0')

while True:
    success, im = cam.read()
    tStart=time.time()
    cv2.putText(im,str(int(fps))+' FPS',pos,font,height,myColor,weight)
    cv2.imshow("Camera", im)
    if cv2.waitKey(1)==ord('q'):
        break
    tEnd=time.time()
    loopTime=tEnd-tStart
    fps=.9*fps + .1*(1/loopTime)
    
cam.release()
cv2.destroyAllWindows()
