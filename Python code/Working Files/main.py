import numpy as np 
import cv2 as cv
import time
import tkinter as tk
from adafruit_servokit import ServoKit

#Initialize the three servos
kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(500, 2500) #Very important
kit.servo[1].set_pulse_width_range(500, 2500) #Very important
kit.servo[2].set_pulse_width_range(500, 2500) #Very important

#Center coordinates
center_x = 250
center_y = 250  

prevX = 0
prevY = 0 

#Min and max angles of each motor
min_motor = np.array([85,85,77])
max_motor = np.array([175,175,165])

#Sets the angle of the motors to the chosen values
def set_angle():
    global initial_angle
    initial_angle[0] = e0.get()
    initial_angle[1] = e1.get()
    initial_angle[2] = e2.get()
    initialPos()

initial_angle = np.array([124,120,111]) #These depend on how level the surface is

#Initial position of the motors
def initialPos():
    kit.servo[0].angle = int(e0.get())
    kit.servo[1].angle = int(e1.get())
    kit.servo[2].angle = int(e2.get())

#Initialize the capture 
cap = cv.VideoCapture(0)

#Moves the motor according to the instructions given by the PID
def move_motors(pid):
    for i in np.arange(0,3,1):
        angle = initial_angle[i] - (pid[i]/150.0 * 20)
        angle = np.floor(angle)
        if angle > max_motor[i]:
            print("angle limit high")
            angle = max_motor[i]
        if angle < min_motor[i]:
            print("angle limit low")
            angle = min_motor[i]
        kit.servo[i].angle = angle 

#Finds the ball's center
def ballFinder():
    ret, frame = cap.read()
    result = (0,0)
    if ret:
        frame = frame[:, 93:550, :]
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lower_value = np.array([00,80,100])
        higher_value = np.array([50,255,255])
        mask = cv.inRange(hsv, lower_value, higher_value)
        mask = cv.blur(mask,(6,6))                        
        mask = cv.erode(mask, None, iterations=2)         
        mask = cv.dilate(mask, None, iterations=2)        
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 1500:
                (x,y), radius = cv.minEnclosingCircle(cnt)
                x = int(x)
                y = 480 - int(y) #In images, y=0 is on top, not on the bottom 
                radius = int(radius)
                if radius > 20:
                    result = (x,y)
    return(result)

#This allows you to properly place the plate beneath the camera
def CenterCalibration():
    for i in np.arange(0,150):
        ret, frame = cap.read()
        if ret:
            frame = frame[:,93:550,:]
            cv.circle(frame,(center_x,center_y),3,[255,130,130],3)
            cv.imshow("Calibration", frame)
            cv.waitKey(1)
        else:
            pass
           
        
started = False
#GUI and looping
def start_program():
    global started,i
    set_values()
    reset_values()
    if started:
        start["text"] = "Start"
        started = False
    else:
        start["text"] = "Stop"
        started = True

errors = np.zeros(3)
pasterrors = np.zeros(3)
#Calculates the error for each line of action of each servo
def get_errors(x,y):
    global center_x, center_y, errors, pasterrors
    print(center_x, center_y)
    pasterrors = np.copy(errors)
    errors[0] = y - center_y  
    errors[1] = (np.sqrt(3)*(x-center_x) + (center_y - y))/2.0
    errors[2] = (np.sqrt(3)*(center_x - x) + (center_y - y))/2.
    #errors[abs(errors - pasterrors) > 40] = pasterrors[abs(errors - pasterrors) > 40]
    #print(abs(errors - pasterrors))
    print(errors[0])

lastderivative = 0 
lastlastderivative = 0 
integral = 0
derivative = 0 
pid = 0
#Calculates the PID and normalizes it based on computation time 
def pid_control(compT):
    global errors, pasterrors
    global lastderivative, lastlastderivative, derivative
    global Kp, Ki, Kd, Ka
    global integral
    global lastpid
    global computation_time
    #output == Kp * error + Ki * integral + Kd * derivative 
    global pid 
    if np.all(errors > 10):
        integral += errors * 0.5 
    else:
        integral = 0
    Kd_normalized = np.copy(Kd) / (compT/np.mean(computation_time))
    Ka_normalized = np.copy(Ka) * (compT/np.mean(computation_time))
    Kp_normalized = np.copy(Kp)
    Kp_normalized[np.abs(errors) < 20] = Kp_normalized[np.abs(errors) < 20] * 0.5
    lastlastderivative = np.copy(lastderivative)
    lastderivative = np.copy(derivative)
    derivative = (errors - pasterrors)
    #print("SUM",np.sum(abs(derivative)))
    if np.sum(abs(derivative)) < 10:
        Kd_normalized = Kd_normalized *  (0.1*(np.sum(abs(derivative))))
    elif np.sum(abs(derivative)) > 20:
        Kd_normalized = Kd_normalized * 1.1
    if np.all(np.abs(derivative) < 0.5):
        Kd_normalized = 0
    lastpid = pid
    pid = Kp_normalized * errors + Ki * integral  + Kd_normalized * (((5*derivative) + (3*lastderivative) + (2*lastlastderivative))/10.0) + Ka_normalized * (((derivative - lastderivative) + (lastderivative - lastlastderivative))/2.0) 
    return pid 


Kp = np.zeros(3)
Kd = np.zeros(3)
Ka = np.zeros(3)
Ki = np.zeros(3)

#Resets the values, if the ball leaves the plate
def reset_values():
    global derivative, lastderivative, lastlastderivative, i
    global pasterrors, errors, pid
    derivative = 0
    lastderivative = 0
    lastlastderivative = 0 
    pid = 0
    i = 0
    print('successfully reset all values')

#sets the coefficients 
def set_values():
    global Kp, Ki, Kd, Ka
    i = int(motor_selector.get())
    if i == 3:  
        Kp[:] = p_slider.get() / 10.0 
        Ki[:] = i_slider.get() / 10.0
        Kd[:] = d_slider.get()
        Ka[:] = a_slider.get()
    else:
        Kp[i] = p_slider.get() / 10.0 
        Ki[i] = i_slider.get() / 10.0
        Kd[i] = d_slider.get()
        Ka[i] = a_slider.get()
    print(Kp, Ki, Kd, Ka)

t = 0 
#Refreshes the 'ball position' graph
def refresh(x, y):
    global t
    graphWindow.deiconify()
    graphCanvas.create_oval(x,y,x,y, fill="#b20000", width=4)
    graphCanvas.create_line(0,240,480,240, fill="#0069b5", width=2)
    graphCanvas.create_line(240,0,240,480, fill="#0069b5", width=2)
    if t >= 480:
        t = 0
        graphCanvas.delete("all")
        graphCanvas.create_line(0,240,480,240, fill="#0069b5", width=2)
        graphCanvas.create_line(240,0,240,480, fill="#0069b5", width=2)
        graphCanvas.create_oval(x,y,x+1,y, fill="#b20000")
    t += 4

computation_time = []
#Main loop
def main():
    global prevX, prevY, computation_time
    global center_x, center_y
    start_time = time.time()
    if started:
        x, y = ballFinder()
        #print("posiiton", x,y)
        if x != y != 0: 
            detected = True
            get_errors(x,y)
            compT = (time.time() - start_time)
            computation_time.append(compT)
            pid = pid_control(compT)  
            #print(pid[0])
            #print(compT / np.mean(computation_time))

            move_motors(pid) 
            prevX = x
            prevY = y
        else:
            initialPos()        
        refresh(x, y)
    lmain.after(1,main)

    
#GUI PART 

window = tk.Tk()
window.geometry("820x500")
window.title("Test ?")

p_slider = tk.Scale(window,  from_=0, to=15, orient="horizontal", label="Proportionnal", length=500, tickinterval=2.5, resolution=0.1)
p_slider.set(3)
p_slider.pack()
i_slider = tk.Scale(window,  from_=0, to=1, orient="horizontal", label="Integral", length=500, tickinterval=0.25, resolution=0.005)
i_slider.set(0)
i_slider.pack()
d_slider = tk.Scale(window,  from_=0, to=10, orient="horizontal", label="Derivative", length=500, tickinterval=10, resolution=0.1)
d_slider.set(6.2)
d_slider.pack()
a_slider = tk.Scale(window,  from_=0, to=10, orient="horizontal", label="Double Derivative", length=500, tickinterval=10, resolution=0.1)
a_slider.set(5.6)
a_slider.pack()

p_slider.place(x=250, y= 0)
i_slider.place(x=250, y= 100)
d_slider.place(x=250, y= 200)
a_slider.place(x=250, y= 300)

tk.Label(window, text="Motor0").place(x=00,y=20)
tk.Label(window, text="Motor1").place(x=00,y=50)
tk.Label(window, text="Motor2").place(x=00,y=80)


lmain = tk.Label(window)
lmain.pack()

graphWindow = tk.Toplevel(window)
graphWindow.title('Ball position')
graphCanvas = tk.Canvas(graphWindow,width=480,height=480)
graphCanvas.pack()

motor0 = tk.StringVar(window)
motor1 = tk.StringVar(window)
motor2 = tk.StringVar(window)
motor_select = tk.StringVar(window)
motor0.set(str(initial_angle[0]))
motor1.set(str(initial_angle[1]))
motor2.set(str(initial_angle[2]))
motor_select.set("3")

e0 = tk.Spinbox(window, from_=100, to=200, command=set_angle, width=3, textvariable=motor0)
e1 = tk.Spinbox(window, from_=100, to=200, command=set_angle, width=3, textvariable=motor1)
e2 = tk.Spinbox(window, from_=100, to=200, command=set_angle, width=3, textvariable=motor2)

motor_selector = tk.Spinbox(window, from_=0, to=3, width=2, textvariable=motor_select)

motor_selector.place(x=100, y= 100)

e0.place(x=50, y=20)
e1.place(x=50, y = 50 )
e2.place(x=50, y = 80)



start = tk.Button(window, text="Start" ,command=start_program)
start.place(x=20, y=225)
Breset = tk.Button(window, text="Reset", command=reset_values)
Breset.place(x=20, y= 250)
set = tk.Button(window, text='set values', command=set_values)
set.place(x=400, y = 350 )
    
#END OF GUI

#Final order of operations : 
initialPos()
CenterCalibration()
cv.destroyAllWindows()
main()
window.mainloop()

