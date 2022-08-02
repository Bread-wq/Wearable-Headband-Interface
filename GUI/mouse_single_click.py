#Import tkinter library
from tkinter import *
import socket
import keyboard
import time

#Create an instance of tkinter frame
win= Tk()
win.geometry("1750x1450")
mode=0
prev=-1
num_mode=4

#set up socket communication
host = '172.26.167.160' #client ip
port = 4005
#server = ('172.26.166.129', 4000) #robot 1082
server = ('172.26.163.219', 4000)  #robot 1075
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# timmer
time_started=0
released=True

def start_timer(event=None):
    global time_started=time.time()
    global released=False


def get_pressed_time():
    global time_started
    if (time_started==0):
        print("Error, timer has not been started")
        return -1
    else:
        time_pressed=time.time()-time_started
        time_started=0
        return time_pressed

def connect_socket(event=None):
    global s
    s.bind((host,port))
    message="Client Connected"
    s.sendto(message.encode('utf-8'), server)

def send_msg(message):
    global server, s
    s.sendto(message.encode('utf-8'), server)
    data, addr = s.recvfrom(1024)
    data = data.decode('utf-8')
    print("Received from server: " + data)
            
    while (data!=message):
        data, addr = s.recvfrom(1024)
        data = data.decode('utf-8')
        s.sendto(message.encode('utf-8'),server)
        print("Received from server: " + data)

def disconnect_socket(event=None):
    global s
    message="Client Disconnected"
    s.sendto(message.encode('utf-8'), server)
    s.close()

#Define a function to highlight the text
def add_highlighter(event=None):
    global mode,prev
    send_msg("Switch")
    if prev==0:
        text.tag_remove("Mode1", "2.1","2.50")
    elif prev==1:
        text.tag_remove("Mode2", "3.1","3.50")
    elif prev==2:
        text.tag_remove("Mode3", "4.1","4.50")
    elif prev==3:
        text.tag_remove("Mode4", "5.1","5.50")

    if mode==0:
        text.tag_add("Mode1", "2.1","2.50")
        text.tag_config("Mode1", background= "black", foreground= "white")
    elif mode==1:
        text.tag_add("Mode2", "3.1","3.50")
        text.tag_config("Mode2", background= "black", foreground= "white")
    elif mode==2:
        text.tag_add("Mode3", "4.1","4.50")
        text.tag_config("Mode3", background= "black", foreground= "white")
    elif mode==3:
        text.tag_add("Mode4", "5.1","5.50")
        text.tag_config("Mode4", background= "black", foreground= "white")
    prev=mode
    mode=(mode+1)%num_mode

def disable_control():
    send_msg("disable")
    text.tag_add("disable", "9.1", "9.100")
    text.tag_config("disable", background="red", foreground="white")

def enable_control():
    send_msg("enable")
    text.tag_remove("disable","9.1","9.100")

def decide_action(event=None):
    time_pressed=get_pressed_time()
    if (time_pressed<1):
        add_highligher()
    else:
        enable_control()


#Create a Tex Field
text= Text(win);
text.insert(INSERT, "Current mode:\n\
        Mode 1: Base movement\n\
        Mode 2: Arm movement\n\
        Mode 3: Wrist movement\n\
        Mode 4: Gripper movement\n\
        \n\n\n\
        Head movement control for the robot is temporarily disabled\n")
text.pack()
win.bind("<space>",add_highlighter)

# Create a Button to highlight text
button1=Button(win, text= "Double click to switch Mode\n\nHold the button to disable \nhead movement control for the \nrobot temporarily", height=10, width=30)#.pack(pady=20)
button1.pack(pady=20)
# button1.bind('<Double-1>', add_highlighter)
button1.bind('<Button-1>', start_timer)
button1.bind('<ButtonRelease-1>', enable_control)

#Create a button to connect socket
button2=Button(win, text="connect socket", command=connect_socket).pack(pady=20)

#Create a button to disconnect socket
button3=Button(win, text="disconnect socket", command=disconnect_socket).pack(pady=20)

win.mainloop()
