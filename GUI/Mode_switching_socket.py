#Import tkinter library
from tkinter import *
import socket
import keyboard
import threading

#Create an instance of tkinter frame
win= Tk()
win.geometry("1750x1450")
mode=0
prev=-1
num_mode=4

# socket global variables 
server = ('172.26.98.229', 4000)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 
def client_connect():
    global server,s
    host ='172.26.98.229'  #client ip
    port = 4005
    s.bind((host,port))
    message ="Connected "
    s.sendto(message.encode('utf-8'), server)


def client_disconnect():
    global s
    s.close()

def send_msg():
    global s,server
    message=""
    while message !='q':
        s.sendto(message.encode('utf-8'), server)
        #data, addr = s.recvfrom(1024)
        #data = data.decode('utf-8')
        #print("Received from server: " + data)
        while True:
            if keyboard.is_pressed(" "):
                message = "Switch"
                break
            elif keyboard.is_pressed("q"):
                message = 'q'
                break
            else:
                message = "No operation"


#Define a function to highlight the text
def add_highlighter(event=None):
    global mode,prev
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

#Create a Tex Field
text= Text(win);
text.insert(INSERT, "Current mode:\n\
        Mode 1: Base movement\n\
        Mode 2: Arm movement\n\
        Mode 3: Wrist movement\n\
        Mode 4: Gripper movement")
text.pack()
win.bind("<space>",add_highlighter)
win.bind("<space>",threading.Thread(target=send_msg).start())
#Create a Button to highlight text
Button(win, text= "Connect", command= threading.Thread(target=client_connect).start()).pack()
Button(win, text= "Disconnect", command= threading.Thread(target=client_disconnect).start()).pack()

win.mainloop()

