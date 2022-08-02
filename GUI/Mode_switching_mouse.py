#Import tkinter library
from tkinter import *
import socket
import keyboard

#Create an instance of tkinter frame
win= Tk()
win.geometry("1750x1450")
mode=0
prev=-1
num_mode=4

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

def disable_control(event=None):
    text.tag_add("disable", "9.1", "9.100")
    text.tag_config("disable", background="red", foreground="white")

def enable_control(event=None):
    text.tag_remove("disable","9.1","9.100")

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

#Create a Button to highlight text
button1=Button(win, text= "Double click to switch Mode\n\nHold the button to disable \nhead movement control for the \nrobot temporarily", height=10, width=30)#.pack(pady=20)
button1.pack(pady=20)
button1.bind('<Double-1>', add_highlighter)
button1.bind('<Button-1>', disable_control)
button1.bind('<ButtonRelease-1>',enable_control)
win.mainloop()
