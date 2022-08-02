# Import the required library
from tkinter import *
import keyboard

# Create an instance of tkinter frame
win=Tk()

# Set the geometry
win.geometry("700x350")

# Add a text widget
text=Text(win, width=80, height=15, font=('Calibri 12'))

# Set default text for text widget
text.insert(INSERT, "Tkinter is a Python Library to create GUI-based applications.")
text.insert(END, "Learning Tkinter is Awesome!!")

# Select Text by adding tags
text.tag_add("start", "1.0","1.7")
text.tag_configure("start", background="OliveDrab1", foreground="black")
text.pack()

win.mainloop()
while True:
    if keyboard.is_pressed(" "):
        print("You pressed space.")
        break

