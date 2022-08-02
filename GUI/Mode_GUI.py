from tkinter import *
 
class Root(Tk):
    def __init__(self):
        super(Root,self).__init__()
        self.title("Stretch Mode")
        self.minsize(500,500)
 
root = Root()
root.mainloop()
