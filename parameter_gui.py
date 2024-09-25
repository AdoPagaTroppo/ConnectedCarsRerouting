import sys
from tkinter import *
class Checkbar(Frame):
   def __init__(self, parent=None, picks=[], side=LEFT, anchor=W):
      Frame.__init__(self, parent)
      self.vars = []
      for pick in picks:
         var = IntVar()
         chk = Checkbutton(self, text=pick, variable=var)
         chk.pack(side=side, anchor=anchor, expand=YES)
         self.vars.append(var)
   def state(self):
      return map((lambda var: var.get()), self.vars)

class Inputbar(Frame):
    def __init__(self, parent=None, picks=[], side=LEFT, anchor=W):
        Frame.__init__(self, parent)
        self.vars = []
        lbl1 = Label(self,text='Number of cars')
        lng = Text(self, height = 1,
                        width = 5,
                        bg = "light yellow")
        lbl2 = Label(self,text='Number of simulation repetitions')
        lng2 = Text(self, height = 1,
                        width = 5,
                        bg = "light yellow")
        lbl1.pack(side=side,anchor=anchor)
        lng.pack(side=side,anchor=anchor)
        lbl2.pack(side=side,anchor=anchor)
        lng2.pack(side=side,anchor=anchor)
        self.vars.append(lng)
        self.vars.append(lng2)
    def state(self):
      return map((lambda var: var.get()), self.vars)

def write_params_on_file(number_of_cars,number_of_repeats,v):
    colorsel = v[0]==1
    colorpaths = v[1]==1
    colorreward = v[2]==1    
    f = open('sim_config.txt','w')
    f.write(str(number_of_cars)+';'+str(number_of_repeats)+';'+str(colorsel)+';'+str(colorpaths)+';'+str(colorreward)+';')
    f.close()

def gui_create_and_run():
   root = Tk()
   simdet = LabelFrame(root,text='Simulation details')
   simdet.grid(row=0,column=0,padx=0,pady=0)
   l1 = Label(simdet,text='Number of cars')
   l1.grid(row=0,column=0,padx=2,pady=10)
   t1 = Entry(simdet,width=20)
   t1.grid(row=0,column=1,padx=2,pady=10)
   l2 = Label(simdet,text='Number of simulation repeats')
   l2.grid(row=1,column=0,padx=2,pady=10)
   t2 = Entry(simdet,width=20)
   t2.grid(row=1,column=1,padx=2,pady=10)
   tgl = Checkbar(simdet, ['Show colour of selected path (suggested for HiL)','Show colours for different paths (suggested for paths analysis)','Show cost map colours (suggested for reward analysis)'],side=TOP)
   tgl.grid(row=2)
   def allstates(): 
      v = list(tgl.state())
      number_of_cars = int(t1.get())
      number_of_repeats = int(t2.get())
      write_params_on_file(number_of_cars,number_of_repeats,v)
      root.destroy()
   def allbreak():
       root.destroy()
       sys.exit(0)
   Button(simdet, text='Start Simulations', command=allstates).grid(row=3,column=0)
   Button(simdet, text='Quit', command=allbreak).grid(row=3,column=1)
   root.mainloop()
   
if __name__ == '__main__':
    gui_create_and_run()