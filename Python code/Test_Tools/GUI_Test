from webbrowser import get
from adafruit_servokit import ServoKit
import tkinter  as tk
from tkinter import ttk

# root window
root = tk.Tk()
root.geometry('300x200')
root.resizable(False, False)
root.title('Slider Demo')


root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=3)


# slider current value
current_value = tk.DoubleVar()


def get_current_value():
    return '{: .2f}'.format(current_value.get())


def slider_changed(event):
    value_label.configure(text=get_current_value())


# label for the slider
slider_label = ttk.Label(
    root,
    text='Slider:'
)

slider_label.grid(
    column=0,
    row=0,
    sticky='w'
)


def set_angle(value):
    angle = slider.get()
    angle = int(angle)
    kit.servo[0].angle = angle


#  slider
slider = ttk.Scale(
    root,
    from_=0,
    to=180,
    orient='horizontal',  # vertical
    command=set_angle,
    variable=current_value
)

slider.grid(
    column=1,
    row=0,
    sticky='we'
)

# current value label
current_value_label = ttk.Label(
    root,
    text='Current Value:'
)

current_value_label.grid(
    row=1,
    columnspan=2,
    sticky='n',
    ipadx=10,
    ipady=10
)

# value label
value_label = ttk.Label(
    root,
    text=get_current_value()
)
value_label.grid(
    row=2,
    columnspan=2,
    sticky='n'
)

kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(500, 2500) #Very important
kit.servo[1].set_pulse_width_range(500, 2500) #Very important


root.mainloop()
