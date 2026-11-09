import tkinter as tk
import roslibpy

ros = roslibpy.Ros(host='192.168.1.72', port=9090)
ros.run()   # connects to rosbridge on the Pi; raises if it doesn't answer

start_stop = roslibpy.Topic(ros, '/start_stop', 'std_msgs/msg/String')
start_stop.advertise()   # announced now, so the first click isn't lost while ROS finds the subscriber

# roslibpy runs these callbacks in its own thread and Tkinter only works from the main one,
# so they just save the text and refresh() shows it.
latest = {'state': 'No state yet', 'sign': '-', 'light': '-'}

def on_state(msg):
    latest['state'] = msg['data']

def on_sign(msg):
    latest['sign'] = msg['data']

def on_light(msg):
    latest['light'] = msg['data']

robot_state = roslibpy.Topic(ros, '/robot_state', 'std_msgs/msg/String')
robot_state.subscribe(on_state)
detections = roslibpy.Topic(ros, '/detections', 'std_msgs/msg/String')
detections.subscribe(on_sign)
traffic_light = roslibpy.Topic(ros, '/traffic_light', 'std_msgs/msg/String')
traffic_light.subscribe(on_light)

def send(command):
    start_stop.publish(roslibpy.Message({'data': command}))

def refresh():
    state_label.config(text=latest['state'])
    info_label.config(text=f"Last sign: {latest['sign']}    Last light: {latest['light']}")
    window.after(200, refresh)

window = tk.Tk()
window.title('Line follower')
state_label = tk.Label(window, font=('Arial', 20))
state_label.pack(padx=20, pady=(15, 5))
info_label = tk.Label(window, font=('Arial', 12))
info_label.pack(padx=20)
tk.Button(window, text='START', bg='green', fg='white', font=('Arial', 24), width=8,
          command=lambda: send('start')).pack(padx=20, pady=10)
tk.Button(window, text='STOP', bg='red', fg='white', font=('Arial', 24), width=8,
          command=lambda: send('stop')).pack(padx=20, pady=10)
refresh()
window.mainloop()

start_stop.unadvertise()
ros.terminate()
