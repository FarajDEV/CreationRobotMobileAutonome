import numpy as np
import matplotlib.pyplot as plt
import serial


port = 'COM13'
baudrate = 230400
ser = serial.Serial(port, baudrate)


x_data = []
y_data = []
way_points_x = []
way_points_y = []

x_robot = 0
y_robot = 0


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
fig2, waypoint_ax = plt.subplots(1, 1, figsize=(10, 4))

ax1.set_title("Trajectoire de notre robot mobile autonome")
ax1.set_xlabel("Position x du robot")
ax1.set_ylabel("Position y du robot")

waypoint_ax.scatter(x_robot, y_robot, c='r', marker='o')
waypoint_ax.set_title("Calcul des waypoints")


total_steps = 100
total_time = 10


for step in range(total_steps):
    ax1.scatter(x_robot, y_robot, c='r', marker='o')
    plt.pause(total_time / total_steps)

   
    line = ser.readline().decode().strip()
    if line:
        try:
          
            data = list(map(float, line.split(',')))
            if len(data) == 4:
                x_robot, y_robot, waypoint_x, waypoint_y = data
                

                x_data.append(x_robot)
                y_data.append(y_robot)
                way_points_x.append(waypoint_x)
                way_points_y.append(waypoint_y)
                
                ax1.scatter(x_robot, y_robot, c='r', marker='o')
                ax2.scatter(x_data, y_data, c='b', marker='o')
                waypoint_ax.scatter(waypoint_x, waypoint_y, c='b', marker='o')
                plt.pause(0.05)
            else:
                print(f"Ligne de données incorrecte: {line}")
        except ValueError as e:
            print(f"Erreur lors de la conversion des données: {e}")


ser.close()

plt.tight_layout()
plt.show()
