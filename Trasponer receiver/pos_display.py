from threading import Thread
from sys import version_info
import serial
import math
import time


if version_info.major == 2:
	import Tkinter as tk
elif version_info.major == 3:
	import tkinter as tk

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from matplotlib.figure import Figure

PLOT_T_INTERVAL=0.25

##### List of transpoder and its position 
g_t_pos = np.array([0.0,0.0], dtype=float)

##### Curret position of reciver (simulaeted by taking average of the first two data)
g_s_pos = np.array([0.0,0.0], dtype=float)

#### time, number of sensor
g_time =(0,0,0)
g_n_sensor=0


##### true if draw is enable
g_draw=False


def r_decode(inLine):
	list_t = inLine.decode().split(",")

	# print("length: ", len(list_t))

	if len(list_t) == 11:
		t_id=int(list_t[0])
		t_fix=list_t[1]
		t_lat_ns=list_t[2]
		t_lat_d=int(list_t[3])
		t_lat_m=float(list_t[4])
		t_lon_ew=list_t[5]
		t_lon_d=int(list_t[6])
		t_lon_m=float(list_t[7])
		t_time_h=int(list_t[8])
		t_time_m=int(list_t[9])
		t_time_s=float(list_t[10])

		return t_id, t_fix, t_lat_ns, t_lat_d, t_lat_m, t_lon_ew, \
				t_lon_d, t_lon_m, t_time_h, t_time_m, t_time_s
	else:
		return 0, 'N', 'N', 0, 0, 'E', 0, 0, 0, 0, 0





def recevier_monitor():
	global g_t_pos, g_s_pos, g_time, g_draw, g_n_sensor

	ser = serial.Serial(port='/dev/cu.SLAB_USBtoUART',
						baudrate=115200,
						parity=serial.PARITY_NONE,
						stopbits=serial.STOPBITS_ONE,
						bytesize=serial.EIGHTBITS,
						timeout=1)

	print("connected to: " + ser.portstr)

	talking_avg=True
	
	active_sensors=list([])

	while True:
		line = ser.readline()

		t_id, t_fix, t_lat_ns, t_lat_d, t_lat_m, t_lon_ew, t_lon_d, \
		t_lon_m, t_time_h, t_time_m, t_time_s =r_decode(line)

		# print("ID:%d, fix:%s, %c-%d:%.5f, %c-%d:%.5f, %d:%d:%.2f"
		# 	  %(t_id, t_fix, t_lat_ns, t_lat_d, t_lat_m, t_lon_ew, 
		# 	  	t_lon_d, t_lon_m, t_time_h, t_time_m, t_time_s))
		print("ID:%d, fix:%s, %c-%.7f, %c-%.7f, %d:%d:%.2f"
			  %(t_id, t_fix, t_lat_ns, t_lat_d+t_lat_m/60.0, t_lon_ew, 
			  	t_lon_d+t_lon_m/60.0, t_time_h, t_time_m, t_time_s))
		if t_id != 0  and t_fix=="Y":
			newSensor=False

			#fin if sensor is on list
			try:
				result = active_sensors.index(t_id)
			except ValueError:
				newSensor=True

				g_t_pos=np.vstack((g_t_pos,np.array([0.0,0.0], dtype=float)))

				active_sensors.append(t_id)

				print('first time seeing id: ', t_id)
				g_n_sensor=len(active_sensors)
				result=g_n_sensor-1


			# print("active sensor", active_sensors)

			##save time
			g_time=(t_time_h, t_time_m, t_time_s)

			##calculate position in array format
			lat=t_lat_d+t_lat_m/60.0
			# lat=t_lat_m
			if t_lat_ns == "S":
				lat=lat*-1
			lon=t_lon_d+t_lon_m/60.0
			# lon=t_lon_m
			if t_lon_ew == "W":
				lon=lon*-1

			#place Data into global variable
			if g_t_pos.ndim != 1:
				g_t_pos[result+1, 0] = lat
				g_t_pos[result+1, 1] = lon
			else:
				g_t_pos[0] = lat
				g_t_pos[1] = lon

			if newSensor and len(active_sensors)<=2:
				g_s_pos = np.true_divide(np.add(g_s_pos, np.array([lat, lon])),len(active_sensors))

			if len(active_sensors) >=1:
				g_draw=True


		# print("self pos",g_s_pos)

		# print("draw pos",g_t_pos)


	ser.close()



def main():

	#startig recevier thread
	recevier_thread = Thread(target=recevier_monitor, daemon=True)
	recevier_thread.start()


	displayApp=App(tk.Tk())



class App:

	def __init__(self, masterWindow):

		self.masterWindow=masterWindow

		masterWindow.title("Relative position of transponder")

		timeFrame=tk.Frame(self.masterWindow, bd=1)
		timeFrame.pack(side=tk.TOP)
		timeTxtLabel=tk.Label(timeFrame, text="UTC Time:", font=("Courier", 16))
		timeTxtLabel.pack(side=tk.LEFT)
		self.timeLabel=tk.Label(timeFrame, text="0:0:0", font=("Courier", 16))
		self.timeLabel.pack(side=tk.RIGHT)

		sensorFrame=tk.Frame(self.masterWindow, bd=1)
		sensorFrame.pack(side=tk.TOP)
		sensorCntTxtLabel=tk.Label(sensorFrame, text="Transponder Count:", font=("Courier", 16))
		sensorCntTxtLabel.pack(side=tk.LEFT)
		self.sensorCntLabel=tk.Label(sensorFrame, text="0", font=("Courier", 16))
		self.sensorCntLabel.pack(side=tk.RIGHT)

		self.drawFrame=tk.Frame(self.masterWindow, bd=1)
		self.drawFrame.pack(side=tk.TOP)


		self.f = Figure(figsize=(5, 4), dpi=100)
		self.ax = self.f.add_subplot(111)

		v1 = (0, -5)
		v2 = (0, 5)

		soa = np.array([[0, 0, v1[0], v1[1]], [0, 0, v2[0], v2[1]]])
		X, Y, U, V = zip(*soa)

		print(soa)
		print("X", X)
		print("Y", Y)
		print("U", U)
		print("V", V)

		self.ax.quiver(X, Y, U, V, angles='xy', scale_units='xy', scale=1, color='gb')
		# self.ax.set_xlim([35.72034, 35.72038])
		# self.ax.set_ylim([139.7555, 139.7559])		
		self.ax.set_xlim([-5, 5])
		self.ax.set_ylim([-5, 5])

		self.canvas = FigureCanvasTkAgg(self.f, master=self.drawFrame)
		self.canvas.draw()
		self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

		yAxisLabel=tk.Label(self.drawFrame, text="Latitude NS (meter)", \
					font=("Courier", 16), wraplength=1)
		yAxisLabel.pack(side=tk.LEFT)
		xAxisLabel=tk.Label(masterWindow, text="Longitude E/W (meter)", font=("Courier", 16))
		xAxisLabel.pack(side=tk.BOTTOM)


		#initilize loop
		self.masterWindow.after(1000, self.update)


		self.angle=0

		tk.mainloop()


	def update(self):

		global g_t_pos, g_s_pos, g_time, g_draw, g_n_sensor

		self.timeLabel.config(text="%d:%d:%.2f"%(g_time[0],g_time[1],g_time[2]))
		self.sensorCntLabel.config(text="%d"%(g_n_sensor))


		if g_draw:
			print("drawing")
			# self.f = Figure(figsize=(5, 4), dpi=100)
			self.ax.clear()
			self.ax = self.f.add_subplot(111)



			# v1 = (4*math.cos(self.angle), 4*math.sin(self.angle))
			# v2 = (4*math.cos(-self.angle), 4*math.sin(-self.angle))

			# if self.angle>=math.pi*2: 
			# 	self.angle=self.angle-math.pi*2+0.5
			# else:
			# 	self.angle=self.angle+0.2


			# soa = np.array([[0, 0, v1[0], v1[1]], [0, 0, v2[0], v2[1]]])
			# X, Y, U, V = zip(*soa)



			soa=0
			for i in range(1,g_n_sensor+1):
				# print("i:%d"%i)
				mag=111321 
				soa_a = np.array([0, 0, (g_t_pos[i,0]-g_s_pos[0])*mag, (g_t_pos[i,1]-g_s_pos[1])*mag])

				if i==1:
					soa = soa_a
				else:
					soa=np.vstack((soa, soa_a))

			# print("soa",soa)
			if soa.ndim==1:
				X=soa[0]
				Y=soa[1]
				U=soa[2]
				V=soa[3]
			else:
				X, Y, U, V = zip(*soa)
			# print("X", X)
			# print("Y", Y)
			# print("U", U)
			# print("V", V)

			self.ax.quiver(X, Y, U, V, angles='xy', scale_units='xy', scale=1, color='gb')
			# self.ax.set_xlim([35.72034, 35.72038])
			# self.ax.set_ylim([139.7555, 139.7559])
			space=10
			try:
				self.ax.set_xlim([min(np.append(U,X))-space, max(np.append(U,X))+space])
				self.ax.set_ylim([min(np.append(Y,V))-space, max(np.append(Y,V))+space])
			except TypeError:
				self.ax.set_xlim([U-space, U+space])
				self.ax.set_ylim([V-space, V+space])

			# self.canvas = FigureCanvasTkAgg(self.f, master=self.drawFrame)
			self.canvas.draw()
			# self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

			g_draw=False
		
		# print("Plotting update function")

		self.masterWindow.after(int(PLOT_T_INTERVAL*1e3), self.update)






if __name__=='__main__':
	main()