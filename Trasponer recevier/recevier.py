import serial

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

		return t_id, t_fix, t_lat_ns, t_lat_d, t_lat_m, t_lon_ew, t_lon_d, t_lon_m, t_time_h, t_time_m, t_time_s
	else:
		return 0, 'N', 'N', 0, 0, 'E', 0, 0, 0, 0, 0




ser = serial.Serial(port='/dev/cu.SLAB_USBtoUART',
					baudrate=115200,
					parity=serial.PARITY_NONE,
					stopbits=serial.STOPBITS_ONE,
					bytesize=serial.EIGHTBITS,
					timeout=1)

print("connected to: " + ser.portstr)
 

while True:
	line = ser.readline()

	t_id, t_fix, t_lat_ns, t_lat_d, t_lat_m, t_lon_ew, t_lon_d, t_lon_m, t_time_h, t_time_m, t_time_s =r_decode(line)

	print("ID:%d, fix:%c, %c-%d:%.5f, %c-%d:%.5f, %d:%d:%.2f"
		  %(t_id, t_fix, t_lat_ns, t_lat_d, t_lat_m, t_lon_ew, 
		  	t_lon_d, t_lon_m, t_time_h, t_time_m, t_time_s))

ser.close()


