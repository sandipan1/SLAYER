import RPi.GPIO as GPIO 
from multiprocessing import Process,Value,Array,Event
from math import sin,cos
from datetime import datetime
from time import sleep

encoder_pins_for_cm=5
dis_bet_wheels=30
cm=29.5818/6

event_l=Event()
event_r=Event()

def encoder_update(delta_time,speed,encoder_pin,motor_pin,event):
	try:
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(encoder_pin,GPIO.IN)
		GPIO.setup(motor_pin,GPIO.IN)
		global event_r,event_l
		while True:
			# sleep(0.00001)
			# print lock
			pins_crossed=0
			start=datetime.now()
			while( (not event_r.is_set()) or  (not event_l.is_set())):
				if(GPIO.input(motor_pin)==1):
					direction=1
				else:
					direction=-1
				# direction = 1
				if(GPIO.input(encoder_pin)==1):
					while((GPIO.input(encoder_pin)==1) and ((not event_r.is_set()) or  (not event_l.is_set()))):
						continue
					pins_crossed+=1*direction
					# print "pins",
					print pins_crossed
				if((abs(pins_crossed)>=encoder_pins_for_cm)):
					event.set()
			while( event_r.is_set() and  event_l.is_set()):
				# print "not here"
				# print thread_sync.value
				continue
			end=datetime.now()
			delta_time.value=((start-end).total_seconds())
			# print delta_time.value
			speed.value=(pins_crossed/abs(pins_crossed))*cm/delta_time.value
			# print "speed.value"
	except Exception as e:
		print e


def coordinate_update(left_motor_pin,right_motor_pin,left_encoder_pin,right_encoder_pin,current_coordinate,cur_angle):
	try:	
		delta_time=Value('d',0.0)
		speed_left=Value('d',0.0)
		speed_right=Value('d',0.0)
		global event_l,event_r
		# event_l=Event()
		# event_r=Event()
		event_r.clear()
		event_l.clear()

		left_encoder_process=Process(target=encoder_update,args=(delta_time,speed_left,left_encoder_pin,left_motor_pin,event_l))
		right_encoder_process=Process(target=encoder_update,args=(delta_time,speed_right,right_encoder_pin,right_motor_pin,event_r))
		left_encoder_process.start()
		right_encoder_process.start()
		current_coordinate[0]=0
		current_coordinate[1]=0
		while True:
			sleep(0.00001)
			# print "stuck"
			event_r.wait()
			event_l.wait()
			sleep(0.0001)
			event_r.clear()
			event_l.clear()
			speed=(speed_left.value+speed_right.value)/2
			omega=(float)(speed_right.value-speed_left.value)/dis_bet_wheels
			# print speed
			if(omega):
				current_coordinate[0]=float(current_coordinate[0])+(speed/float(omega))*(sin(float(cur_angle.value)+float(omega*delta_time.value))-sin(cur_angle.value))
				current_coordinate[1]=float(current_coordinate[1])+(speed/float(omega))*(-cos(float(cur_angle.value)+float(omega*delta_time.value))+cos(cur_angle.value))
			cur_angle.value=cur_angle.value+omega*delta_time.value
			#print current_coordinate, cur_angle.value
			# print "clear"
	except Exception as e:
		print e

