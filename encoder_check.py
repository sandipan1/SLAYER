import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
left_encoder=8
right_encoder=7


GPIO.setup(left_encoder,GPIO.IN)
GPIO.setup(right_encoder,GPIO.IN)

c=0
d=0
def encoder():
		while True:
			l_old=GPIO.input(left_encoder)
			r_old=GPIO.input(right_encoder)
			print (l_old,r_old)
'''while True:

	l_new=GPIO.input(left_encoder)
	r_new=GPIO.input(right_encoder)
	if l_new != l_old and (r_new ==r_old):
		print l_new
	elif l_new == l_old and r_new !=r_old:
		print  r_new
	elif l_new!=l_old and r_old!=r_new:
		c=c+1
		d=d+1
		print c,d
	l_old=l_new
	r_old=r_new'''

sensPins = [21, 20, 16, 12, 13, 6, 5]
for j in sensPins:
	GPIO.setup(j,GPIO.IN)

def sensor(pin):
	while True:
		for i in pin:
			print(GPIO.input(i),end='')
		print()	

sensor(sensPins)	
		

