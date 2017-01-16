import RPi.GPIO as GPIO
from time import sleep


V = 27

graph = [[0,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[15,0,70,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,70,0,0,0,0],
		[0,70,0,145,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,145,0,145,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,145,0,70,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,70,0,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,70,0,0,0,0],
		[0,0,0,0,0,15,0,55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,55,0],
		[0,0,0,0,0,0,55,0,55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,55,0,15,0,0,0,0,0,0,0,0,0,0,0,0,0,55,0,0,0],
		[0,0,0,0,0,0,0,0,15,0,35,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,35],
		[0,0,0,0,0,0,0,0,0,35,0,70,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,70,0,70,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,70,0,35,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,35,0,15,0,0,0,0,0,0,0,0,0,0,0,35],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,15,0,15,0,0,0,0,15,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,0,15,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,0,15,0,0,0,15,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,0,35,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,35,0,15,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,0,15,15,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,0,0,0,0,15,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,0,0,15,0,0,0,0,0,0,0],
		[0,70,0,0,0,70,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,115,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,115,0,115,0],
		[0,0,0,0,0,0,55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,115,0,0],
		[0,0,0,0,0,0,0,0,0,35,0,0,0,35,0,0,0,0,0,0,0,0,0,0,0,0,0]]

angle = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,-90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0],
		[0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0],
		[0,0,0,0,0,0,0,-90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0],
		[0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,-90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90],
		[0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,180],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,-90,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-90,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,-90,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-90,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-90,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,90,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0,0,0,0],
		[0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0,0],
		[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,90,0],
		[0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]


sensPins = [21, 20, 16, 12, 13, 6, 5]
sensPins.reverse()
lmotor1 = 18
lmotor2 = 23
rmotor1 = 25
rmotor2 = 24


servo_l = 23
servo_m = 24
servo_r = 25

encoder_l = 8
encoder_r = 7

K = 8
dspeed = 50

pins_for_360_turn = 100 #find tihs value
pins_for_30_cm = 32   #find tihs value
cm_30 = 30

#Setup

GPIO.setmode(GPIO.BCM)
for i in range(7):
	GPIO.setup(sensPins[i],GPIO.IN)
GPIO.setup(lmotor1,GPIO.OUT)
GPIO.setup(lmotor2,GPIO.OUT)
GPIO.setup(rmotor1,GPIO.OUT)
GPIO.setup(rmotor2,GPIO.OUT)

GPIO.setup(servo_l,GPIO.OUT)
GPIO.setup(servo_m,GPIO.OUT)
GPIO.setup(servo_r,GPIO.OUT)

GPIO.setup(encoder_l,GPIO.IN)
GPIO.setup(encoder_r,GPIO.IN)

pwm_l_motor_1 = GPIO.PWM(lmotor1,100)
pwm_l_motor_1.start(0)
pwm_l_motor_2 = GPIO.PWM(lmotor2,100)
pwm_l_motor_2.start(0)
pwm_r_motor_1 = GPIO.PWM(rmotor1,100)
pwm_r_motor_1.start(0)
pwm_r_motor_2 = GPIO.PWM(rmotor2,100)
pwm_r_motor_2.start(0)

#pwm_l_servo = GPIO.PWM(servo_l,100)
#pwm_l_servo.start(25)
#pwm_m_servo = GPIO.PWM(servo_m,100)
#pwm_m_servo.start(15)
#pwm_r_servo = GPIO.PWM(servo_r,100)
#pwm_r_servo.start(5)


# def servo_set_angle(servo,angle):
# 	duty = 5 + angle/9
# 	servo.ChangeDutyCycle(duty)


def minDistance(dist,sptSet):
	min = float('inf')
	min_index = -1

	for v in range (V):
		if (sptSet[v] == False and dist[v]<min):
			min = dist[v]
			min_index = v

	return min_index

def findPath(dist, prev, path, n, des):
	temp = des
	i = 0
	path_rev = [None]*V
	print("\nPath\n")

	while(temp != -1):
		path_rev[i] = temp
		temp = prev[temp]
		i = i + 1

	for j in range(i-1,-1,-1):
		path[j] = path_rev[i-j-1]
	path[i] = -1

	print path

def dijkstra(path,src,des):
	dist = [None]*V
	prev = [None]*V
	sptSet = [False]*V

	for i in range(V):
		dist[i] = float('inf')
		sptSet[i] = False
		prev[i] = -1
	dist[src] = 0

	for count in range(V-1):
		u = minDistance(dist, sptSet)
		sptSet[u] = True
		for v in range(V):
			if( (sptSet[v]==False) and (graph[u][v]!=0) and (dist[u]!=float('inf')) and (dist[u]+graph[u][v]<dist[v])):
				dist[v] = dist[u] + graph[u][v]
				prev[v] = u

	findPath(dist, prev, path, V, des)

def set_state(readings):
	for i in range(7):
		readings[i] = 1 - GPIO.input(sensPins[i])
	# print readings

def start_pid():
	readings = [None]*7
	exit_pid = False
	while True:
		sleep(0.015)
		set_state(readings)
		som = 0
		pid = 0
		for i in range(7):
			pid+=readings[i]*(4-i)
			som+=readings[i]
		if(som>=4):
			pwm_r_motor_1.ChangeDutyCycle(0)
			pwm_r_motor_2.ChangeDutyCycle(0)
			pwm_l_motor_1.ChangeDutyCycle(0)
			pwm_l_motor_2.ChangeDutyCycle(0)
			break
		pid*=K
		#print pid
		#print som
		if(pid<=50 and pid>=-50):
			pwm_r_motor_1.ChangeDutyCycle(dspeed + pid)
			pwm_r_motor_2.ChangeDutyCycle(0)
			pwm_l_motor_1.ChangeDutyCycle(dspeed - pid)
			pwm_l_motor_2.ChangeDutyCycle(0)
		elif(pid>50):
			pwm_r_motor_1.ChangeDutyCycle(dspeed + dspeed)
			pwm_r_motor_2.ChangeDutyCycle(0)
			pwm_l_motor_1.ChangeDutyCycle(dspeed - dspeed)
			pwm_l_motor_2.ChangeDutyCycle(0)
		elif(pid<-50):
			pwm_r_motor_1.ChangeDutyCycle(dspeed - dspeed)
			pwm_r_motor_2.ChangeDutyCycle(0)
			pwm_l_motor_1.ChangeDutyCycle(dspeed + dspeed)
			pwm_l_motor_2.ChangeDutyCycle(0)

'''start'''
ang=[[-1 for _ in range(25)] for _ in range(25)]
gr = [[0 for _ in range(25)] for _ in range(25)]
def start_pid2():
    readings = [None]*7
    exit_pid = False
    pins_crossed=0
    while True:
        sleep(0.015)
        set_state(readings)
        som = 0
        pid = 0
        for i in range(7):
            pid+=readings[i]*(4-i)
            som+=readings[i]
        if(som>=4):
            pwm_r_motor_1.ChangeDutyCycle(0)
            pwm_r_motor_2.ChangeDutyCycle(0)
            pwm_l_motor_1.ChangeDutyCycle(0)
            pwm_l_motor_2.ChangeDutyCycle(0)
            break
        pid*=K
        #print pid
        #print som
        if(GPIO.input(encoder_l)==1):
            while((GPIO.input(encoder_l)==1)):
                continue
            pins_crossed+=1
        if(pid<=50 and pid>=-50):
            pwm_r_motor_1.ChangeDutyCycle(dspeed + pid)
            pwm_r_motor_2.ChangeDutyCycle(0)
            pwm_l_motor_1.ChangeDutyCycle(dspeed - pid)
            pwm_l_motor_2.ChangeDutyCycle(0)
        elif(pid>50):
            pwm_r_motor_1.ChangeDutyCycle(dspeed + dspeed)
            pwm_r_motor_2.ChangeDutyCycle(0)
            pwm_l_motor_1.ChangeDutyCycle(dspeed - dspeed)
            pwm_l_motor_2.ChangeDutyCycle(0)
        elif(pid<-50):
            pwm_r_motor_1.ChangeDutyCycle(dspeed - dspeed)
            pwm_r_motor_2.ChangeDutyCycle(0)
            pwm_l_motor_1.ChangeDutyCycle(dspeed + dspeed)
            pwm_l_motor_2.ChangeDutyCycle(0)
    return pins_crossed/27.1

def online():
    a=[0 for i in range(7)]
    set_state(a)
    som=0
    for i in a:
        if i ==1:
            som+=1
    if(som<4):
        return True
    else:
        return False

def graphCons(nodeId,visited=[]):
    if(nodeId not in visited):
        visited.append(nodeId)
        k=0
        while(k<360):
            nodeId1=0
            turn_by(15)
            while not online():
                turn_by(1)
                k+=1
            nodeId1=nodeId+1
            ang[nodeId][nodeId1]=k
            ang[nodeId1][nodeId]=k
        visited.append(nodeId)
        if(nodeId1==nodeId):
            return
        for i in range(len(ang[nodeId])):
            if(ang[nodeId][i]!=-1):
                turn_by(ang[nodeId])
                d = start_pid2()
                gr[nodeId][i]=d
                graphCons(i,visited)
    else:
        turn_by(180)
        start_pid2()
'''end'''
def turn_by(angle):
	if(angle>=0):
		pins_crossed = 0
		pwm_r_motor_1.ChangeDutyCycle(dspeed)
		pwm_r_motor_2.ChangeDutyCycle(0)
		pwm_l_motor_1.ChangeDutyCycle(0)
		pwm_l_motor_2.ChangeDutyCycle(dspeed)
		while(pins_crossed<(angle*pins_for_360_turn)/360):
			val = GPIO.input(encoder_l)
			if(val==1):
				while(GPIO.input(encoder_l)==1):
					continue
				pins_crossed+=1
		pwm_r_motor_1.ChangeDutyCycle(0)
		pwm_r_motor_2.ChangeDutyCycle(0)
		pwm_l_motor_1.ChangeDutyCycle(0)
		pwm_l_motor_2.ChangeDutyCycle(0)
	else:
		pins_crossed = 0
		pwm_r_motor_1.ChangeDutyCycle(0)
		pwm_r_motor_2.ChangeDutyCycle(dspeed)
		pwm_l_motor_1.ChangeDutyCycle(dspeed)
		pwm_l_motor_2.ChangeDutyCycle(0)
		while(pins_crossed<((-angle)*pins_for_360_turn)/360):
			val = GPIO.input(encoder_l)
			if(val==1):
				while(GPIO.input(encoder_l)==1):
					continue
				pins_crossed+=1
		pwm_r_motor_1.ChangeDutyCycle(0)
		pwm_r_motor_2.ChangeDutyCycle(0)
		pwm_l_motor_1.ChangeDutyCycle(0)
		pwm_l_motor_2.ChangeDutyCycle(0)

def move_ahead_by(distance):
	pins_crossed = 0
	pwm_r_motor_1.ChangeDutyCycle(dspeed)
	pwm_r_motor_2.ChangeDutyCycle(0)
	pwm_l_motor_1.ChangeDutyCycle(dspeed)
	pwm_l_motor_2.ChangeDutyCycle(0)
	while(pins_crossed<((distance*pins_for_30_cm)/cm_30)):
		val = GPIO.input(encoder_l)
		if(val==1):
			while(GPIO.input(encoder_l)==1):
				continue
			# print pins_crossed
			pins_crossed+=1
	pwm_r_motor_1.ChangeDutyCycle(0)
	pwm_r_motor_2.ChangeDutyCycle(0)
	pwm_l_motor_1.ChangeDutyCycle(0)
	pwm_l_motor_2.ChangeDutyCycle(0)
	# sleep(0.5)

# def pick_obj(pick):
# 	if(pick):
# 		servo_set_angle(pwm_l_servo,90)
# 		servo_set_angle(pwm_r_servo,90)
# 		sleep(0.5)
# 		servo_set_angle(pwm_m_servo,180)
# 		sleep(0.5)
# 		servo_set_angle(pwm_l_servo,180)
# 		servo_set_angle(pwm_r_servo,0)
# 		sleep(0.5)
# 		servo_set_angle(pwm_m_servo,90)
# 		sleep(0.5)
# 	else:
# 		servo_set_angle(pwm_m_servo,180)
# 		sleep(0.5)
# 		servo_set_angle(pwm_l_servo,90)
# 		servo_set_angle(pwm_r_servo,90)
# 		sleep(0.5)
# 		servo_set_angle(pwm_m_servo,90)
# 		sleep(0.5)
# 		servo_set_angle(pwm_l_servo,180)
# 		servo_set_angle(pwm_r_servo,0)
# 		sleep(0.5)

def reach(start,end,pick):
	path = [None]*(V+1)
	dijkstra(path,start,end)
	i=0
	while(path[i]!=-1):
		if (i==0):
			print("reached ",path[i])
			#move_ahead_by(10)
			turn_by(0)
			start_pid()
		elif(path[i+1]==-1):
			print("reached ",path[i])
			#pick_obj(pick)
			move_ahead_by(8)
			turn_by(180)
		else:
			print("reached ",path[i])
			temp_angle=180-(angle[path[i]][path[i+1]] - angle[path[i]][path[i-1]])
			temp_angle=temp_angle%360
			if(temp_angle>=180):
				temp_angle-=360
			move_ahead_by(12)
			turn_by(temp_angle)
			start_pid()
		i+=1

#graphCons(0)
reach(2,10,True)
# reach(25,0,False)
# servo_set_angle(pwm_r_servo,0)
# servo_set_angle(pwm_m_servo,180)
# servo_set_angle(pwm_l_servo,180)
# sleep(2)
# pick_obj(True)
# move_ahead_by(30)
# turn_by(90)
# sleep(2)
# turn_by(90)
#turn_by(-90)
pwm_l_motor_1.stop()
pwm_l_motor_2.stop()
pwm_r_motor_1.stop()
pwm_r_motor_2.stop()
GPIO.cleanup()
'''
def isnode():
	for i in sensPins:
		s+=GPIO.input(i)

	if s>3:
		[0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,180,0,0,0,0],
		return True


def check_for_edges():
	p1= Process(target=turn_by(360))                            
	p1.start()
	p2=Process (target=isline())                           isline
	p2.start()
		

