neighb={(id1,id2):-1}
angles={id1:[]}

import RPi.GPIO as GPIO
from time import sleep
import position_layer as pl

lmot1 = 5
lmot2 = 6
rmot1 = 13
rmot2 = 19
IR1_in = 17
IR2_in = 18
IR3_in = 22
IR4_in = 23
IR5_in = 24
IR6_in = 27

GPIO.setup(IR6_in,GPIO.IN)
GPIO.setup(IR5_in,GPIO.IN)
GPIO.setup(IR4_in,GPIO.IN)
GPIO.setup(IR3_in,GPIO.IN)
GPIO.setup(IR2_in,GPIO.IN)
GPIO.setup(IR1_in,GPIO.IN)

GPIO.setup(lmot1,GPIO.OUT)
GPIO.setup(lmot2,GPIO.OUT)
GPIO.setup(rmot1,GPIO.OUT)
GPIO.setup(rmot2,GPIO.OUT)

pwm_l_motor_1 = GPIO.PWM(lmotor1,100)
pwm_l_motor_1.start(0)
pwm_l_motor_2 = GPIO.PWM(lmotor2,100)
pwm_l_motor_2.start(0)
pwm_r_motor_1 = GPIO.PWM(rmotor1,100)
pwm_r_motor_1.start(0)
pwm_r_motor_2 = GPIO.PWM(rmotor2,100)
pwm_r_motor_2.start(0)

encoderl = 20
encoderr = 16
pinsper360 = 30

def start_pid():
    readings = [None]*5
    exit_pid = False
    while True:
        sleep(0.015)
        set_state(readings)
        som = 0
        pid = 0
        for i in range(5):
            pid+=readings[i]*(2-i)
            som+=readings[i]
        if(som>=4):
            pwm_r_motor_1.ChangeDutyCycle(0)
            pwm_r_motor_2.ChangeDutyCycle(0)
            pwm_l_motor_1.ChangeDutyCycle(0)
            pwm_l_motor_2.ChangeDutyCycle(0)
            break
        pid*=K
        if(pid<=50 and pid>=-50):
            pwm_r_motor_1.ChangeDutyCycle(dspeed + pid)
            pwm_r_motor_2.ChangeDutyCycle(0)
            pwm_l_motor_1.ChangeDutyCycle(dspeed + pid)
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

def move(nodeId):
    pinscrossed1=0
    pinscrossed2=0
	while(not isnode()):
		GPIO.output(lmot1,1)
		GPIO.output(lmot2,0)
		GPIO.output(rmot1,1)
		GPIO.output(rmot2,0)
        val1 = GPIO.input(encoderl)
        val2 = GPIO.input(encoderr)
        if(val1==1):
            while(GPIO.input(encoderl)==1):
                continue
            pinscrossed1+=1
        if(val2==1):
            while(GPIO.input(encoderr)==1):
                continue
            pinscrossed2+=1
    dist = (pinscrossed1+pinscrossed2)/2
	idnew=getId()
    if idnew not in angles.keys():  
    	neighb[(nodeId,idnew)]=dist
    	neighb[(idnew,nodeId)]=dist

def isnode():
	in1 = -GPIO.input(IR1_in)
	in2 = -GPIO.input(IR2_in)
	in3 = GPIO.input(IR3_in)
	in4 = GPIO.input(IR4_in)
	in5 = GPIO.input(IR5_in)
	in6 = GPIO.input(IR6_in)
	tot = in6+in5+in4+in3+in2+in1
	if(tot>=5):
		return True
	else:
		return False

def online():
	in1 = GPIO.input(IR1_in)
	in2 = GPIO.input(IR2_in)
	in3 = GPIO.input(IR3_in)
	in4 = GPIO.input(IR4_in)
	in5 = GPIO.input(IR5_in)
	in6 = GPIO.input(IR6_in)
	if(in3 and in4):
		return True
	else:
		return False

def turn(nodeId):
	ang=0
	while(ang<360):
		while(not online()):
			rotate()
		if(ang not in angles[nodeId] and ang!=180):
			angles[nodeId].append(ang)


def rotate(ang=1):
	if(ang>180 and ang<360):
		pinscrossed = 0
		while(pinscrossed<(ang*pinsper360)/360):
			val = GPIO.input(encoderl)
			if(val==1):
				while(GPIO.input(encoderl)==1):
					continue
				pinscrossed+=1

    else:
        pinscrossed = 0
        while(pinscrossed<(ang*pinsper360)/360):
            val = GPIO.input(encoderr)
            if(val==1):
                while(GPIO.input(encoderr)==1):
                    continue
                pinscrossed+=1


def graphCons(nodeId,visited=[]):
	if(nodeId not in visited):
		visited.append(nodeId)
		turn(nodeId)
		visited.append(nodeId)
		if(len(angles[nodeId])==0):
			return
		for(i in range(len(angles[nodeId]))):
			rotate(angles[nodeId][i])
			move(nodeId)
			for(j in neighb.keys()):
				if(j[0]==nodeId):
					graphCons(j[1])
				elif(j[1]==nodeId):
					graphCons(j[0])
    else:
        rotate(180)
        move(nodeId)

class Graph(object):
	def __init__(self):
		self.edgelist={}
		self.nodelist=[]
		self.connected={}
	def add_node(self,a):
		if(a not in self.nodelist):
			self.nodelist.append(a);
	def add_edge(self,a,b,weight=1):
		self.edgelist.update({(a,b):weight})
		self.edgelist.update({(b,a):weight})
		self.add_node(a)
		self.add_node(b)
		if(a not in self.connected.keys()):
			self.connected.update({a:[b]})
		else:
			self.connected[a].append(b)
		if(b not in self.connected.keys()):
			self.connected.update({b:[a]})
		else:
			self.connected[b].append(a)
	def neighbors(self,a):
		t=[]
		for i in self.connected[a]:
			t.append(i)
		return t
	def nodes(self):
		return self.nodelist
	def edges(self):
		return self.edgelist
	def number_of_nodes(self):
		return len(self.nodelist)
	def number_of_edges(self):
		return len(self.edgelist.keys())
	def getConnected(self):
		return self.connected

G=Graph()

for i in neighb.keys():
	G.add_edge(i[0],i[1],neighb[(i[0],i[1])])

def dijkstras(graph,start,target):
	dist={start:0}
	prev={start:None}
	for i in graph.nodes():
		if(i!=start):
			dist[i]=10000000000
			prev[i]=None
	visited=set()
	queue=graph.nodes()[:]
	while queue:
		u=queue[0]
		for i in queue:
			if(dist[i]<dist[u]):
				u=i
		visited.add(u)
		queue.remove(u)
		for i in graph.neighbors(u):
			if(dist[i]>dist[u]+graph[(u,i)]):
				dist[i]=dist[u]+graph[(u,i)]
				prev[i]=u
	path=[]
	t=target
	while t!=start:
		path=[t]+path
		t=prev[t]
	path=[start]+path
	return path

print dijkstras(G,startId,StopId)[0]
