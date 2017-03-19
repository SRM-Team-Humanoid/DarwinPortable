import random
import sys, getopt
sys.path.append('.')
import pypot.dynamixel
import time
import RTIMU
import os.path
import numpy as np
from pprint import pprint
import xml.etree.cElementTree as ET
from collections import Counter
from copy import deepcopy

class Dxl(object):
    def __init__(self,port_id=0, scan_limit=25, lock=-1):
        # Initializes Dynamixel Object
        # Port ID is zero by default
        ports = pypot.dynamixel.get_available_ports()
        if not ports:
            raise IOError('no port found!')

        print('ports found', ports)
        print('connecting on the first available port:', ports[port_id])
        dxl_io = pypot.dynamixel.DxlIO(ports[port_id])
        ids = dxl_io.scan(range(25))
        print(ids)

        if lock > 0:
            if len(ids) < lock:
                raise RuntimeError("Couldn't detect all motors.")

        self.dxl_io = dxl_io
        self.ids = ids
    def setPos(self,pose):
        '''
        for k in pose.keys():
            if k not in self.ids:
                del pose[k]
        '''
        writ = {key: value for key, value in pose.items() if key in self.ids}
        #print writ
        self.dxl_io.set_goal_position(writ)

    def getPos(self):
        return Motion(1," ".join(map(str,self.dxl_io.get_present_position(self.ids))),0)


class XmlTree(object):

    def __init__(self,str):
        try:
            with open(str) as f:
                pass
            self.tree = ET.ElementTree(file=str)
        except:
            raise RuntimeError("File not found.")

    def parsexml(self,text):
        find = "PageRoot/Page[@name='" + text + "']/steps/step"
        motions = []
        prev_frame = 0
        steps = [x for x in self.tree.findall(find)]
        if len(steps)==0:
            print find
            raise RuntimeError("ParseFail!")
        for step in steps:
            motion = Motion(step.attrib['frame'], step.attrib['pose'], prev_frame)
            prev_frame = step.attrib['frame']
            motions.append(motion)

        return motions

    def superparsexml(self,text,exclude=[],offsets=[]):
        find = "FlowRoot/Flow[@name='"+text+"']/units/unit"
        steps = [x for x in self.tree.findall(find)]
        if len(steps)==0:
            print find
            raise RuntimeError("ParseFail!")
        motionsets = []
        for step in steps:
            motionsets.append(MotionSet(self.parsexml(step.attrib['main']),speed=float(step.attrib['mainSpeed']),exclude=exclude,offsets=offsets))

        return motionsets



class Motion(object):
    def __init__(self,frame,pose,prev_frame):
        self.frame = int(frame)
        self.pose = {}
        self.delay = self.frame-int(prev_frame)
        for i,p in enumerate(pose.split()):
            self.pose[i+1] =float(p)

    def __str__(self):
        return "Frame:"+str(self.frame) + "      Delay:"+str(self.delay) + "     Pose:"+" ".join(map(str,self.pose.values()))

    def updatePose(self,offset,add=True):
        if add:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] += offset[k]
        else:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] -= offset[k]


    def write(self,state, speed,exclude=[],offset={}):
        begpos = state.pose
        endpos = self.pose
        frames = []
        ids = []
        for k in endpos.keys():
            try:
                begpos[k]
            except:
                begpos[k]=0
            if begpos[k]!=endpos[k] and k not in exclude:
                frames.append(np.linspace(begpos[k],endpos[k],self.delay))
                ids.append(k)

        frames = zip(*frames)
        for f in frames:
            writ = dict(zip(ids, f))
            dxl.setPos(writ)
            time.sleep(0.008 / speed)
            print writ



class MotionSet(object):
    def __init__(self,motions,speed=1.0,exclude =[],offsets=[]):
        self.motions = motions
        self.speed = speed
        self.exclude = exclude
        self.offsets = offsets
        self.loaded = False

    def setExclude(self,list):
        self.exclude = list

    def setSpeed(self,speed):
        self.speed = speed

    def execute(self,speed=-1,iter=1):
        global state
        if speed<0:
            speed = self.speed
        if not self.loaded:
            for offset in self.offsets:
                for motion in self.motions:
                    motion.updatePose(offset)
            self.loaded = True

        while iter>0:
            for motion in self.motions:
                motion.write(state,speed,self.exclude)
                state = deepcopy(motion)
            iter-=1


class Action():
    def __init__(self,motionsets):
        self.motionsets=motionsets

    def add(self,motionsets):
        self.motionsets.extend(motionsets)

    def execute(self,iter=1,speed=1):
        while iter>0:
            for motionset in self.motionsets:
                for m in motionset.motions:
                    print m
                orig = motionset.speed
                motionset.speed = motionset.speed*speed
                motionset.execute()
                motionset.speed = orig
            iter -= 1

#IMU INIT =================================------------->
SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")


imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)
return imu


def imu_read(imu):
  if imu.IMURead():
    # x, y, z = imu.getFusionData()
    # print("%f %f %f" % (x,y,z))
    data = imu.getIMUData()
    fusionPose = data["fusionPose"]
    print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), 
        math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
    time.sleep(poll_interval*1.0/1000.0)





#=========================================================================="



if __name__=='__main__':
    darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
    abmath = {11: 15, 12: -15, 13: -10, 14: 10, 15: -5, 16: 5}
    hand = {5: 60, 6: -60}
    offsets = [darwin, abmath, hand]
    offsets = [darwin, hand]
    dxl = Dxl(lock=20)
    tree = XmlTree('data2.xml')
    tree2 = XmlTree('DRIBLE.xml')
    walk = Action(tree.superparsexml("22 F_S_L",offsets=[darwin]))
    balance = MotionSet(tree.parsexml("152 Balance"), offsets=[darwin,hand])
    moon_walk = Action(tree2.superparsexml("11 B_L_S", offsets=[darwin]))
    lback = MotionSet(tree2.parsexml("18 B_L_E"), offsets=[darwin])
    rback = MotionSet(tree2.parsexml("17 B_R_E"), offsets=[darwin])
    state = dxl.getPos()
    print state
    raw_input("Proceed?")
    #balance.execute()
    raw_input("Sure?")
    #balance.execute()
    imu_read(imu)
    


    # #state = dxl.getPos()
    #
    # balance = MotionSet(tree.parsexml("152 Balance"),offsets=offsets)
    # l_attack = MotionSet(tree.parsexml("21 L attack"),speed=1.2,offsets=offsets)
    # kick = MotionSet(tree.parsexml("18 L kick"),speed=2,offsets=offsets)
    # f_getup = MotionSet(tree.parsexml("27 F getup"),speed=2.7,offsets=offsets)
    # b_getup = MotionSet(tree.parsexml("28 B getup  "),speed=1.5,offsets=offsets)
    # r_inv = MotionSet(tree2.parsexml("19 RFT"),speed=1.2,offsets=offsets)
    # l_inv = MotionSet(tree2.parsexml("20 LFT"),speed=1.2,offsets=offsets)
    # r_turn = MotionSet(tree2.parsexml("27 RT"),speed=1.2,offsets=offsets)
    # l_turn = MotionSet(tree2.parsexml("28 LT"),speed=1.2,offsets=offsets)
    #
    # #walk goes in superclass
    #
    # w1 = MotionSet(tree.parsexml("32 F_S_L"),speed=2.1,offsets=offsets)
    # w2 = MotionSet(tree.parsexml("33 "),speed=2.1,offsets=offsets)
    # w3 = MotionSet(tree.parsexml("38 F_M_R"),speed=2.7,offsets=offsets)
    # w4 = MotionSet(tree.parsexml("39 "),speed=2.1,offsets=offsets)
    # w5 = MotionSet(tree.parsexml("36 F_M_L"),speed=2.7,offsets=offsets)
    # w6 = MotionSet(tree.parsexml("37 "),speed=2.1,offsets=offsets)
    #
    # l_step = MotionSet(tree2.parsexml("10 ff_l_r"), speed=1.5, offsets=offsets2)
    # r_step = MotionSet(tree2.parsexml("9 ff_r_l"), speed=1.5, offsets=offsets2)
    # balance2 = MotionSet(tree.parsexml("152 Balance"), offsets=offsets2)
    # boom_walk = Action([l_step,r_step])
    #
    # #print state
    #
    # walk_init = Action([w1,w2])
    # walk_motion = Action([w3,w4,w5,w6])
    #
    # balance2.execute()
    # x = raw_input("Proceed (y/n)?")
    # prev =''
    # if x == 'y':
    #     boom_walk.execute(10,speed=1.4)
    #     #walk_init.execute(speed=2)
    #     #walk_motion.execute(10,speed=2)
    #     balance2.execute()
    #
    #     while True:
    #         x = raw_input()
    #         if x=='l':
    #             l_turn.execute()
    #             prev = ''
    #         if x=='w' and prev =='w':
    #             walk_motion.execute()
    #             prev = 'w'
    #         elif x=='w':
    #             walk_init.execute()
    #             walk_motion.execute()
    #             prev = 'w'
    #         elif x=='r':
    #             r_turn.execute()
    #             prev = ''
    #         elif x=='a':
    #             l_inv.execute()
    #             prev = ''
    #         elif x=='d':
    #             r_inv.execute()
    #             prev = ''
    #         elif x=='k':
    #             kick.execute()
    #             prev = ''
    #
    #
    #
    #
    #
    #
    #
