#coding=utf-8
#!/usr/bin/env python
from pymavlink import mavutil
from std_msgs.msg import String
import threading
import time,codecs,os
import rospy


wt_hb_flag = 0
run_flag = 1

findFlag = 0
leftFlag = 0

#params
selfRotRate = 1000
straightSpeed = 20
twitingRate = 1000

def turn_to_find():
    print("turning to find target object...")
    #while find_flag:
	#turn round
	#find_flag = objrvt_detect_data_validation;
	

#def run_to_object():
    #calculate straight speed
    
def param_init():
    getFlag = 0
    file = open('/home/nvidia/yolo_ws/param.txt')
    for line in file.readlines():
	if line[0] == 'a':
	   getFlag = 1
	   continue	
	if getFlag:
	   getFlag = 0
	   index1=line.find(',')
	   index2=line.find(';')
           global selfRotRate, straightSpeed, twitingRate#global var
           selfRotRate=line[0:index1]#string type
           straightSpeed=line[index1+1:index2]
	   twitingRate=line[index2+1:len(line)]
           selfRotRate=int(selfRotRate)
           straightSpeed=int(straightSpeed)
           twitingRate=int(twitingRate)
           print("params: ", selfRotRate, straightSpeed, twitingRate)


def sychron_msg_rcv_task():
    print("Start Listen!")
    while not rospy.is_shutdown():
	msg = the_connection.recv_match(blocking=True)
	#if msg.get_type() == "HEARTBEAT" and wt_hb_flag:
	    #print(msg)

	if msg.get_type() == "COMMAND_LONG":
	    print("command is {}".format(msg.command))
	    print("param1 is {}".format(msg.param1))
    	    global run_flag
	    if msg.command == 9010:
		run_flag = 1
	    elif msg.command == 9011:
		run_flag = 2
	    elif msg.command == 9012:
		run_flag = 0
	    elif msg.command == 8999:
		run_flag = 3
	elif msg.get_type() == "SET_POSITION_TARGET_LOCAL_NED":
	    print("target_system:{}, target_system:{}").format(msg.target_system, msg.target_component)
	    print("vx:{}, vy:{}, vz:{}").format(msg.vx, msg.vy, msg.vz)
	    print("yaw_rate:{}").format(msg.yaw_rate)

def run_task():
    while not rospy.is_shutdown():
	#time.sleep(1)
	'''
	if run_flag == 1:
	    print("sending message 9010...")
	    the_connection.mav.set_position_target_local_ned_send(0,
		1, 0,
		8, 0b010011000011,
		0, 0, 0,
		0.8, 0, 0,
		0, 0, 0,
		0, 0)
	elif run_flag == 2:
	    print("sending message 9000...")
	    the_connection.mav.command_long_send(1,
		0, 8010,
		0, -20,
		1000, 3,
		4, 5,
		6, 7)
	'''
        global findFlag, leftFlag
	if run_flag == 1 and findFlag == 1:
	    # print("sending message 8010...")#angle speed
	    the_connection.mav.command_long_send(1,
		0, 8010,
		0, straightSpeed,
		0, 0,
		0, 0,
		0, 0)
	    if leftFlag == 1:#target in left
               the_connection.mav.command_long_send(1,
		   0, 8010,
		   0, straightSpeed,
		   -twitingRate, 0,
		   0, 0,
		   0, 0)
	    elif leftFlag == -1:#right in right
               the_connection.mav.command_long_send(1,
		   0, 8010,
		   0, straightSpeed,
		   twitingRate, 0,
		   0, 0,
		   0, 0)
	elif run_flag == 1 and findFlag == 0:#find target
	    the_connection.mav.command_long_send(1,
		0, 8010,
		0, 0,
		selfRotRate, 0,
		0, 0,
		0, 0)

def callback(data):
    index=data.data.find(',')
    global findFlag, leftFlag#global var
    findFlag=data.data[0:index]#string type
    leftFlag=data.data[index+1:len(data.data)]
    findFlag=float(findFlag)
    leftFlag=float(leftFlag)
    #print("findFlag: %d", findFlag)
    #print("object in left: (1 for T, -1 for F) %d", leftFlag)


#1Hz hb send task
def main():
    #print("waiting for hb...")
    #the_connection.wait_heartbeat()

    sy = threading.Thread(target=sychron_msg_rcv_task, args=())
    #hb = threading.Thread(target=heart_beat_task, args=())
    rt = threading.Thread(target=run_task, args=())
	
    sy.start()
    #hb.start()
    rt.start()
    time.sleep(1)
    #print("Heartbeat from system (system %u component %u)"
	#(the_connection.target_system, the_connection.target_component))

    while not rospy.is_shutdown():
	'''
	str_cmd = int(input("num(0-9)"))
	# time_boot_ms target_system target_component coordinate_frame type_mask
	# x y z vx vy vz afx afy afz yaw yaw_rate force_mavlink1=False
	if str_cmd == 1: #straignt
	    the_connection.mav.set_position_target_local_ned_send(0,
		1, 0,
		8, 0b010011000011,
		0, 0, 0,
		0.8, 0, 0,
		0, 0, 0,
		0, 0)

	if str_cmd == 2: #turn 
	    the_connection.mav.set_position_target_local_ned_send(0,
		1, 0,
		8, 0b010011000011,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0.8)
	'''
	# type autopilot base_mode custom_mode system_status
	#global findFlag, leftFlag, run_flag
        #print("var run_flag is:{}".format(run_flag))
        #print("findFlag in run_task: %d", findFlag)
        #print("leftFlag in run_task: %d", leftFlag)
	the_connection.mav.heartbeat_send(6,
		8, 0,
		0, 0)
	#print("heartbeat sent...")
	time.sleep(1)
	

if __name__ == "__main__":
    rospy.init_node('motion_decision', anonymous=True)
    # rate = rospy.Rate(10) # 10hz #not used yet 
    param_init()   
    the_connection=mavutil.mavlink_connection('/dev/ttyUSB0',57600)  
    # print("connect successfully!")
    rospy.Subscriber("camera/messages_test", String, callback)
    main()
    rospy.spin()
    
























