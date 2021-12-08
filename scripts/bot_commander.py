#!/usr/bin/env python3

import rospy

from std_msgs.msg import String 

usr_idx_to_obj = {
            '1': '51',
            '2': '11',
            '3': '13',
            '4': '74',
            '5': '10'
        }
        
if __name__=="__main__":
	print('Enter the index of the item you would like to rescue')
	#print('1) Bowl')
	print('2) Fire hydrant')
	#print('3) Stop sign')
	print('4) Mouse')
	print('5) Traffic light')

	rospy.init_node('bot_commander')
	pub = rospy.Publisher('/rescue_cmd', String, queue_size=5)
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown(): # We will need to stop this while loop while the object list has been published to the topic. 
		try:
			ip_cmd = input("Enter the object to be rescued: ")
			objects = [s.strip() for s in ip_cmd.split(',')]
			translated_objs = []
			for obj in objects:
			    if obj in usr_idx_to_obj:
			        translated_objs.append(usr_idx_to_obj[obj])
			        
			print(f'Sending message: {",".join(translated_objs)}')
			pub.publish(",".join(translated_objs))
		except:
			break
	
