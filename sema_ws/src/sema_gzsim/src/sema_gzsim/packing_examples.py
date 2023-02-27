#!/usr/bin/env python3

import rospy
from sema_gzsim.box_spawner import boxes_prms

# inefficient Horizontal Paking Demo
class PackingExample1(object):
	
	def __init__(self):
		self.variables_init()
		

	def variables_init(self):
		self.palet_prms = None
		self.target = None
		self.offset = None
		self.box_prms = None
		
		self.count = 0	


	def set_first_box_pose(self):
		self.target["x"] = self.palet_prms["x"] - self.palet_prms["size"]["x"]/2 + self.box_prms["size"]["x"]/2 + self.offset
		self.target["y"] = self.palet_prms["y"] - self.palet_prms["size"]["y"]/2 + self.box_prms["size"]["y"]/2 + self.offset
		self.target["z"] = self.palet_prms["z"] + self.box_prms["size"]["z"]/2 + self.box_prms["size"]["z"]/2 + self.offset
	
	def get_next_pose(self):
		self.count += 1

		if self.count%3 == 0:
			self.target["x"] = self.palet_prms["x"] - self.palet_prms["size"]["x"]/2 + self.box_prms["size"]["x"]/2 + self.offset
			self.target["y"] += self.box_prms["size"]["y"] + self.offset
		
		else:
			self.target["x"] += self.box_prms["size"]["x"] + self.offset

# inefficient Vertical Paking Demo
class PackingExample2(object):
	
	def __init__(self):
		self.palet_prms = None
		self.target = None
		self.offset = None
		self.box_prms = None
		
		self.count = 0	


	def set_first_box_pose(self):
		self.target["x"] = self.palet_prms["x"] - self.palet_prms["size"]["x"]/2 + self.box_prms["size"]["x"]/2 + self.offset
		self.target["y"] = self.palet_prms["y"] + self.offset
		self.target["z"] = self.palet_prms["z"] + self.box_prms["size"]["z"]/2 + self.box_prms["size"]["z"]/2 + self.offset
	
	def get_next_pose(self):
		self.count += 1

		if self.count%3 == 0:
			self.target["x"] = self.palet_prms["x"] - self.palet_prms["size"]["x"]/2 + self.box_prms["size"]["x"]/2 + self.offset
			self.target["z"] += self.box_prms["size"]["z"] + self.offset
		
		else:
			self.target["x"] += self.box_prms["size"]["x"] + self.offset


# efficient Horizontal Paking Demo
class PackingExample3(object):
	
	def __init__(self):
		self.palet_prms = None
		self.target = None
		self.offset = None
		self.box_prms = None
		
		self.count = 0	


	def set_first_box_pose(self):
		self.target["x"] = self.palet_prms["x"] + self.palet_prms["size"]["x"]/2 - self.box_prms["size"]["x"]/2 + self.offset
		self.target["y"] = self.palet_prms["y"] + self.palet_prms["size"]["y"]/2 - self.box_prms["size"]["y"]/2 + self.offset
		self.target["z"] = self.palet_prms["z"] + self.box_prms["size"]["z"]/2 + self.box_prms["size"]["z"]/2 + self.offset
	
	def get_next_pose(self):
		self.count += 1

		if self.count%3 == 0:
			self.target["x"] = self.palet_prms["x"] + self.palet_prms["size"]["x"]/2 - self.box_prms["size"]["x"]/2 + self.offset
			self.target["y"] -= self.box_prms["size"]["y"] + self.offset
		
		else:
			self.target["x"] -= self.box_prms["size"]["x"] + self.offset


# efficient Verical Paking Demo
class PackingExample4(object):
	
	def __init__(self):
		self.palet_prms = None
		self.target = None
		self.offset = None
		self.box_prms = None
		
		self.count = 0	


	def set_first_box_pose(self):
		self.target["x"] = self.palet_prms["x"] + self.palet_prms["size"]["x"]/2 - self.box_prms["size"]["x"]/2 + self.offset
		self.target["y"] = self.palet_prms["y"] + self.offset
		self.target["z"] = self.palet_prms["z"] + self.box_prms["size"]["z"]/2 + self.box_prms["size"]["z"]/2 + self.offset
	
	def get_next_pose(self):
		self.count += 1

		if self.count%3 == 0:
			self.target["x"] = self.palet_prms["x"] + self.palet_prms["size"]["x"]/2 - self.box_prms["size"]["x"]/2 + self.offset
			self.target["z"] += self.box_prms["size"]["z"] + self.offset
		
		else:
			self.target["x"] -= self.box_prms["size"]["x"] + self.offset


if __name__ == "__main__":

	paking_demo = PackingExample2() # PackingExample{1/2/3/4}()

	paking_demo.palet_prms = {"x":0.1, "y":0.75, "z":0.65, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}
	paking_demo.target = {"model":"m", "id":0, "x":0.0, "y":0.0, "z":0.0, "yaw":0.0, "static":True}
	paking_demo.box_prms = boxes_prms[paking_demo.target["model"]]
	paking_demo.offset = 0.001

	paking_demo.set_first_box_pose()
	print(paking_demo.target)
	
	for count in range(1,9):
		paking_demo.target["id"] = count
		paking_demo.get_next_pose()
		print(paking_demo.target)




