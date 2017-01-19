#!/usr/bin/env python
import roslib; roslib.load_manifest('sbpl_multimaster')
import rospy
import rospkg
from collections import defaultdict
import IPython

class MultimasterConfigGenerator:
	def __init__(self):

		rospack = rospkg.RosPack()
		self.package_path = rospack.get_path('sbpl_multimaster')

		#read in rosparams
		if not rospy.has_param('generator_gateway_list'):
			rospy.logerr('generator parameters are not on the param server' +
			'try loading config/gateway_topics.yaml')

		# gateway_list should be ['dagobah', 'alan1', ...]
		self.gateway_list = rospy.get_param('generator_gateway_list')

		self.pr2_machine = rospy.get_param('generator_pr2') # should always be 'alan1'
		self.commander_machine = rospy.get_param('generator_commander') # for now, 'dagobah'
		self.roman_machine = rospy.get_param('generator_roman') # TBD, 'tatooine' for now
		# **new robots should go here**

		self.robot_dict = defaultdict(str)
		self.robot_dict[self.pr2_machine] = 'pr2/'
		self.robot_dict[self.roman_machine] = 'roman/'
		# **new robots should go here**

		self.defaultAdvertisements = defaultdict(list) # K-gateway, V-list of topics 
		self.defaultPulls = defaultdict(list) # K-gateway, V-list of topics

		for gw in self.gateway_list:
			self.defaultAdvertisements[gw] = []
			self.defaultPulls[gw] = []


	def writeConfigHeader(self, file, gateway):
		file.write("watch_loop_period: 1\n")
		file.write("firewall: false\n")
		file.write("name: "+gateway+"_gateway\n")
		if gateway == "alan1":
			file.write("network_interface: lan0\n")
		else:
			file.write("network_interface: eth0\n")

	def writeDefaultAdvertisements(self, file, gateway):
		#TODO
		file.write("default_advertisements:\n")
		#for...
		file.write("   - name: /" + publisher_topic +"\n")
		file.write("     node: None\n")
		file.write("     type: publisher\n")
		rospy.loginfo("     topic: " + publisher_topic)

	def writeDefaultPulls(self, file, gateway):
		#TODO
		file.write("\ndefault_pulls:\n")
		#for ...
		file.write("   - gateway: "+other_gateway+"_gateway\n")
		file.write("     rule:\n")
		file.write("       name: /"+topic+"\n")
		file.write("       node: None\n")
		file.write("       type: publisher\n\n")

	def run(self):
		# first generate the list of default advertisements from actions


		# next add the list of default advertisements from publsihers
		for gateway in self.gateway_list:
			prefix = ''
			if self.robot_dict[gateway]:
				prefix = self.robot_dict[gateway]
			publisher_topics = rospy.get_param("generator_"+gateway+"_publishers")
			for publisher_topic in publisher_topics:
				self.defaultAdvertisements[gateway].append(prefix + publisher_topic)

		IPython.embed()
		# add the list of default pulls from publishers
		for gateway in self.gateway_list:
			for other_gateway in self.gateway_list:
				if other_gateway!= gateway:
					prefix = ''
					if self.robot_dict[other_gateway]:
						prefix = self.robot_dict[other_gateway]
					topics = rospy.get_param("generator_"+other_gateway+"_publishers")
					for topic in topics:
						self.defaultPulls[gateway].append(prefix + topic)
		IPython.embed()
		# Write the accumulated advertisements and pull to config file
		for gateway in self.gateway_list:
			rospy.loginfo("Creating configuration for " + gateway)
			file = open(self.package_path + "/config/generated/" + gateway+"_config.yaml", 'w')
			self.writeConfigHeader(file, gateway)
			self.writeDefaultAdvertisements(file, gateway)
			self.writeDefaultPulls(file, gateway)
			file.close()

		# write the remapping topic to the generated launch file


if __name__ == '__main__':
	rospy.init_node('generate_multimaster_configs')
	MCG = MultimasterConfigGenerator()
	MCG.run()
	rospy.loginfo('All config files successfully generated!')