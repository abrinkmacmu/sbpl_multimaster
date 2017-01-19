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

		self.actions = rospy.get_param('generator_commander_action_clients')
		#IPython.embed()

		self.defaultAdvertisements = defaultdict(list) # K-gateway, V-list of topics 
		self.defaultPullsTopics = defaultdict(list) # K-gateway, V-list of topics
		self.defaultPullsGateway = defaultdict(list) # K-gateway, V-list of 'other_gateway'

		for gw in self.gateway_list:
			self.defaultAdvertisements[gw] = []
			self.defaultPullsTopics[gw] = []
			self.defaultPullsGateway[gw] = []


	def writeConfigHeader(self, file, gateway):
		file.write("watch_loop_period: 1\n")
		file.write("firewall: false\n")
		file.write("name: "+gateway+"_gateway\n")
		if gateway == "alan1":
			file.write("network_interface: lan0\n")
		else:
			file.write("network_interface: eth0\n")

	def writeDefaultAdvertisements(self, file, gateway):
		file.write("default_advertisements:\n")
		for topic in self.defaultAdvertisements[gateway]:
			file.write("   - name: /" + topic +"\n")
			file.write("     node: None\n")
			file.write("     type: publisher\n")
			rospy.loginfo("     topic: " + topic)

	def writeDefaultPulls(self, file, gateway):
		file.write("\ndefault_pulls:\n")
		for i in range(0, len(self.defaultPullsTopics[gateway])):
			topic = self.defaultPullsTopics[gateway][i]
			other_gateway = self.defaultPullsGateway[gateway][i]
			file.write("   - gateway: "+other_gateway+"_gateway\n")
			file.write("     rule:\n")
			file.write("       name: /"+topic+"\n")
			file.write("       node: None\n")
			file.write("       type: publisher\n\n")

	def writePublishedRemaps(self, file, gateway):
		for topic in self.defaultAdvertisements[gateway]:
			topic_unprefixed = topic[ topic.find('/')+1: len(topic)]
			##topic_name = topic_unprefixed[0:topic_unprefixed.find('/')] #not unique enough
			topic_name = topic_unprefixed.replace('/','_')
			file.write("  <node name=\""+topic_name+"_relay_node\" pkg=\"topic_tools\" type=\"relay\" ")
			file.write("args=\""+topic_unprefixed+" "+topic +"\" >\n")
			file.write("    <param name=\"lazy\" value=\"true\"/>\n")
			file.write("  </node>\n\n")


	def writeSubscribedRemaps(self, file, gateway, robot_prefix):
		for topic in self.defaultPullsTopics[gateway]:
			if(topic.find(robot_prefix) > -1 ):
				topic_unprefixed = topic[ topic.find('/')+1: len(topic)]
				##topic_name = topic_unprefixed[0:topic_unprefixed.find('/')] #not unique enough
				topic_name = topic_unprefixed.replace('/','_')
				file.write("  <node name=\""+topic_name+"_relay_node\" pkg=\"topic_tools\" type=\"relay\" ")
				file.write("args=\""+topic+" "+topic_unprefixed +"\" >\n")
				file.write("    <param name=\"lazy\" value=\"true\"/>\n")
				file.write("  </node>\n\n")

	def run(self):
		# first generate the list of default advertisements from actions

		for robot in self.actions.keys():
			for action_root in self.actions[robot]:
				robot_machine = "unknown"
				if robot == "pr2":
					robot_machine = self.pr2_machine
				elif robot == "roman":
					robot_machine = self.roman_machine
				else:
					rospy.loginfo("Error parsing generator_commander_action_clients, unknown robot name")
				## Add new robots here
				

				self.defaultAdvertisements[self.pr2_machine].append(robot+"/" + action_root + "/status")
				self.defaultAdvertisements[self.pr2_machine].append(robot+"/" + action_root + "/result")
				self.defaultAdvertisements[self.pr2_machine].append(robot+"/" + action_root + "/feedback")

				self.defaultPullsTopics[self.pr2_machine].append(robot+"/" + action_root + "/goal")
				self.defaultPullsGateway[self.pr2_machine].append(self.commander_machine)
				self.defaultPullsTopics[self.pr2_machine].append(robot+"/" + action_root + "/cancel")
				self.defaultPullsGateway[self.pr2_machine].append(self.commander_machine)

				self.defaultAdvertisements[self.commander_machine].append(robot+"/" + action_root + "/goal")
				self.defaultAdvertisements[self.commander_machine].append(robot+"/" + action_root + "/cancel")

				self.defaultPullsTopics[self.commander_machine].append(robot+"/" + action_root + "/status")
				self.defaultPullsGateway[self.commander_machine].append(self.pr2_machine)
				self.defaultPullsTopics[self.commander_machine].append(robot+"/" + action_root + "/result")
				self.defaultPullsGateway[self.commander_machine].append(self.pr2_machine)
				self.defaultPullsTopics[self.commander_machine].append(robot+"/" + action_root + "/feedback")
				self.defaultPullsGateway[self.commander_machine].append(self.pr2_machine)



		# next add the list of default advertisements from publsihers
		for gateway in self.gateway_list:
			prefix = ''
			if self.robot_dict[gateway]:
				prefix = self.robot_dict[gateway]
			publisher_topics = rospy.get_param("generator_"+gateway+"_publishers")
			for publisher_topic in publisher_topics:
				self.defaultAdvertisements[gateway].append(prefix + publisher_topic)


		# add the list of default pulls from publishers
		for gateway in self.gateway_list:
			for other_gateway in self.gateway_list:
				if other_gateway!= gateway:
					prefix = ''
					if self.robot_dict[other_gateway]:
						prefix = self.robot_dict[other_gateway]
					topics = rospy.get_param("generator_"+other_gateway+"_publishers")
					for topic in topics:
						self.defaultPullsTopics[gateway].append(prefix + topic)
						self.defaultPullsGateway[gateway].append(other_gateway)

		# Write the accumulated advertisements and pull to config file
		for gateway in self.gateway_list:
			rospy.loginfo("Creating configuration for " + gateway)
			file = open(self.package_path + "/config/generated/" + gateway+"_config.yaml", 'w')
			self.writeConfigHeader(file, gateway)
			self.writeDefaultAdvertisements(file, gateway)
			self.writeDefaultPulls(file, gateway)
			file.close()

		# write the remapping topic to the generated launch file
		gateway = self.pr2_machine
		robot_prefix = "pr2"
		file = open(self.package_path + "/launch/generated/" + gateway+"_remappings.launch", 'w')
		file.write("<launch>\n")

		self.writePublishedRemaps(file, gateway)
		self.writeSubscribedRemaps(file, gateway, robot_prefix)

		file.write("</launch>")
		file.close()


if __name__ == '__main__':
	rospy.init_node('generate_multimaster_configs')
	MCG = MultimasterConfigGenerator()
	MCG.run()
	rospy.loginfo('All config files successfully generated!')
