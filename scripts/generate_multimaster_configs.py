#!/usr/bin/env python
import roslib; roslib.load_manifest('sbpl_multimaster')
import rospy
import rospkg

rospy.init_node('generate_multimaster_configs')
rospack = rospkg.RosPack()
package_path = rospack.get_path('sbpl_multimaster')

if not rospy.has_param('generator_gateway_list'):
	rospy.logerr('generator parameters are not on the param server' +
		'try loading config/gateway_topics.yaml')

gateway_list = rospy.get_param('generator_gateway_list')

for gateway in gateway_list:
	rospy.loginfo("Creating configuration for " + gateway)
	file = open(package_path + "/config/generated/" + gateway+"_config.yaml", 'w')
	file.write("watch_loop_period: 1\n")
	file.write("firewall: false\n")
	file.write("name: "+gateway+"_gateway\n")
	if gateway == "alan1":
		file.write("network_interface: lan0\n")
	else:
		file.write("network_interface: eth0\n")


	# now create the default advertisements
	publisher_topics = rospy.get_param("generator_"+gateway+"_publishers")
	file.write("default_advertisements:\n")
	for publisher_topic in publisher_topics:
		file.write("   - name: /" + publisher_topic +"\n")
		file.write("     node: None\n")
		file.write("     type: publisher\n")

	# not create the default pulls for other machines
	file.write("\ndefault_pulls:\n")
	for other_gateway in gateway_list:
		if other_gateway!= gateway:
			topics = rospy.get_param("generator_"+other_gateway+"_publishers")
			for topic in topics:
				file.write("   - gateway: "+other_gateway+"_gateway\n")
				file.write("     rule:\n")
				file.write("       name: /"+topic+"\n")
				file.write("       node: None\n")
				file.write("       type: publisher\n\n")

	file.close()

rospy.loginfo('All config files successfully generated!')
