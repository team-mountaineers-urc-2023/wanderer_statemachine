#!/usr/bin/env python3

from threading import Lock

from smach import StateMachine

import rospy
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import SetBool, Trigger, Empty
from geometry_msgs.msg import Point

from aruco_finder.msg import FoundMarkerList
from marker_interfacing.msg import ENUMarker, GeodeticMarkerList
from marker_interfacing.srv import RemoveCurrentMarker, PublishCurrentMarker, PlanHomeMarker
from planner_interfacing.srv import SetPrecision

from states import ArrivedAtMarker, GoingToWaypoint, GoingToMarker, \
	Loiter, MarkerSearch, GateTraversal, ReturnHome, Startup, TeleopControl, \
	state_globals

### main #####################################################################

def main():

	rospy.init_node("state_machine")

	### local variables ######################################################
	
	state_globals.planner_status = 'inactive'
	state_globals.planner_status_lock = Lock()

	state_globals.current_marker_type = None
	state_globals.current_aruco_id = None
	state_globals.current_marker_lock = Lock()

	state_globals.marker_count = 0
	state_globals.marker_count_lock = Lock()

	state_globals.found_marker_dict = FoundMarkerList
	state_globals.found_marker_dict_lock = Lock()

	state_globals.selected_mode = 'teleop'
	state_globals.selected_mode_lock = Lock()

	state_globals.wait_at_marker_duration = rospy.get_param("~wait_at_marker_duration")
	state_globals.gnss_only_precision = rospy.get_param("~gnss_only_precision")
	state_globals.marker_precision = rospy.get_param("~marker_precision")
	state_globals.gate_precision = rospy.get_param("~gate_precision")

	### connect to ROS #######################################################

	planner_status_topic = rospy.get_param("~planner_status_topic")
	current_marker_topic = rospy.get_param("~current_marker_topic")
	marker_list_topic = rospy.get_param("~marker_list_topic")
	found_marker_list_topic = rospy.get_param("~found_marker_list_topic")
	selected_mode_topic = rospy.get_param("~selected_mode_topic")
	current_waypoint_topic = rospy.get_param("~current_waypoint_topic")
	state_topic = rospy.get_param("~state_topic")
	led_topic = rospy.get_param("~led_topic")
	remove_current_marker_service = rospy.get_param("~remove_current_marker_service")
	publish_current_marker_service = rospy.get_param("~publish_current_marker_service")
	publish_search_waypoint_service = rospy.get_param("~publish_search_waypoint_service")
	through_next_gate_waypoint_service = rospy.get_param("~through_next_gate_waypoint_service")
	through_update_gate_waypoint_service = rospy.get_param("~through_update_gate_waypoint_service")
	circle_next_gate_waypoint_service = rospy.get_param("~circle_next_gate_waypoint_service")
	circle_update_gate_waypoint_service = rospy.get_param("~circle_update_gate_waypoint_service")
	plan_home_marker_service = rospy.get_param("~plan_home_marker_service")
	planner_enabled_service = rospy.get_param("~planner_enabled_service")
	planner_precision_service = rospy.get_param("~planner_precision_service")
	teleop_enabled_service = rospy.get_param("~teleop_enabled_service")
	frequency = rospy.get_param("~frequency")

	rospy.wait_for_service(remove_current_marker_service)
	rospy.wait_for_service(publish_current_marker_service)
	rospy.wait_for_service(publish_search_waypoint_service)
	rospy.wait_for_service(through_next_gate_waypoint_service)
	rospy.wait_for_service(through_update_gate_waypoint_service)
	rospy.wait_for_service(circle_next_gate_waypoint_service)
	rospy.wait_for_service(circle_update_gate_waypoint_service)
	rospy.wait_for_service(plan_home_marker_service)
	rospy.wait_for_service(planner_enabled_service)
	rospy.wait_for_service(planner_precision_service)
	rospy.wait_for_service(teleop_enabled_service)

	state_globals.planner_status_sub = rospy.Subscriber(planner_status_topic, String, planner_status_callback)
	state_globals.current_marker_sub = rospy.Subscriber(current_marker_topic, ENUMarker, current_marker_callback)
	state_globals.marker_list_sub = rospy.Subscriber(marker_list_topic, GeodeticMarkerList, marker_list_callback)
	state_globals.found_marker_list_sub = rospy.Subscriber(found_marker_list_topic, FoundMarkerList, found_marker_list_callback)
	state_globals.selected_mode_sub = rospy.Subscriber(selected_mode_topic, String, selected_mode_callback)
	state_globals.current_waypoint_pub = rospy.Publisher(current_waypoint_topic, Point, queue_size=1)
	state_globals.state_pub = rospy.Publisher(state_topic, String, queue_size=1)
	state_globals.led_pub = rospy.Publisher(led_topic, ColorRGBA, queue_size=1)
	state_globals.remove_current_marker_srv = rospy.ServiceProxy(remove_current_marker_service, RemoveCurrentMarker)
	state_globals.publish_current_marker_srv = rospy.ServiceProxy(publish_current_marker_service, PublishCurrentMarker)
	state_globals.publish_search_waypoint_srv = rospy.ServiceProxy(publish_search_waypoint_service, Trigger)
	state_globals.through_next_gate_waypoint_srv = rospy.ServiceProxy(through_next_gate_waypoint_service, Trigger)
	state_globals.through_update_gate_waypoint_srv = rospy.ServiceProxy(through_update_gate_waypoint_service, Empty)
	state_globals.circle_next_gate_waypoint_srv = rospy.ServiceProxy(circle_next_gate_waypoint_service, Trigger)
	state_globals.circle_update_gate_waypoint_srv = rospy.ServiceProxy(circle_update_gate_waypoint_service, Empty)
	state_globals.plan_home_marker_srv = rospy.ServiceProxy(plan_home_marker_service, PlanHomeMarker)
	state_globals.planner_enabled_srv = rospy.ServiceProxy(planner_enabled_service, SetBool)
	state_globals.planner_precision_srv = rospy.ServiceProxy(planner_precision_service, SetPrecision)
	state_globals.teleop_enabled_srv = rospy.ServiceProxy(teleop_enabled_service, SetBool)
	state_globals.rate = rospy.Rate(frequency)

	### begin statemachine ###################################################

	state_globals.set_state('StateMachine Initialization')
	sm = StateMachine(outcomes=['Success', 'End'])
	with sm:
		StateMachine.add('Startup', Startup(),
			transitions={'startup_complete':'TeleopControl',
						 'quit':'End'})

		StateMachine.add('TeleopControl', TeleopControl(),
			transitions={'failed_to_disable_planner':'TeleopControl',
						 'failed_to_enable_teleop':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'autonomy_selected':'Loiter',
						 'return_home_selected':'ReturnHome',
						 'quit':'End'})
		
		StateMachine.add('Loiter', Loiter(),
			transitions={'failed_to_disable_teleop':'TeleopControl',
						 'teleop_selected':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'marker_available':'GoingToWaypoint',
						 'return_home_selected':'ReturnHome',
						 'quit':'End'})

		StateMachine.add('GoingToWaypoint', GoingToWaypoint(),
			transitions={'failed_to_publish_waypoint':'Loiter',
						 'failed_to_set_planner_precision':'GoingToWaypoint',
						 'failed_to_enable_planner':'Loiter',
						 'teleop_selected':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'found_marker':'GoingToMarker',
						 'near_waypoint':'MarkerSearch',
						 'return_home_selected':'ReturnHome',
						 'quit':'End'})

		StateMachine.add('GoingToMarker', GoingToMarker(),
			transitions={'failed_to_set_planner_precision':'GoingToMarker',
						 'teleop_selected':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'near_marker':'ArrivedAtMarker',
						 'through_gate_traversal':'ThroughGateTraversal',
						 'circle_gate_traversal':'CircleGateTraversal',
						 'return_home_selected':'ReturnHome',
						 'quit':'End'})

		StateMachine.add('MarkerSearch', MarkerSearch(),
			transitions={'failed_to_set_planner_precision':'MarkerSearch',
						 'failed_to_publish_search_waypoint':'TeleopControl',
						 'teleop_selected':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'through_gate_traversal':'ThroughGateTraversal',
						 'circle_gate_traversal':'CircleGateTraversal',
						 'found_marker':'GoingToMarker',
						 'return_home_selected':'ReturnHome',
						 'quit':'End'})

		StateMachine.add('ThroughGateTraversal', GateTraversal(approach='through'),
			transitions={'failed_to_set_planner_precision':'ThroughGateTraversal',
						 'failed_to_publish_gate_waypoint':'TeleopControl',
						 'teleop_selected':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'gate_traversal_complete':'ArrivedAtMarker',
						 'return_home_selected':'ReturnHome',
						 'quit':'End'})

		StateMachine.add('CircleGateTraversal', GateTraversal(approach='circle'),
			transitions={'failed_to_set_planner_precision':'CircleGateTraversal',
						 'failed_to_publish_gate_waypoint':'TeleopControl',
						 'teleop_selected':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'gate_traversal_complete':'ArrivedAtMarker',
						 'return_home_selected':'ReturnHome',
						 'quit':'End'})

		StateMachine.add('ArrivedAtMarker', ArrivedAtMarker(),
			transitions={'intermediary_arrival_complete':'Loiter',
						 'failed_to_remove_current_marker':'ArrivedAtMarker',
						 'failed_to_disable_planner':'ArrivedAtMarker',
						 'wait_at_marker_timeout':'Loiter',
						 'teleop_selected':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'autonomy_selected':'Loiter',
						 'return_home_selected':'ReturnHome',
						 'quit':'End'})

		StateMachine.add('ReturnHome', ReturnHome(),
			transitions={'failed_to_plan_home_marker':'ReturnHome',
						 'failed_to_set_planner_precision':'ReturnHome',
						 'failed_to_enable_planner':'ReturnHome',
						 'teleop_selected':'TeleopControl',
						 'estop_selected':'TeleopControl',
						 'autonomy_selected':'Loiter',
						 'reached_home':'ArrivedAtMarker',
						 'quit':'End'})
	sm.execute()

	### end init #############################################################

### callbacks ################################################################

def planner_status_callback(status: String):
	with state_globals.planner_status_lock:
		state_globals.planner_status = status.data

def current_marker_callback(current_marker: ENUMarker):
	with state_globals.current_marker_lock:
		state_globals.current_aruco_id = current_marker.aruco_id
		state_globals.current_aruco_id_2 = current_marker.aruco_id_2
		state_globals.current_marker_type = current_marker.marker_type

def marker_list_callback(marker_list: GeodeticMarkerList):
	with state_globals.marker_count_lock:
		state_globals.marker_count = marker_list.count

def found_marker_list_callback(found_marker_list: FoundMarkerList):
	with state_globals.found_marker_dict_lock:
		state_globals.found_marker_dict = {}
		for found_marker in found_marker_list.markers:
			state_globals.found_marker_dict[found_marker.aruco_id] = found_marker.marker_enu

def selected_mode_callback(mode: String):
	with state_globals.selected_mode_lock:
		state_globals.selected_mode = mode.data

if __name__ == '__main__':
	main()
