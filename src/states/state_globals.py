from enum import Enum
from threading import Lock

import rospy

from std_msgs.msg import ColorRGBA

### global variables #########################################################

# publishers and subscribers
planner_status_sub:					rospy.Subscriber
current_marker_sub:					rospy.Subscriber
marker_list_sub:					rospy.Subscriber
found_marker_list_sub:				rospy.Subscriber
selected_mode_sub:					rospy.Subscriber
current_waypoint_pub:				rospy.Publisher
state_pub:							rospy.Publisher
led_pub:							rospy.Publisher
remove_current_marker_srv:			rospy.ServiceProxy
publish_current_marker_srv:			rospy.ServiceProxy
publish_search_waypoint_srv:		rospy.ServiceProxy
through_next_gate_waypoint_srv:		rospy.ServiceProxy
through_update_gate_waypoint_srv:	rospy.ServiceProxy
circle_next_gate_waypoint_srv:		rospy.ServiceProxy
circle_update_gate_waypoint_srv:	rospy.ServiceProxy
plan_home_marker_srv:				rospy.ServiceProxy
planner_enabled_srv:				rospy.ServiceProxy
planner_precision_srv:				rospy.ServiceProxy
teleop_enabled_srv:					rospy.ServiceProxy

# subscribed info (synchronized)
planner_status_lock:				Lock
planner_status:						str

current_marker_lock:				Lock
current_aruco_id:					int
current_aruco_id_2:					int
current_marker_type:				str

marker_count_lock:					Lock
marker_count:						int

found_marker_dict_lock:				Lock
found_marker_dict:					dict

selected_mode_lock:					Lock
selected_mode:						str

# static
rate:								rospy.Rate
wait_at_marker_duration:			int
gnss_only_precision:				int
marker_precision:					int
gate_precision:						int

# current state
current_state:						str

### enums ####################################################################

class PlannerStatus(str, Enum):
	# ability = enabled and has location
	running = 'active'		# ability and has waypoint
	idle = 'idle'			# ability and has no waypoint
	blocked = 'blocked'		# no ability and has waypoint
	inactive = 'inactive'	# no ability and has no waypoint

class ControlModes(str, Enum):
	teleop = 'teleop'
	autonomy = 'autonomy'
	emergency_stop = 'emergency_stop'
	return_home = 'return_home'
	takeoff = 'takeoff'
	land = 'land'

class LedModes(int, Enum):
	solid = 255
	flashing = 128

# not an Enum but can be used like one
class LedColors(ColorRGBA):
	teleop = ColorRGBA(0, 0, 255, LedModes.solid)		# solid blue
	autonomy = ColorRGBA(255, 0, 0, LedModes.solid)		# solid red
	startup = ColorRGBA(255, 255, 0, LedModes.solid)	# solid yellow
	arrival = ColorRGBA(0, 255, 0, LedModes.flashing)	# flashing green

class MarkerTypes(str, Enum):
	no_marker = 'no_marker'
	one_marker = 'one_marker'
	gate = 'gate'
	intermediary_marker = 'intermediary_marker'

### change detection wrappers ################################################

def mode_change_checker() -> str:
	global rate
	rate.sleep()
	state_pub.publish(current_state)

	if rospy.is_shutdown():
		return 'quit', 'Shutting down...'

	global selected_mode_lock, selected_mode
	with selected_mode_lock:
		mode = selected_mode
		selected_mode = ''
		if mode == ControlModes.teleop:
			return 'teleop_selected', 'Teleop selected'
		if mode == ControlModes.autonomy:
			return 'autonomy_selected', 'Autonomy selected'
		if mode == ControlModes.emergency_stop:
			return 'estop_selected', 'Emergency stop selected'
		if mode == ControlModes.return_home:
			return 'return_home_selected', 'Return home selected'
		if mode == ControlModes.takeoff:
			return 'takeoff_selected', 'Takeoff selected'
		if mode == ControlModes.land:
			return 'land_selected', 'Land selected'

	return '', ''

### service request wrapper to handle errors #################################

class ResponseTests:
	def success_response_test(response):
		return response is not None and response.success
	
	def empty_response_test(response):
		return response is not None

def request_service(pre_request_log_msg, error_log_msg, success_log_msg, service_proxy, request, response_test=ResponseTests.success_response_test) -> bool:
	loginfo(pre_request_log_msg)

	try:
		response = service_proxy(request)
	except rospy.service.ServiceException as e:
		logerr(error_log_msg)
		return None, False
	
	success = response_test(response)
	
	if success:
		loginfo(success_log_msg)
	else:
		logerr(error_log_msg)

	return response, success

### log wrappers to print messages with the state they come from #############

def logdebug(msg: str):
	global current_state
	rospy.logdebug(f'[{current_state}] {msg}')

def loginfo(msg: str):
	global current_state
	rospy.loginfo(f'[{current_state}] {msg}')

def logwarn(msg: str):
	global current_state
	rospy.logwarn(f'[{current_state}] {msg}')

def logerr(msg: str):
	global current_state
	rospy.logerr(f'[{current_state}] {msg}')

### set state machine state ##################################################

def set_state(state: str):
	
	# clear selected mode after state change
	global selected_mode_lock, selected_mode
	with selected_mode_lock:
		selected_mode = ''
	
	# update state
	global current_state
	current_state = state
	state_pub.publish(current_state)
