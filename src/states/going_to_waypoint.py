from smach import State

from std_srvs.srv import SetBoolRequest

from marker_interfacing.srv import PublishCurrentMarkerRequest
from planner_interfacing.srv import SetPrecisionRequest

from states import state_globals

class GoingToWaypoint(State):
	def __init__(self):
		self.outcomes = [
			'failed_to_publish_waypoint',
			'failed_to_set_planner_precision',
			'failed_to_enable_planner',
			'teleop_selected',
			'estop_selected',
			'return_home_selected',
			'found_marker',
			'near_waypoint',
			'quit'
		]
		State.__init__(self, self.outcomes)

	def execute(self, _):
		state_globals.set_state('GoingToWaypoint')
		state_globals.loginfo('Going to waypoint...')
		state_globals.led_pub.publish(state_globals.LedColors.autonomy)

		# check for shutdown or mode changes
		transition, message = state_globals.mode_change_checker()
		if transition in self.outcomes:
			state_globals.loginfo(message)
			return transition

		# ask marker manager to publish current marker
		publish_current_marker_response, publish_current_marker_success = state_globals.request_service(
			pre_request_log_msg='Asking marker manager to publish current marker...',
			error_log_msg='Marker manager failed to publish current marker',
			success_log_msg='Marker manager published current marker',
			service_proxy=state_globals.publish_current_marker_srv,
			request=PublishCurrentMarkerRequest(),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not publish_current_marker_success:
			return 'failed_to_publish_waypoint'
		
		# set planner precision
		precision_response, precision_success = state_globals.request_service(
			pre_request_log_msg='Setting planner precision for waypoints...',
			error_log_msg='Failed to set planner precision...',
			success_log_msg='Set planner precision',
			service_proxy=state_globals.planner_precision_srv,
			request=SetPrecisionRequest(precision=state_globals.gnss_only_precision),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not precision_success:
			return 'failed_to_set_planner_precision'

		# enable planner
		planner_response, planner_success = state_globals.request_service(
			pre_request_log_msg='Enabling planner...',
			error_log_msg='Failed to enable planner...',
			success_log_msg='Enabled planner',
			service_proxy=state_globals.planner_enabled_srv,
			request=SetBoolRequest(data=True),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not planner_success:
			return 'failed_to_enable_planner'

		# check what marker we are searching for
		with state_globals.current_marker_lock:
			current_aruco_id = state_globals.current_aruco_id
			current_aruco_id_2 = state_globals.current_aruco_id_2
			current_marker_type = state_globals.current_marker_type
		searching_for_marker = current_marker_type == state_globals.MarkerTypes.one_marker
		searching_for_gate = current_marker_type == state_globals.MarkerTypes.gate
		if not (searching_for_marker or searching_for_gate):
			return 'found_marker'

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition

			# check if marker has been seen
			with state_globals.found_marker_dict_lock:
				found_marker_dict = state_globals.found_marker_dict
				found_marker_1 = current_aruco_id in found_marker_dict
				found_marker_2 = current_aruco_id_2 in found_marker_dict

				marker_seen = found_marker_1 or (searching_for_gate and found_marker_2)
				if marker_seen:
					state_globals.loginfo('Marker found')
					return 'found_marker'

			# check if robot is near waypoint
			# 	if planner_status has changed to idle (no longer running)
			# 	the robot is near the waypoint
			with state_globals.planner_status_lock:
				planner_status = state_globals.planner_status
				if planner_status == state_globals.PlannerStatus.idle:
					state_globals.loginfo('Robot is near waypoint')
					return 'near_waypoint'
