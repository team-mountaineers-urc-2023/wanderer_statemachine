from smach import State

from std_srvs.srv import TriggerRequest

from planner_interfacing.srv import SetPrecisionRequest

from states import state_globals

class MarkerSearch(State):
	def __init__(self):
		self.outcomes = [
			'teleop_selected',
			'estop_selected',
			'return_home_selected',
			'failed_to_set_planner_precision',
			'failed_to_publish_search_waypoint',
			'through_gate_traversal',
			'circle_gate_traversal',
			'found_marker',
			'quit'
		]
		State.__init__(self, self.outcomes)

	def execute(self, _):
		state_globals.set_state('MarkerSearch')
		state_globals.loginfo('Starting marker search...')

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

		# check what marker we are searching for
		with state_globals.current_marker_lock:
			current_aruco_id = state_globals.current_aruco_id
			current_aruco_id_2 = state_globals.current_aruco_id_2
			current_marker_type = state_globals.current_marker_type
		searching_for_marker = current_marker_type == state_globals.MarkerTypes.one_marker
		searching_for_gate = current_marker_type == state_globals.MarkerTypes.gate

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition

			# ask marker searcher to publish search waypoint
			publish_search_waypoint_response, publish_search_waypoint_success = state_globals.request_service(
				pre_request_log_msg='Asking marker searcher to publish search waypoint...',
				error_log_msg='Marker searcher failed to publish search waypoint',
				success_log_msg='Marker searcher published search waypoint',
				service_proxy=state_globals.publish_search_waypoint_srv,
				request=TriggerRequest(),
				response_test=state_globals.ResponseTests.success_response_test
			)
			if not publish_search_waypoint_success:
				return 'failed_to_publish_search_waypoint'

			while True:

				# check for shutdown or mode changes
				transition, message = state_globals.mode_change_checker()
				if transition in self.outcomes:
					state_globals.loginfo(message)
					return transition

				# update search target
				with state_globals.found_marker_dict_lock:
					found_marker_dict = state_globals.found_marker_dict
					found_marker_1 = current_aruco_id in found_marker_dict
					found_marker_2 = current_aruco_id_2 in found_marker_dict

					if searching_for_gate and found_marker_1 and found_marker_2:
						state_globals.loginfo('Robot has found both markers')
						return 'through_gate_traversal'

					elif searching_for_gate and found_marker_1 or found_marker_2:
						state_globals.loginfo('Robot has found one gate marker')
						return 'circle_gate_traversal'

					elif searching_for_marker and found_marker_1:
						state_globals.loginfo('Marker found')
						return 'found_marker'

				# check if robot is near search waypoint
				# 	if planner_status has changed to idle (no longer running)
				# 	the robot is near the search waypoint
				with state_globals.planner_status_lock:
					planner_status = state_globals.planner_status
					if planner_status == state_globals.PlannerStatus.idle:
						state_globals.loginfo('Robot reached search waypoint')
						break
