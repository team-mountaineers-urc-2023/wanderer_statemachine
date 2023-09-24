from smach import State

from planner_interfacing.srv import SetPrecisionRequest

from states import state_globals

class GoingToMarker(State):
	def __init__(self):
		self.outcomes = [
			'failed_to_set_planner_precision',
			'teleop_selected',
			'estop_selected',
			'return_home_selected',
			'near_marker',
			'through_gate_traversal',
			'circle_gate_traversal',
			'quit'
		]
		State.__init__(self, self.outcomes)

	def execute(self, _):
		state_globals.set_state('GoingToMarker')
		state_globals.loginfo('Going to marker...')

		# check for shutdown or mode changes
		transition, message = state_globals.mode_change_checker()
		if transition in self.outcomes:
			state_globals.loginfo(message)
			return transition

		# check what marker we are searching for
		with state_globals.current_marker_lock:
			current_aruco_id = state_globals.current_aruco_id
			current_aruco_id_2 = state_globals.current_aruco_id_2
			current_marker_type = state_globals.current_marker_type
		going_to_intermediary_marker = current_marker_type == state_globals.MarkerTypes.intermediary_marker
		searching_for_marker = current_marker_type == state_globals.MarkerTypes.one_marker
		searching_for_gate = current_marker_type == state_globals.MarkerTypes.gate

		if going_to_intermediary_marker or searching_for_marker or searching_for_gate:
			# set planner precision
			precision_response, precision_success = state_globals.request_service(
				pre_request_log_msg='Setting planner precision for markers...',
				error_log_msg='Failed to set planner precision...',
				success_log_msg='Set planner precision',
				service_proxy=state_globals.planner_precision_srv,
				request=SetPrecisionRequest(precision=state_globals.marker_precision),
				response_test=state_globals.ResponseTests.success_response_test
			)
			if not precision_success:
				return 'failed_to_set_planner_precision'

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition

			# check if robot is near marker
			# 	if planner_status has changed to idle (no longer running)
			# 	the robot is near the marker
			with state_globals.planner_status_lock:
				planner_status = state_globals.planner_status
				if planner_status == state_globals.PlannerStatus.idle:

					# if there was a gate, we are near it but need to find the other marker
					if searching_for_gate:
						state_globals.loginfo('Robot is near one marker, but did not find the other')
						return 'circle_gate_traversal'

					state_globals.loginfo('Robot is near marker')
					return 'near_marker'

			# update search target
			with state_globals.found_marker_dict_lock:
				found_marker_dict = state_globals.found_marker_dict
				found_marker_1 = current_aruco_id in found_marker_dict
				found_marker_2 = current_aruco_id_2 in found_marker_dict

				if searching_for_gate and found_marker_1 and found_marker_2:
					state_globals.loginfo('Robot has found both markers')
					return 'through_gate_traversal'
			
				elif searching_for_marker or searching_for_gate and found_marker_1:
					marker_enu = found_marker_dict[current_aruco_id]
					state_globals.current_waypoint_pub.publish(marker_enu)

				elif searching_for_gate and found_marker_2:
					marker_2_enu = found_marker_dict[current_aruco_id_2]
					state_globals.current_waypoint_pub.publish(marker_2_enu)
