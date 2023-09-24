from smach import State

import rospy
from std_srvs.srv import SetBoolRequest

from marker_interfacing.srv import RemoveCurrentMarkerRequest

from states import state_globals

class ArrivedAtMarker(State):
	def __init__(self):
		self.pre_arrival_outcomes = [
			'intermediary_arrival_complete',
			'failed_to_remove_current_marker',
			'failed_to_disable_planner',
			'wait_at_marker_timeout',
			'teleop_selected',
			'estop_selected',
			'return_home_selected',
			'quit'
		]
		self.outcomes = self.pre_arrival_outcomes + ['autonomy_selected']
		State.__init__(self, self.outcomes)

	def execute(self, _):
		state_globals.set_state('ArrivedAtMarker')
		state_globals.loginfo('Arrived at marker...')

		# check for shutdown or mode changes (excluding autonomy)
		transition, message = state_globals.mode_change_checker()
		if transition in self.pre_arrival_outcomes:
			state_globals.loginfo(message)
			return transition

		# ask marker manager to remove current marker
		remove_current_marker_response, remove_current_marker_success = state_globals.request_service(
			pre_request_log_msg='Removing current marker from marker manager...',
			error_log_msg='Failed to remove current marker from marker manager',
			success_log_msg='Removed current marker from marker manager',
			service_proxy=state_globals.remove_current_marker_srv,
			request=RemoveCurrentMarkerRequest(),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not remove_current_marker_success:
			return 'failed_to_remove_current_marker'

		# disable planner
		planner_response, planner_success = state_globals.request_service(
			pre_request_log_msg='Disabling planner...',
			error_log_msg='Failed to disable planner...',
			success_log_msg='Disabled planner',
			service_proxy=state_globals.planner_enabled_srv,
			request=SetBoolRequest(data=False),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not planner_success:
			return 'failed_to_disable_planner'

		state_globals.loginfo('Arrival sequence complete')

		with state_globals.current_marker_lock:
			current_marker_type = state_globals.current_marker_type
		if current_marker_type == state_globals.MarkerTypes.intermediary_marker:
			return 'intermediary_arrival_complete'

		state_globals.led_pub.publish(state_globals.LedColors.arrival)

		if state_globals.wait_at_marker_duration > 0:
			wait_at_marker_timeout = rospy.Time.now() + rospy.Duration(state_globals.wait_at_marker_duration)
			state_globals.loginfo(f'Waiting at marker for {state_globals.wait_at_marker_duration} seconds...')
		else:
			wait_at_marker_timeout = None
			state_globals.loginfo(f'Waiting at marker for instructions...')

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition

			# if waiting for too long, proceed to loiter
			if wait_at_marker_timeout and rospy.Time.now() > wait_at_marker_timeout:
				state_globals.loginfo('Waiting at marker for too long')
				return 'wait_at_marker_timeout'
