from smach import State

from std_srvs.srv import SetBoolRequest

from states import state_globals

class Loiter(State):
	def __init__(self):
		self.outcomes = [
			'failed_to_disable_teleop',
			'teleop_selected',
			'estop_selected',
			'return_home_selected',
			'marker_available',
			'quit'
		]
		State.__init__(self, self.outcomes)

	def execute(self, _):
		state_globals.set_state('Loiter')
		state_globals.loginfo('Loitering...')
		state_globals.led_pub.publish(state_globals.LedColors.autonomy)

		# check for shutdown or mode changes
		transition, message = state_globals.mode_change_checker()
		if transition in self.outcomes:
			state_globals.loginfo(message)
			return transition

		# disable teleop control
		teleop_response, teleop_success = state_globals.request_service(
			pre_request_log_msg='Disabling teleop control...',
			error_log_msg='Failed to disable teleop control...',
			success_log_msg='Disabled teleop control',
			service_proxy=state_globals.teleop_enabled_srv,
			request=SetBoolRequest(data=False),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not teleop_success:
			return 'failed_to_disable_teleop'

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition

			# check if there is a marker available
			with state_globals.marker_count_lock:
				marker_count = state_globals.marker_count
				if marker_count > 0:
					state_globals.loginfo('A marker has become available')
					return 'marker_available'
