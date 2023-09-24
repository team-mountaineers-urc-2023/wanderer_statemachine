from smach import State

from std_srvs.srv import SetBoolRequest

from states import state_globals

class TeleopControl(State):
	def __init__(self):
		self.outcomes = [
			'failed_to_disable_planner',
			'failed_to_enable_teleop',
			'estop_selected',
			'autonomy_selected',
			'return_home_selected',
			'quit'
		]
		State.__init__(self, self.outcomes)

	def execute(self, _):
		state_globals.set_state('TeleopControl')
		state_globals.loginfo('Switching to teleop control...')

		# check for shutdown or mode changes
		transition, message = state_globals.mode_change_checker()
		if transition in self.outcomes:
			state_globals.loginfo(message)
			return transition

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

		# enable teleop control
		teleop_response, teleop_success = state_globals.request_service(
			pre_request_log_msg='Enabling teleop control...',
			error_log_msg='Failed to enable teleop control',
			success_log_msg='Enabled teleop control',
			service_proxy=state_globals.teleop_enabled_srv,
			request=SetBoolRequest(data=True),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not teleop_success:
			return 'failed_to_enable_teleop'

		state_globals.led_pub.publish(state_globals.LedColors.teleop)

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition
