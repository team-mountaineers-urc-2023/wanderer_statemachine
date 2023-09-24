from smach import State

from std_srvs.srv import SetBoolRequest

from marker_interfacing.srv import PlanHomeMarkerRequest
from planner_interfacing.srv import SetPrecisionRequest

from states import state_globals

class ReturnHome(State):
	def __init__(self):
		self.outcomes = [
			'failed_to_plan_home_marker',
			'failed_to_set_planner_precision',
			'failed_to_enable_planner',
			'teleop_selected',
			'estop_selected',
			'autonomy_selected',
			'reached_home',
			'quit'
		]
		State.__init__(self, self.outcomes)

	def execute(self, _):
		state_globals.set_state('ReturnHome')
		state_globals.loginfo('Returning home...')
		state_globals.led_pub.publish(state_globals.LedColors.autonomy)

		# check for shutdown or mode changes
		transition, message = state_globals.mode_change_checker()
		if transition in self.outcomes:
			state_globals.loginfo(message)
			return transition

		# ask marker manager to set home marker
		home_marker_response, home_marker_success = state_globals.request_service(
			pre_request_log_msg='Asking marker manager to set home marker...',
			error_log_msg='Marker manager failed to plan home marker',
			success_log_msg='Marker manager planned home marker',
			service_proxy=state_globals.plan_home_marker_srv,
			request=PlanHomeMarkerRequest(),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not home_marker_success:
			return 'failed_to_plan_home_marker'

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

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition

			# check if robot is near waypoint
			# 	if planner_status has changed to idle (no longer running)
			# 	the robot is near the waypoint
			with state_globals.planner_status_lock:
				planner_status = state_globals.planner_status
				if planner_status == state_globals.PlannerStatus.idle:
					state_globals.loginfo('Robot has reached home')
					return 'reached_home'
