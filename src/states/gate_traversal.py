from smach import State

import rospy
from std_srvs.srv import TriggerRequest, EmptyRequest

from planner_interfacing.srv import SetPrecisionRequest

from states import state_globals

class GateTraversal(State):
	def __init__(self, approach: str):
		self.outcomes = [
			'failed_to_set_planner_precision',
			'teleop_selected',
			'estop_selected',
			'return_home_selected',
			'failed_to_publish_gate_waypoint',
			'gate_traversal_complete',
			'quit'
		]
		State.__init__(self, self.outcomes)
		
		self.approach = approach

		if approach == 'through':
			self.next_gate_waypoint_srv = state_globals.through_next_gate_waypoint_srv
			self.update_gate_waypoint_srv = state_globals.through_update_gate_waypoint_srv
			self.state_name = 'ThroughGateTraversal'
		elif approach == 'circle':
			self.next_gate_waypoint_srv = state_globals.circle_next_gate_waypoint_srv
			self.update_gate_waypoint_srv = state_globals.circle_update_gate_waypoint_srv
			self.state_name = 'CircleGateTraversal'
		else:
			raise ValueError('Invalid approach')

	def execute(self, _):
		state_globals.set_state(self.state_name)
		state_globals.loginfo(f'Starting {self.approach} gate traversal...')

		# check for shutdown or mode changes
		transition, message = state_globals.mode_change_checker()
		if transition in self.outcomes:
			state_globals.loginfo(message)
			return transition

		# set planner precision
		precision_response, precision_success = state_globals.request_service(
			pre_request_log_msg='Setting planner precision for gates...',
			error_log_msg='Failed to set planner precision...',
			success_log_msg='Set planner precision',
			service_proxy=state_globals.planner_precision_srv,
			request=SetPrecisionRequest(precision=state_globals.gate_precision),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not precision_success:
			return 'failed_to_set_planner_precision'

		### go to the initial gate waypoint ###

		# ask gate traversal to publish gate waypoint
		publish_gate_waypoint_response, publish_gate_waypoint_success = state_globals.request_service(
			pre_request_log_msg='Asking gate planner to publish initial gate waypoint...',
			error_log_msg='Gate Planner failed to publish initial gate waypoint',
			success_log_msg='Gate Planner published initial gate waypoint',
			service_proxy=self.next_gate_waypoint_srv,
			request=TriggerRequest(),
			response_test=state_globals.ResponseTests.success_response_test
		)
		if not publish_gate_waypoint_success:
			return 'failed_to_publish_gate_waypoint'

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition

			# check if robot is near initial gate waypoint
			# 	if planner_status has changed to idle (no longer running)
			# 	the robot is near the initial gate waypoint
			with state_globals.planner_status_lock:
				planner_status = state_globals.planner_status
				if planner_status == state_globals.PlannerStatus.idle:
					state_globals.loginfo('Robot reached initial gate waypoint')
					break

			# ask gate traversal to update gate waypoint
			try:
				self.update_gate_waypoint_srv(EmptyRequest())
			except rospy.service.ServiceException as e:
				pass

		### complete gate traversal ###

		while True:

			# check for shutdown or mode changes
			transition, message = state_globals.mode_change_checker()
			if transition in self.outcomes:
				state_globals.loginfo(message)
				return transition

			# ask gate traversal to publish gate waypoint
			publish_gate_waypoint_response, publish_gate_waypoint_success = state_globals.request_service(
				pre_request_log_msg='Asking gate planner to publish gate waypoint...',
				error_log_msg='Gate Planner failed to publish gate waypoint',
				success_log_msg='Gate Planner published gate waypoint',
				service_proxy=self.next_gate_waypoint_srv,
				request=TriggerRequest(),
				response_test=state_globals.ResponseTests.success_response_test
			)
			if not publish_gate_waypoint_success:
				return 'failed_to_publish_gate_waypoint'
			if publish_gate_waypoint_response.message == 'Complete':
				return 'gate_traversal_complete'

			while True:

				# check for shutdown or mode changes
				transition, message = state_globals.mode_change_checker()
				if transition in self.outcomes:
					state_globals.loginfo(message)
					return transition

				# check if robot is near gate waypoint
				# 	if planner_status has changed to idle (no longer running)
				# 	the robot is near the gate waypoint
				with state_globals.planner_status_lock:
					planner_status = state_globals.planner_status
					if planner_status == state_globals.PlannerStatus.idle:
						state_globals.loginfo('Robot reached gate waypoint')
						break
				
				# ask gate traversal to update gate waypoint
				try:
					self.update_gate_waypoint_srv(EmptyRequest())
				except rospy.service.ServiceException as e:
					pass
