from smach import State

from states import state_globals

class Startup(State):
	def __init__(self):
		self.outcomes = [
			'startup_complete',
			'quit'
		]
		State.__init__(self, self.outcomes)

	def execute(self, _):
		state_globals.set_state('Startup')
		state_globals.loginfo('Starting up...')
		state_globals.led_pub.publish(state_globals.LedColors.startup)

		state_globals.loginfo('Running startup checks...')
		# TODO: startup checks (batteries, cameras, motors...)
		state_globals.logwarn('No startup checks configured')
		state_globals.loginfo('Startup checks passed')

		state_globals.loginfo('Startup complete')
		return 'startup_complete'
