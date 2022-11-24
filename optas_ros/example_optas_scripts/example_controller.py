import optas
import rospy
from optas_ros import Controller
from optas_ros import StateListener, JointStateListener, TfListener

class PositionListener(TfListener):

    def get(self):
        tr = self._msg.transform.translation
        return optas.DM([tr.x, tr.y, tr.z])


class ExampleController(Controller):

    def setup_robot(self):
        self.robot = optas.RobotModel(
            urdf_string=rospy.get_param('robot_description'),
            time_derivs=[0, 1],
        )

    def setup_state_listener(self):

        states = {
            'joints': JointStateListener(self.robot),
            'target': PositionListener('world', 'target'),
        }

        self.state_listener = StateListener(states=states)

    def specify_problem(self):

        # Setup builder
        self.builder = optas.OptimizationBuilder(T=2, robots=self.robot)

        # Setup parameters
        targ = self.builder.add_parameter('targ', 3)
        qc = self.builder.add_parameter('qc', self.robot.ndof)

        # Constraint: initial configuration
        self.builder.initial_configuration('initial_configuration', qc)

        # Constraint: dynamics
        self.builder.integrate_model_states('dynamics', time_deriv=1, dt=self.config['dt'])

        # Cost: minimize joint velocity
        w_mjv = 1e-2
        qd = self.builder.get_model_state(self.robot.get_name(), 0, time_deriv=1)
        self.builder.add_cost_term('minimize_joint_velocity', w_mjv*optas.sumsqr(qd))

        # Cost: goal target position
        qf = self.builder.get_model_state(self.robot.get_name(), 1)
        pf = self.robot.get_global_link_position(qf)
        self.builder.add_cost_term('goal_target_position', optas.sumsqr(pf - targ))

    def setup_solver(self):
        self.solver = optas.CasADiSolver(self.optimization).setup('ipopt')

    def reset_problem(self):

        # Get states from listener
        targ = self.state_listener.get_state('target')
        qc = self.state_listener.get_state('joints')

        # Reset initial seed with previous solution
        if self._solution is not None:
            if self.solver.reset_initial_seed(self._solution)
        else:
            self.solver.reset_initial_seed({f'{self.robot.get_name()}/q': optas.horzcat(qc, qc)})

        # Reset parameters
        self.solver.reset_parameters({'targ': targ, 'qc': qc})

    def compute_next_state(self):

        # Solve problem
        self._solution = self.solver.solve()

        # Return target
        Q = self._solution[f'{self.robot.get_name()}/q']
        return Q[:, 1]
