import optas
import rospy
from optas_ros import Controller
from optas_ros import StateListener, JointStateListener, TfListener

class PositionListener(TfListener):

    def get(self):
        tr = self._msg.transform.translation
        return optas.DM([tr.x, tr.y, tr.z])


class ExampleJointController(Controller):

    def specify_problem(self):

        self.robot = optas.RobotModel(
            urdf_string=rospy.get_param('robot_description'),
            time_derivs=[0, 1],
        )

        # Setup builder
        self.builder = optas.OptimizationBuilder(T=2, robots=self.robot)

        # Setup parameters
        targ = self.builder.add_parameter('targ', 3)
        qc = self.builder.add_parameter('qc', self.robot.ndof)

        # Constraint: initial configuration
        self.builder.initial_configuration(self.robot.get_name(), qc)

        # Constraint: dynamics
        self.builder.integrate_model_states(self.robot.get_name(), time_deriv=1, dt=self.config['dt'])

        # Cost: minimize joint velocity
        w_mjv = 1
        qd = self.builder.get_model_state(self.robot.get_name(), 0, time_deriv=1)
        self.builder.add_cost_term('minimize_joint_velocity', w_mjv*optas.sumsqr(qd))

        # Cost: goal target position
        w_g = 1e8
        J = self.robot.get_global_linear_jacobian('lwr_arm_7_link', qc)
        pd = J @ qd
        pc = self.robot.get_global_link_position('lwr_arm_7_link', qc)
        pf = pc + self.config['dt']*pd
        self.builder.add_cost_term('goal_target_position', w_g*optas.sumsqr(pf - targ))

        # Constraint: joint velocity limit
        max_joint_vel = self.config['max_joint_vel']
        self.builder.add_bound_inequality_constraint(
            'maximum_joint_velocity', -max_joint_vel, qd, max_joint_vel
        )

    def setup_state_listener(self):

        states = {
            'joints': JointStateListener(self.robot, topic_name='rpbi/kuka_lwr/joint_states'),
            'target': PositionListener('rpbi/world', 'target'),
        }

        self.state_listener = StateListener(states=states)

    def setup_solver(self):
        # self.solver = optas.CasADiSolver(self.optimization).setup('ipopt')
        # self.solver = optas.CasADiSolver(self.optimization).setup('sqpmethod')
        self.solver = optas.OSQPSolver(self.optimization).setup(True)
        # self.solver = optas.CVXOPTSolver(self.optimization).setup()

    def reset_problem(self):

        # Get states from listener
        targ = self.state_listener.get_state('target')
        qc = self.state_listener.get_state('joints')

        # Reset initial seed with previous solution
        if self._solution is not None:
            self.solver.reset_initial_seed(self._solution)
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
