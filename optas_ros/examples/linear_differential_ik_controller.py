import optas
from optas.templates import ROSController

from copy import deepcopy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class LinearDifferentialIKController(ROSController):

    state_listener = {
        'joint_states': JointState,
        'linear_velocity_goal': Float64MultiArray,
    }

    def setup_solver(self):

        # Setup variables
        self.dt = 1./float(self.hz)
        link_ee = self.config['end_effector_name']
        w_minimize_joint_velocity = self.config['w_minimize_joint_velocity']
        w_minimize_joint_acceleration = self.config['w_minimize_joint_acceleration']

        # Setup robot
        self.robot = optas.RobotModel(urdf_string=self.urdf_string, time_derivs=[1])
        self.name = self.robot.get_name()

        # Setup optimization builder
        builder = optas.OptimizationBuilder(T=1, robots=self.robot, derivs_align=True)

        # Add parameters
        qc = builder.add_parameter('qc', self.robot.ndof)  # current joint position
        dqc = builder.add_parameter('dqc', self.robot.ndof)  # current joint velocity
        vg = builder.add_parameter('vg', 3)  # goal linear velocity

        # Extract decision variables
        dq = builder.get_model_state(self.name, 0, time_deriv=1)

        # Pre-prep for cost terms and constraints
        J = self.robot.get_global_linear_jacobian(link_ee, qc)
        dv = J @ dq

        # Cost: linear velocity goal
        builder.add_cost_term('linear_velocity_goal', optas.sumsqr(dv - vg))

        # Cost: minimize joint velocity
        builder.add_cost_term(
            'minimize_joint_velocity',
            w_minimize_joint_velocity*optas.sumsqr(dq),
        )

        # Cost: minimize joint acceleration
        ddq = (dq - dqc)/self.dt
        builder.add_cost_term(
            'minimize_joint_acceleration',
            w_minimize_joint_acceleration*optas.sumsqr(ddq),
        )

        # Constraint: joint position limits
        lower = self.robot.lower_actuated_joint_limits
        upper = self.robot.upper_actuated_joint_limits
        qn = qc + self.dt*dq
        builder.add_bound_inequality_constraint('joint_position_limits', lower, qn, upper)

        # Setup solver
        solver = optas.OSQPSolver(builder.build()).setup(use_warm_start=True)

        return solver

    def _get_current_joint_states(self):
        msg = deepcopy(self.msgs['joint_states'])
        qc = []
        for joint_name in self.robot.actuated_joint_names:
            joint_index = msg.name.index(joint_name)
            qc.append(msg.position[joint_index])
        return optas.DM(qc)

    def _get_current_linear_velocity_goal(self):
        return self.msgs['linear_velocity_goal']

    def reset(self):
        self.qc = self._get_current_joint_states()
        if not self.is_first_solve():
            self.solver.reset_initial_seed(self.solution)
        self.solver.reset_parameters({
            'qc': self.qc,
            'vg': self._get_current_linear_velocity_goal(),
        })

    def get_target(self):
        dq = self.solution[f'{self.name}/dq']
        q_target = self.qc + self.dt*dq
        return q_target
