import optas
import rospy
import tf2_ros
from copy import deepcopy
from optas_ros import Controller
from optas_ros import StateListener, TfListener

class PositionListener(TfListener):

    @staticmethod
    def get_position(msg):
        tf = msg.transform.translation
        return optas.DM([tr.x, tr.y, tr.z])

    def get(self):
        return self.get_position(self._msg)

class VelocityListener(TfListener):

    n_window = 2

    def __init__(self, parent, child, hz=50):
        super().__init__(parent, child, hz=hz)
        self._msg = []

    def _update(self, event):
        if self._msg is None: return
        try:
            self._msg.append(self._buf.lookup_transform(self.parent, self.child, rospy.Time()))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        if len(self._msg) > self.n_window:
            self._msg.pop(0)

    def recieved(self):
        return len(self._msg) == self.n_window

    @staticmethod
    def extract_time(msg):
        stamp = msg.header.stamp
        return float(stamp.secs) + float(stamp.nsecs)/1e9

    def _extract(self, msg):
        return self.extract_time(msg), PositionListener.get_position(msg)

    def get(self):

        # Extract messages
        msgs = deepcopy(self._msgs)
        msg_new = msgs[-1]
        msg_old = msgs[-2]

        # Extract time/position and compute/return velocity
        t_new, p_new = self._extract(msg_new)
        t_old, p_old = self._extract(msg_old)
        return (p_new - p_old)/(t_new - t_old)

class ExampleMPCController(Controller):

    def specify_problem(self):

        # Planner attributes
        dt = self.config['dt_mpc']  # time step for mpc plan
        obs_rad = self.config['obs_rad']  # obstacle radii

        # Setup point mass model
        pm_radius = self.config['pm_rad']  # point mass radii
        pm_dim = 3 # x, y, z dimensions
        dlim = {0: [[-1.5]*3, [1.5]*3], 1: [[-0.2]*3, [0.2]*3]}  # pos/vel limits
        point_mass = optas.TaskModel('point_mass', pm_dim, time_derivs=[0, 1], dlim=dlim)
        pm_name = point_mass.get_name()

        # Setup optimization builder
        T = self.config['T'] # number of time steps
        builder = optas.OptimizationBuilder(T, tasks=point_mass, derivs_align=True)

        # Add parameters
        init_pos = builder.add_parameter('init_pos', 3)  # initial point mass position
        init_vel = builder.add_parameter('init_vel', 3)  # initial point mass velocity
        goal = builder.add_parameter('goal', 3)  # goal point mass position
        obs = builder.add_parameter('obs', 3)  # obstacle position

        # Compute predicted model (straight line model)
        _init_pos = optas.SX.sym('init_pos', 3, T)
        _goal = optas.SX.sym('goal', 3)
        _human_model = optas.SX.zeros(3, T)
        _human_model[:, 0] = init_pos
        for i in range(T-1):
            _human_model[:, i+1] = _human_model[:, i] + _goal - _init_pos
        human_model = optas.Function('human_model', [_init_pos, _goal], [_human_model])

        # Constraint: limits
        builder.enforce_model_limits(pm_name, time_deriv=0)
        builder.enforce_model_limits(pm_name, time_deriv=1)

        # Constraint: dynamics
        builder.integrate_model_states(pm_name, time_deriv=1, dt=dt)

        # Constraint: initial state
        builder.initial_configuration(pm_name, init=init_pos)
        builder.initial_configuration(pm_name, init=init_vel, time_deriv=1)

        # Constraint: obstacle avoidance
        X = builder.get_model_states(pm_name)
        safe_dist_sq = (obs_rad + pm_radius)**2
        for i in range(T):
            dist_sq = optas.sumsqr(obs - X[:, i])
            builder.add_geq_inequality_constraint(f'obs_avoid_{i}', dist_sq, safe_dist_sq)

        # Cost: follow human model
        builder.add_cost_term('follow_human_model', optas.sumsqr(X - human_model(init_pos, goal)))

        # Cost: minimize acceleration
        w = 0.005/float(T)  # weight on cost term
        dX = builder.get_model_states(pm_name, time_deriv=1)
        ddX = (dX[:, 1:] - dX[:, :-1])/dt
        builder.add_cost_term('minimize_acceleration', w*optas.sumsqr(ddX))

        # Set class attributes
        self.builder = builder
        self.T = T
        self.dt = dt

    def setup_state_listener(self):
        states = dict(
            goal=PositionListener('rpbi/world', 'goal'),
            obstacle=PositionListener('rpbi/world', 'obstacle'),
            init_pos=PositionListener('rpbi/world', 'rpbi/kuka_lwr/lwr_arm_7_link'),
            init_vel=VelocityListener('rpbi/world', 'rpbi/kuka_lwr/lwr_arm_7_link'),
        )
        self.state_listener = StateListener(states=states)

    def setup_solver(self):
        self.solver = optas.CasADiSolver(self.optimization).setup('ipopt')
        # self.solver = optas.CasADiSolver(self.optimization).setup('sqpmethod')

    def reset_problem(self):

        # Get states from listener
        goal = self.state_listener.get_state('goal')
        obstacle = self.state_listener.get_state('obstacle')
        init_pos = self.state_listener.get_state('init_pos')
        init_vel = self.state_listener.get_state('init_vel')

        # Reset initial seed with previous solution
        if self._solution is not None:
            self.solver.reset_initial_seed(self._solution)
        else:
            goal_vel = (goal - init_pos)/self.dt
            self.solver.reset_initial_seed({
                'point_mass/x': optas.diag(goal)@optas.DM.ones(3, self.T),
                'point_mass/dx': optas.diag(goal_vel)@optas.DM.ones(3, self.T),
            })

        # Reset parameters
        self.solver.reset_parameters({'goal': goal

        
