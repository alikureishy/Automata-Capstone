import rospy
from enum import Enum

class Scope(Enum):
    """
    Caller must set this scoping on the parameter value being defined:
    -   Local           : Resolves within the node's scope
    -   Relative        : Resolves relative to the namespace
    -   Root            : Resolves relative to the root "/"
    -   FirstAvailable  : Attempts all 3 scopes, in the order Local, Relative, Root, Default-Value
    """
    LOCAL = "LOCAL"
    RELATIVE = "RELATIVE"
    ROOT = "ROOT"
    FIRST_AVAILABLE = "FIRST_AVAILABLE"

class ScopedResolver(object):
    def __init__(self, param_name, param_default, scope, dynamic = False):
        self.param_default = param_default
        self.scope = scope
        self.dynamic = dynamic

        # Strip away any "/" or "~" character from the beginning
        self.param_name = param_name.strip("~").strip("/")

        # Save 3 versions of the param name:
        self.local_name = "~{}".format(param_name)
        self.relative_name = param_name
        self.root_name = "/{}".format(param_name)
        self.any_name = [self.local_name, self.relative_name, self.root_name]

        rospy.logdebug("[%s] [%s] Parameter '%s' registered as '%s', with '%s' scope",
                       self.scope,
                       "Dynamic" if self.dynamic else "Static",
                       param_name, self.param_name)

        self.saved_value = None

    def Get(self): #, default_override = None):
        if self.saved_value is None or self.dynamic:
            default_value = self.param_default #if default_override is None else default_override
            value = None
            name_attempted = None
            if self.scope == Scope.LOCAL:
                name_attempted = self.local_name
                value = rospy.get_param(name_attempted, default_value)
            elif self.scope == Scope.RELATIVE:
                name_attempted = self.relative_name
                value = rospy.get_param(name_attempted, default_value)
            elif self.scope == Scope.ROOT:
                name_attempted = self.root_name
                value = rospy.get_param(name_attempted, default_value)
            elif self.scope == Scope.FIRST_AVAILABLE:
                for name in self.any_name:
                    name_attempted = name
                    value = rospy.get_param(name_attempted, None)
                    if value is not None:
                        break
                value = value if value is not None else default_value
            else:
                raise ("Unknown scope: {}".format(self.scope))
            self.saved_value = value

            rospy.logdebug("[%s] [%s] param lookup: '%s' [=> '%s']. Retrieved value = '%s' (Default: '%s')", self.scope,
                           "Dynamic" if self.dynamic else "Static",
                           self.param_name, name_attempted, self.saved_value, default_value)
        else:
            rospy.logdebug("[%s] [Saved] param lookup: '%s' [=> '%s']. Saved value = '%s'", self.scope,
                           self.param_name, self.local_name, self.saved_value)

        return self.saved_value

class Params(object):
    class Shared(object):
        SimulationMode = ScopedResolver             ('sim_mode',                        0,                                  Scope.ROOT)  # Simulator_mode parameter (1== ON, 0==OFF)

    class Server(object):
        Port = ScopedResolver                       ('port',                            4567,                               Scope.LOCAL)
        ImageDropProbability = ScopedResolver       ('image_drop_prob',                 0.75,                               Scope.LOCAL)  # If > 0.0, we drop images by that prbability

    class CamInfo(object):
        GrasshopperCalibrationYaml = ScopedResolver ('/grasshopper_calibration_yaml',   None,                               Scope.ROOT)

    class Classifier(object):
        ModelFilePath = ScopedResolver              ('model_file',                      "model/graph_optimized.pb",         Scope.LOCAL)
        DataFolder = ScopedResolver                 ('data_folder',                     "../../../images",                  Scope.LOCAL)
        SavingImages = ScopedResolver               ('save_images',                     0,                                  Scope.LOCAL)  # 0 = False / 1 = True
        CheatMode = ScopedResolver                  ('cheat_mode',                      1,                                  Scope.LOCAL)  # 0 = False / 1 = True
        ImageNameFormat = ScopedResolver            ('image_name_format',               "{}-{}.jpeg",                       Scope.LOCAL)
        StateCountThreshold = ScopedResolver        ('state_count_threshold',           3,                                  Scope.LOCAL)
        TrafficLightConfig = ScopedResolver         ('/traffic_light_config',           None,                               Scope.ROOT)

    class Controller(object):
        Kp = ScopedResolver                         ('~kp',                              0.3,                               Scope.LOCAL)
        Ki = ScopedResolver                         ('~ki',                              0.1,                               Scope.LOCAL)
        Kd = ScopedResolver                         ('~kd',                              0.,                                Scope.LOCAL)
        Mn = ScopedResolver                         ('~mn',                              0.,                                Scope.LOCAL)  # Minimum throttle value
        Mx = ScopedResolver                         ('~mx',                              0.2,                               Scope.LOCAL)  # Maximum throttle value
        Tau = ScopedResolver                        ('~tau',                             0.5,                               Scope.LOCAL)  # 1/(2pi*tau) = cutoff frequency
        Ts = ScopedResolver                         ('~ts',                              0.02,                              Scope.LOCAL)  # Sample time

    class DBW(object):
        VehicleMass = ScopedResolver                ('~vehicle_mass',                    1736.35,                           Scope.LOCAL)
        FuelCapacity = ScopedResolver               ('~fuel_capacity',                   13.5,                              Scope.LOCAL)
        BrakeDeadband = ScopedResolver              ('~brake_deadband',                  .1,                                Scope.LOCAL)
        DecelLimit = ScopedResolver                 ('~decel_limit',                     -5,                                Scope.LOCAL)
        AccelLimit = ScopedResolver                 ('~accel_limit',                     1.,                                Scope.LOCAL)
        WheelRadius = ScopedResolver                ('~wheel_radius',                    0.2413,                            Scope.LOCAL)
        WheelBase = ScopedResolver                  ('~wheel_base',                      2.8498,                            Scope.LOCAL)
        SteerRatio = ScopedResolver                 ('~steer_ratio',                     14.8,                              Scope.LOCAL)
        MaxLatAccel = ScopedResolver                ('~max_lat_accel',                   3.,                                Scope.LOCAL)
        MaxSteerAngle = ScopedResolver              ('~max_steer_angle',                 8.,                                Scope.LOCAL)

    class WaypointLoader(object):
        VelocityKmph = ScopedResolver               ('velocity',                        None,                               Scope.LOCAL)
        Path = ScopedResolver                       ('path',                            None,                               Scope.LOCAL)
        
    class WaypointUpdater(object):
        LookaheadWaypointCount = ScopedResolver     ('~lookahead_waypoint_count',        50,                                Scope.LOCAL)
