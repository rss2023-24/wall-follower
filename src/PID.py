import rospy
import csv

class PID:

    setpoint = rospy.get_param("wall_follower/desired_distance")
    MAX_STEERING_ANGLE = rospy.get_param("wall_follower/max_steering_angle")

    def __init__(self):

        # self.Kp = 0.8
        # self.Ki = 0.7
        # self.Kd = 0.15

        # self.Kp = 0.5
        self.Kp = rospy.get_param("wall_follower/Kp")
        self.Ki = rospy.get_param("wall_follower/Ki")
        self.Kd = rospy.get_param("wall_follower/Kd")

        self.last_time = rospy.Time().now()
        self.last_error = 0
    
    def step(self, curr_distance):
        
        params_filename = '/home/racecar/racecar_ws/src/team-24/wall-follower/src/consts.csv'
        with open(params_filename,'r') as csvfile:
            lines = list(csv.reader(csvfile, delimiter=','))
            Kp, Ki, Kd = float(lines[0][0]), float(lines[0][1]), float(lines[0][2])
        
        print(Kp, Ki, Kd)
        print(lines)
        
        curr_time = rospy.Time().now()
        time_delta = (curr_time - self.last_time).to_sec()

        error_signal = self.setpoint - curr_distance

        proportional_output = self.Kp * error_signal
        integral_output = self.Ki * (time_delta * (error_signal + self.last_error) / 2)
        derivative_output = self.Kd * ((error_signal - self.last_error) / time_delta)

        control_signal = proportional_output + integral_output + derivative_output
        regulated_control_signal = max( min(control_signal, self.MAX_STEERING_ANGLE), -self.MAX_STEERING_ANGLE )

        self.last_time = curr_time
        self.last_error = error_signal

        return regulated_control_signal
