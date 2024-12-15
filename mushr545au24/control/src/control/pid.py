from __future__ import division
import numpy as np

from control.controller import BaseController
from control.controller import compute_position_in_frame
import ros


class PIDController(BaseController):
    def __init__(self, **kwargs):
        self.kp = kwargs.pop("kp")
        self.kd = kwargs.pop("kd")

        # Get the keyword args that we didn't consume with the above initialization
        super(PIDController, self).__init__(**kwargs)


    def get_error(self, pose, reference_xytv):
        """Compute the PD error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: across-track and cross-track error
        """
        return compute_position_in_frame(pose, reference_xytv[:3])

    def get_control(self, pose, reference_xytv, error):
        """Compute the PD control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: np.array of velocity and steering angle
                (velocity should be copied from reference velocity)
        """
        # BEGIN QUESTION 2.1
        "*** REPLACE THIS LINE ***"
        # PID calculations
        # ki = 0
        # P = self.kp*error
        # I = I + self.ki*error*(ros.time() - t_prev)
        # D = self.kd*(error - error_prev)/(t - t_prev)
        # MV = MV_bar + P + I + D
        
        P = self.kp * error[1]
        velocity = reference_xytv[3]
        D = self.kd * velocity * (np.sin(pose[2] - reference_xytv[2]))
        steering_angle = -1 * float(P + D)
        return np.array([velocity, steering_angle])
        raise NotImplementedError
        # END QUESTION 2.1
