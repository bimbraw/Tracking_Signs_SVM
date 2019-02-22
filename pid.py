# pid.py

class PID:
    """

    Class implementation of PID controller

    """

    def __init__(self, dt, max_out, min_out, Kp, Kd, Ki):
        self.dt = float(dt)
        self.max_out = max_out
        self.min_out = min_out
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.pre_error = float(0)
        self.integral = float(0)


    def calculate(self, error):
        # Get error
        self.error = error

        # Proportional term
        self.Pout = self.Kp * self.error

        # Integral term
        self.integral += self.error * self.dt
        self.Iout = self.Ki * self.integral

        # Derivative term
        self.derivative = (self.error - self.pre_error) / self.dt
        self.Dout = self.Kd * self.derivative

        # Total output
        self.output = self.Pout + self.Iout + self.Dout

        # Restrict by Max and Min
        if (self.output > self.max_out):
            self.output = self.max_out
        elif (self.output < self.min_out):
            self.output = self.min_out

        # Save error to previous error
        self.pre_error = self.error

        return self.output




