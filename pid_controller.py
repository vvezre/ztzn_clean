class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=100, output_limit=100):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.integral = 0
        self.prev_error = 0
        self.integral_limit = integral_limit
        self.output_limit = output_limit

    def update(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.prev_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, self.output_limit), -self.output_limit)

        self.prev_error = error
        return output
