import numpy as np
import matplotlib.pyplot as plt


class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = np.clip(output, -self.max_output, self.max_output)
        self.prev_error = error
        return output


class VehicleModel:
    def __init__(self, wheelbase=2.5):
        # 轴距
        self.wheelbase = wheelbase
        self.x = 0
        self.y = 0
        # 偏航角（yaw）为车辆自身坐标系下的旋转角度，范围-180°~180°‌
        self.yaw = 0
        self.velocity = 5  # m/s

    def update(self, steering_angle, dt):
        self.yaw += (self.velocity / self.wheelbase) * np.tan(steering_angle) * dt
        self.x += self.velocity * np.cos(self.yaw) * dt
        self.y += self.velocity * np.sin(self.yaw) * dt
        return self.x, self.y, self.yaw


def generate_reference_path():
    # x = np.linspace(0, 100, 100)
    # y = 5 * np.sin(x / 10)
    x = np.linspace(0, 100, 10)
    y = x
    return np.column_stack((x, y))


def find_nearest_point(vehicle_state, path):
    distances = np.linalg.norm(path - vehicle_state[:2], axis=1)
    return np.argmin(distances)


def main():
    # 初始化
    pid = PIDController(Kp=0.8, Ki=0.01, Kd=0.2, max_output=np.pi / 6)
    vehicle = VehicleModel()
    path = generate_reference_path()

    # 模拟参数
    dt = 0.1
    sim_time = 30
    steps = int(sim_time / dt)

    # 记录轨迹
    trajectory = np.zeros((steps, 3))

    for i in range(steps):
        # 1. 计算预瞄点（1秒后位置）
        lookahead_dist = vehicle.velocity * 1.0
        nearest_idx = find_nearest_point([vehicle.x, vehicle.y], path)
        lookahead_idx = min(nearest_idx + int(lookahead_dist / 0.5), len(path) - 1)
        target_point = path[lookahead_idx]

        # 2. 计算航向误差
        desired_yaw = np.arctan2(target_point[1] - vehicle.y, target_point[0] - vehicle.x)
        yaw_error = (desired_yaw - vehicle.yaw + np.pi) % (2 * np.pi) - np.pi

        # 3. PID控制
        steering_angle = pid.update(yaw_error, dt)
        print(steering_angle)

        # 4. 更新车辆状态
        vehicle.update(steering_angle, dt)
        trajectory[i] = [vehicle.x, vehicle.y, vehicle.yaw]

        # 可视化
        if i % 10 == 0:
            plt.clf()
            plt.plot(path[:, 0], path[:, 1], 'r--', label='参考路径')
            plt.plot(trajectory[:i, 0], trajectory[:i, 1], 'b-', label='车辆轨迹')
            plt.scatter(vehicle.x, vehicle.y, c='g', s=50)
            plt.axis('equal')
            plt.legend()
            plt.pause(0.01)

    plt.show()


if __name__ == "__main__":
    main()
