import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#plt.switch_backend('TkAgg')

# 定义模糊变量
distance = ctrl.Antecedent(np.arange(0, 3.51, 0.01), 'distance')
angle = ctrl.Antecedent(np.arange(-180, 181, 1), 'angle')
goal_angle = ctrl.Antecedent(np.arange(-180, 181, 1), 'goal_angle')

angular_velocity = ctrl.Consequent(np.arange(-1.82, 1.83, 0.01), 'angular_velocity')
velocity = ctrl.Consequent(np.arange(0, 0.27, 0.01), 'velocity')

# 定义模糊集
distance['close'] = fuzz.trimf(distance.universe, [0, 0.5, 1.0])
distance['medium'] = fuzz.trimf(distance.universe, [0.5, 1.25, 2.0])
distance['far'] = fuzz.trimf(distance.universe, [1.5, 2.5, 3.51])

angle['right'] = fuzz.trimf(angle.universe, [-180, -180, 0])
angle['front'] = fuzz.trimf(angle.universe, [-45, 0, 45])
angle['left'] = fuzz.trimf(angle.universe, [0, 180, 180])

goal_angle['left'] = fuzz.trimf(goal_angle.universe, [0, 180, 180])
goal_angle['front'] = fuzz.trimf(goal_angle.universe, [-45, 0, 45])
goal_angle['right'] = fuzz.trimf(goal_angle.universe, [-180, -180, 0])

angular_velocity['large_right'] = fuzz.trimf(angular_velocity.universe, [-1.82, -1.82, -0.9])
angular_velocity['small_right'] = fuzz.trimf(angular_velocity.universe, [-1.2, -0.6, 0])
angular_velocity['straight'] = fuzz.trimf(angular_velocity.universe, [-0.25, 0, 0.25])
angular_velocity['small_left'] = fuzz.trimf(angular_velocity.universe, [0, 0.6, 1.2])
angular_velocity['large_left'] = fuzz.trimf(angular_velocity.universe, [0.9, 1.82, 1.82])

velocity['slow'] = fuzz.trimf(velocity.universe, [0, 0, 0.13])
velocity['medium'] = fuzz.trimf(velocity.universe, [0.1, 0.15, 0.2])
velocity['fast'] = fuzz.trimf(velocity.universe, [0.15, 0.26, 0.26])

# 定义模糊规则
rule1 = ctrl.Rule(distance['close'] & angle['left'], (angular_velocity['large_right'], velocity['slow']))  # Mamdani 推理方法
rule2 = ctrl.Rule(distance['close'] & angle['front'] & goal_angle['left'], (angular_velocity['small_left'], velocity['slow']))
rule3 = ctrl.Rule(distance['close'] & angle['front'] & goal_angle['right'], (angular_velocity['small_right'], velocity['slow']))
rule4 = ctrl.Rule(distance['close'] & angle['front'] & goal_angle['front'], (angular_velocity['straight'], velocity['slow']))
rule5 = ctrl.Rule(distance['close'] & angle['right'], (angular_velocity['large_left'], velocity['slow']))

rule6 = ctrl.Rule(distance['medium'] & angle['left'], (angular_velocity['small_right'], velocity['medium']))
rule7 = ctrl.Rule(distance['medium'] & angle['front'] & goal_angle['left'], (angular_velocity['small_left'], velocity['medium']))
rule8 = ctrl.Rule(distance['medium'] & angle['front'] & goal_angle['right'], (angular_velocity['small_right'], velocity['medium']))
rule9 = ctrl.Rule(distance['medium'] & angle['front'] & goal_angle['front'], (angular_velocity['straight'], velocity['medium']))
rule10 = ctrl.Rule(distance['medium'] & angle['right'], (angular_velocity['small_left'], velocity['medium']))

rule11 = ctrl.Rule(distance['far'] & angle['left'], (angular_velocity['straight'], velocity['fast']))


# 创建控制系统
avoidance_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11])

# 创建模糊控制器实例

avoidance = ctrl.ControlSystemSimulation(avoidance_ctrl)        # 质心法
# 定义一个函数来获取避障输出

# distance.view()
# angle.view()
# turn_angle.view()
# velocity.view()

def get_avoidance_output(distance_input, angle_input, goal_angle_input):
    avoidance.input['distance'] = distance_input
    avoidance.input['angle'] = angle_input
    avoidance.input['goal_angle'] = goal_angle_input
    avoidance.compute()
    return avoidance.output['velocity'], avoidance.output['angular_velocity']
# 示例：使用模糊控制器

# distance_input = 1.4
# angle_input = 30
# goal_angle_input = -90
#
# velocity_output, turn_angle_output = get_avoidance_output(distance_input, angle_input, goal_angle_input)
# print(f"Turn angle: {turn_angle_output:.2f} rad/s, Velocity: {velocity_output:.2f} m/s")

#
# # 采样输入空间
# distance_samples = np.linspace(0, 3.5, 35)
# angle_samples = np.linspace(-180, 180, 36)
#
# # 计算对应的输出
# turn_angle_outputs = np.zeros((len(distance_samples), len(angle_samples)))
# velocity_outputs = np.zeros((len(distance_samples), len(angle_samples)))
#
# for i, d in enumerate(distance_samples):
#     for j, a in enumerate(angle_samples):
#         turn_angle, velocity = get_avoidance_output(d, a)
#         turn_angle_outputs[i, j] = turn_angle
#         velocity_outputs[i, j] = velocity
#
# # 创建3D图像
# fig = plt.figure(figsize=(12, 6))
#
# # 绘制转向角度曲面图
# ax1 = fig.add_subplot(121, projection='3d')
# D, A = np.meshgrid(distance_samples, angle_samples, indexing='ij')
#
# ax1.plot_surface(D, A, turn_angle_outputs, cmap='viridis')
# ax1.set_xlabel('Distance (m)')
# ax1.set_ylabel('Angle (rad)')
# ax1.set_zlabel('Turn Angle (rad)')
# ax1.set_title('Output angle')
#
# # 绘制速度曲面图
# ax2 = fig.add_subplot(122, projection='3d')
# ax2.plot_surface(D, A, velocity_outputs, cmap='viridis')
# ax2.set_xlabel('Distance (m)')
# ax2.set_ylabel('Angle (rad)')
# ax2.set_zlabel('Velocity (m/s)')
# ax2.set_title('Output velocity')
#
#
# # 显示图像
# plt.show()
