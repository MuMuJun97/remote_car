std_msgs/Header header
int16 steering_wheel
int16 accelerator

# 修改数据类型

# 刹车 0-99.0
float32 brake

# 急刹 开关F 向上拨急刹 向下拨:正常状态
int16 emergency_braking

# 档位 开关C
# 中间-0空挡 向前拨-1前进档 向后拨-2后退档
int16 gear

# 增加请求进入转向自动控制的标志
# 开关D
# 向前拨0 向后拨1:请求进入自动驾驶
int16 Enter_steering_control
