# coding=utf-8
"""路线点位规则的统一参数目录（厘米、角度）。

最终执行点采用15cm统一合并；几何浮点去重、边界闭合、转向角度等仍是
不同问题，继续使用各自阈值，不能因为数值相近而互相替代。
完整适用条件见 docs/route_point_rules.md。
"""

# 原始边界的规划副本：最后记录点回到首点附近时闭合。
BOUNDARY_CLOSURE_MERGE_CM = 30.0

# 几何接入：计算点与记录点对齐；桥头接入还受可通行检查约束。
# 预览会把坐标保存到0.001cm；距离边界不超过0.1cm只视为浮点/序列化误差，
# 不能据此把本来就在边界上的桥点判断为“区域内部”。
BOUNDARY_CLASSIFICATION_TOLERANCE_CM = 0.1
BOUNDARY_CORNER_SNAP_CM = 5.0
LANE_ENDPOINT_ANCHOR_SNAP_CM = 5.0
BRIDGE_BOUNDARY_JOIN_CM = 10.0
SAME_EDGE_DIRECT_MAX_OFFSET_CM = 2.0

# 清扫折线：同位点去重、近点上下文判断。
CLEAN_DUPLICATE_POINT_CM = 5.0
CLEAN_NEAR_POINT_CM = 30.0
CLEAN_BACKTRACK_DEG = 150.0

# 最终执行点：相邻目标点不超过15cm时，小车的10cm到点判断和停车误差会让
# 这段路线退化成“只转向、几乎不移动”。规划结果统一吸收这类短任务，原始
# 建模点不删除。阈值略高于执行到点半径，用于覆盖整数厘米取整和RTK停车误差。
EXECUTION_POINT_MERGE_CM = 15.0

# 跨区转场：去重、局部近点簇、折返和普通移动直线简化。
TRANSITION_DUPLICATE_POINT_CM = 5.0
TRANSITION_NEAR_POINT_CM = 30.0
TRANSITION_BACKTRACK_DEG = 150.0
TRANSFER_MAX_LATERAL_DEVIATION_CM = 20.0

# 停车转向阈值；现有旧简化函数也引用它，后续需要拆开职责。
TRANSFER_HARD_TURN_DEG = 55.0
CLEAN_PATH_HARD_TURN_DEG = 55.0
