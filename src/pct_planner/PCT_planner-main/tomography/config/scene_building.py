from .scene import ScenePCD, SceneMap, SceneTrav


class SceneBuilding():
    pcd = ScenePCD()
    pcd.file_name = 'MultifloorBuilding.pcd'

    # 初始化地图场景
    map = SceneMap()
    # resilution是分辨率
    map.resolution = 0.25
    # map.resolution = 0.02
    map.ground_h = 0.0
    map.slice_dh = 0.25  
    # 切片层数太少了
    # map.slice_dh = 0.05

    # 初始化场景可通行性分析对象
    trav = SceneTrav()
    # 设置内核大小，用于形态学操作
    trav.kernel_size = 7
    # 设置最小间隔，单位为米
    trav.interval_min = 0.50
    # 设置机器人正常工作高度，单位为米
    trav.interval_free = 0.65

    trav.slope_max = 0.40   
    trav.step_max = 0.17
    # 设置可站立区域的最小比例
    trav.standable_ratio = 0.01
    # 设置障碍物成本值
    trav.cost_barrier = 50.0
    # 设置安全边距，单位为米
    trav.safe_margin = 0.2
    # 设置膨胀半径，单位为米
    trav.inflation = 0.1


