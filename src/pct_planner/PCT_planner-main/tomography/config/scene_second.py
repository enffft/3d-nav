from .scene import ScenePCD, SceneMap, SceneTrav


class SceneSecond():
    pcd = ScenePCD()
    pcd.file_name = 'second_building.pcd'

    map = SceneMap()
    # resilution是分辨率
    # map.resolution = 0.15
    map.resolution = 0.3
    map.ground_h = 0.0
    # 切片层数太少了
    map.slice_dh = 0.01

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 2.0
    trav.interval_free = 0.65
    # 坡度
    trav.slope_max = 2.0
    # 台阶
    trav.step_max = 0.00

    trav.standable_ratio = 0.001

    trav.cost_barrier = 50
    # 安全边际
    trav.safe_margin = 0.25
    # 膨胀米数
    trav.inflation = 0.2

