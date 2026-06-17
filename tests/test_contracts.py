"""Basic test scaffold for the 3D calibration portfolio project."""


def test_icp_result_contract():
    result = {
        "rotation_angles": {
            "tx": 0.003,
            "ty": -0.001,
            "tz": 0.006,
            "rx": 0.12,
            "ry": -0.05,
            "rz": 0.30,
        },
        "fitness": 0.82,
        "inlier_rmse": 0.006,
        "identity_transform": False,
    }

    assert set(result["rotation_angles"].keys()) == {"tx", "ty", "tz", "rx", "ry", "rz"}
    assert result["fitness"] >= 0.01
    assert result["inlier_rmse"] <= 0.02


def test_camera_config_contract():
    config = {"width": 848, "height": 480, "fps": 30}
    assert config["width"] > 0
    assert config["height"] > 0
    assert config["fps"] == 30
