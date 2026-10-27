import numpy as np
from mzj_perception.detector import detect_marker


def test_marker_and_empty_image():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    assert detect_marker(image) == []
    image[20:60, 30:70] = (0, 255, 0)
    result = detect_marker(image)
    assert len(result) == 1
    assert result[0][0] == "demo_green_marker"
    assert result[0][2:] == (50., 40., 40., 40.)
