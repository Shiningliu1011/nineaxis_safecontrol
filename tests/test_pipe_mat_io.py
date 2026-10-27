import numpy as np
import pytest

from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat, save_mat


def test_mat_struct_round_trip_preserves_coordinates_and_units(tmp_path):
    filename = tmp_path / "path.mat"
    trajectory = {
        "tcpPoints": np.array([[20.0, 0.0, 0.0], [20.0, 0.0, 10.0]]),
        "lengthUnit": "mm",
        "pathDefinition": {"type": "c2-spline"},
        "optionalLimit": None,
    }
    save_mat(filename, {"trajectory": trajectory})
    restored = load_mat(filename)["trajectory"]
    np.testing.assert_array_equal(restored["tcpPoints"], trajectory["tcpPoints"])
    assert restored["lengthUnit"] == "mm"
    assert restored["pathDefinition"]["type"] == "c2-spline"
    assert np.size(restored["optionalLimit"]) == 0


def test_mat_input_requires_existing_mat_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_mat(tmp_path / "missing.mat")
    filename = tmp_path / "input.csv"
    filename.write_text("x,y,z\n1,2,3\n")
    with pytest.raises(ValueError, match="MAT"):
        load_mat(filename)
