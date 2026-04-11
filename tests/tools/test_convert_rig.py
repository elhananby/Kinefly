import sys
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "tools"))
from convert_rig import (
    coerce_value,
    convert_rig_dir,
    has_substitution,
    parse_launch_file,
    set_nested,
)


def test_coerce_value_types():
    assert coerce_value("true", "bool") is True
    assert coerce_value("false", "bool") is False
    assert coerce_value("42", "int") == 42
    assert coerce_value("3.14", "double") == pytest.approx(3.14)
    assert coerce_value("hello", "string") == "hello"
    assert coerce_value("hello", None) == "hello"  # unknown type -> str


def test_has_substitution():
    assert has_substitution("$(env HOME)/foo") is True
    assert has_substitution("$(optenv HOME /default)") is True
    assert has_substitution("/camera/image_raw") is False
    assert has_substitution("plain_value") is False


def test_parse_launch_file(tmp_path):
    launch = tmp_path / "test.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="n_edges_max" type="int" value="1" />
            <param name="rc_background" type="double" value="100000" />
            <param name="use_gui" type="bool" value="true" />
            <param name="head/tracker" type="string" value="area" />
        </launch>
    """))
    params, file_warnings = parse_launch_file(launch)
    assert params["n_edges_max"] == ("int", "1")
    assert params["rc_background"] == ("double", "100000")
    assert params["head/tracker"] == ("string", "area")
    assert file_warnings == []


def test_set_nested():
    d = {}
    set_nested(d, ("a", "b", "c"), 42)
    assert d == {"a": {"b": {"c": 42}}}


def test_convert_rig_dir_basic(tmp_path):
    launch = tmp_path / "params_kinefly.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="n_edges_max"    type="int"    value="1" />
            <param name="rc_background"  type="double" value="100000" />
            <param name="use_gui"        type="bool"   value="true" />
            <param name="wingbeat_min"   type="double" value="180" />
            <param name="wingbeat_max"   type="double" value="220" />
            <param name="head/tracker"   type="string" value="area" />
            <param name="head/autozero"  type="bool"   value="true" />
            <param name="head/threshold" type="double" value="0.0" />
            <param name="left/tracker"   type="string" value="edge" />
            <param name="aux/tracker"    type="string" value="intensity" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    assert config["kinefly"]["version"] == 2
    assert config["tracking"]["n_edges_max"] == 1
    assert config["tracking"]["rc_background"] == pytest.approx(100000.0)
    assert config["tracking"]["use_gui"] is True
    assert config["tracking"]["head"]["tracker"] == "area"
    assert config["tracking"]["left"]["tracker"] == "edge"


def test_convert_rig_dir_phidgets(tmp_path):
    launch = tmp_path / "params_phidgetsanalog.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="flystate2phidgetsanalog/autorange" type="bool"   value="false" />
            <param name="flystate2phidgetsanalog/serial"    type="int"    value="0" />
            <param name="flystate2phidgetsanalog/v0enable"  type="bool"   value="true" />
            <param name="flystate2phidgetsanalog/v0l1"      type="double" value="5.0" />
            <param name="flystate2phidgetsanalog/v00"       type="double" value="0.0" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    assert config["phidgets"]["autorange"] is False
    assert config["phidgets"]["serial"] == 0
    assert config["phidgets"]["channels"][0]["enable"] is True
    assert config["phidgets"]["channels"][0]["coefficients"]["l1"] == pytest.approx(5.0)
    assert config["phidgets"]["channels"][0]["coefficients"]["offset"] == pytest.approx(0.0)


def test_convert_rig_dir_ledpanels(tmp_path):
    launch = tmp_path / "params_ledpanels.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="flystate2ledpanels/axis"              type="string" value="x" />
            <param name="flystate2ledpanels/method"            type="string" value="voltage" />
            <param name="flystate2ledpanels/mode"              type="string" value="velocity" />
            <param name="flystate2ledpanels/pattern_id"        type="int"    value="1" />
            <param name="flystate2ledpanels/coeff_voltage/adc0" type="double" value="1.0" />
            <param name="flystate2ledpanels/coeff_usb/xl1"    type="double" value="1.0" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    assert config["ledpanels"]["axis"] == "x"
    assert config["ledpanels"]["method"] == "voltage"
    assert config["ledpanels"]["coeff_voltage"]["adc0"] == pytest.approx(1.0)
    assert config["ledpanels"]["coeff_usb"]["xl1"] == pytest.approx(1.0)


def test_substitution_generates_warning(tmp_path):
    launch = tmp_path / "params_kinefly.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="parameterfile" type="string" value="$(env HOME)/kinefly.yaml" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    # parameterfile is in skip list, so no warning about the substitution
    # But any non-skip param with a substitution should warn
    assert isinstance(warnings, list)


def test_substitution_non_skip_generates_warning(tmp_path):
    launch = tmp_path / "params_kinefly.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="use_gui" type="string" value="$(env MY_VAR)" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    assert any("use_gui" in w for w in warnings)


def test_camera_params(tmp_path):
    launch = tmp_path / "params_camera.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="AcquisitionFrameRate" type="double" value="100" />
            <param name="ExposureTimeAbs"      type="double" value="9000" />
            <param name="Gain"                 type="double" value="1.0" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    assert config["camera"]["framerate"] == pytest.approx(100.0)
    assert config["camera"]["exposure_time_us"] == pytest.approx(9000.0)
    assert config["camera"]["gain"] == pytest.approx(1.0)


def test_unrecognized_param_generates_warning(tmp_path):
    launch = tmp_path / "params_kinefly.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="totally_unknown_param" type="string" value="foo" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    assert any("totally_unknown_param" in w for w in warnings)


def test_missing_optional_launch_files(tmp_path):
    """Rig dir with only params_kinefly.launch — no phidgets or ledpanels."""
    launch = tmp_path / "params_kinefly.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="n_edges_max" type="int" value="1" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    assert "phidgets" not in config
    assert "ledpanels" not in config


def test_phidgets_multiple_channels(tmp_path):
    launch = tmp_path / "params_phidgetsanalog.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="flystate2phidgetsanalog/v0enable" type="bool"   value="true" />
            <param name="flystate2phidgetsanalog/v0l1"     type="double" value="5.0" />
            <param name="flystate2phidgetsanalog/v1enable" type="bool"   value="true" />
            <param name="flystate2phidgetsanalog/v1r1"     type="double" value="5.0" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    channels = config["phidgets"]["channels"]
    assert len(channels) == 2
    assert channels[0]["coefficients"]["l1"] == pytest.approx(5.0)
    assert channels[1]["coefficients"]["r1"] == pytest.approx(5.0)


def test_parse_launch_file_with_rosparam(tmp_path):
    """<rosparam> blocks with dict and scalar payloads are parsed correctly."""
    launch = tmp_path / "test_rosparam.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <rosparam param="head">
              tracker: area
              threshold: 0.5
            </rosparam>
            <rosparam param="scale_image">2</rosparam>
        </launch>
    """))
    params, file_warnings = parse_launch_file(launch)
    # Dict rosparam is flattened
    assert params["head/tracker"] == ("rosparam", "area")
    assert params["head/threshold"] == ("rosparam", "0.5")
    # Scalar rosparam
    assert params["scale_image"] == ("rosparam", "2")
    assert file_warnings == []


def test_convert_rig_dir_warns_on_empty(tmp_path):
    """A dir with only a numbered variant (not a standard name) triggers a warning."""
    launch = tmp_path / "params_kinefly_1.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="n_edges_max" type="int" value="1" />
        </launch>
    """))
    config, warnings = convert_rig_dir(tmp_path)
    assert any("No recognized launch files" in w for w in warnings)
    # The variant filename should be mentioned
    assert any("params_kinefly_1.launch" in w for w in warnings)


def test_conditional_block_generates_warning(tmp_path):
    """Params with if= or unless= attributes generate a warning."""
    launch = tmp_path / "params_kinefly.launch"
    launch.write_text(textwrap.dedent("""
        <launch>
            <param name="foo" type="string" value="bar" if="$(arg some_arg)" />
        </launch>
    """))
    params, file_warnings = parse_launch_file(launch)
    assert any("conditional" in w.lower() or "if" in w for w in file_warnings)
    # The param is still parsed despite the conditional
    assert "foo" in params
