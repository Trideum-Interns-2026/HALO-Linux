"""The city worlds (Worlds/city) are well formed PX4 worlds. No sim needed."""
import glob
import os
import xml.etree.ElementTree as ET

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORLDS = sorted(glob.glob(os.path.join(REPO, "Worlds", "city", "*.sdf")))
MODELS = {"city_basic": "large_basic_city", "city_cat_mouse": "my_city_v3"}
ASSETS = os.path.join(REPO, "external", "RoboticsInfrastructure", "CustomRobots")


def test_both_city_worlds_exist():
    assert [os.path.basename(w) for w in WORLDS] == ["city_basic.sdf", "city_cat_mouse.sdf"]


@pytest.mark.parametrize("path", WORLDS, ids=os.path.basename)
def test_world_is_a_complete_px4_world(path):
    name = os.path.splitext(os.path.basename(path))[0]
    world = ET.parse(path).getroot().find("world")
    assert world.get("name") == name                       # PX4: /world/<name>/ == the file name
    for tag in ("physics", "gravity", "magnetic_field", "scene", "light", "spherical_coordinates"):
        assert world.find(tag) is not None, tag
    ground = [m for m in world.findall("model") if m.get("name") == "ground_plane"]
    assert len(ground) == 1 and ground[0].find(".//collision") is not None
    includes = world.findall("include/uri")
    assert [i.text for i in includes] == ["model://%s" % MODELS[name]]


@pytest.mark.parametrize("path", WORLDS, ids=os.path.basename)
def test_world_declares_no_gazebo_plugins_of_its_own(path):
    # PX4's server.config loads the system plugins; a second copy in the world would load twice
    assert "<plugin" not in open(path).read()


@pytest.mark.parametrize("path", WORLDS, ids=os.path.basename)
def test_world_matches_px4s_default_world(path):
    def canon(e):    # ignores whitespace between elements
        return (e.tag, dict(e.attrib), (e.text or "").strip(), [canon(c) for c in e])

    px4 = "/opt/px4-gazebo/share/gz/worlds/default.sdf"
    if not os.path.exists(px4):
        pytest.skip("PX4 is not installed here")
    mine = ET.parse(path).getroot().find("world")
    ref = ET.parse(px4).getroot().find("world")
    for tag in ("physics", "gravity", "magnetic_field", "spherical_coordinates"):
        assert canon(mine.find(tag)) == canon(ref.find(tag)), tag


@pytest.mark.parametrize("name,model", sorted(MODELS.items()))
def test_model_assets_when_cloned(name, model):
    package = "city_assets" if model == "large_basic_city" else "drone_cat_mouse"
    base = os.path.join(ASSETS, package, "models", model)
    if not os.path.isdir(base):
        pytest.skip("external/RoboticsInfrastructure is not cloned (see Worlds/city/README.md)")
    assert os.path.isfile(os.path.join(base, "model.sdf"))
    assert os.path.isfile(os.path.join(base, "model.config"))
    uris = [u.text for u in ET.parse(os.path.join(base, "model.sdf")).getroot().iter("uri")]
    for uri in uris:                                        # model://<model>/meshes/...
        assert uri.startswith("model://%s/" % model)
        assert os.path.isfile(os.path.join(base, uri[len("model://%s/" % model):])), uri


def _floats(text):
    return [float(v) for v in text.split()]


def test_basic_city_keeps_its_dark_look():
    # the roads are black by design (Color.png is pure black), so the ground and sky must be dark
    # too: on PX4's light gray the city washes out into a flat plane with black walls
    world = ET.parse(os.path.join(REPO, "Worlds", "city", "city_basic.sdf")).getroot().find("world")
    ground = [m for m in world.findall("model") if m.get("name") == "ground_plane"][0]
    assert max(_floats(ground.find(".//visual//diffuse").text)[:3]) < 0.1
    assert max(_floats(world.find("scene/background").text)) < 0.1


def test_cat_mouse_city_keeps_its_green_ground_and_blue_sky():
    world = ET.parse(
        os.path.join(REPO, "Worlds", "city", "city_cat_mouse.sdf")).getroot().find("world")
    ground = [m for m in world.findall("model") if m.get("name") == "ground_plane"][0]
    r, g, b = _floats(ground.find(".//visual//diffuse").text)[:3]
    assert g > r and g > b
    r, g, b, _a = _floats(world.find("scene/background").text)
    assert b > r
