"""ROS-free helpers for Lab 7's temporary Gazebo Classic assets.

The stock robot and arena files are never rewritten. Camera configuration follows
ros-simulation/gazebo_ros_pkgs (ros2), gazebo_plugins/src/gazebo_ros_camera.cpp:
camera_name selects the image transport base and camera_info; frame_name is exact.
Gazebo stamps these messages with the sensor's simulation measurement time.
"""

import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


CAMERA_XYZ = (0.075, 0.0, 0.15)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)
CAMERA_HFOV = math.pi / 3


def numbers(values):
    return " ".join(format(value, ".15g") for value in values)


def xml_bytes(root):
    ET.indent(root, space="  ")

    return ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"


def child(parent, tag, value=None, **attributes):
    item = ET.SubElement(parent, tag, attributes)
    if value is not None:
        item.text = str(value)

    return item


def finite_vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{label} must contain {length} numbers")

    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f"{label} must contain finite numbers")

    return value


def load_task(path):
    import yaml  # rusty-linter: ignore
    task = yaml.safe_load(Path(path).read_text(encoding="utf-8"))

    return validate_task(task)


def validate_task(task):
    if not isinstance(task, dict) or task.get("schema_version") != 1 or task.get("profile") != "sim_practice":
        raise ValueError("Expected schema_version 1 and profile sim_practice")

    if task.get("marker_dictionary") != "DICT_4X4_50":
        raise ValueError("This practice model uses DICT_4X4_50")

    markers = task["markers"]
    black, board = markers["black_square_m"], markers["board_size_m"]
    finite_vector([black, board], 2, "Marker sizes")
    if not 0 < black < board:
        raise ValueError("The board must include a white margin around its black square")

    placements, seen = markers["placements"], set()
    if not isinstance(placements, dict) or not placements:
        raise ValueError("At least one marker placement is required")

    labels = task["marker_id_to_fruit"]
    for station, placement in placements.items():
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", station):
            raise ValueError("Station names must be simple identifiers")

        marker_id = placement["id"]
        if type(marker_id) is not int or not 0 <= marker_id < 50 or marker_id in seen:
            raise ValueError("Each placement needs a unique DICT_4X4_50 ID")

        seen.add(marker_id)
        if not isinstance(labels.get(marker_id), str) or not labels[marker_id]:
            raise ValueError(f"Missing label for marker {marker_id}")

        xyz = finite_vector(placement["position_arena_m"], 3, "Marker position")
        finite_vector([placement["facing_yaw_arena_rad"]], 1, "Marker facing yaw")
        if xyz[2] <= board / 2:
            raise ValueError("A marker board must be above the floor")

    finite_vector(task["start"]["pose_arena"], 3, "SIM start pose")

    return task


def arena_pose_to_world(geometry, pose):
    x, y, yaw = finite_vector(pose, 3, "Arena pose")
    transform = geometry["world_from_arena"]
    tx, ty, _ = transform["translation_m"]
    angle = transform["yaw_rad"]
    c, s = math.cos(angle), math.sin(angle)

    return tx + c * x - s * y, ty + s * x + c * y, yaw + angle


def _tiny_inertia(link, mass):
    inertial = child(link, "inertial")
    child(inertial, "mass", mass)
    inertia = child(inertial, "inertia")
    for name in ("ixx", "iyy", "izz"):
        child(inertia, name, mass * 0.001)

    for name in ("ixy", "ixz", "iyz"):
        child(inertia, name, 0)


def prepare_robot(sdf_xml, urdf_xml, robot_name, *, camera_transport="raw"):
    """Return an augmented SDF and URDF; existing robot collision shapes stay intact."""
    if not re.fullmatch(r"tb_[A-Za-z0-9_]+", robot_name):
        raise ValueError("robot_name must be the full tb_<NAME> namespace")

    if camera_transport not in ("raw", "h264"):
        raise ValueError("camera_transport must be raw or h264")

    sdf, urdf = ET.fromstring(sdf_xml), ET.fromstring(urdf_xml)
    model = sdf.find("model")
    if model is None or model.find("link[@name='base_link']") is None or urdf.find("link[@name='base_link']") is None:
        raise ValueError("Expected a stock Burger model with base_link")

    if model.find("link[@name='camera_link']") is not None or urdf.find("link[@name='camera_link']") is not None:
        raise ValueError("The supplied model already has camera_link")

    drive = model.find("plugin[@filename='libgazebo_ros_diff_drive.so']")
    if drive is None:
        raise ValueError("Expected the Gazebo Classic TurtleBot3 drive plugin")

    for tag in ("odometry_frame", "robot_base_frame", "frame_name"):
        for element in model.iter(tag):
            frame = (element.text or "").strip().lstrip("/")
            if frame and frame != "world" and not frame.startswith(robot_name + "/"):
                element.text = f"{robot_name}/{frame}"

    for plugin in model.iter("plugin"):
        ros = plugin.find("ros")
        if ros is None:
            ros = child(plugin, "ros")

        namespace = ros.find("namespace")
        if namespace is None:
            namespace = child(ros, "namespace")

        namespace.text = f"/{robot_name}"
    odometry = drive.find("odometry_source")
    if odometry is None:
        odometry = child(drive, "odometry_source")

    odometry.text = "0"
    for sensor in model.iter("sensor"):
        if sensor.get("type") in ("ray", "gpu_ray"):
            visible = sensor.find("visualize")
            if visible is None:
                visible = child(sensor, "visualize")

            visible.text = "false"

    # in Gazebo 11, SDF 1.7 supports explicit pose reference frames
    sdf.set("version", "1.7")
    camera_link = child(model, "link", name="camera_link")
    child(camera_link, "pose", numbers([*CAMERA_XYZ, 0, 0, 0]), relative_to="base_link")
    _tiny_inertia(camera_link, 0.001)
    sensor = child(camera_link, "sensor", name="lab7_rgb", type="camera")
    child(sensor, "always_on", "true")
    child(sensor, "update_rate", 15)
    child(sensor, "visualize", "false")
    camera = child(sensor, "camera")
    child(camera, "horizontal_fov", numbers([CAMERA_HFOV]))
    image = child(camera, "image")
    for key, value in (("width", 640), ("height", 480), ("format", "R8G8B8")):
        child(image, key, value)

    clip = child(camera, "clip")
    child(clip, "near", 0.03)
    child(clip, "far", 10)
    plugin = child(sensor, "plugin", name="lab7_camera", filename="libgazebo_ros_camera.so")
    ros = child(plugin, "ros")
    child(ros, "namespace", f"/{robot_name}")
    if camera_transport == "h264":
        child(ros, "remapping", "camera/image_raw/ffmpeg:=camera/video_h264")
        child(ros, "parameters", Path(__file__).resolve().parents[1] / "config/static/sim_camera.yaml")
    else:
        child(ros, "remapping", "camera/image_raw:=camera/video_raw")

    child(plugin, "camera_name", "camera")
    child(plugin, "frame_name", f"{robot_name}/camera_optical_frame")
    # an optical frame is a coordinate convention, not another physical body
    optical = child(model, "frame", name="camera_optical_frame", attached_to="camera_link")
    child(optical, "pose", numbers([0, 0, 0, *OPTICAL_RPY]), relative_to="camera_link")
    for name, parent, link, xyz, rpy in (
        ("camera_joint", "base_link", "camera_link", CAMERA_XYZ, (0, 0, 0)),
        ("camera_optical_joint", "camera_link", "camera_optical_frame", (0, 0, 0), OPTICAL_RPY),
    ):
        if link == "camera_link":
            joint = child(model, "joint", name=name, type="fixed")
            child(joint, "parent", parent)
            child(joint, "child", link)

        child(urdf, "link", name=link)
        joint = child(urdf, "joint", name=name, type="fixed")
        child(joint, "parent", link=parent)
        child(joint, "child", link=link)
        child(joint, "origin", xyz=numbers(xyz), rpy=numbers(rpy))
    # in Humble, spawn_entity.py parses file text as a Unicode XML string
    # a Unicode string containing an XML encoding declaration is rejected

    ET.indent(sdf, space="  ")
    ET.indent(urdf, space="  ")

    return ET.tostring(sdf, encoding="unicode"), ET.tostring(urdf, encoding="unicode")


def marker_model_name(marker_id):
    # distinct model names prevent collision with older many-box Gazebo assets
    return f"lab7_textured_aruco_{marker_id}"


def marker_model(marker_id, grid, black_side, board_side):
    """One textured plane in place of many black boxes. Front normal is local +X.

    Lab 7's world placement yaws the model toward the robot. The visual's
    roll=+90, yaw=+90 make its textured +Z face point along model +X;
    image-right follows model +Y and image-up follows model +Z. For the
    rear-wall yaw (-90 degrees), this is exactly the successful Phase 2
    upright sign orientation (roll=+90, no yaw).
    """
    if len(grid) != 6 or any(len(row) != 6 or any(v not in (0, 255) for v in row) for row in grid):
        raise ValueError("DICT_4X4_50 artwork must be a 6 by 6 black/white grid")

    if not 0 < black_side < board_side:
        raise ValueError("The image requires a black square and white margin")

    name = marker_model_name(marker_id)
    root = ET.Element("sdf", version="1.6")
    root.append(ET.Comment(" Generated by scripts/build_markers.py; SIM practice only. "))
    model = child(root, "model", name=name)
    child(model, "static", "true")
    link = child(model, "link", name="board")
    visual = child(link, "visual", name="single_textured_face")
    child(visual, "pose", numbers([0, 0, 0, math.pi / 2, 0, math.pi / 2]))
    child(visual, "cast_shadows", "false")
    plane = child(child(visual, "geometry"), "plane")
    child(plane, "normal", "0 0 1")
    child(plane, "size", numbers([board_side, board_side]))
    script = child(child(visual, "material"), "script")
    child(script, "uri", f"model://{name}/materials/scripts")
    child(script, "uri", f"model://{name}/materials/textures")
    child(script, "name", f"Lab7Textured/ArUco{marker_id}")

    return xml_bytes(root)


def add_markers(world_xml, geometry, task, model_dir):
    """Use the same model:// Ogre-texture loading mechanism proven in Phase 3.

    Models and textures are Lab 7-local. The original arena world is read only.
    """
    validate_task(task)
    root = ET.fromstring(world_xml)
    world = root.find("world")
    if world is None:
        raise ValueError("Arena file contains no SDF world")

    for station, placement in task["markers"]["placements"].items():
        marker_id = placement["id"]
        name = marker_model_name(marker_id)
        directory = Path(model_dir) / name
        expected = (
            directory / "model.config",
            directory / "model.sdf",
            directory / "materials" / "scripts" / f"aruco_{marker_id}.material",
            directory / "materials" / "textures" / f"aruco_{marker_id}.png",
        )
        for path in expected:
            if not path.is_file():
                raise FileNotFoundError(f"Missing {path}; run make markers")

        x, y, z = placement["position_arena_m"]
        wx, wy, yaw = arena_pose_to_world(
            geometry, (x, y, placement["facing_yaw_arena_rad"]))
        include = child(world, "include")
        child(include, "uri", f"model://{name}")
        child(include, "name", f"lab7_marker_{station}")
        child(include, "pose", numbers([wx, wy, z, 0, 0, yaw]))

    return xml_bytes(root).decode()
