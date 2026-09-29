"""场景拼装器：从 Panda 臂 + 平行夹爪 + 物体资产生成双臂 MJCF 场景。

被 scripts/gen_scene.py 调用（读取 yaml 场景配置）。约定：

  - 机器人资产：`assets/robot/panda/`（robopal 版 Panda.xml + meshes/）、
    `assets/robot/gripper.xml`（平行夹爪）。左右实例加 `l_`/`r_` 前缀，
    执行器顺序：[l_actuator1..7, l_gripper, r_actuator1..7, r_gripper] = 16 维。
  - 物体资产：`assets/objects/<name>/object.xml`，内含
    `<mujoco><asset>…</asset><worldbody><body name="<name>" …>…`，
    底部中心为原点、底面 z=0。MJCF 的 `<include>` 无法修改被引用文件里
    的 body 属性，因此这里采用「解析 object.xml 后内联」的策略：把物体
    的 asset 片段（材质/mesh 名加物体前缀防重名）合并进场景 asset，
    并把其 worldbody 下的 body 原样内联到场景 worldbody、由调用方给定 pos。

R0 阶段物体只支持内置 box（无 mesh），接口已按通用物体留好。
"""

import os
import re
import xml.etree.ElementTree as ET

ROBOT_DIR = os.path.join("assets", "robot")
PANDA_XML = os.path.join(ROBOT_DIR, "panda", "Panda.xml")
GRIPPER_XML = os.path.join(ROBOT_DIR, "gripper.xml")

TABLE_POS = (0.0, 0.0, 0.45)      # box halfsize -> 桌顶 z = 0.50
TABLE_SIZE = (0.45, 0.40, 0.05)
TABLE_TOP_Z = TABLE_POS[2] + TABLE_SIZE[2]

PANDA_BASE = {
    "l": {"pos": (-0.62, 0.0, 0.0), "quat": "1 0 0 0"},
    "r": {"pos": (0.62, 0.0, 0.0), "quat": "0 0 0 1"},  # yaw pi: 面向 -x
}

# 无碰撞 home 位姿（关节空间）。若 tune_home.py 报碰撞，改这里重生成。
ARM_HOME = {
    "l": [-0.5841, -0.3123, 0.2378, -1.3030, -0.9488, 1.8973, 0.4535],
    "r": [-0.5929, -0.3362, 0.1876, -1.3412, -0.8648, 2.2275, 0.3358],
}
GRIPPER_HOME = 0.03               # home 时夹爪半开（间隙 ~64mm）

ARM_KP = [300.0, 300.0, 300.0, 300.0, 100.0, 100.0, 100.0]
ARM_FORCE = [(87.0, 87.0)] * 5 + [(12.0, 12.0)] * 2

HEAD_CAMERA = {"pos": "0 0 1.6", "xyaxes": "1 0 0 0 1 0", "fovy": "60"}


def serialize(elem, indent="  "):
    from xml.dom import minidom

    rough = ET.tostring(elem, encoding="unicode")
    pretty = minidom.parseString(rough).toprettyxml(indent=indent)
    lines = [ln for ln in pretty.splitlines() if ln.strip() and not ln.startswith("<?")]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Panda 臂处理（加前缀，motor -> position）
# ---------------------------------------------------------------------------
PANDA_CLASSES = ("panda", "panda/visual", "panda/collision")
PANDA_MATERIALS = ("white", "off_white", "dark_grey", "green", "light_blue")


def _prefix_tree(root, prefix, named, mesh_names, materials, classes,
                 ref_attrs=("mesh", "material", "joint")):
    """通用加前缀：元素名 + 名字引用属性 + default class。"""
    for elem in root.iter():
        if "name" in elem.attrib and named.match(elem.attrib["name"]):
            elem.attrib["name"] = prefix + elem.attrib["name"]
        elif elem.tag == "material" and "name" in elem.attrib:
            elem.attrib["name"] = prefix + elem.attrib["name"]
        for attr in ref_attrs:
            if attr in elem.attrib and (
                elem.attrib[attr] in mesh_names
                or (attr == "material" and elem.attrib[attr] in materials)
            ):
                elem.attrib[attr] = prefix + elem.attrib[attr]
        for attr in ("class", "childclass"):
            if elem.attrib.get(attr) in classes:
                elem.attrib[attr] = prefix + elem.attrib[attr]


def process_arm(prefix, mesh_dir):
    """prefix: 'l_' / 'r_'。mesh_dir: 场景 XML 到 meshes 的相对路径前缀。
    返回 dict(asset/default/worldbody/actuator 字符串)。"""
    tree = ET.parse(PANDA_XML)
    root = tree.getroot()

    mesh_names = set()
    for mesh in root.iter("mesh"):
        mesh.attrib["file"] = mesh_dir + mesh.attrib["file"]
        if "name" in mesh.attrib:
            mesh_names.add(mesh.attrib["name"])
        else:
            base = os.path.splitext(os.path.basename(mesh.attrib["file"]))[0]
            mesh.attrib["name"] = prefix + base
            mesh_names.add(base)

    named = re.compile(
        r"^(link\d+(?:_c\d*|_c|_\d+)?|joint\d+|attachment(?:_site)?|actuator\d+)$")
    _prefix_tree(root, prefix, named, mesh_names, PANDA_MATERIALS, PANDA_CLASSES)

    asset = root.find("asset")
    default = root.find("default")
    worldbody = root.find("worldbody")
    side = prefix.rstrip("_")
    base = ET.Element("body", name=f"{side}_base",
                      pos=" ".join(str(v) for v in PANDA_BASE[side]["pos"]),
                      quat=PANDA_BASE[side]["quat"])
    for child in list(worldbody):
        worldbody.remove(child)
        base.append(child)
    worldbody.append(base)

    old_act = root.find("actuator")
    joint_ranges = {}
    for j in root.iter("joint"):
        if "name" in j.attrib:
            joint_ranges[j.attrib["name"]] = j.attrib.get("range", "-2.8973 2.8973")
    actuator = ET.Element("actuator")
    for i in range(1, 8):
        lo, hi = (float(x) for x in joint_ranges[prefix + f"joint{i}"].split())
        ET.SubElement(
            actuator, "position",
            name=prefix + f"actuator{i}", joint=prefix + f"joint{i}",
            ctrlrange=f"{lo} {hi}", kp=str(ARM_KP[i - 1]),
            forcerange=f"{-ARM_FORCE[i-1][0]} {ARM_FORCE[i-1][1]}",
        )
    root.remove(old_act)

    return {
        "asset": "\n".join(serialize(c) for c in list(asset)),
        "default": "\n".join(serialize(c) for c in list(default)),
        "worldbody": "\n".join(serialize(c) for c in list(worldbody)),
        "actuator": "\n".join(serialize(c) for c in list(actuator)),
    }


# ---------------------------------------------------------------------------
# 平行夹爪处理（加前缀，挂到 attachment body 内）
# ---------------------------------------------------------------------------
GRIPPER_CLASSES = ("gripper",)
GRIPPER_NAMED = re.compile(
    r"^(gripper_mount|finger1|finger2|finger_joint|finger_mimic_joint|"
    r"gripper_sync|gripper_site|gripper_base_geom|finger1_pad|finger2_pad|gripper)$")


def process_gripper(prefix):
    """返回 dict(worldbody/equality/actuator 字符串)，body 可整体插入 attachment。"""
    tree = ET.parse(GRIPPER_XML)
    root = tree.getroot()
    _prefix_tree(root, prefix, GRIPPER_NAMED, set(), set(), GRIPPER_CLASSES,
                 ref_attrs=("joint1", "joint2", "joint"))
    worldbody = "\n".join(serialize(c) for c in list(root.find("worldbody")))
    equality = "\n".join(serialize(c) for c in list(root.find("equality")))
    actuator = "\n".join(serialize(c) for c in list(root.find("actuator")))
    return {"worldbody": worldbody, "equality": equality, "actuator": actuator}


# ---------------------------------------------------------------------------
# 物体资产内联（object.xml -> asset 片段 + worldbody 片段）
# ---------------------------------------------------------------------------
def load_object(object_dir, obj_prefix):
    """解析 assets/objects/<name>/object.xml。

    返回 (asset_xml, worldbody_xml, body_name)。asset 内所有 material/mesh
    名加 `obj_prefix` 前缀防重名，worldbody 内 body 保持原名（由调用方摆位）。
    """
    path = os.path.join(object_dir, "object.xml")
    tree = ET.parse(path)
    root = tree.getroot()

    named = re.compile(r".*")
    mesh_names = set()
    asset = root.find("asset")
    if asset is not None:
        for mesh in asset.iter("mesh"):
            if "name" in mesh.attrib:
                mesh_names.add(mesh.attrib["name"])
        for elem in asset.iter():
            if "name" in elem.attrib and elem.tag in ("material", "mesh", "texture"):
                elem.attrib["name"] = obj_prefix + elem.attrib["name"]
        asset_xml = "\n".join(serialize(c) for c in list(asset))
    else:
        asset_xml = ""

    worldbody = root.find("worldbody")
    bodies = list(worldbody)
    assert len(bodies) >= 1, f"{path}: worldbody 为空"
    body = bodies[0]
    # worldbody 内的 material/mesh 引用加前缀
    for elem in body.iter():
        for attr in ("material", "mesh"):
            if elem.attrib.get(attr) in mesh_names:
                elem.attrib[attr] = obj_prefix + elem.attrib[attr]
    body_xml = "\n".join(serialize(c) for c in bodies)
    return asset_xml, body_xml, body.attrib["name"]


# ---------------------------------------------------------------------------
# 场景拼装
# ---------------------------------------------------------------------------
def build_scene(objects, out_path, arm_home=None):
    """objects: list of dict(name=资产目录名, dir=资产目录路径, pos=(x,y,z))。
    返回生成的 XML 文本。第一个物体的 body 即 env 的目标物体（默认名 target）。"""
    arm_home = arm_home or ARM_HOME
    # 场景 XML 位于 assets/scenes/，mesh 相对路径指向 assets/robot/panda/
    mesh_dir = "../robot/panda/"

    arm = {"l": process_arm("l_", mesh_dir), "r": process_arm("r_", mesh_dir)}
    grip = {"l": process_gripper("l_"), "r": process_gripper("r_")}

    obj_assets, obj_bodies = [], []
    for obj in objects:
        asset_xml, body_xml, body_name = load_object(obj["dir"], obj["name"] + "_")
        # 摆位：object.xml 约定底部中心为原点、底面 z=0，直接给 body pos
        root = ET.fromstring(f"<worldbody>{body_xml}</worldbody>")
        root[0].attrib["pos"] = " ".join(f"{v}" for v in obj["pos"])
        obj_assets.append(asset_xml)
        obj_bodies.append(serialize(root[0]))

    arm_home_l = " ".join(str(v) for v in arm_home["l"])
    arm_home_r = " ".join(str(v) for v in arm_home["r"])
    # 关节顺序：目标物体 freejoint(7) | l_arm7 l_f1 l_f2 | r_arm7 r_f1 r_f2
    qpos = " ".join([
        f"0 0 {TABLE_TOP_Z + 0.03} 1 0 0 0",
        arm_home_l, str(GRIPPER_HOME), str(-GRIPPER_HOME),
        arm_home_r, str(GRIPPER_HOME), str(-GRIPPER_HOME),
    ])
    ctrl = " ".join([arm_home_l, str(GRIPPER_HOME), arm_home_r, str(GRIPPER_HOME)])

    def mount_gripper(arm_xml, grip_xml, side):
        pat = re.compile(r'(<body name="' + side + r'_attachment"[^>]*>)')
        out, n = pat.subn(lambda m: m.group(1) + "\n" + grip_xml["worldbody"],
                          arm_xml, count=1)
        assert n == 1, f"attachment body not found for {side}"
        return out

    worldbody_arms = (
        mount_gripper(arm["l"]["worldbody"], grip["l"], "l"),
        mount_gripper(arm["r"]["worldbody"], grip["r"], "r"),
    )
    actuator_order = (
        arm["l"]["actuator"], grip["l"]["actuator"],
        arm["r"]["actuator"], grip["r"]["actuator"],
    )

    scene = f"""<mujoco model="bimanual_gripper">
  <compiler angle="radian" autolimits="true"/>

  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.5 0.5 0.5" specular="0.15 0.15 0.15"/>
    <quality shadowsize="1024"/>
    <rgba haze="0.35 0.35 0.35 1"/>
  </visual>

  <option integrator="implicitfast" solver="Newton" iterations="200" ls_iterations="50"
    ls_tolerance="1e-4" cone="elliptic" impratio="10" timestep="0.002"/>

  <size memory="100M"/>

  <default>
    <geom solimp="0.9 0.95 0.001 0.5 2" solref="0.01 1"/>
    <default class="table">
      <geom type="box" material="table_wood" friction="1.0 0.3 0.05"/>
    </default>
{arm['l']['default']}
{arm['r']['default']}
  </default>

  <asset>
    <material name="table_wood" rgba="0.6 0.45 0.3 1" specular="0.2" shininess="0.1"/>
{chr(10).join(a for a in obj_assets if a)}
{arm['l']['asset']}
{arm['r']['asset']}
  </asset>

  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" pos="0 0 0" rgba="0.85 0.85 0.87 1" friction="1.2 0.3 0.05"/>
    <geom name="table" class="table" size="{' '.join(str(v) for v in TABLE_SIZE)}" pos="{' '.join(str(v) for v in TABLE_POS)}"/>

{chr(10).join(obj_bodies)}

    <light name="key_light" directional="true" pos="-0.5 -0.7 2.2"
      dir="0.25 0.35 -1" diffuse="0.5 0.5 0.5" specular="0.1 0.1 0.1"/>
    <light name="fill_light" directional="true" pos="0.6 0.5 1.8"
      dir="-0.3 -0.25 -1" diffuse="0.3 0.3 0.3" specular="0.05 0.05 0.05"/>

    <camera name="head" pos="{HEAD_CAMERA['pos']}" xyaxes="{HEAD_CAMERA['xyaxes']}" fovy="{HEAD_CAMERA['fovy']}"/>
{chr(10).join(worldbody_arms)}
  </worldbody>

  <equality>
{grip['l']['equality']}
{grip['r']['equality']}
  </equality>

  <actuator>
{chr(10).join(actuator_order)}
  </actuator>

  <keyframe>
    <key name="home" qpos="{qpos}" ctrl="{ctrl}"/>
  </keyframe>
</mujoco>
"""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(scene)
    return scene
