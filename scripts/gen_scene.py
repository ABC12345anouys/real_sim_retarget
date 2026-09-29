"""生成双臂夹爪场景 XML。

读取 configs/scene_bimanual.yaml（物体资产目录列表 + 摆放），调用
retarget/scene_builder.py 生成 assets/scenes/bimanual_gripper_scene.xml，
并做 mujoco 加载自检（nq/nv/nu/执行器顺序）。

运行（项目根目录）：python scripts/gen_scene.py [config.yaml]
"""
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from retarget import scene_builder  # noqa: E402


def main():
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, "configs", "scene_bimanual.yaml")
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    out_path = cfg["output"]
    if not os.path.isabs(out_path):
        out_path = os.path.join(ROOT, out_path)

    objects = []
    for obj in cfg.get("objects", []):
        d = obj["dir"]
        if not os.path.isabs(d):
            d = os.path.join(ROOT, d)
        objects.append({"name": obj["name"], "dir": d, "pos": tuple(obj["pos"])})

    scene = scene_builder.build_scene(objects, out_path)
    print(f"wrote {out_path} ({len(scene)} bytes)")

    import mujoco

    model = mujoco.MjModel.from_xml_path(out_path)
    print(f"nq={model.nq} nv={model.nv} nactuator={model.nu} ncam={model.ncam}")
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
             for i in range(model.nu)]
    print("actuators:", names)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    print("target body id:", bid)


if __name__ == "__main__":
    main()
