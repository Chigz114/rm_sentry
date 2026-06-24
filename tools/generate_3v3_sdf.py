#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple
import xml.etree.ElementTree as ET


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    WORKSPACE_ROOT
    / "src"
    / "pb_rm_simulation"
    / "src"
    / "rm_simulation"
    / "pb_rm_simulation"
    / "world"
    / "RM3V3"
    / "rm3v3_sym_v1.world"
)


@dataclass
class BoxPrimitive:
    name: str
    cx: float
    cy: float
    sx: float
    sy: float
    sz: float
    rgba: Tuple[float, float, float, float]
    z_base: float = 0.0
    collide: bool = True


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    pad = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        for child in elem:
            indent_xml(child, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad


def add_box_model(world: ET.Element, box: BoxPrimitive) -> None:
    model = ET.SubElement(world, "model", {"name": box.name})
    ET.SubElement(model, "static").text = "1"
    ET.SubElement(model, "pose").text = f"{box.cx:.4f} {box.cy:.4f} {box.z_base + box.sz / 2.0:.4f} 0 0 0"

    link = ET.SubElement(model, "link", {"name": "link"})

    if box.collide:
        collision = ET.SubElement(link, "collision", {"name": "collision"})
        c_geom = ET.SubElement(collision, "geometry")
        c_box = ET.SubElement(c_geom, "box")
        ET.SubElement(c_box, "size").text = f"{box.sx:.4f} {box.sy:.4f} {box.sz:.4f}"

    visual = ET.SubElement(link, "visual", {"name": "visual"})
    v_geom = ET.SubElement(visual, "geometry")
    v_box = ET.SubElement(v_geom, "box")
    ET.SubElement(v_box, "size").text = f"{box.sx:.4f} {box.sy:.4f} {box.sz:.4f}"

    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "{:.3f} {:.3f} {:.3f} {:.3f}".format(*box.rgba)
    ET.SubElement(material, "diffuse").text = "{:.3f} {:.3f} {:.3f} {:.3f}".format(*box.rgba)


def mirror_center(cx: float, cy: float, center_x: float, center_y: float) -> Tuple[float, float]:
    return 2.0 * center_x - cx, 2.0 * center_y - cy


def build_world() -> ET.Element:
    # --------------- 基础参数（按截图第一版理解）---------------
    field_w = 12.0
    field_h = 8.0
    center_x = field_w / 2.0
    center_y = field_h / 2.0

    wall_t = 0.2
    wall_h = 0.6

    floor_z = 0.02

    # 启动区：1500 x 2000 mm（仅标注，不参与碰撞）
    spawn_w = 1.5
    spawn_h = 2.0
    spawn_z = 0.01
    mark_z_base = floor_z + 0.003

    # 控制区：3000 x 3000 mm（仅标注，不参与碰撞）
    ctrl_size = 3.0
    ctrl_z = 0.01

    # 高地：2000 x 5000 mm, 高 200 mm
    high_w = 2.0
    high_l = 5.0
    high_z = 0.2

    # 窄挡板：宽 150 mm；沿高地长度方向分段：900(200高) + 1800(400高) + 2300(200高)
    # 这里仅生成中间 1800mm 的 400mm 区段（其余由高地本体 200mm 覆盖）
    lip_w = 0.15
    lip_z_total = 0.4
    lip_mid_len = 1.8
    lip_near_center_len = 0.9

    # 左上半场高地（其对称体自动生成）
    left_high = BoxPrimitive(
        name="highland_a",
        cx=3.5,
        cy=5.5,
        sx=high_w,
        sy=high_l,
        sz=high_z,
        rgba=(0.63, 0.70, 0.79, 1.0),
    )

    # 靠近中心侧窄挡板（左高地右侧靠中心）
    left_lip = BoxPrimitive(
        name="highland_lip_a",
        cx=left_high.cx + (high_w / 2.0 - lip_w / 2.0),
        cy=(left_high.cy - high_l / 2.0) + lip_near_center_len + lip_mid_len / 2.0,
        sx=lip_w,
        sy=lip_mid_len,
        sz=lip_z_total,
        rgba=(0.83, 0.87, 0.92, 1.0),
    )

    root = ET.Element("sdf", {"version": "1.7"})
    world = ET.SubElement(root, "world", {"name": "default"})

    ET.SubElement(world, "gravity").text = "0 0 -9.81"

    physics = ET.SubElement(world, "physics", {"name": "default_physics", "default": "0", "type": "ode"})
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1"
    ET.SubElement(physics, "real_time_update_rate").text = "1000"

    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.4 0.4 0.4 1"
    ET.SubElement(scene, "background").text = "0.7 0.7 0.7 1"
    ET.SubElement(scene, "shadows").text = "1"

    # 地板
    add_box_model(
        world,
        BoxPrimitive(
            name="floor",
            cx=center_x,
            cy=center_y,
            sx=field_w,
            sy=field_h,
            sz=floor_z,
            rgba=(0.72, 0.72, 0.72, 1.0),
        ),
    )

    # 四周围墙
    walls: List[BoxPrimitive] = [
        BoxPrimitive("wall_left", wall_t / 2.0, center_y, wall_t, field_h, wall_h, (0.15, 0.15, 0.16, 1.0)),
        BoxPrimitive("wall_right", field_w - wall_t / 2.0, center_y, wall_t, field_h, wall_h, (0.15, 0.15, 0.16, 1.0)),
        BoxPrimitive("wall_bottom", center_x, wall_t / 2.0, field_w, wall_t, wall_h, (0.15, 0.15, 0.16, 1.0)),
        BoxPrimitive("wall_top", center_x, field_h - wall_t / 2.0, field_w, wall_t, wall_h, (0.15, 0.15, 0.16, 1.0)),
    ]
    for w in walls:
        add_box_model(world, w)

    # 启动区（左上 + 右下对称）
    spawn_red = BoxPrimitive(
        name="spawn_red",
        cx=0.75,
        cy=7.0,
        sx=spawn_w,
        sy=spawn_h,
        sz=spawn_z,
        rgba=(0.95, 0.20, 0.20, 1.0),
        z_base=mark_z_base,
        collide=False,
    )
    m_x, m_y = mirror_center(spawn_red.cx, spawn_red.cy, center_x, center_y)
    spawn_blue = BoxPrimitive(
        name="spawn_blue",
        cx=m_x,
        cy=m_y,
        sx=spawn_w,
        sy=spawn_h,
        sz=spawn_z,
        rgba=(0.10, 0.30, 0.95, 1.0),
        z_base=mark_z_base,
        collide=False,
    )

    # 控制区（中心）
    control_zone = BoxPrimitive(
        name="control_zone",
        cx=center_x,
        cy=center_y,
        sx=ctrl_size,
        sy=ctrl_size,
        sz=ctrl_z,
        rgba=(0.05, 0.95, 0.25, 1.0),
        z_base=mark_z_base,
        collide=False,
    )

    # 高地和窄挡板（中心对称）
    mirror_high_x, mirror_high_y = mirror_center(left_high.cx, left_high.cy, center_x, center_y)
    right_high = BoxPrimitive(
        name="highland_b",
        cx=mirror_high_x,
        cy=mirror_high_y,
        sx=left_high.sx,
        sy=left_high.sy,
        sz=left_high.sz,
        rgba=left_high.rgba,
    )

    mirror_lip_x, mirror_lip_y = mirror_center(left_lip.cx, left_lip.cy, center_x, center_y)
    right_lip = BoxPrimitive(
        name="highland_lip_b",
        cx=mirror_lip_x,
        cy=mirror_lip_y,
        sx=left_lip.sx,
        sy=left_lip.sy,
        sz=left_lip.sz,
        rgba=left_lip.rgba,
    )

    for item in [spawn_red, spawn_blue, control_zone, left_high, right_high, left_lip, right_lip]:
        add_box_model(world, item)

    # 光照
    sun = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    ET.SubElement(sun, "cast_shadows").text = "1"
    ET.SubElement(sun, "pose").text = "6 4 10 0 0 0"
    ET.SubElement(sun, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(sun, "specular").text = "0.2 0.2 0.2 1"
    ET.SubElement(sun, "direction").text = "0.2 0.2 -1"

    return root


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a center-symmetric 3v3 arena SDF world (first-pass from screenshots).")
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT),
        help="Output world file path",
    )
    args = parser.parse_args()

    root = build_world()
    indent_xml(root)

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    tree = ET.ElementTree(root)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"Generated: {args.output}")


if __name__ == "__main__":
    main()
