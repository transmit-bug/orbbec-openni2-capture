"""在 Blender 里打开 wheal 导出的 PLY —— 打开就能看见，不用手动配。

用法（GUI）::

    blender --factory-startup --python scripts/view_in_blender.py -- output/scene_indoor/mesh.ply
    # 想在 PLY 旁边存一份 .blend（默认不存）：末尾加 --save

为什么需要这么一个脚本。裸 `wm.ply_import` 之后 Blender 里看到的是一团黑，而黑
**同时**是“数据没问题但没设对”和“数据真的坏了”的样子——从屏幕上分不出来。这里
把三件必须做的事一次做掉：

1. **着色模式切到 Workbench + VERTEX。** 默认是 EEVEE + 材质着色，而导入的网格
   没有材质，那就是黑的。Workbench 不需要灯光，且能直接读顶点色属性。
2. **法向朝观察者。** wheal 导出时已经把法向翻向相机了；这里再设一次
   ``use_backface_culling = False``，保证从任何一侧转过去都看得见。
3. **把视角摆到一个能看出几何的角度。** 这个模型是一张几乎平的 2.5D 曲面，
   正对着看什么也看不出来——那些行相关伪影必须斜着看才现形。

同时会在 PLY 旁边存一份 ``.blend``，下次双击就能开，不用再跑这个脚本。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import bpy
import mathutils

#: 观察方向：从相机的左上前方斜看。正对（-Z）会让这张近乎平面的网格看起来
#: 一片平坦，斜 45° 才能同时看到整体轮廓和表面的起伏。
VIEW_DIRECTION = mathutils.Vector((0.55, -0.75, -0.62)).normalized()


def _arguments() -> list[str]:
    """``--`` 之后的参数。Blender 自己会吃掉前面的，必须靠这个分隔符。"""
    if "--" not in sys.argv:
        return []
    return sys.argv[sys.argv.index("--") + 1 :]


def _clear_default_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def _import_ply(path: Path):
    if not path.is_file():
        raise SystemExit(f"找不到 PLY: {path}")
    bpy.ops.wm.ply_import(filepath=str(path))
    selected = [obj for obj in bpy.context.selected_objects if obj.type == "MESH"]
    if not selected:
        raise SystemExit(f"导入后没有网格对象: {path}")
    obj = selected[0]
    print(f"[wheal] 导入 {path.name}: {len(obj.data.vertices)} 顶点 / {len(obj.data.polygons)} 面")
    return obj


def _bounds(obj) -> tuple[mathutils.Vector, mathutils.Vector]:
    corners = [obj.matrix_world @ mathutils.Vector(c) for c in obj.bound_box]
    low = mathutils.Vector((min(c[i] for c in corners) for i in range(3)))
    high = mathutils.Vector((max(c[i] for c in corners) for i in range(3)))
    return low, high


def _add_vertex_colour_material(obj) -> bool:
    """给网格挂一个读 ``Col`` 属性的材质，让 EEVEE/Cycles 渲染也有颜色。

    Workbench 的 VERTEX 模式不需要材质，但用户一旦按 F12 或切到 EEVEE，
    没有材质就又是黑的。这里顺手把这个坑填上。
    """
    attribute = next(iter(obj.data.color_attributes), None)
    if attribute is None:
        print("[wheal] 网格没有顶点色属性；EEVEE 里会用中性灰")
        return False

    material = bpy.data.materials.new("wheal_vertex_colour")
    material.use_nodes = True
    tree = material.node_tree
    nodes = tree.nodes
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    shader = nodes.new("ShaderNodeBsdfPrincipled")
    source = nodes.new("ShaderNodeVertexColor")
    source.layer_name = attribute.name
    tree.links.new(source.outputs["Color"], shader.inputs["Base Color"])
    tree.links.new(shader.outputs["BSDF"], output.inputs["Surface"])
    # 双面可见：单视角 2.5D 面从背面看过去也该有东西，而不是凭空消失。
    material.use_backface_culling = False
    obj.data.materials.append(material)
    print(f"[wheal] 顶点色属性 '{attribute.name}' -> 材质 Base Color")
    return True


def _place_camera(center, radius: float) -> None:
    """摆一台相机，并把渲染引擎设成不需要灯光的 Workbench。

    **视口直接切到相机视角**（见 :func:`_configure_viewport`），所以这里的取景就是
    实际看到的取景。之前用的是“手算 view_location / view_distance”——那次打开了
    一个极近距离的视口，看到的是一片巨大的“绿色尖刺”（其实是行相关伪影被放大），
    而模型看起来像坏掉了。相机视角没有这个歧义：位置和朝向都是真实几何量。
    """
    direction = VIEW_DIRECTION
    location = center - direction * (radius * 2.9)

    camera_data = bpy.data.cameras.new("wheal_camera")
    camera_data.clip_start = max(radius * 0.01, 1e-4)
    camera_data.clip_end = radius * 50.0
    camera = bpy.data.objects.new("wheal_camera", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = location
    camera.rotation_euler = (center - location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = camera


def _configure_viewport(center, radius: float) -> None:
    """把 3D 视图摆好。背景模式下没有 area，所以整段都要能跳过。"""
    screen = getattr(bpy.context, "screen", None)
    if screen is None:
        return

    for area in screen.areas:
        if area.type != "VIEW_3D":
            continue
        for space in area.spaces:
            if space.type != "VIEW_3D":
                continue
            # Workbench 不需要灯光，且 VERTEX 直接读顶点色——导入后立刻可见。
            space.shading.type = "SOLID"
            space.shading.color_type = "VERTEX"
            space.shading.show_cavity = True
            space.overlay.show_floor = False
            space.overlay.show_axis_x = False
            space.overlay.show_axis_y = False

            region = space.region_3d
            # 关键：切到相机视角。取景完全由场景相机决定，不再依赖对
            # view_distance / view_rotation 约定的人脑推导。
            region.view_perspective = "CAMERA"
            # 同时把自由视角也摆好——按 Numpad 0 切回去时不会落到奇怪的机位。
            region.view_location = center
            region.view_distance = radius * 2.9
            region.view_rotation = (
                center - bpy.context.scene.camera.location
            ).to_track_quat("-Z", "Y")
        area.tag_redraw()


def _report(obj) -> None:
    low, high = _bounds(obj)
    extent = high - low
    print(
        f"[wheal] 包围盒 {extent.x:.3f} x {extent.y:.3f} x {extent.z:.3f} m "
        f"（原点在相机处，表面在 z≈{low.z:.3f}..{high.z:.3f} m）"
    )
    print("[wheal] 视口已是 Workbench + 顶点色，且已切到相机视角。")
    print("[wheal] 鼠标中键旋转、滚轮缩放（会自动回到自由视角）；Numpad 0 切回相机视角。")
    print("[wheal] 想用材质渲染：右上角改成 EEVEE（材质已接好顶点色，不用再加灯）。")


def main() -> None:
    arguments = _arguments()
    if not arguments:
        raise SystemExit(
            "用法: blender --factory-startup --python scripts/view_in_blender.py -- "
            "<file.ply> [--save]"
        )
    save = "--save" in arguments
    path = Path(next(a for a in arguments if not a.startswith("--"))).resolve()

    _clear_default_scene()
    obj = _import_ply(path)
    _add_vertex_colour_material(obj)

    low, high = _bounds(obj)
    center = (low + high) / 2.0
    radius = max((high - low).length / 2.0, 1e-3)

    _place_camera(center, radius)
    _configure_viewport(center, radius)

    # 存盘是**可选**的，默认不存。每跑一次就在 PLY 旁边留下一个 16 MB 的 .blend
    # 再加一个 16 MB 的 .blend1 自动备份，而它完全可以从 PLY 重新生成——
    # 这类“顺手写一个派生大文件”正是仓库里冗余产物的来源。想看就当场看，
    # 真想让下次能双击打开时再加 --save。
    if save:
        blend_path = path.with_suffix(".blend")
        bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))
        print(f"[wheal] 已存 {blend_path}（下次直接打开这个就不用跑脚本了）")

    _report(obj)
    print("[wheal] 本次未写任何文件；要存一份 .blend 请加 --save。")


main()
