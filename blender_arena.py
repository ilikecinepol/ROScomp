import bpy
import math
import json
from mathutils import Vector
from pathlib import Path


ROOT = Path(r"C:\Users\drmma\Documents\ChatGPT\ROS2 Competition")
BLEND_PATH = ROOT / "true_tech_arena_scale_1to1.blend"
GLB_PATH = ROOT / "true_tech_arena_visual.glb"
PREVIEW_PATH = ROOT / "true_tech_arena_preview.png"
DIMENSIONS_PATH = ROOT / "true_tech_arena_dimensions.json"

# All values are metres. Obstacle dimensions are taken from the contest brief.
# Room size and obstacle positions are a working reconstruction from photographs.
ROOM_X = 12.0
ROOM_Y = 8.0
ROOM_H = 3.2


def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for datablocks in (bpy.data.meshes, bpy.data.curves, bpy.data.materials,
                       bpy.data.cameras, bpy.data.lights):
        for block in list(datablocks):
            if block.users == 0:
                datablocks.remove(block)


def collection(name):
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(col)
    return col


def move_to_collection(obj, col):
    for old in list(obj.users_collection):
        old.objects.unlink(obj)
    col.objects.link(obj)


def material(name, color, metallic=0.0, roughness=0.55):
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    mat.diffuse_color = (*color[:3], color[3])
    bsdf = next((n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED'), None)
    if bsdf:
        bsdf.inputs['Base Color'].default_value = color
        bsdf.inputs['Roughness'].default_value = roughness
        bsdf.inputs['Metallic'].default_value = metallic
    return mat


def finish(obj, name, col, mat=None, bevel=0.015):
    obj.name = name
    move_to_collection(obj, col)
    if mat:
        obj.data.materials.append(mat)
    if bevel and obj.type == 'MESH':
        mod = obj.modifiers.new('Edge softening', 'BEVEL')
        mod.width = bevel
        mod.segments = 2
    return obj


def box(name, loc, dims, mat, col, rot=(0, 0, 0), bevel=0.015):
    bpy.ops.mesh.primitive_cube_add(location=loc, rotation=rot)
    obj = bpy.context.object
    obj.dimensions = dims
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return finish(obj, name, col, mat, bevel)


def cylinder(name, loc, radius, depth, mat, col, rot=(0, 0, 0), vertices=24):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth,
                                        location=loc, rotation=rot)
    return finish(bpy.context.object, name, col, mat, 0.006)


def cyl_between(name, p1, p2, radius, mat, col, vertices=16):
    p1, p2 = Vector(p1), Vector(p2)
    vec = p2 - p1
    mid = (p1 + p2) / 2
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius,
                                        depth=vec.length, location=mid)
    obj = bpy.context.object
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = vec.to_track_quat('Z', 'Y')
    return finish(obj, name, col, mat, 0.003)


def text_obj(name, body, loc, size, mat, col):
    bpy.ops.object.text_add(location=loc)
    obj = bpy.context.object
    obj.data.body = body
    obj.data.align_x = 'CENTER'
    obj.data.align_y = 'CENTER'
    obj.data.size = size
    obj.data.extrude = 0.008
    obj.data.bevel_depth = 0.002
    return finish(obj, name, col, mat, 0)


def group_marker(name, loc, props, col):
    obj = bpy.data.objects.new(name, None)
    obj.empty_display_type = 'CUBE'
    obj.empty_display_size = 0.25
    obj.location = loc
    col.objects.link(obj)
    for key, value in props.items():
        obj[key] = value
    return obj


def camera_look_at(obj, point):
    obj.rotation_euler = (Vector(point) - obj.location).to_track_quat('-Z', 'Y').to_euler()


clear_scene()
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.length_unit = 'METERS'
scene.unit_settings.scale_length = 1.0
scene['model_scale'] = '1 Blender unit = 1 metre'
scene['obstacle_dimensions_source'] = 'True Tech Championship 2026 Track 2 brief'
scene['room_layout_status'] = 'Approximate reconstruction from photographs; replace after field measurement'
scene['room_assumed_size_m'] = [ROOM_X, ROOM_Y, ROOM_H]

ENV = collection('00_ENVIRONMENT_APPROX')
OBS = collection('10_OBSTACLES_EXACT_BRIEF')
MARK = collection('20_LABELS_AND_ASSUMPTIONS')
LIGHT = collection('90_LIGHTS_CAMERAS')

mat_floor_a = material('MAT_Floor_Dark', (0.12, 0.14, 0.16, 1))
mat_floor_b = material('MAT_Floor_Light', (0.28, 0.31, 0.33, 1))
mat_wall = material('MAT_Wall', (0.58, 0.60, 0.60, 1))
mat_black = material('MAT_BlackCurtain', (0.012, 0.015, 0.02, 1), roughness=0.82)
mat_board = material('MAT_ObstacleBoard', (0.16, 0.18, 0.19, 1), roughness=0.72)
mat_edge = material('MAT_Grip', (0.035, 0.045, 0.05, 1), roughness=0.75)
mat_metal = material('MAT_Metal', (0.18, 0.21, 0.23, 1), metallic=0.65, roughness=0.35)
mat_orange = material('MAT_Orange', (0.95, 0.20, 0.025, 1), roughness=0.45)
mat_white = material('MAT_White', (0.92, 0.94, 0.95, 1), roughness=0.45)
mat_yellow = material('MAT_Start', (0.96, 0.73, 0.04, 1), roughness=0.5)
mat_text = material('MAT_Labels', (0.65, 0.84, 1.0, 1), roughness=0.4)

# Room: 12 x 8 m is an explicit provisional reconstruction.
box('ENV_FloorBase_12x8m', (0, 0, -0.035), (ROOM_X, ROOM_Y, 0.07), mat_floor_a, ENV, bevel=0)
stripe_w = 0.5
for i in range(int(ROOM_X / stripe_w)):
    x = -ROOM_X / 2 + stripe_w / 2 + i * stripe_w
    mat = mat_floor_b if i % 2 == 0 else mat_floor_a
    box(f'ENV_FloorStripe_{i+1:02d}', (x, 0, 0.008), (stripe_w, ROOM_Y, 0.016), mat, ENV, bevel=0)

box('ENV_BackWall', (0, ROOM_Y/2, ROOM_H/2), (ROOM_X, 0.08, ROOM_H), mat_wall, ENV, bevel=0)
box('ENV_LeftWall', (-ROOM_X/2, 0, ROOM_H/2), (0.08, ROOM_Y, ROOM_H), mat_wall, ENV, bevel=0)
box('ENV_RightCurtain', (ROOM_X/2, 0, ROOM_H/2), (0.06, ROOM_Y, ROOM_H), mat_black, ENV, bevel=0)
for i, x in enumerate((-4.0, -1.35, 1.35, 4.0), 1):
    box(f'ENV_Window_{i}', (x, ROOM_Y/2-0.055, 2.05), (2.05, 0.035, 1.15), mat_black, ENV, bevel=0.01)
    box(f'ENV_WindowGlass_{i}', (x, ROOM_Y/2-0.08, 2.05), (1.88, 0.02, 0.98), mat_floor_b, ENV, bevel=0)

# 1. A-frame: slope length 1.35 m each, height 0.50 m, width 0.60 m.
af_x, af_y = -3.25, 1.45
af_slope = 1.35
af_h = 0.50
af_run = math.sqrt(af_slope**2 - af_h**2)
af_angle = math.atan2(af_h, af_run)
for side, sign in (('A', 1), ('B', -1)):
    cy = af_y - sign * af_run/2
    board = box(f'AFRAME_Slope_{side}', (af_x, cy, af_h/2),
                (0.60, af_slope, 0.045), mat_board, OBS,
                rot=(sign * af_angle, 0, 0), bevel=0.012)
    board['width_m'] = 0.60
    board['slope_length_m'] = af_slope
    board['peak_height_m'] = af_h
    for j in range(1, 7):
        t = j / 7
        y = af_y - sign * af_run * (1-t)
        z = af_h * t + 0.035
        box(f'AFRAME_Grip_{side}_{j:02d}', (af_x, y, z), (0.66, 0.038, 0.035),
            mat_edge, OBS, rot=(sign * af_angle, 0, 0), bevel=0.004)
group_marker('DIM_AFRAME', (af_x, af_y, af_h + 0.3), {
    'width_m': 0.60, 'slope_each_m': 1.35, 'peak_height_m': 0.50,
    'slope_angle_deg': round(math.degrees(af_angle), 2), 'source': 'contest brief midpoint'
}, MARK)
text_obj('LABEL_AFRAME', '1  A-FRAME  0.60 m', (af_x, af_y+1.25, 0.025), 0.23, mat_text, MARK)

# 2. Suspended bridge: span 1.75 m, width 0.55 m, deck height 0.20 m.
br_x, br_y = 1.05, -1.55
br_len, br_w, br_z = 1.75, 0.55, 0.20
plank_count = 10
plank_len = br_len / plank_count * 0.88
for i in range(plank_count):
    x = br_x - br_len/2 + (i + 0.5) * br_len/plank_count
    box(f'BRIDGE_Plank_{i+1:02d}', (x, br_y, br_z), (plank_len, br_w, 0.045),
        mat_board, OBS, bevel=0.01)
for sx in (-1, 1):
    for sy in (-1, 1):
        px = br_x + sx * (br_len/2 + 0.16)
        py = br_y + sy * (br_w/2 + 0.10)
        cylinder(f'BRIDGE_Post_{sx}_{sy}', (px, py, 0.52), 0.035, 1.04,
                 mat_metal, OBS)
        deck_corner = (br_x + sx*br_len/2, br_y + sy*br_w/2, br_z+0.035)
        cyl_between(f'BRIDGE_Hanger_{sx}_{sy}', (px, py, 1.0), deck_corner,
                    0.012, mat_metal, OBS)
for sy in (-1, 1):
    cyl_between(f'BRIDGE_Rail_{sy}', (br_x-br_len/2-0.16, br_y+sy*(br_w/2+0.10), 1.0),
                (br_x+br_len/2+0.16, br_y+sy*(br_w/2+0.10), 1.0), 0.012, mat_metal, OBS)
group_marker('DIM_SUSPENDED_BRIDGE', (br_x, br_y, 1.25), {
    'span_m': br_len, 'width_m': br_w, 'deck_height_m': br_z,
    'allowed_sway_deg': 8.0, 'source': 'contest brief midpoint/high limit'
}, MARK)
text_obj('LABEL_BRIDGE', '2  SUSPENDED BRIDGE  1.75 x 0.55 m',
         (br_x, br_y+0.72, 0.025), 0.19, mat_text, MARK)

# 3. Teeter: 1.80 x 0.55 m, axle 0.175 m, rest tilt 11 degrees.
tt_x, tt_y = -3.15, -1.50
tt_len, tt_w, tt_axis, tt_deg = 1.80, 0.55, 0.175, 11.0
tt_ang = math.radians(tt_deg)
box('TEETER_Board', (tt_x, tt_y, tt_axis), (tt_len, tt_w, 0.05), mat_board, OBS,
    rot=(0, -tt_ang, 0), bevel=0.012)
cylinder('TEETER_Axle', (tt_x, tt_y, tt_axis), 0.045, tt_w+0.18, mat_metal, OBS,
         rot=(math.pi/2, 0, 0))
for sy in (-1, 1):
    box(f'TEETER_Support_{sy}', (tt_x, tt_y+sy*(tt_w/2+0.07), 0.085),
        (0.30, 0.06, 0.17), mat_metal, OBS, bevel=0.008)
group_marker('DIM_TEETER', (tt_x, tt_y, 0.72), {
    'board_length_m': tt_len, 'width_m': tt_w, 'axis_height_m': tt_axis,
    'rest_tilt_deg': tt_deg, 'dynamic_joint_required_in_webots': True,
    'source': 'contest brief midpoint'
}, MARK)
text_obj('LABEL_TEETER', '3  TEETER  1.80 x 0.55 m', (tt_x, tt_y+0.68, 0.025),
         0.20, mat_text, MARK)

# 4. Slalom: six 1 m poles at 0.90 m spacing.
sl_y = -3.10
sl_start = -2.25
for i in range(6):
    x = sl_start + i * 0.90
    cylinder(f'SLALOM_Base_{i+1:02d}', (x, sl_y, 0.035), 0.145, 0.07, mat_orange, OBS)
    for seg in range(5):
        z = 0.10 + (seg + 0.5) * 0.18
        cylinder(f'SLALOM_Pole_{i+1:02d}_{seg+1}', (x, sl_y, z), 0.027, 0.18,
                 mat_orange if seg % 2 == 0 else mat_white, OBS)
group_marker('DIM_SLALOM', (0, sl_y, 1.25), {
    'pole_count': 6, 'pole_height_m': 1.0, 'spacing_m': 0.90,
    'corridor_half_width_m': 0.40, 'base_diameter_m_assumed': 0.29,
    'source': 'contest brief midpoint; base diameter assumed'
}, MARK)
text_obj('LABEL_SLALOM', '4  SLALOM  6 x 1.00 m   STEP 0.90 m',
         (0, sl_y+0.45, 0.025), 0.19, mat_text, MARK)

# 5. Double platform: ramp -> 0.275 m platform -> +0.125 m step -> 0.40 m platform -> ramp.
dp_x, dp_y = 3.40, 1.15
dp_w = 0.50
h1, h2 = 0.275, 0.400
plat_len = 0.60
ramp_angle = math.radians(22.5)
r1_run = h1 / math.tan(ramp_angle)
r2_run = h2 / math.tan(ramp_angle)
r1_len = math.hypot(r1_run, h1)
r2_len = math.hypot(r2_run, h2)
y0 = dp_y - (r1_run + plat_len*2 + r2_run)/2
box('DOUBLE_RampUp', (dp_x, y0+r1_run/2, h1/2), (dp_w, r1_len, 0.045),
    mat_board, OBS, rot=(ramp_angle, 0, 0), bevel=0.01)
y1 = y0 + r1_run
box('DOUBLE_PlatformLow', (dp_x, y1+plat_len/2, h1), (dp_w, plat_len, 0.055),
    mat_board, OBS, bevel=0.01)
y2 = y1 + plat_len
box('DOUBLE_StepFace', (dp_x, y2, (h1+h2)/2), (dp_w, 0.055, h2-h1),
    mat_edge, OBS, bevel=0.006)
box('DOUBLE_PlatformHigh', (dp_x, y2+plat_len/2, h2), (dp_w, plat_len, 0.055),
    mat_board, OBS, bevel=0.01)
y3 = y2 + plat_len
box('DOUBLE_RampDown', (dp_x, y3+r2_run/2, h2/2), (dp_w, r2_len, 0.045),
    mat_board, OBS, rot=(-ramp_angle, 0, 0), bevel=0.01)
group_marker('DIM_DOUBLE_PLATFORM', (dp_x, dp_y, 0.95), {
    'width_m': dp_w, 'platform_length_each_m': plat_len,
    'low_height_m': h1, 'high_height_m': h2, 'step_height_m': h2-h1,
    'ramp_angle_deg': 22.5, 'source': 'contest brief midpoint'
}, MARK)
text_obj('LABEL_DOUBLE_PLATFORM', '5  DOUBLE PLATFORM  W 0.50 m',
         (dp_x, dp_y+1.85, 0.025), 0.19, mat_text, MARK)

# Provisional start line near the open end of the room.
for i in range(12):
    box(f'START_Segment_{i+1:02d}', (-5.5+i, -3.70, 0.035),
        (0.92, 0.10, 0.025), mat_yellow if i % 2 == 0 else mat_white, MARK,
        bevel=0.002)
start = group_marker('ASSUMPTION_StartLine', (0, -3.70, 0.2), {
    'status': 'provisional - exact start line was not measurable from supplied photos'
}, MARK)

# Camera and light for an inspectable isometric overview.
bpy.ops.object.camera_add(location=(10.6, -12.4, 10.8))
cam = finish(bpy.context.object, 'CAM_ArenaOverview', LIGHT, None, 0)
camera_look_at(cam, (0, 0.2, 0.15))
cam.data.lens = 48
scene.camera = cam

bpy.ops.object.light_add(type='AREA', location=(0, -0.5, 8.0))
key = finish(bpy.context.object, 'LIGHT_KeyArea', LIGHT, None, 0)
key.data.energy = 1700
key.data.shape = 'RECTANGLE'
key.data.size = 8.0
key.data.size_y = 5.0

bpy.ops.object.light_add(type='SUN', location=(0, 0, 6))
sun = finish(bpy.context.object, 'LIGHT_Sun', LIGHT, None, 0)
sun.rotation_euler = (math.radians(24), math.radians(-18), math.radians(28))
sun.data.energy = 1.2

scene.world.color = (0.025, 0.028, 0.035)
try:
    scene.render.engine = 'BLENDER_EEVEE_NEXT'
except TypeError:
    pass
scene.render.resolution_x = 1400
scene.render.resolution_y = 900
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.filepath = str(PREVIEW_PATH)
scene.render.film_transparent = False

# Attach a compact machine-readable manifest to the scene and write it alongside the model.
manifest = {
    'units': 'metres',
    'scale': '1:1',
    'room': {'size_m': [ROOM_X, ROOM_Y, ROOM_H], 'status': 'approximate from photos'},
    'obstacles': {
        'a_frame': {'width': 0.60, 'slope_each': 1.35, 'height': 0.50},
        'suspended_bridge': {'span': 1.75, 'width': 0.55, 'deck_height': 0.20, 'sway_deg': 8.0},
        'teeter': {'length': 1.80, 'width': 0.55, 'axis_height': 0.175, 'tilt_deg': 11.0},
        'slalom': {'count': 6, 'height': 1.0, 'spacing': 0.90, 'corridor_half_width': 0.40},
        'double_platform': {'width': 0.50, 'platform_each': 0.60, 'heights': [0.275, 0.400], 'ramp_deg': 22.5}
    },
    'webots_note': 'Use GLB for visuals. Add simplified collision solids and joints in Webots PROTO; bridge and teeter dynamics are not carried by GLB.'
}
scene['dimension_manifest_json'] = json.dumps(manifest, ensure_ascii=False)
DIMENSIONS_PATH.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')

# Save native scene first. Export and render are best-effort so the .blend remains available.
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))
try:
    bpy.ops.export_scene.gltf(filepath=str(GLB_PATH), export_format='GLB',
                              use_selection=False, export_cameras=False, export_lights=False)
except Exception as exc:
    scene['glb_export_error'] = repr(exc)
try:
    bpy.ops.render.render(write_still=True)
except Exception as exc:
    scene['preview_render_error'] = repr(exc)
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))

# Select obstacle collection for immediate inspection in the open Blender window.
bpy.context.view_layer.objects.active = bpy.data.objects.get('AFRAME_Slope_A')
for obj in bpy.context.selected_objects:
    obj.select_set(False)
if bpy.context.view_layer.objects.active:
    bpy.context.view_layer.objects.active.select_set(True)

print(f'ARENA_BUILD_OK objects={len(scene.objects)} blend={BLEND_PATH}')
