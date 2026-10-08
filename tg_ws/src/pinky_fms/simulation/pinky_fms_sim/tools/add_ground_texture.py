#!/usr/bin/env python3
"""Add a calibrated PNG visual to an existing SDF without rebuilding its walls."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


def image_size(path):
    with Image.open(path) as image:
        if image.format != 'PNG':
            raise ValueError('Ground texture must be a PNG')
        size = image.size
        image.verify()
    return size


def fit_transform(calibration, texture_size, map_size, meta):
    """Landmarks use continuous image edge coordinates: top-left=(0,0).

    A pixel center is (column+0.5,row+0.5); map origin is the bottom-left
    image edge. The fitted affine transform allows rotation and unequal scales.
    """
    if tuple(calibration['texture_size']) != tuple(texture_size):
        raise ValueError('Calibration texture_size does not match the PNG')
    points = calibration['landmarks']
    source = np.asarray([p['texture_px'] for p in points], dtype=float)
    target = np.asarray([p['map_px'] for p in points], dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2:
        raise ValueError('Landmarks must be pairs of two coordinates')
    if len(source) < 3 or not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError('Need at least three finite landmark pairs')
    for coords, size in ((source, texture_size), (target, map_size)):
        if (coords < 0).any() or (coords > np.asarray(size)).any():
            raise ValueError('A landmark is outside its image')
    design = np.column_stack((source, np.ones(len(source))))
    transform, _, rank, _ = np.linalg.lstsq(design, target, rcond=None)
    if rank != 3 or abs(np.linalg.det(transform[:2])) < 1e-12:
        raise ValueError('Landmarks are degenerate; use non-collinear points')
    # Mirroring a course is usually a calibration error, not a valid alignment.
    if np.linalg.det(transform[:2]) <= 0:
        raise ValueError('Calibration reflects the image; check landmark order')
    resolution = float(meta['resolution'])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Map resolution must be positive and finite')
    errors = np.linalg.norm(design @ transform - target, axis=1) * resolution
    limit = float(calibration.get('max_error_m', 0.01))
    if not math.isfinite(limit) or limit < 0 or errors.max() > limit:
        raise ValueError(f'Landmark error {errors.max():.6f}m exceeds {limit}m')
    return transform, errors


def world_vertices(transform, texture_size, map_size, meta):
    width, height = texture_size
    corners = np.array([[0, 0, 1], [width, 0, 1],
                        [width, height, 1], [0, height, 1]], dtype=float)
    pixels = corners @ transform
    return pixels_to_world(pixels, map_size, meta)


def pixels_to_world(pixels, map_size, meta):
    local = np.column_stack((pixels[:, 0], map_size[1] - pixels[:, 1]))
    local *= float(meta['resolution'])
    ox, oy, yaw = map(float, meta['origin'])
    if not np.isfinite([ox, oy, yaw]).all():
        raise ValueError('Map origin must contain three finite values')
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    return local @ rotation.T + [ox, oy]


def grid_mesh(calibration, texture_size, map_size, meta):
    """Separable, monotone registration for an illustrated orthogonal course.

    The PNG bytes stay unchanged; positions and UVs of a tessellated plane
    align the measured wall/obstacle anchors. No physical collisions change.
    """
    if tuple(calibration['texture_size']) != tuple(texture_size):
        raise ValueError('Calibration texture_size does not match the PNG')
    axes = []
    for key, length, map_length in zip(('x', 'y'), texture_size, map_size):
        pairs = np.asarray(calibration['grid'][key], dtype=float)
        if (pairs.ndim != 2 or pairs.shape[1] != 2 or len(pairs) < 2
                or not np.isfinite(pairs).all()
                or not (np.diff(pairs, axis=0) > 0).all()
                or pairs[0, 0] != 0 or pairs[-1, 0] != length
                or pairs[0, 1] < 0 or pairs[-1, 1] > map_length):
            raise ValueError('Grid axes must increase strictly and span the texture inside the map')
        axes.append(pairs)
    xs, ys = axes
    points = np.array([[x[1], y[1]] for y in ys for x in xs])
    vertices = pixels_to_world(points, map_size, meta)
    uvs = [(x[0] / texture_size[0], 1 - y[0] / texture_size[1]) for y in ys for x in xs]
    triangles = []
    for row in range(len(ys) - 1):
        for col in range(len(xs) - 1):
            top = row * len(xs) + col
            bottom = top + len(xs)
            triangles.extend([(top, bottom + 1, top + 1), (top, bottom, bottom + 1)])
    return vertices, uvs, triangles


def mesh_text(vertices, uvs=None, triangles=None):
    # Top-left texture corner is UV (0,1). Reverse image winding so normals
    # point upwards. Both triangles share the explicit position/UV indices.
    positions = ' '.join(f'{v:.10f}' for x, y in vertices for v in (x, y, 0))
    uvs = uvs if uvs is not None else [(0, 1), (1, 1), (1, 0), (0, 0)]
    triangles = triangles if triangles is not None else [(0, 2, 1), (0, 3, 2)]
    texcoords = ' '.join(f'{v:.10f}' for uv in uvs for v in uv)
    indices = ' '.join(f'{i} {i} 0' for triangle in triangles for i in triangle)
    return f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="course_image"><init_from>course.png</init_from></image></library_images>
  <library_effects><effect id="course_effect"><profile_COMMON>
    <newparam sid="course_surface"><surface type="2D"><init_from>course_image</init_from></surface></newparam>
    <newparam sid="course_sampler"><sampler2D><source>course_surface</source></sampler2D></newparam>
    <technique sid="common"><lambert><diffuse><texture texture="course_sampler" texcoord="UVMap"/></diffuse></lambert></technique>
  </profile_COMMON></effect></library_effects>
  <library_materials><material id="course_material"><instance_effect url="#course_effect"/></material></library_materials>
  <library_geometries><geometry id="course"><mesh>
    <source id="positions"><float_array id="positions-array" count="{len(vertices)*3}">{positions}</float_array>
      <technique_common><accessor source="#positions-array" count="{len(vertices)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="uv"><float_array id="uv-array" count="{len(uvs)*2}">{texcoords}</float_array>
      <technique_common><accessor source="#uv-array" count="{len(uvs)}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
    <source id="normals"><float_array id="normals-array" count="3">0 0 1</float_array>
      <technique_common><accessor source="#normals-array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
    <triangles material="course_material" count="{len(triangles)}"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><input semantic="NORMAL" source="#normals" offset="2"/><p>{indices}</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node id="ground"><instance_geometry url="#course"><bind_material><technique_common><instance_material symbol="course_material" target="#course_material"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''


def add_texture(world, map_yaml, texture, calibration_file, output):
    world, map_yaml, texture, calibration_file, output = map(
        Path, (world, map_yaml, texture, calibration_file, output))
    if output.resolve() == world.resolve():
        raise ValueError('Output must differ from the existing world')
    # No silent overwrites of an existing local calibration or generated world.
    assets = output.parent / (output.stem + '_assets')
    if output.exists() or assets.exists():
        raise FileExistsError('Output or asset directory already exists; choose a fresh output path')
    meta = yaml.safe_load(map_yaml.read_text())
    with Image.open(map_yaml.parent / meta['image']) as image:
        map_size = image.size
        image.load()
    texture_size = image_size(texture)
    calibration = json.loads(calibration_file.read_text())
    expected_hash = calibration.get('texture_sha256')
    if expected_hash and hashlib.sha256(texture.read_bytes()).hexdigest() != expected_hash:
        raise ValueError('PNG checksum does not match its calibration')
    if 'grid' in calibration:
        vertices, uvs, triangles = grid_mesh(calibration, texture_size, map_size, meta)
        registration = {'method': 'monotone_grid', 'grid': calibration['grid']}
    else:
        transform, errors = fit_transform(calibration, texture_size, map_size, meta)
        vertices = world_vertices(transform, texture_size, map_size, meta)
        uvs, triangles = None, None
        registration = {'method': 'affine', 'texture_to_map': transform.tolist(),
                        'landmark_errors_m': errors.tolist()}
    tree = ET.parse(world)
    sdf_world = tree.getroot().find('world')
    if sdf_world is None:
        raise ValueError('Input must contain an SDF world')
    if any(m.get('name') == 'course_ground_texture' for m in sdf_world.findall('model')):
        raise ValueError('World already contains a course texture')
    model = ET.SubElement(sdf_world, 'model', name='course_ground_texture')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '0 0 0.001 0 0 0'
    link = ET.SubElement(model, 'link', name='ground_texture')
    visual = ET.SubElement(link, 'visual', name='course')
    ET.SubElement(visual, 'cast_shadows').text = 'false'
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = f'{assets.name}/ground.dae'
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'diffuse').text = '1 1 1 1'
    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    ET.SubElement(metal, 'albedo_map').text = f'{assets.name}/course.png'
    ET.SubElement(metal, 'metalness').text = '0'
    ET.SubElement(metal, 'roughness').text = '1'
    output.parent.mkdir(parents=True, exist_ok=True)
    assets.mkdir()
    shutil.copyfile(texture, assets / 'course.png')
    (assets / 'ground.dae').write_text(mesh_text(vertices, uvs, triangles))
    (assets / 'calibration.json').write_text(json.dumps({
        'calibration': calibration, 'map_size': map_size,
        'map_resolution': meta['resolution'], 'map_origin': meta['origin'],
        **registration,
        'world_vertices': vertices.tolist(),
    }, indent=2))
    tree.write(output, encoding='utf-8', xml_declaration=True)
    print(f'{output}: visual only; registration={registration["method"]}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world', help='Existing SDF; walls and collisions are retained')
    parser.add_argument('map_yaml')
    parser.add_argument('texture', help='Actual course PNG, not an annotated diagram')
    parser.add_argument('calibration', help='JSON landmark pairs in image coordinates')
    parser.add_argument('output', help='New SDF path, adjacent to generated assets')
    args = parser.parse_args()
    try:
        add_texture(args.world, args.map_yaml, args.texture, args.calibration, args.output)
    except (ValueError, KeyError, OSError, ET.ParseError) as error:
        parser.exit(1, f'Ground texture failed: {error}\n')


if __name__ == '__main__':
    main()
