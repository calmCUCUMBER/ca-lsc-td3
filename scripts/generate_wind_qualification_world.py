#!/usr/bin/env python3
"""Generate a Gazebo Sim world/model pair for wind/payload tests.

The stock PX4 `standard_vtol` model is included by name in the nominal world.
For F2 wind qualification we need a variant whose aircraft links explicitly
enable Gazebo's wind mode.  This script creates that variant without editing
the frozen PX4 low-level transition allocator or the upstream PX4 model tree.

The same generated model can also carry a centered fixed payload link for
F2-A mass-sensitivity tests.  This keeps the disturbance in Gazebo physics
rather than only changing an algorithm-side mass estimate.
"""

from __future__ import annotations

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _indent(tree: ET.ElementTree) -> None:
    try:
        ET.indent(tree, space='  ')
    except AttributeError:  # pragma: no cover - Python < 3.9 compatibility.
        pass


def _set_text(parent: ET.Element, tag: str, text: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    child.text = text
    return child


def _link_masses(model: ET.Element) -> list[tuple[str, float]]:
    masses: list[tuple[str, float]] = []
    for link in model.findall('link'):
        mass_text = link.findtext('inertial/mass')
        if mass_text is None:
            continue
        try:
            mass = float(mass_text)
        except ValueError:
            continue
        masses.append((link.get('name', ''), mass))
    return masses


def _append_centered_payload(
    model: ET.Element,
    *,
    payload_mass_kg: float,
    payload_link_name: str,
    payload_pose: tuple[float, float, float, float, float, float],
    payload_cube_size_m: float,
) -> None:
    if payload_mass_kg <= 0.0:
        return
    if model.find(f"./link[@name='{payload_link_name}']") is not None:
        raise RuntimeError(f'payload link already exists: {payload_link_name}')
    if model.find("./link[@name='base_link']") is None:
        raise RuntimeError('payload insertion requires a base_link')

    inertia = payload_mass_kg * payload_cube_size_m ** 2 / 6.0
    pose_text = ' '.join(f'{value:.9g}' for value in payload_pose)
    size_text = ' '.join(f'{payload_cube_size_m:.9g}' for _ in range(3))

    link = ET.SubElement(model, 'link', {'name': payload_link_name})
    ET.SubElement(link, 'pose').text = pose_text
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = f'{payload_mass_kg:.12g}'
    inertia_element = ET.SubElement(inertial, 'inertia')
    for tag, value in (
        ('ixx', inertia), ('ixy', 0.0), ('ixz', 0.0),
        ('iyy', inertia), ('iyz', 0.0), ('izz', inertia),
    ):
        ET.SubElement(inertia_element, tag).text = f'{value:.12g}'

    for tag in ('collision', 'visual'):
        geometry_parent = ET.SubElement(
            link, tag, {'name': f'{payload_link_name}_{tag}'}
        )
        geometry = ET.SubElement(geometry_parent, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = size_text

    joint = ET.SubElement(
        model,
        'joint',
        {'name': f'{payload_link_name}_fixed_joint', 'type': 'fixed'},
    )
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = payload_link_name


def _instrument_main_wings(
    model: ET.Element,
    *,
    instrument_elevator_moment: bool = False,
    elevator_id_dither_amplitude_rad: float = 0.0,
    elevator_id_dither_frequency_hz: float = 0.0,
    elevator_id_dither_phase_rad: float = 0.0,
) -> dict[str, str]:
    """Replace only the two main-wing LiftDrag force producers.

    The diagnostic plugin applies the same aerodynamic force and publishes
    the exact lift vector / optional moment wrench used by Gazebo physics.
    By default the elevator LiftDrag instance remains untouched; F3-B1 can
    explicitly replace it with the same equations plus telemetry.
    """
    main_wings = []
    for plugin in model.findall('plugin'):
        if 'lift-drag' not in plugin.get('filename', ''):
            continue
        try:
            area = float(plugin.findtext('area', default='nan'))
        except ValueError:
            continue
        if area >= 0.1:
            main_wings.append(plugin)
    if len(main_wings) != 2:
        raise RuntimeError(
            f'expected exactly two main-wing LiftDrag plugins, got '
            f'{len(main_wings)}'
        )
    topics: dict[str, str] = {}
    for plugin in main_wings:
        cp = [float(value) for value in plugin.findtext('cp').split()]
        side = 'left' if cp[1] > 0.0 else 'right'
        topic = f'/ca_lsc/f3/wing_{side}/lift'
        plugin.set('filename', 'libca_lsc_instrumented_lift_drag.so')
        plugin.set('name', 'ca_lsc::InstrumentedLiftDrag')
        _set_text(plugin, 'force_topic', topic)
        _set_text(plugin, 'publish_rate_hz', '100')
        topics[f'wing_{side}_lift_gz_topic'] = topic
    if set(topics) != {
        'wing_left_lift_gz_topic', 'wing_right_lift_gz_topic'
    }:
        raise RuntimeError('main-wing center-of-pressure sides are ambiguous')
    if instrument_elevator_moment:
        elevator_plugins = []
        for plugin in model.findall('plugin'):
            if 'lift-drag' not in plugin.get('filename', ''):
                continue
            if plugin.findtext('control_joint_name', default='') != 'servo_2':
                continue
            try:
                area = float(plugin.findtext('area', default='nan'))
            except ValueError:
                continue
            if area < 0.1:
                elevator_plugins.append(plugin)
        if len(elevator_plugins) != 1:
            raise RuntimeError(
                f'expected exactly one elevator LiftDrag plugin, got '
                f'{len(elevator_plugins)}'
            )
        elevator_wrench_topic = '/ca_lsc/f3/elevator/wrench'
        elevator_effective_angle_topic = '/ca_lsc/f3/elevator/effective_angle'
        elevator_dither_scale_topic = '/ca_lsc/f3/elevator/dither_scale'
        elevator_plugin = elevator_plugins[0]
        elevator_plugin.set(
            'filename', 'libca_lsc_instrumented_lift_drag.so')
        elevator_plugin.set('name', 'ca_lsc::InstrumentedLiftDrag')
        _set_text(elevator_plugin, 'force_topic', elevator_wrench_topic)
        _set_text(
            elevator_plugin, 'effective_angle_topic',
            elevator_effective_angle_topic,
        )
        _set_text(
            elevator_plugin, 'id_dither_scale_topic',
            elevator_dither_scale_topic,
        )
        _set_text(
            elevator_plugin, 'id_dither_amplitude_rad',
            f'{elevator_id_dither_amplitude_rad:.12g}',
        )
        _set_text(
            elevator_plugin, 'id_dither_frequency_hz',
            f'{elevator_id_dither_frequency_hz:.12g}',
        )
        _set_text(
            elevator_plugin, 'id_dither_phase_rad',
            f'{elevator_id_dither_phase_rad:.12g}',
        )
        _set_text(elevator_plugin, 'publish_rate_hz', '100')
        topics['elevator_wrench_gz_topic'] = elevator_wrench_topic
        topics['elevator_effective_angle_gz_topic'] = (
            elevator_effective_angle_topic
        )
        topics['elevator_dither_scale_gz_topic'] = (
            elevator_dither_scale_topic
        )
    elevator_topic = '/ca_lsc/f3/elevator/joint_position'
    telemetry = ET.SubElement(model, 'plugin', {
        'filename': 'libca_lsc_instrumented_lift_drag.so',
        'name': 'ca_lsc::JointPositionTelemetry',
    })
    ET.SubElement(telemetry, 'joint_name').text = 'servo_2'
    ET.SubElement(telemetry, 'topic').text = elevator_topic
    ET.SubElement(telemetry, 'publish_rate_hz').text = '100'
    topics['elevator_joint_position_gz_topic'] = elevator_topic
    return topics


def _copy_condition_model(
    source_model: Path,
    destination_model: Path,
    model_name: str,
    *,
    mass_scale: float,
    payload_link_name: str,
    payload_pose: tuple[float, float, float, float, float, float],
    payload_cube_size_m: float,
    instrument_main_wings: bool,
    instrument_elevator_moment: bool,
    elevator_id_dither_amplitude_rad: float,
    elevator_id_dither_frequency_hz: float,
    elevator_id_dither_phase_rad: float,
) -> dict[str, object]:
    if destination_model.exists():
        shutil.rmtree(destination_model)
    shutil.copytree(source_model, destination_model)

    model_sdf = destination_model / 'model.sdf'
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    model = root.find('model')
    if model is None:
        raise RuntimeError(f'no <model> element found in {model_sdf}')
    model.set('name', model_name)

    if mass_scale < 1.0:
        raise RuntimeError('mass_scale below 1.0 is not supported by payload insertion')
    masses = _link_masses(model)
    nominal_mass_kg = sum(mass for _, mass in masses)
    payload_mass_kg = nominal_mass_kg * (mass_scale - 1.0)
    _append_centered_payload(
        model,
        payload_mass_kg=payload_mass_kg,
        payload_link_name=payload_link_name,
        payload_pose=payload_pose,
        payload_cube_size_m=payload_cube_size_m,
    )

    instrumentation = (
        _instrument_main_wings(
            model,
            instrument_elevator_moment=instrument_elevator_moment,
            elevator_id_dither_amplitude_rad=(
                elevator_id_dither_amplitude_rad
            ),
            elevator_id_dither_frequency_hz=elevator_id_dither_frequency_hz,
            elevator_id_dither_phase_rad=elevator_id_dither_phase_rad,
        )
        if instrument_main_wings else {}
    )

    enabled_links = 0
    for link in model.findall('link'):
        _set_text(link, 'enable_wind', 'true')
        enabled_links += 1

    _indent(tree)
    tree.write(model_sdf, encoding='utf-8', xml_declaration=True)

    config = destination_model / 'model.config'
    if config.exists():
        config_tree = ET.parse(config)
        config_root = config_tree.getroot()
        _set_text(config_root, 'name', model_name)
        _indent(config_tree)
        config_tree.write(config, encoding='utf-8', xml_declaration=True)
    return {
        'aircraft_links_enable_wind_true': enabled_links,
        'nominal_model_mass_kg': nominal_mass_kg,
        'mass_scale': mass_scale,
        'payload_mass_kg': payload_mass_kg,
        'actual_model_mass_kg': nominal_mass_kg + payload_mass_kg,
        'payload_link_count': 1 if payload_mass_kg > 0.0 else 0,
        'source_link_masses': [
            {'link': name, 'mass_kg': mass} for name, mass in masses
        ],
        'instrumented_main_wing_count': (
            2 if instrument_main_wings else 0
        ),
        **instrumentation,
    }


def _write_world(
    template_world: Path,
    output_world: Path,
    model_name: str,
    world_name: str,
    wind_enu: tuple[float, float, float],
    dynamic_wind: bool,
) -> None:
    tree = ET.parse(template_world)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        raise RuntimeError(f'no <world> element found in {template_world}')
    world.set('name', world_name)

    wind = world.find('wind')
    if wind is None:
        wind = ET.SubElement(world, 'wind')
    _set_text(
        wind,
        'linear_velocity',
        f'{wind_enu[0]:.6g} {wind_enu[1]:.6g} {wind_enu[2]:.6g}',
    )

    if dynamic_wind:
        existing = world.find(
            "./plugin[@name='gz::sim::systems::WindEffects']"
        )
        if existing is None:
            plugin = ET.SubElement(world, 'plugin', {
                'filename': 'gz-sim-wind-effects-system',
                'name': 'gz::sim::systems::WindEffects',
            })
            ET.SubElement(
                plugin, 'force_approximation_scaling_factor'
            ).text = '0'

    include = None
    for candidate in world.findall('include'):
        name = candidate.findtext('name', default='')
        uri = candidate.findtext('uri', default='')
        if name == 'standard_vtol' or 'standard_vtol' in uri:
            include = candidate
            break
    if include is None:
        raise RuntimeError(f'no standard_vtol <include> found in {template_world}')
    _set_text(include, 'uri', f'model://{model_name}')
    _set_text(include, 'name', model_name)

    output_world.parent.mkdir(parents=True, exist_ok=True)
    _indent(tree)
    tree.write(output_world, encoding='utf-8', xml_declaration=True)


def main() -> None:
    root = _project_root()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'output_root',
        type=Path,
        help='Directory that will receive worlds/, models/, and condition.json.',
    )
    parser.add_argument(
        '--dynamic-wind',
        action='store_true',
        help='Load Gazebo WindEffects so /world/<name>/wind/ commands can '
             'apply a deterministic time-varying profile.',
    )
    parser.add_argument(
        '--wind-enu',
        type=float,
        nargs=3,
        metavar=('X', 'Y', 'Z'),
        default=(0.0, 0.0, 0.0),
        help='Steady Gazebo world wind vector in ENU m/s.',
    )
    parser.add_argument('--world-name', default='default')
    parser.add_argument('--model-name', default='standard_vtol_wind')
    parser.add_argument(
        '--mass-scale',
        type=float,
        default=1.0,
        help='Actual aircraft mass scale implemented as a fixed payload link. '
             'Values below 1.0 are rejected.',
    )
    parser.add_argument('--payload-link-name', default='payload_mass')
    parser.add_argument(
        '--payload-pose',
        type=float,
        nargs=6,
        metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
        default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        help='Pose of the added payload link relative to the model frame.',
    )
    parser.add_argument(
        '--payload-cube-size-m',
        type=float,
        default=0.08,
        help='Side length used for the payload inertia/collision/visual cube.',
    )
    parser.add_argument(
        '--instrument-main-wings',
        action='store_true',
        help='Replace the two main-wing LiftDrag instances with the F3 '
             'force-publishing equivalent.',
    )
    parser.add_argument(
        '--instrument-elevator-moment',
        action='store_true',
        help='When --instrument-main-wings is enabled, also replace the '
             'elevator LiftDrag instance and publish its applied aerodynamic '
             'wrench for F3-B1 moment validation.',
    )
    parser.add_argument(
        '--elevator-id-dither-amplitude-rad',
        type=float,
        default=0.0,
        help='Optional small effective-elevator perturbation used only by '
             'the instrumented elevator LiftDrag plugin.',
    )
    parser.add_argument(
        '--elevator-id-dither-frequency-hz',
        type=float,
        default=0.0,
        help='Frequency of the optional F3-B1 elevator identification dither.',
    )
    parser.add_argument(
        '--elevator-id-dither-phase-rad',
        type=float,
        default=0.0,
        help='Phase of the optional F3-B1 elevator identification dither.',
    )
    parser.add_argument(
        '--template-world',
        type=Path,
        default=root / 'ros2_ws/src/ca_lsc_transition/worlds/nominal_empty.sdf',
    )
    parser.add_argument(
        '--source-model',
        type=Path,
        default=root / 'PX4-Autopilot/Tools/simulation/gz/models/standard_vtol',
    )
    args = parser.parse_args()
    if args.instrument_elevator_moment and not args.instrument_main_wings:
        parser.error('--instrument-elevator-moment requires --instrument-main-wings')
    if (
        abs(args.elevator_id_dither_amplitude_rad) > 0.0
        and (
            not args.instrument_elevator_moment
            or args.elevator_id_dither_frequency_hz <= 0.0
        )
    ):
        parser.error(
            'elevator dither requires --instrument-elevator-moment and a '
            'positive --elevator-id-dither-frequency-hz'
        )

    output_root = args.output_root.resolve()
    model_dir = output_root / 'models' / args.model_name
    world_path = output_root / 'worlds' / f'{args.model_name}.sdf'
    condition_path = output_root / 'condition.json'

    model_metadata = _copy_condition_model(
        args.source_model.resolve(),
        model_dir,
        args.model_name,
        mass_scale=args.mass_scale,
        payload_link_name=args.payload_link_name,
        payload_pose=tuple(args.payload_pose),
        payload_cube_size_m=args.payload_cube_size_m,
        instrument_main_wings=args.instrument_main_wings,
        instrument_elevator_moment=args.instrument_elevator_moment,
        elevator_id_dither_amplitude_rad=(
            args.elevator_id_dither_amplitude_rad
        ),
        elevator_id_dither_frequency_hz=args.elevator_id_dither_frequency_hz,
        elevator_id_dither_phase_rad=args.elevator_id_dither_phase_rad,
    )
    _write_world(
        args.template_world.resolve(), world_path, args.model_name,
        args.world_name, tuple(args.wind_enu), args.dynamic_wind,
    )

    condition = {
        'wind_model': (
            'dynamic_gazebo_world_wind' if args.dynamic_wind
            else 'steady_gazebo_world_wind'
        ),
        'dynamic_wind_enabled': args.dynamic_wind,
        'gz_wind_topic': f'/world/{args.world_name}/wind/',
        'wind_frame': 'ENU',
        'wind_velocity_mps': list(args.wind_enu),
        'world': str(world_path),
        'world_name': args.world_name,
        'model_name': args.model_name,
        'model_resource_path': str((output_root / 'models').resolve()),
        'mass_frame': 'model',
        'payload_link_name': (
            args.payload_link_name
            if model_metadata['payload_mass_kg'] > 0.0 else ''
        ),
        'payload_pose': list(args.payload_pose),
        'payload_cube_size_m': args.payload_cube_size_m,
        'qualification_status': 'generated_not_validated',
        'wing_force_ground_truth_enabled': args.instrument_main_wings,
        'elevator_moment_ground_truth_enabled': (
            args.instrument_main_wings and args.instrument_elevator_moment
        ),
        'elevator_id_dither_amplitude_rad': (
            args.elevator_id_dither_amplitude_rad
        ),
        'elevator_id_dither_frequency_hz': (
            args.elevator_id_dither_frequency_hz
        ),
        'elevator_id_dither_phase_rad': args.elevator_id_dither_phase_rad,
    }
    condition.update(model_metadata)
    condition_path.write_text(
        json.dumps(condition, indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(condition, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
