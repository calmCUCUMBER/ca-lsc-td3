"""Extract model-based aerodynamic and propulsion calibration from PX4 SDF.

The Gazebo ESC field named ``esc_rpm`` carries the motor angular-speed
command unchanged for this model, so all curves in this module use rad/s.
They are ideal simulation-model quantities, not measured electrical power.
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

from .capabilities import piecewise_lift_coefficient


@dataclass(frozen=True)
class LiftSurface:
    joint_name: str
    area_m2: float
    alpha_offset_rad: float
    lift_slope_per_rad: float
    stall_angle_rad: float
    post_stall_slope_per_rad: float
    control_lift_slope_per_rad: float
    cp_m: tuple[float, float, float]

    def lift_coefficient(self, alpha_rad: float, deflection_rad: float = 0.0) -> float:
        """Evaluate the SDF LiftDrag piecewise-linear coefficient model."""
        if not math.isfinite(alpha_rad) or not math.isfinite(deflection_rad):
            raise ValueError('alpha and deflection must be finite')
        return piecewise_lift_coefficient(
            geometric_alpha_rad=alpha_rad,
            alpha_offset_rad=self.alpha_offset_rad,
            lift_slope_per_rad=self.lift_slope_per_rad,
            stall_angle_rad=self.stall_angle_rad,
            post_stall_slope_per_rad=self.post_stall_slope_per_rad,
            control_lift_increment=(
                self.control_lift_slope_per_rad * deflection_rad
            ),
        )


@dataclass(frozen=True)
class Rotor:
    motor_number: int
    link_name: str
    max_omega_rad_s: float
    thrust_coefficient: float
    moment_constant_m: float

    def thrust_n(self, omega_rad_s: float) -> float:
        omega = _bounded_omega(omega_rad_s, self.max_omega_rad_s)
        return self.thrust_coefficient * omega * omega

    def ideal_mechanical_power_w(self, omega_rad_s: float) -> float:
        omega = _bounded_omega(omega_rad_s, self.max_omega_rad_s)
        return self.thrust_coefficient * self.moment_constant_m * omega ** 3


@dataclass(frozen=True)
class AirframeCalibration:
    source_sdf: str
    mass_kg: float
    air_density_kg_m3: float
    main_wings: tuple[LiftSurface, ...]
    elevator: LiftSurface
    elevator_min_rad: float
    elevator_max_rad: float
    rotors: tuple[Rotor, ...]

    @property
    def wing_area_m2(self) -> float:
        return sum(surface.area_m2 for surface in self.main_wings)

    def main_wing_cl(self, alpha_rad: float) -> float:
        area = self.wing_area_m2
        if area <= 0.0:
            raise ValueError('main-wing area must be positive')
        return sum(
            surface.area_m2 * surface.lift_coefficient(alpha_rad)
            for surface in self.main_wings
        ) / area

    @property
    def elevator_moment_derivative_per_dynamic_pressure(self) -> float:
        """Return dM_pitch/d(delta_e)/q in m^3 using the SDF force arm."""
        return (
            -self.elevator.cp_m[0]
            * self.elevator.area_m2
            * self.elevator.control_lift_slope_per_rad
        )

    @property
    def lift_rotors(self) -> tuple[Rotor, ...]:
        return tuple(rotor for rotor in self.rotors if rotor.motor_number < 4)

    @property
    def pusher(self) -> Rotor:
        matching = [rotor for rotor in self.rotors if rotor.motor_number == 4]
        if len(matching) != 1:
            raise ValueError('expected exactly one motor_number=4 pusher')
        return matching[0]


def _bounded_omega(value: float, maximum: float) -> float:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError('omega must be finite and non-negative')
    return min(value, maximum)


def _required_float(element: ET.Element, tag: str) -> float:
    child = element.find(tag)
    if child is None or child.text is None:
        raise ValueError(f'missing <{tag}> in SDF element')
    value = float(child.text)
    if not math.isfinite(value):
        raise ValueError(f'non-finite <{tag}> in SDF element')
    return value


def _required_text(element: ET.Element, tag: str) -> str:
    child = element.find(tag)
    if child is None or not child.text or not child.text.strip():
        raise ValueError(f'missing <{tag}> in SDF element')
    return child.text.strip()


def parse_standard_vtol_sdf(path: Path) -> AirframeCalibration:
    """Parse the stock standard_vtol SDF and reject incomplete models."""
    source = Path(path).expanduser().resolve()
    root = ET.parse(source).getroot()
    masses = [
        float(item.text)
        for item in root.findall('.//link/inertial/mass')
        if item.text is not None
    ]
    if not masses or any(not math.isfinite(item) or item <= 0.0 for item in masses):
        raise ValueError('SDF must contain positive finite link masses')

    surfaces: list[LiftSurface] = []
    densities: list[float] = []
    for plugin in root.findall('.//plugin'):
        if 'lift-drag' not in plugin.attrib.get('filename', ''):
            continue
        cp = tuple(float(item) for item in _required_text(plugin, 'cp').split())
        if len(cp) != 3:
            raise ValueError('lift surface cp must contain three coordinates')
        surface = LiftSurface(
            joint_name=_required_text(plugin, 'control_joint_name'),
            area_m2=_required_float(plugin, 'area'),
            alpha_offset_rad=_required_float(plugin, 'a0'),
            lift_slope_per_rad=_required_float(plugin, 'cla'),
            stall_angle_rad=_required_float(plugin, 'alpha_stall'),
            post_stall_slope_per_rad=_required_float(plugin, 'cla_stall'),
            control_lift_slope_per_rad=_required_float(
                plugin, 'control_joint_rad_to_cl'
            ),
            cp_m=(cp[0], cp[1], cp[2]),
        )
        surfaces.append(surface)
        densities.append(_required_float(plugin, 'air_density'))
    main_wings = tuple(surface for surface in surfaces if surface.area_m2 >= 0.1)
    elevators = [surface for surface in surfaces if surface.joint_name == 'servo_2']
    if len(main_wings) != 2 or len(elevators) != 1:
        raise ValueError('expected two main-wing surfaces and servo_2 elevator')
    if max(densities) - min(densities) > 1e-9:
        raise ValueError('inconsistent air density across lift surfaces')

    joint = root.find(".//joint[@name='servo_2']/axis/limit")
    if joint is None:
        raise ValueError('missing servo_2 joint limits')
    elevator_min = _required_float(joint, 'lower')
    elevator_max = _required_float(joint, 'upper')

    rotors: list[Rotor] = []
    for plugin in root.findall('.//plugin'):
        if 'multicopter-motor-model' not in plugin.attrib.get('filename', ''):
            continue
        rotors.append(Rotor(
            motor_number=int(_required_float(plugin, 'motorNumber')),
            link_name=_required_text(plugin, 'linkName'),
            max_omega_rad_s=_required_float(plugin, 'maxRotVelocity'),
            thrust_coefficient=_required_float(plugin, 'motorConstant'),
            moment_constant_m=_required_float(plugin, 'momentConstant'),
        ))
    rotors.sort(key=lambda rotor: rotor.motor_number)
    if [rotor.motor_number for rotor in rotors] != [0, 1, 2, 3, 4]:
        raise ValueError('expected motor numbers 0..4')

    return AirframeCalibration(
        source_sdf=str(source),
        mass_kg=sum(masses),
        air_density_kg_m3=densities[0],
        main_wings=main_wings,
        elevator=elevators[0],
        elevator_min_rad=elevator_min,
        elevator_max_rad=elevator_max,
        rotors=tuple(rotors),
    )


def _float_range(start: float, stop: float, step: float) -> Iterable[float]:
    count = int(round((stop - start) / step))
    for index in range(count + 1):
        yield start + index * step


def write_calibration(calibration: AirframeCalibration, output_dir: Path) -> dict[str, object]:
    """Write JSON plus CL, elevator-authority, and rotor curve CSV files."""
    destination = Path(output_dir).expanduser().resolve()
    destination.mkdir(parents=True, exist_ok=True)
    gravity = 9.80665
    hover_omega = math.sqrt(
        calibration.mass_kg * gravity
        / sum(rotor.thrust_coefficient for rotor in calibration.lift_rotors)
    )
    hover_omega_below_limit = all(
        hover_omega < rotor.max_omega_rad_s
        for rotor in calibration.lift_rotors
    )
    thrust_margin = (
        sum(rotor.thrust_n(rotor.max_omega_rad_s)
            for rotor in calibration.lift_rotors)
        / (calibration.mass_kg * gravity)
    )
    required_parameters_valid = bool(
        calibration.mass_kg > 0.0
        and calibration.wing_area_m2 > 0.0
        and len(calibration.main_wings) == 2
        and len(calibration.lift_rotors) == 4
        and calibration.elevator_min_rad < calibration.elevator_max_rad
        and math.isfinite(
            calibration.elevator_moment_derivative_per_dynamic_pressure
        )
        and abs(calibration.elevator_moment_derivative_per_dynamic_pressure) > 1e-9
    )
    model_calibration_pass = bool(
        required_parameters_valid
        and hover_omega_below_limit
        and math.isfinite(thrust_margin)
        and thrust_margin > 1.0
    )
    summary = {
        'calibration_kind': 'model_based_sdf',
        'lift_drag_a0_semantics': 'alpha_plugin_rad=alpha_geometric_rad+a0_rad',
        'power_kind': 'ideal_mechanical_model_not_electrical_measurement',
        'esc_speed_unit': 'rad_s_despite_esc_rpm_field_name_in_gz_sitl',
        'source_sdf': calibration.source_sdf,
        'mass_kg': calibration.mass_kg,
        'air_density_kg_m3': calibration.air_density_kg_m3,
        'wing_area_m2': calibration.wing_area_m2,
        'main_wings': [asdict(item) for item in calibration.main_wings],
        'elevator': asdict(calibration.elevator),
        'elevator_min_rad': calibration.elevator_min_rad,
        'elevator_max_rad': calibration.elevator_max_rad,
        'elevator_dmoment_ddelta_per_q_m3': (
            calibration.elevator_moment_derivative_per_dynamic_pressure
        ),
        'rotors': [asdict(item) for item in calibration.rotors],
        'hover_omega_rad_s': hover_omega,
        'hover_ideal_lift_power_w': sum(
            rotor.ideal_mechanical_power_w(hover_omega)
            for rotor in calibration.lift_rotors
        ),
        'checks': {
            'required_parameters_valid': required_parameters_valid,
            'hover_omega_below_lift_limit': hover_omega_below_limit,
            'lift_thrust_margin_over_weight': thrust_margin,
            'test_5_model_calibration_pass': model_calibration_pass,
        },
    }
    (destination / 'airframe_calibration.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8'
    )

    with (destination / 'cl_alpha.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=('alpha_deg', 'alpha_rad', 'cl'))
        writer.writeheader()
        for alpha_deg in _float_range(-10.0, 15.0, 0.5):
            alpha_rad = math.radians(alpha_deg)
            writer.writerow({
                'alpha_deg': alpha_deg,
                'alpha_rad': alpha_rad,
                'cl': calibration.main_wing_cl(alpha_rad),
            })

    with (destination / 'elevator_authority.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=('airspeed_mps', 'dynamic_pressure_pa', 'dmoment_ddelta_nm_per_rad'),
        )
        writer.writeheader()
        for airspeed in _float_range(0.0, 25.0, 0.5):
            dynamic_pressure = 0.5 * calibration.air_density_kg_m3 * airspeed ** 2
            writer.writerow({
                'airspeed_mps': airspeed,
                'dynamic_pressure_pa': dynamic_pressure,
                'dmoment_ddelta_nm_per_rad': (
                    dynamic_pressure
                    * calibration.elevator_moment_derivative_per_dynamic_pressure
                ),
            })

    with (destination / 'rotor_curves.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                'motor_number', 'omega_rad_s', 'thrust_n',
                'reaction_torque_nm', 'ideal_mechanical_power_w',
            ),
        )
        writer.writeheader()
        for rotor in calibration.rotors:
            for fraction in _float_range(0.0, 1.0, 0.02):
                omega = fraction * rotor.max_omega_rad_s
                thrust = rotor.thrust_n(omega)
                writer.writerow({
                    'motor_number': rotor.motor_number,
                    'omega_rad_s': omega,
                    'thrust_n': thrust,
                    'reaction_torque_nm': rotor.moment_constant_m * thrust,
                    'ideal_mechanical_power_w': rotor.ideal_mechanical_power_w(omega),
                })
    return summary
