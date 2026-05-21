#!/usr/bin/env python3
"""Generate a pedestrian-only intent-uncertainty stress world.

The generated scene is designed for CVaR-style planners: a small set of
pedestrians have low-probability branches that cut across the circular UAV
reference path, while background pedestrians keep the scene crowded without
dominating the benchmark.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "worlds" / "intent_uncertain" / "intent_branch_cvar_stress.world"
STRESS_VELOCITY_SCALE = 1.35


def fmt_pt(pt):
    return f"{pt[0]:.2f} {pt[1]:.2f} {pt[2]:.2f}"


def person_block(name, velocity, waypoints, choices, note):
    model_name = name if name.startswith("person") else f"person_{name}"
    velocity *= STRESS_VELOCITY_SCALE
    lines = [
        f"    <!-- {note} -->",
        f'    <model name="{model_name}_0.5_0.5_1.8">',
        "      <pose>0 0 0 0 0 0</pose>",
        '      <link name="link">',
        '        <collision name="bottom">',
        "          <pose>0 0 0.01 0 0 1.57</pose>",
        "          <geometry><box><size>0.35 0.75 0.02</size></box></geometry>",
        "        </collision>",
        '        <visual name="visual">',
        "          <pose>0 0 -0.02 0 0 1.57</pose>",
        "          <geometry><mesh><uri>model://person/meshes/walking.dae</uri></mesh></geometry>",
        "        </visual>",
        "      </link>",
        '      <plugin name="obstacle_motion" filename="libobstaclePathPlugin.so">',
        "        <loop>0</loop>",
        f"        <velocity>{velocity:.2f}</velocity>",
        "        <angular_velocity>0.8</angular_velocity>",
        "        <path>",
    ]
    for pt in waypoints:
        lines.append(f"          <waypoint>{fmt_pt(pt)}</waypoint>")
    lines.append("          <branch_point>")
    for prob, pts in choices:
        lines.append(f'            <choice prob="{prob:.2f}">')
        for pt in pts:
            lines.append(f"              <waypoint>{fmt_pt(pt)}</waypoint>")
        lines.append("            </choice>")
    lines.extend(
        [
            "          </branch_point>",
            "        </path>",
            "      </plugin>",
            "    </model>",
            "",
        ]
    )
    return "\n".join(lines)


def world_header():
    return """<sdf version='1.7'>
  <!--
    CVaR intent-uncertainty stress benchmark.

    This world is pedestrian-only and removes static obstacles. It is meant to
    expose low-probability high-consequence intent branches:

      * critical pedestrians share a pre-branch approach, so the realized branch
        is hidden until close to the UAV circular path;
      * danger branches use probability 0.18-0.25, matching cvar_alpha=0.20;
      * background pedestrians keep density comparable without making the start
        zone unsafe.

    obstaclePathPlugin samples one branch per pedestrian at Gazebo load time
    using OBS_BRANCH_SEED mixed with model name hash. All planners see the same
    realized motions for the same seed.
  -->
  <world name='default'>

"""


def world_footer():
    return """    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface>
            <contact><collide_bitmask>65535</collide_bitmask><ode/></contact>
            <friction><ode><mu>100</mu><mu2>50</mu2></ode><torsional><ode/></torsional></friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
  </world>
</sdf>
"""


CRITICAL = [
    ("critical01_left_upper", 0.72, [(-10.20, 4.30, 0.0), (-7.60, 4.30, 0.0)],
     [(0.60, [(-9.40, 5.90, 0.0), (-10.80, 5.90, 0.0)]),
      (0.22, [(-5.20, 4.30, 0.0), (-2.60, 4.30, 0.0)]),
      (0.18, [(-7.00, 5.10, 0.0), (-7.00, 6.20, 0.0)])],
     "critical: low-probability branch cuts across left-upper arc"),
    ("critical02_left_mid", 0.68, [(-10.40, 1.50, 0.0), (-7.45, 1.50, 0.0)],
     [(0.58, [(-9.10, 2.80, 0.0), (-10.60, 2.80, 0.0)]),
      (0.24, [(-4.80, 1.50, 0.0), (-2.80, 1.50, 0.0)]),
      (0.18, [(-7.00, 0.40, 0.0), (-7.00, -0.80, 0.0)])],
     "critical: danger branch enters the circular corridor from the left"),
    ("critical03_left_lower", 0.74, [(-9.80, -3.00, 0.0), (-6.90, -3.00, 0.0)],
     [(0.62, [(-8.80, -4.60, 0.0), (-10.20, -4.60, 0.0)]),
      (0.20, [(-4.40, -3.00, 0.0), (-2.60, -3.00, 0.0)]),
      (0.18, [(-6.30, -1.80, 0.0), (-6.30, -0.60, 0.0)])],
     "critical: rare branch crosses lower-left arc"),
    ("critical04_bottom_left", 0.78, [(-6.20, -10.00, 0.0), (-4.50, -6.90, 0.0)],
     [(0.60, [(-6.80, -8.60, 0.0), (-7.80, -9.60, 0.0)]),
      (0.22, [(-2.20, -5.40, 0.0), (-0.60, -4.60, 0.0)]),
      (0.18, [(-4.80, -5.60, 0.0), (-4.80, -4.20, 0.0)])],
     "critical: diagonal danger branch cuts into the lower arc"),
    ("critical05_bottom", 0.70, [(-1.80, -10.50, 0.0), (-0.40, -7.60, 0.0)],
     [(0.58, [(-2.90, -9.00, 0.0), (-4.60, -9.40, 0.0)]),
      (0.24, [(1.30, -5.80, 0.0), (2.90, -4.90, 0.0)]),
      (0.18, [(-0.30, -6.60, 0.0), (-0.30, -5.40, 0.0)])],
     "critical: bottom branch creates a late crossing near the UAV path"),
    ("critical06_bottom_right", 0.76, [(6.10, -10.10, 0.0), (4.50, -6.90, 0.0)],
     [(0.60, [(6.80, -8.70, 0.0), (8.10, -9.40, 0.0)]),
      (0.22, [(2.20, -5.40, 0.0), (0.80, -4.60, 0.0)]),
      (0.18, [(4.90, -5.60, 0.0), (4.90, -4.20, 0.0)])],
     "critical: mirrored lower-right crossing"),
    ("critical07_right_mid", 0.69, [(10.40, -1.20, 0.0), (7.45, -1.20, 0.0)],
     [(0.60, [(9.40, -2.90, 0.0), (10.70, -2.90, 0.0)]),
      (0.22, [(4.70, -1.20, 0.0), (2.80, -1.20, 0.0)]),
      (0.18, [(7.00, -0.10, 0.0), (7.00, 1.10, 0.0)])],
     "critical: rare branch enters from the right"),
    ("critical08_right_upper", 0.73, [(10.30, 3.40, 0.0), (7.25, 3.40, 0.0)],
     [(0.62, [(9.00, 5.00, 0.0), (10.60, 5.20, 0.0)]),
      (0.20, [(4.90, 3.40, 0.0), (2.70, 3.40, 0.0)]),
      (0.18, [(6.60, 4.70, 0.0), (6.60, 5.90, 0.0)])],
     "critical: right-upper danger branch crosses the path"),
    ("critical09_top_right", 0.67, [(7.60, 9.80, 0.0), (4.90, 5.60, 0.0)],
     [(0.60, [(7.20, 7.60, 0.0), (8.60, 7.80, 0.0)]),
      (0.22, [(2.30, 5.50, 0.0), (0.70, 5.30, 0.0)]),
      (0.18, [(4.90, 6.70, 0.0), (4.90, 7.80, 0.0)])],
     "critical: top-right branch stays outside start exclusion but crosses later"),
    ("critical10_top_left", 0.67, [(-7.60, 9.80, 0.0), (-4.90, 5.60, 0.0)],
     [(0.60, [(-7.20, 7.60, 0.0), (-8.60, 7.80, 0.0)]),
      (0.22, [(-2.30, 5.50, 0.0), (-0.70, 5.30, 0.0)]),
      (0.18, [(-4.90, 6.70, 0.0), (-4.90, 7.80, 0.0)])],
     "critical: top-left branch avoids takeoff but challenges the first lap"),
    ("critical11_inner_left", 0.62, [(-2.20, 10.80, 0.0), (-2.20, 7.60, 0.0)],
     [(0.65, [(-3.80, 8.60, 0.0), (-5.40, 8.80, 0.0)]),
      (0.20, [(-2.20, 5.00, 0.0), (-2.20, 3.40, 0.0)]),
      (0.15, [(-1.10, 7.20, 0.0), (0.30, 7.20, 0.0)])],
     "critical: delayed top-side crossing outside the start exclusion"),
    ("critical12_inner_right", 0.62, [(2.20, 10.80, 0.0), (2.20, 7.60, 0.0)],
     [(0.65, [(3.80, 8.60, 0.0), (5.40, 8.80, 0.0)]),
      (0.20, [(2.20, 5.00, 0.0), (2.20, 3.40, 0.0)]),
      (0.15, [(1.10, 7.20, 0.0), (-0.30, 7.20, 0.0)])],
     "critical: symmetric top-side low-probability crossing"),
]


BACKGROUND = [
    ("background01", 0.50, [(-12.00, -8.00, 0.0), (-12.00, -2.00, 0.0)],
     [(0.50, [(-12.00, 6.80, 0.0)]), (0.25, [(-13.40, 1.00, 0.0), (-13.40, 6.80, 0.0)]), (0.25, [(-10.80, 1.00, 0.0), (-10.80, 6.80, 0.0)])],
     "background: outside-density pedestrian"),
    ("background02", 0.55, [(12.00, 8.00, 0.0), (12.00, 2.00, 0.0)],
     [(0.50, [(12.00, -6.80, 0.0)]), (0.25, [(13.40, -1.00, 0.0), (13.40, -6.80, 0.0)]), (0.25, [(10.80, -1.00, 0.0), (10.80, -6.80, 0.0)])],
     "background: outside-density pedestrian"),
    ("background03", 0.52, [(-11.50, 8.80, 0.0), (-8.80, 8.80, 0.0)],
     [(0.50, [(-4.80, 8.80, 0.0)]), (0.25, [(-7.40, 10.20, 0.0), (-4.80, 10.20, 0.0)]), (0.25, [(-7.40, 7.50, 0.0), (-4.80, 7.50, 0.0)])],
     "background: upper crowd flow"),
    ("background04", 0.58, [(11.50, -8.80, 0.0), (8.80, -8.80, 0.0)],
     [(0.50, [(4.80, -8.80, 0.0)]), (0.25, [(7.40, -10.20, 0.0), (4.80, -10.20, 0.0)]), (0.25, [(7.40, -7.50, 0.0), (4.80, -7.50, 0.0)])],
     "background: lower crowd flow"),
    ("background05", 0.60, [(-13.00, 0.00, 0.0), (-10.20, 0.00, 0.0)],
     [(0.55, [(-10.20, -5.80, 0.0)]), (0.25, [(-11.60, -2.20, 0.0), (-11.60, -5.80, 0.0)]), (0.20, [(-9.00, -2.20, 0.0), (-9.00, -5.80, 0.0)])],
     "background: lateral traffic outside the circle"),
    ("background06", 0.60, [(13.00, 0.00, 0.0), (10.20, 0.00, 0.0)],
     [(0.55, [(10.20, 5.80, 0.0)]), (0.25, [(11.60, 2.20, 0.0), (11.60, 5.80, 0.0)]), (0.20, [(9.00, 2.20, 0.0), (9.00, 5.80, 0.0)])],
     "background: lateral traffic outside the circle"),
    ("background07", 0.48, [(-14.00, 4.80, 0.0), (-11.20, 4.80, 0.0)],
     [(0.55, [(-11.20, 9.20, 0.0)]), (0.25, [(-12.80, 6.40, 0.0), (-12.80, 9.20, 0.0)]), (0.20, [(-10.00, 6.40, 0.0), (-10.00, 9.20, 0.0)])],
     "background: upper-left density away from takeoff"),
    ("background08", 0.48, [(14.00, -4.80, 0.0), (11.20, -4.80, 0.0)],
     [(0.55, [(11.20, -9.20, 0.0)]), (0.25, [(12.80, -6.40, 0.0), (12.80, -9.20, 0.0)]), (0.20, [(10.00, -6.40, 0.0), (10.00, -9.20, 0.0)])],
     "background: lower-right density away from the circle"),
    ("background09", 0.54, [(-3.40, -12.20, 0.0), (-3.40, -9.40, 0.0)],
     [(0.50, [(-7.60, -9.40, 0.0)]), (0.25, [(-5.20, -10.80, 0.0), (-7.60, -10.80, 0.0)]), (0.25, [(-5.20, -8.20, 0.0), (-7.60, -8.20, 0.0)])],
     "background: bottom-left density"),
    ("background10", 0.54, [(3.40, 12.20, 0.0), (3.40, 9.40, 0.0)],
     [(0.50, [(7.60, 9.40, 0.0)]), (0.25, [(5.20, 10.80, 0.0), (7.60, 10.80, 0.0)]), (0.25, [(5.20, 8.20, 0.0), (7.60, 8.20, 0.0)])],
     "background: top-right density outside start zone"),
    ("background11", 0.57, [(-14.00, -5.80, 0.0), (-11.00, -5.80, 0.0)],
     [(0.55, [(-8.40, -5.80, 0.0)]), (0.25, [(-10.00, -7.20, 0.0), (-8.40, -7.20, 0.0)]), (0.20, [(-10.00, -4.40, 0.0), (-8.40, -4.40, 0.0)])],
     "background: lower-left lateral flow"),
    ("background12", 0.57, [(14.00, 5.80, 0.0), (11.00, 5.80, 0.0)],
     [(0.55, [(8.40, 5.80, 0.0)]), (0.25, [(10.00, 7.20, 0.0), (8.40, 7.20, 0.0)]), (0.20, [(10.00, 4.40, 0.0), (8.40, 4.40, 0.0)])],
     "background: upper-right lateral flow"),
]


def main():
    blocks = []
    for spec in CRITICAL + BACKGROUND:
        blocks.append(person_block(*spec))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(world_header() + "\n".join(blocks) + world_footer(), encoding="utf-8")
    print(OUT)


if __name__ == "__main__":
    main()
