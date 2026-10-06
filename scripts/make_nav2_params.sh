#!/bin/bash
# Builds ~/wheelbot/nav2_params.yaml from the Nav2 defaults with this robot's shape + safe speeds.
# Re-run after changing a number here.
#
# Footprint (m, from the point between the back wheels; x forward, y left):
#   front 0.27 = caster at 242 mm + a bit, back 0.07 = tyre radius 37 mm + margin,
#   half width 0.17 = (305.5 mm track + tyre) / 2. Body itself is 282.7 mm wide.
FOOTPRINT='[[0.27, 0.17], [0.27, -0.17], [-0.07, -0.17], [-0.07, 0.17]]'
VX_MAX=0.25      # m/s forward (default 0.5) - slow for events
VX_MIN=-0.15     # m/s reverse
WZ_MAX=0.8       # rad/s turning (default 1.9)
INFLATION=0.45   # m keep-away from obstacles (default 0.70 is too fat for tight rooms)

src=/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml
dst=~/wheelbot/nav2_params.yaml
sed -e "s/^\( *\)robot_radius: 0.22/\1footprint: \"$FOOTPRINT\"/" \
    -e "s/^\( *\)vx_max: 0.5/\1vx_max: $VX_MAX/" \
    -e "s/^\( *\)vx_min: -0.35/\1vx_min: $VX_MIN/" \
    -e "s/^\( *\)wz_max: 1.9/\1wz_max: $WZ_MAX/" \
    -e "s/^\( *\)inflation_radius: 0.70*$/\1inflation_radius: $INFLATION/" \
    -e "s/^\( *\)max_velocity: \[0.5, 0.0, 2.0\]/\1max_velocity: [$VX_MAX, 0.0, $WZ_MAX]/" \
    -e "s/^\( *\)min_velocity: \[-0.5, 0.0, -2.0\]/\1min_velocity: [$VX_MIN, 0.0, -$WZ_MAX]/" \
    -e "s/^\( *\)controller_frequency: 20.0/\1controller_frequency: 10.0/" \
    -e "s/^\( *\)model_dt: 0.05/\1model_dt: 0.1/" \
    -e "s/^\( *\)transform_tolerance: 0.[12]$/\1transform_tolerance: 0.5/" \
    -e "s/^\( *\)default_server_timeout: 20$/\1default_server_timeout: 500/" \
    -e "s/^      global_frame: map$/&\n      initial_transform_timeout: 600.0  # wait 10 min for 2D Pose Estimate (default 60 s aborts Nav2)/" \
    "$src" > "$dst"

# Lighter CPU load for the Pi 5 (overload made the collision monitor stop the robot on late scans):
#   MPPI 1000 samples x 40 steps (was 2000 x 56), local costmap 3 Hz (was 5),
#   collision monitor accepts scans up to 3 s old (was 1), TF tolerance 1.0 s (was 0.5).
sed -i -e "s/^\( *\)batch_size: 2000$/\1batch_size: 1000/" \
       -e "s/^\( *\)time_steps: 56$/\1time_steps: 40/" \
       -e "s/^\( *\)update_frequency: 5.0$/\1update_frequency: 3.0/" \
       -e "s/^\( *\)source_timeout: 1.0$/\1source_timeout: 3.0/" \
       -e "s/^\( *\)transform_tolerance: 0.5$/\1transform_tolerance: 1.0/" \
       -e "s/^\( *\)expected_planner_frequency: 20.0$/\1expected_planner_frequency: 5.0/" \
    "$dst"

# Localization accuracy (AMCL was +-0.9 m / +-18 deg in the corridor):
#   120 beams (was 60), update every 10 cm / ~6 deg (was 25 cm / 11 deg),
#   trust the map more: z_hit 0.7 / z_rand 0.3 (was 0.5 / 0.5), sigma_hit 0.15 (was 0.2).
# Goal reached within 15 cm (was 25).
sed -i -e "s/^\( *\)max_beams: 60$/\1max_beams: 120/" \
       -e "s/^\( *\)update_min_d: 0.25$/\1update_min_d: 0.1/" \
       -e "s/^\( *\)update_min_a: 0.2$/\1update_min_a: 0.1/" \
       -e "s/^\( *\)z_hit: 0.5$/\1z_hit: 0.7/" \
       -e "s/^\( *\)z_rand: 0.5$/\1z_rand: 0.3/" \
       -e "s/^\( *\)sigma_hit: 0.2$/\1sigma_hit: 0.15/" \
       -e "s/^\( *\)xy_goal_tolerance: 0.25$/\1xy_goal_tolerance: 0.15/" \
    "$dst"

# Fail loudly if a default changed and a substitution silently missed.
for want in "footprint: \"" "vx_max: $VX_MAX" "wz_max: $WZ_MAX" "inflation_radius: $INFLATION" "max_velocity: \[$VX_MAX" "controller_frequency: 10.0" "model_dt: 0.1" "initial_transform_timeout: 600.0" "default_server_timeout: 500" "batch_size: 1000" "time_steps: 40" "source_timeout: 3.0" "max_beams: 120" "update_min_d: 0.1" "z_hit: 0.7" "sigma_hit: 0.15" "xy_goal_tolerance: 0.15"; do
  grep -q "$want" "$dst" || { echo "make_nav2_params: '$want' not applied - check $src"; exit 1; }
done
grep -c "footprint: \"" "$dst" | grep -q 2 || { echo "make_nav2_params: footprint not set in both costmaps"; exit 1; }
echo "wrote $dst"
