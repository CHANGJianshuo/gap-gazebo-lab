#!/usr/bin/env python3
"""Small, pinned Humble JTC extension: interpolate and apply effort feedforward."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
repo=ROOT/'vendor_ws/src/ros2_controllers'
def replace(path,old,new):
    text=path.read_text()
    if new in text:return
    if text.count(old)!=1:raise RuntimeError(f'Unexpected upstream source: {path.name}')
    path.write_text(text.replace(old,new))
controller=repo/'joint_trajectory_controller/src/joint_trajectory_controller.cpp'
replace(controller,'tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +',
'''tmp_command_[i] =
              ((has_effort_command_interface_ && state_desired_.effort.size() == dof_)
                 ? state_desired_.effort[i] : 0.0) +
              (state_desired_.velocities[i] * ff_velocity_scale_[i]) +''')
replace(controller,'''    // reject effort entries
    if (!points[i].effort.empty())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Trajectories with effort fields are currently not supported.");
      return false;
    }''','''    // MTC: optional, finite effort feedforward is valid only for an effort controller.
    if (!points[i].effort.empty())
    {
      if (!has_effort_command_interface_ ||
          !validate_trajectory_point_field(joint_count, points[i].effort, "effort", i, false) ||
          !std::all_of(points[i].effort.begin(), points[i].effort.end(),
                       [](double value) { return std::isfinite(value); }))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid effort feedforward field");
        return false;
      }
    }''')
replace(repo/'joint_trajectory_controller/src/trajectory.cpp',
'''  double t[6];
  generate_powers(5, duration_so_far.seconds(), t);''',
'''  // MTC: effort is a feedforward signal, linearly interpolated independently of q/v/a.
  output.effort.clear();
  if (!state_a.effort.empty() || !state_b.effort.empty())
  {
    output.effort.resize(dim, 0.0);
    const double blend = duration_btwn_points.seconds() > 0.0
      ? duration_so_far.seconds() / duration_btwn_points.seconds() : 0.0;
    for (size_t i = 0; i < dim; ++i)
    {
      const double a = state_a.effort.size() == dim ? state_a.effort[i] : 0.0;
      const double b = state_b.effort.size() == dim ? state_b.effort[i] : 0.0;
      output.effort[i] = (1.0 - blend) * a + blend * b;
    }
  }
  double t[6];
  generate_powers(5, duration_so_far.seconds(), t);''')
(ROOT/'scripts/jtc_effort_feedforward.patch').write_text(subprocess.check_output(['git','diff','--','joint_trajectory_controller'],cwd=repo,text=True))
print('Pinned JTC effort feedforward extension applied.')
