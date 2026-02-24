# launch/dwa_pso_planner.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument("controller_step_ms", default_value="100.0"),
        DeclareLaunchArgument("eps_goal",           default_value="1e-2"),

        DeclareLaunchArgument("linear_vel_max",     default_value="10.0"),
        DeclareLaunchArgument("angular_vel_max",    default_value="5.0"),
        DeclareLaunchArgument("linear_acc_max",     default_value="2.0"),
        DeclareLaunchArgument("angular_acc_max",    default_value="5.0"),

        DeclareLaunchArgument("alpha",              default_value="1.0"),
        DeclareLaunchArgument("gamma",              default_value="0.2"),

        DeclareLaunchArgument("pso_max_iter",       default_value="30"),
        DeclareLaunchArgument("particles",          default_value="30"),

        DeclareLaunchArgument("eps_head",           default_value="1e-2"),
        DeclareLaunchArgument("eps_cost",           default_value="1e-4"),
        DeclareLaunchArgument("patience",           default_value="5"),

        DeclareLaunchArgument("cognitive_coeff",    default_value="2.0"),
        DeclareLaunchArgument("social_coeff",       default_value="2.0"),
        DeclareLaunchArgument("init_inertia_coeff", default_value="0.9"),
        DeclareLaunchArgument("final_inertia_coeff",default_value="0.4"),
    ]

    node = Node(
        package="ros2_dwa_pso",
        executable="dwa_pso_planner",
        name="dwa_pso_planner",
        output="screen",
        parameters=[{
            "dt_ms":      LaunchConfiguration("controller_step_ms"),
            "eps_goal":   LaunchConfiguration("eps_goal"),

            "limits.max_vel.linear":  LaunchConfiguration("linear_vel_max"),
            "limits.max_vel.angular": LaunchConfiguration("angular_vel_max"),
            "limits.max_acc.linear":  LaunchConfiguration("linear_acc_max"),
            "limits.max_acc.angular": LaunchConfiguration("angular_acc_max"),

            "alpha": LaunchConfiguration("alpha"),
            "gamma": LaunchConfiguration("gamma"),

            "imax":  LaunchConfiguration("pso_max_iter"),
            "n_par": LaunchConfiguration("particles"),

            "eps_head": LaunchConfiguration("eps_head"),
            "eps_cost": LaunchConfiguration("eps_cost"),
            "patience": LaunchConfiguration("patience"),

            "acc_cog":    LaunchConfiguration("cognitive_coeff"),
            "acc_soc":    LaunchConfiguration("social_coeff"),
            "iner_start": LaunchConfiguration("init_inertia_coeff"),
            "iner_end":   LaunchConfiguration("final_inertia_coeff"),
        }]
    )

    return LaunchDescription(args + [node])