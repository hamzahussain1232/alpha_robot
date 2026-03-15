from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "articubot_one"

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    image_topic = LaunchConfiguration("image_topic", default="/camera/image_raw")
    output_root = LaunchConfiguration(
        "output_root",
        default=PathJoinSubstitution([FindPackageShare(package_name), "assets", "ml", "dataset", "images_all"]),
    )
    class_name = LaunchConfiguration("class_name", default="medicine_bottle")
    save_every_n_frames = LaunchConfiguration("save_every_n_frames", default="5")
    max_images = LaunchConfiguration("max_images", default="300")
    show_preview = LaunchConfiguration("show_preview", default="false")

    capture_node = Node(
        package=package_name,
        executable="capture_dataset_node.py",
        name="capture_dataset_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "image_topic": image_topic,
                "output_root": output_root,
                "class_name": class_name,
                "save_every_n_frames": save_every_n_frames,
                "max_images": max_images,
                "show_preview": show_preview,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("output_root", default_value=output_root),
            DeclareLaunchArgument("class_name", default_value="medicine_bottle"),
            DeclareLaunchArgument("save_every_n_frames", default_value="5"),
            DeclareLaunchArgument("max_images", default_value="300"),
            DeclareLaunchArgument("show_preview", default_value="false"),
            capture_node,
        ]
    )
