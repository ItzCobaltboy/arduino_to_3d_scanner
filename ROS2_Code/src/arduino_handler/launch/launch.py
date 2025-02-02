import launch
import launch.actions
import launch_ros.actions
import launch.events

def generate_launch_description():
    serial_handler = launch_ros.actions.Node(
        package='arduino_handler',
        executable='serial_handler',
        name='serial_handler',
        output='screen'
    )

    data_handler = launch_ros.actions.Node(
        package='arduino_handler',
        executable='data_handler',
        name='data_handler',
        output='screen'
    )

    # Event handler to start csv_to_3d_plot.py after data_handler exits
    csv_plot_script = launch.actions.ExecuteProcess(
        cmd=['python3', 'csv_to_3d_plot.py'],  # Make sure the path is correct
        output='screen'
    )

    event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=data_handler,  # Wait for data_handler to exit
            on_exit=[csv_plot_script]
        )
    )

    return launch.LaunchDescription([
        serial_handler,
        data_handler,
        event_handler  # Add event handler to the launch description
    ])
