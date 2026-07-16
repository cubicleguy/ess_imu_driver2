"""Launch file for Epson IMU node publishing on /imu/data_raw without orientation data for ess_imu_driver2 package"""

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial port name",
            ),
            DeclareLaunchArgument(
                name="frame_id",
                default_value="imu_link",
                description="IMU message frame_id field",
            ),
            DeclareLaunchArgument(
                name="imu_dout_rate",
                # value: output rate (Hz)     Recommended Moving Average Filter
                # 0: 2000                     TAP>=0
                # 1: 1000                     TAP>=2
                # 2: 500                      TAP>=4
                # 3: 250                      TAP>=8
                # 4: 125                      TAP>=16
                # 5: 62.5                     TAP>=32
                # 6: 31.25                    TAP>=64
                # 7: 15.625                   TAP=128
                # 8: 400                      TAP>=8
                # 9: 200                      TAP>=16
                # 10: 100                     TAP>=32
                # 11: 80                      TAP>=32
                # 12: 50                      TAP>=64
                # 13: 40                      TAP>=64
                # 14: 25                      TAP=128
                # 15: 20                      TAP=128
                default_value="4",
                description="Sets data output rate of IMU",
            ),
            DeclareLaunchArgument(
                name="imu_filter_sel",
                # value: Filter Setting
                # 0: bypass
                # 1: Moving Average TAP2
                # 2: Moving Average TAP4
                # 3: Moving Average TAP8
                # 4: Moving Average TAP16
                # 5: Moving Average TAP32
                # 6: Moving Average TAP64
                # 7: Moving Average TAP128
                # 8: KAISER TAP32 Fc=50 Hz
                # 9: KAISER TAP32 Fc=100 Hz
                # 10: KAISER TAP32 Fc=200 Hz
                # 11: KAISER TAP32 Fc=400 Hz
                # 12: KAISER TAP64 Fc=50 Hz
                # 13: KAISER TAP64 Fc=100 Hz
                # 14: KAISER TAP64 Fc=200 Hz
                # 15: KAISER TAP64 Fc=400 Hz
                # 16: KAISER TAP128 Fc=50 Hz
                # 17: KAISER TAP128 Fc=100 Hz
                # 18: KAISER TAP128 Fc=200 Hz
                # 19: KAISER TAP128 Fc=400 Hz
                default_value="6",
                description="Sets the IMU filter",
            ),
            DeclareLaunchArgument(
                name="quaternion_output_en",
                default_value="0",
                description="Disables quaternion output for orientation",
            ),
            DeclareLaunchArgument(
                name="output_32bit_en",
                default_value="true",
                description="Enables all sensor data output in 32-bit resolution or 16-bit resolution.",
            ),
            DeclareLaunchArgument(
                name="time_correction_en",
                default_value="false",
                description="Enables using IMU external counter reset function for timestamp with external 1PPS connected to IMU input pin for GPIO2/EXT",
            ),
            launch_ros.actions.Node(
                package="ess_imu_driver2",
                executable="ess_imu_driver2_node",
                output="screen",
                parameters=[
                    {
                        "__log_level": "INFO",
                        "serial_port": LaunchConfiguration("serial_port"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "imu_dout_rate": LaunchConfiguration("imu_dout_rate"),
                        "imu_filter_sel": LaunchConfiguration("imu_filter_sel"),
                        "quaternion_output_en": LaunchConfiguration(
                            "quaternion_output_en"
                        ),
                        "output_32bit_en": LaunchConfiguration("output_32bit_en"),
                        "time_correction_en": LaunchConfiguration("time_correction_en"),
                    },
                ],
                remappings=[
                    ("/epson_imu/data", "/imu/data"),
                    ("/epson_imu/data_raw", "/imu/data_raw"),
                ],
            ),
        ]
    )
