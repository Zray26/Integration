import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args1 = ['si_utils', 'zl_ar_tracker_with_kinect.launch']

roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)

launch_files = roslaunch_file1

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

parent.start()
parent.spin()