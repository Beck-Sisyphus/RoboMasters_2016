trapezoid

ROS driver for the trapezoid board arduino system.

	Published Topics
		/trapezoid/pose (geometry_msgs/PoseStamped)
			Pose of the trapezoid board in quaternion representation. Position is unused.

	Subscribed Topics
		/trapezoid/turret_pose (geometry_msgs/PoseStamped)
			Requested orientation of the turret in quaternion representation. Only pitch and yaw components of the quarternion are used.

	Services
		/trapezoid/shoot (trapezoid/Shoot)
			Shoot the projectile.
			trapezoid/Shoot.srv
				int32 pwm_speed # The PWM setting. Controls the projectile speed.
				int32 duration  # Milliseconds to keep auto-shooting. If set to 0, fire only one projectile.
				---
				bool result # Returns true if successful, false otherwise.