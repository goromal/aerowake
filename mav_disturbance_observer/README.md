# Disturbance observer based on an UKF
If using this code, please cite:
```
@article{tagliabue2019robust,
  title={Robust collaborative object transportation using multiple mavs},
  author={Tagliabue, Andrea and Kamel, Mina and Siegwart, Roland and Nieto, Juan},
  journal={The International Journal of Robotics Research},
  volume={38},
  number={9},
  pages={1020--1044},
  year={2019},
  publisher={SAGE Publications Sage UK: London, England}
}
```

## Instructions
### Force/Torque estimator
* Prerequisite: message definitions and utilities found in https://github.com/ethz-asl/mav_comm
* UKF force estimator publishes external force/torque estimate on */external_forces_moments*
* In order to work, make sure it is subscribed to 
	* */odometry*
	* */motor_speed* topics
* */odometry* should be remapped into the appropriate topic
```sh
<remap from="odometry" to="msf_core/odometry"/>
```
* And similarly for the /motor_speed topic
