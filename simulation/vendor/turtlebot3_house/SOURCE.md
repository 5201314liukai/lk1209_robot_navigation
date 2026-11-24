# TurtleBot3 House

ROBOTIS TurtleBot3 Simulations, Jazzy branch, commit `45633014a14e8f438495b532a723e4ad45cbbd31`.
The world and house model come from [the pinned official repository](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/45633014a14e8f438495b532a723e4ad45cbbd31/turtlebot3_gazebo). The repository license is included in `LICENSE`.

The original world is `worlds/turtlebot3_house.world`. The runnable LK1209 entry is `../../worlds/turtlebot3_house.sdf`. The entry resolves Ground Plane and Sun from the local resource directory and omits the obsolete empty GUI camera pose frame attribute. The official apartment's layout, model poses, visuals, lights and collision geometry are retained.

The packaged house model has four visual identifiers with an added `_visual` suffix on the table legs. The original names duplicated collision identifiers in the same link and failed the SDFormat 14 uniqueness check. Original model SHA-256 `d3159d9a357b62fb5794301a641fba40f4f47e302033988115a67df9db82fc33`; deployed model SHA-256 `c315e5c53007d3c7e7b245996bae7008242506716fcf83833f4fa254bdef8b3d`. Two relative links next to `cafe_table.dae` resolve its original texture names to the packaged image files.

Fuel dependencies by OpenRobotics, both [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/):

- [Ground Plane v5](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground%20Plane/5/Ground%20Plane.zip), ZIP SHA-256 `bb1394826b055aae349363859e58c04383dc25a34251a288593a1f283746836e`.
- [Sun v3](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun/3/Sun.zip), ZIP SHA-256 `4d5a9ae9cc9ba0a5c713521c817fb061e581b734b2cf268691cd702f9b0c285e`.

Both Gazebo resource search paths, `GZ_SIM_RESOURCE_PATH` for runtime and `SDF_PATH` for the standalone checker, must include this directory's `models/`.
