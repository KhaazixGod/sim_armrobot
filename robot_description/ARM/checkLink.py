import os
src = os.path.realpath(os.path.join('..', '..', 'src'))
urdf_path = os.path.join(src, 'robot_description', 'ARM', 'robot.urdf')
print(f'URDF path: {urdf_path}')