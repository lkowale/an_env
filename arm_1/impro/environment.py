import os
for var in ['PYTHONPATH', 'ROS_ROOT', 'ROS_MASTER_URI', 'ROS_PACKAGE_PATH']:
    print('{} {}'.format(var, os.environ[var]))

# os.environ['PYTHONPATH'] = "/home/aa/an_env/devel/lib/python3/dist-packages:/home/aa/image_transport_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages:/home/aa/tensorflow/models/research:/home/aa/tensorflow/models/research/slim:/opt/ros/kinetic/lib/python2.7/dist-packages/"
# print(os.environ['PYTHONPATH'])