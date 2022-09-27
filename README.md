# ROS2 Humble tutorials
This is a simple package to get started with ROS2 Humble.

## Create a ROS2 package

To start, first make sure ROS2 Humble is installed. If this is the case, you can source the ROS2 Humble installation with:

```sh
source /opt/ros/humble/setup.bash
```

Next, create a workspace in which you will place all the source code of this tutorial. A common name for the workspace is `ros2_ws`. To create the workspace, use:

```sh
mkdir -p ~/ros2_ws/src/
```

This will create a directory called `ros2_ws` and inside another directory called `src`. Now enter the top level directory with:

```sh
cd ~/ros2_ws/
```

Now, you can build this workspace with:

```sh
colcon build
```

Now, you could either clone this package from GitHub with the following line:

```sh
cd src
git clone https://github.com/Dave-van-der-Meer/ros_tutorials.git
```

Or you can follow this tutorial by creating all the files by yourself and get a better understanding on what you are doing. To do this, start by creating a package called `my_turtlesim`:

```sh
ros2 pkg create --build-type ament_python my_turtlesim
```

The folloiwng text will be displayed in the terminal:

```
going to create a new package
package name: my_turtlesim
destination directory: /home/dave/drs_ws/src/ros_tutorials
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['dave <dave@davesroboshack.com>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: []
creating folder ./my_turtlesim
creating ./my_turtlesim/package.xml
creating source folder
creating folder ./my_turtlesim/my_turtlesim
creating ./my_turtlesim/setup.py
creating ./my_turtlesim/setup.cfg
creating folder ./my_turtlesim/resource
creating ./my_turtlesim/resource/my_turtlesim
creating ./my_turtlesim/my_turtlesim/__init__.py
creating folder ./my_turtlesim/test
creating ./my_turtlesim/test/test_copyright.py
creating ./my_turtlesim/test/test_flake8.py
creating ./my_turtlesim/test/test_pep257.py
```

**Note:** With the flag `--build-type ament_python` in the previous command, we tell the terminal that we want to create a Python package. In ROS2, C++ packages and Python packages have a slightly different architecture.

When you now list the content of the package directory with:

```sh
ls my_turtlesim
```

You will find the following content:

```
my_turtlesim  package.xml  resource  setup.cfg  setup.py  test
```

The directory `my_turtlesim` is meant to contain the Python scripts. The file `setup.py` contains information on how to build this package when using `colcon build` in the parent directory of the workspace. The content of the `setup.py` file will look as follows:

```py
from setuptools import setup

package_name = 'my_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dave',
    maintainer_email='dave@davesroboshack.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

The file `package.xml` contains meta data about the package such as the required dependencies. The content of that file looks as fllows:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_turtlesim</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="dave@davesroboshack.com">dave</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Create a ROS2 Python file

As mentionned before, the Python scripts will be placed inside the `my_turtlesim/my_turtlesim` folder. You can create a new file with the command:

```sh
cd my_turltesim/my_turtlesim/
gedit my_first_program.py
```

A text editor will open with a blank document. This document will be your first Python ROS2 script. Add the following code to the document and save it:

```python
import rclpy
from rclpy.node import Node


class MyFirstProgram(Node):

    def __init__(self):
        super().__init__('my_first_program')
        self.get_logger().info('Hello, World')


def main(args=None):
    rclpy.init(args=args)

    my_first_program = MyFirstProgram()

    my_first_program.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Now, you can close the program and in the terminal, use the following command to make this Python script exectuable:

```sh
chmod +x my_firstprogram.py
```
Next, you need to tell ROS to actually use this new script as part of this package. This is done by adding a line in the `setup.py` file. Open that file by navigating to it:

```sh
cd ~/ros2_ws/src/my_turtlesim
gedit config.py
```

At the bottom of this document, you can see the following part:

```python
    entry_points={
        'console_scripts': [
        ],
```

You need to add one line so that it looks as follows:

```python
    entry_points={
        'console_scripts': [
            'my_first_node = my_turtlesim.my_first_program:main',
        ],
```
When you build the package later, this line will make sure that the Python script will be seen as a valid ROS node with the name `my_first_node` which can be found inside the `my_turtlesim` package and the source code is saved in the file `my_first_program.py`.

After that, go to the main directory of the `ros2_ws` and compile build your package:

```sh
cd ~/ros2_ws/
colcon build
```
The terminal should give you the following output:

```
Starting >>> my_turtlesim
Finished <<< my_turtlesim [1.06s]

Summary: 1 package finished [1.28s]
```

Next, you need to source this workspace so that ROS2 will take into account all the packages in this workspace:

```sh
source install/local_setp.bash
```
Now, everything is ready to run your new program. For this, use the following command:

```sh
ros2 run my_turtlesim my_first_node
```

The terminal should output the following text:

```sh
[INFO] [1663744547.799484529] [my_first_program]: Hello, World!
```

This is it, you just grated your first Python program in ROS2! Congratulations! Of course, this program is not doing anything useful yet. But ths is the absolute minimum you need to run a proper ROS2 Python program.

## Create a launch file for a Python package

Source: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

When you started the ROS2 node, you were using the `ros2 run` command. This command allows you to run a single node. In many cases, you will need to run several nodes at the same time so they can work together. Or, you want to run the node with different input arguments (which is a more advanced topic already). In these cases, it can be useful to create a launch file.

In ROS2, launch files can be one of thre file types: Python files, YAML files, or XML files. The most popular type are Python files as they allow for more complex logic compared to YAML or XML.

Start by creating a new directory inside the main directory of your package and name it `launch`:

```sh
cd ~/ros2_ws/src/my_turtlesim/
mkdir launch/
cd launch/
```
Inside this folder, you can create a file with the name `simple_launch.launch.py`. Note that the launch files usually end with the extensions `.launch.py`, `.launch.yaml` or `.launch.xml`.

```sh
gedit simple_launch.launch.py
```

Now, inside this document, enter the following code:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_turtlesim',
            executable='my_first_node',
            name='my_first_program',
            output='screen',
        )
    ])
```

**Note:** The `executable` name is not the name of the Python script, but the name indicated in the `setup.py` script. It is also the same name as when you use `ros2 run` to start the individual node.

Save the program and close the text editor. Now, make the file executable:

```sh
chmod +x simple_launch.launch.py
```

Next, you need to modify the file called `setup.py`:

```sh
cd ../
gedit setup.py
```

The file will currently look like this:

```python
from setuptools import setup

package_name = 'my_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dave',
    maintainer_email='dave@davesroboshack.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_first_node = my_turtlesim.my_first_program:main',
        ],
    },
)
```

First, we need to import the `glob` module and the `os` module at the top. Next, we need to add the path to the launch file to the package build instructions. To do this, modify the part of `data_files`.

```python
from setuptools import setup
from glob import glob
import os

package_name = 'my_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dave',
    maintainer_email='dave@davesroboshack.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_first_node = my_turtlesim.my_first_program:main',
        ],
    },
)
```

It is also recommended to add the `ros2launch` dependency to the `package.xml` file. Open it with:

```sh
gedit package.xml
```

And then add the following line:

```xml
<exec_depend>ros2launch</exec_depend>
```
The file should then look similar to the following example:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_turtlesim</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="dave@davesroboshack.com">dave</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
To be able to use the launch file, the package needs to be build again. This needs to be done each time when nw files are added to the package or when existing files are modified. Go to the root directory of the workspace:

```sh
cd ~/ros2_ws/
colcon build
```
The terminal should give you the following output:

```
Starting >>> my_turtlesim
Finished <<< my_turtlesim [1.10s]

Summary: 1 package finished [1.32s]
```

Next, you need to source this workspace so that ROS2 will know about the added launch file:

```sh
source install/local_setp.bash
```

Now, you can start the launch file with the following command:

```sh
ros2 launch my_turtlesim simple_launch.launch.py
```

The terminal will give you the following output:

```
ros2 launch my_turtlesim simple_launch.launch.py 
[INFO] [launch]: All log files can be found below /home/dave/.ros/log/2022-09-27-10-15-55-369032-adminuser-Precision-5560-140732
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [my_first_node-1]: process started with pid [140734]
[my_first_node-1] [INFO] [1664266555.799257203] [my_first_program]: Hello, World!
[INFO] [my_first_node-1]: process has finished cleanly [pid 140734]
```
Here, the line `[my_first_node-1] [INFO] [1664266555.799257203] [my_first_program]: Hello, World!` is the actual output of the node that you started, and the other lines just tell you that the launc file has started and finished successfully.

That's it for now. Of course, launch files have much more possibilities such as adding arguments and parameters or even remapping some names. In order to work with these more advanced tools, you need to get a wider foundation about how ROS works.

In the next section, you will learn more about the ROS eco system.