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

In the next section, you will learn more about the ROS eco system.