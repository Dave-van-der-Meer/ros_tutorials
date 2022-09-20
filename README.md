# ros_tutorials
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
my_turtlesim  package.xml  resource  setup.cfg  setup.py  test
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