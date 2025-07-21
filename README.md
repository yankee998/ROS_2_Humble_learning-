# ROS 2 Humble Learning 🚀

Welcome to the **ROS_2_Humble_learning-** repository! This project is designed to help you learn and explore **ROS 2 Humble** through simple, practical examples. It includes two packages: `minimal_pkg` for a basic ROS 2 node and `pubsub_py` for a publisher-subscriber system that generates and visualizes a sine wave. Whether you're a beginner or brushing up on ROS 2, this repo is a great starting point! 🎉

## 📋 Project Overview

This repository contains two ROS 2 packages:
- **`minimal_pkg`**: A minimal ROS 2 Python package with a basic node to demonstrate the structure of a ROS 2 package.
- **`pubsub_py`**: A Python package implementing a publisher-subscriber system where `sine_wave_pub` publishes a sine wave signal, and `sine_wave_sub_and_plot` subscribes to it and plots the data using `matplotlib`.

## 🌟 Features

- **Simple Publisher-Subscriber Example**: Learn how to create and use ROS 2 topics with `pubsub_py`.
- **Real-Time Data Visualization**: Visualize sine wave data using `matplotlib` in the `sine_wave_sub_and_plot` node.
- **Clean and Modular Code**: Follows ROS 2 best practices with clear package structure and documentation.
- **Easy Setup**: Detailed instructions for configuring and running the project on Ubuntu with ROS 2 Humble.

## 📂 Project Structure

```
📦 ROS_2_Humble_learning-
 ┣ 📂 minimal_pkg
 ┃ ┣ 📂 minimal_pkg
 ┃ ┃ ┣ 📜 __init__.py
 ┃ ┃ ┗ 📜 minimal_node.py
 ┃ ┣ 📜 package.xml
 ┃ ┣ 📂 resource
 ┃ ┃ ┗ 📜 minimal_pkg
 ┃ ┣ 📜 setup.cfg
 ┃ ┣ 📜 setup.py
 ┃ ┗ 📂 test
 ┃   ┣ 📜 test_copyright.py
 ┃   ┣ 📜 test_flake8.py
 ┃   ┗ 📜 test_pep257.py
 ┗ 📂 pubsub_py
   ┣ 📜 package.xml
   ┣ 📂 pubsub_py
   ┃ ┣ 📜 __init__.py
   ┃ ┣ 📜 sine_wave_pub.py
   ┃ ┗ 📜 sine_wave_sub_and_plot.py
   ┣ 📂 resource
   ┃ ┗ 📜 pubsub_py
   ┣ 📜 setup.cfg
   ┣ 📜 setup.py
   ┗ 📂 test
     ┣ 📜 test_copyright.py
     ┣ 📜 test_flake8.py
     ┗ 📜 test_pep257.py
```

## 🛠 Prerequisites

To run this project, you need:
- **Ubuntu 22.04 (Jammy Jellyfish)** 🐧
- **ROS 2 Humble Hawksbill** installed (follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html))
- **Python 3.10** (default for ROS 2 Humble)
- **colcon** build tool
- Python packages: `rclpy`, `matplotlib`

## ⚙️ Setup Instructions

Follow these steps to configure and run the project:

1. **Clone the Repository**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/yankee998/ROS_2_Humble_learning-.git
   ```

2. **Install Dependencies**:
   Ensure ROS 2 Humble and required Python packages are installed:
   ```bash
   sudo apt update
   sudo apt install python3-rosdep python3-colcon-common-extensions
   sudo apt install python3-matplotlib
   pip3 install rclpy
   ```

3. **Build the Workspace**:
   Navigate to your ROS 2 workspace and build the packages:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. **Source the Workspace**:
   Source the setup file to make the packages available:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

   Add this to your `~/.bashrc` for convenience:
   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## 🚀 Running the Nodes

### 1. Run the Minimal Node
The `minimal_pkg` contains a simple ROS 2 node to demonstrate basic functionality.

```bash
ros2 run minimal_pkg minimal_node
```

### 2. Run the Publisher-Subscriber System
The `pubsub_py` package includes two nodes:
- `sine_wave_pub`: Publishes a sine wave signal to the `sinusoidal_signal` topic.
- `sine_wave_sub_and_plot`: Subscribes to the topic and plots the data using `matplotlib`.

**Step 1**: Start the publisher in one terminal:
```bash
ros2 run pubsub_py sine_wave_pub
```

**Step 2**: In another terminal, start the subscriber and plotter:
```bash
ros2 run pubsub_py sine_wave_sub_and_plot
```

The subscriber will collect data for 15 seconds and display a plot of the sine wave.

**Tip**: To debug, use the `--ros-args --log-level DEBUG` flag:
```bash
ros2 run pubsub_py sine_wave_sub_and_plot --ros-args --log-level DEBUG
```

## 🐞 Troubleshooting

- **No executable found**:
  - Ensure the `entry_points` in `pubsub_py/setup.py` include:
    ```python
    'console_scripts': [
        'sine_wave_pub = pubsub_py.sine_wave_pub:main',
        'sine_wave_sub_and_plot = pubsub_py.sine_wave_sub_and_plot:main',
    ]
    ```
  - Verify the script files exist in `pubsub_py/pubsub_py/`.
  - Rebuild and source: `colcon build --packages-select pubsub_py && source install/setup.bash`.

- **Missing dependencies**:
  - Install `matplotlib`: `pip3 install matplotlib`.
  - Ensure `rclpy` and `std_msgs` are available: `pip3 install rclpy`.

- **Plot not displaying**:
  - Check that `matplotlib` is installed and the display environment is set (e.g., run `export DISPLAY=:0` if on a Linux GUI).

## 📚 Learning Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Understanding ROS 2 Nodes](https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Nodes.html)
- [Publisher-Subscriber in ROS 2](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

## 🤝 Contributing

Contributions are welcome! Feel free to:
- Open issues for bugs or suggestions.
- Submit pull requests with improvements 

## 📧 Contact

For questions or feedback, reach out to [yaredgenanaw99@gmail.com](mailto:yaredgenanaw99@gmail.com).

Happy ROS 2 learning! 🌈