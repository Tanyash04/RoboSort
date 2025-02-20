# RoboSort: Automated Lid Separation for Yogurt Containers

## Overview

RoboSort is an innovative robotic system designed to automate the separation of aluminum lids from plastic yogurt containers, enhancing recycling efficiency and reducing manual intervention in waste processing facilities. The system leverages a digital twin simulation framework using the Franka Emika Panda robot within the PyBullet physics simulation environment.

## Key Features

- **Digital Twin Simulation**: Utilizes PyBullet to create a realistic simulation environment for the Franka Emika Panda robot, modeling interactions between the robot, cup, and lid.
- **CNN-Based Grasp Detection**: Employs a Convolutional Neural Network (CNN) to accurately detect optimal grasp points for the robot to lift and separate lids from containers.
- **Dynamic Mesh Modeling**: Supports various lid configurations through dynamic mesh modeling, allowing the system to adapt to different container designs.
- **Synthetic Dataset Generation**: Generates synthetic datasets to train the CNN model, enhancing its accuracy in grasp point detection.
- **Real-Time Depth Sensing and Motion Planning**: Incorporates depth sensing and motion planning algorithms to enable the robot to perform precise and adaptive movements during the separation process.
- **Progressive Force Adaptation**: Implements force control strategies to ensure reliable and gentle separation of lids without damaging the containers.

## System Architecture

The project comprises several interconnected components:

1. **3D Modeling & Simulation**: Creates a simulated environment with dynamic mesh models of yogurt cups and lids, configurable lid angles, and physics properties using PyBullet.
2. **CNN-Based Grasp Detection**: Utilizes a ResNet50v2-based CNN model trained on a diverse dataset, achieving high accuracy in real-time grasp point prediction.
3. **Motion Planning & Control**: Employs inverse kinematics, force-sensitive grasping, collision avoidance, and trajectory optimization to execute precise robotic movements.

## Project Structure

- **3dmodels/**: Contains 3D models used for simulation.
- **CNN_Model/**: Includes the implementation and training scripts for the CNN-based grasp detection model.
- **scripts/**: Houses scripts for robot operations, control sequences, and integration of the CNN classifier with hardware commands.
- **static/** and **templates/**: Provide assets and HTML templates for the web interface/dashboard used to monitor and control the system.
- **Help.docx**: Offers additional documentation with setup details, usage guidelines, and troubleshooting information.
- **requirements.txt**: Lists Python dependencies required to run the project.
- **README.md**: This file, providing an overview of the project, installation instructions, usage details, and more.

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/RoboSort.git
   ```
2. **Navigate to the Project Directory**:
   ```bash
   cd RoboSort
   ```
3. **Install Dependencies**:
   Ensure you have Python 3.8 or higher installed, then run:
   ```bash
   pip install -r requirements.txt
   ```
4. **Additional Setup**:
   - Refer to `Help.docx` for detailed setup instructions and hardware configuration.
   - Adjust configuration settings as per your environment.

## Usage

- **Launching the Simulation**:
  - Run the main application script to start the simulation:
    ```bash
    python app.py
    ```
  - Access the web-based dashboard by navigating to `http://localhost:5000` in your browser to monitor and control operations.

## Contributing

Contributions are welcome! To contribute:

- Fork the repository.
- Create a new branch for your feature or bug fix.
- Ensure your code follows the project’s style guidelines and includes appropriate tests.
- Submit a pull request with a clear description of your changes.

## Acknowledgments

Special thanks to all contributors, mentors, and supporters who have helped bring RoboSort to life. This project builds upon various open-source libraries and frameworks that have made this work possible.

## Contact

For questions or further information, please open an issue on GitHub or contact the repository maintainer. 
