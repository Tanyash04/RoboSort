# RoboSort

This project focuses on simulating robotic grasping tasks using the Franka Emika Panda robot within the PyBullet physics simulation environment. The primary objective is to develop and execute various grasping scenarios, such as lifting a lid from a cup, by leveraging PyBullet's capabilities to model and control robotic interactions.

## Project Structure

- **3dmodels/**  
  Contains 3D models used for simulation or design purposes.
  
- **CNN_Model/**  
  Houses the implementation, training scripts, and resources for the CNN model that performs object classification.
  
- **scripts/**  
  Contains scripts for managing robot operations, control sequences, and integrating the CNN classifier with hardware commands.
  
- **static/** and **templates/**  
  Provide assets and HTML templates for the web interface/dashboard used to monitor and control the system.
  
- **Help.docx**  
  Additional documentation with setup details, usage guidelines, and troubleshooting information.
  
- **requirements.txt**  
  A list of Python dependencies required to run the project.

- **README.md**  
  (This file) provides an overview of the project, installation instructions, usage details, and more.

## Features

- **Automated Lid Separation :**  
  Utilizes a CNN-based classifier to detect the grasping point and separate lid.
  
- **Robotic Control Integration:**  
  Seamless coordination between machine learning models and robotic control scripts to execute separation task.
  
- **User Interface:**  
  A web-based dashboard for monitoring system performance and managing operations.
  
- **3D Simulation:**  
  Leverages 3D models for simulating sorting operations, assisting in design and testing (if applicable).

## Installation

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/Tanyash04/RoboSort.git
   ```
2. **Navigate to the Project Directory:**
   ```bash
   cd RoboSort
   ```
3. **Install Dependencies:**
   Ensure you have Python installed, then run:
   ```bash
   pip install -r requirements.txt
   ```
4. **Additional Setup:**
   - Refer to `Help.docx` for detailed setup instructions and hardware configuration.
   - Adjust configuration settings (if any) as per your environment.

## Usage

- **Launching the Dashboard:**
  - Start the web server to access the system dashboard:
    ```bash
    python app.py
    ```
  - Open your browser and navigate to `http://localhost:5000` (or the configured port) to monitor operations.

## Contributing

Contributions are welcome! To contribute:
- Fork the repository.
- Create a new branch for your feature or bug fix.
- Ensure your code follows the project’s style guidelines and includes appropriate tests.
- Submit a pull request with a clear description of your changes.


## Acknowledgments

- Special thanks to all contributors, mentors, and supporters who have helped bring RoboSort to life.
- This project builds upon various open source libraries and frameworks that have made this work possible.

## Contact

For questions or further information, please open an issue on GitHub or contact the repository maintainer.
