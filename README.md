# Smart Plant Application

## Overview

The Smart Plant Application is a project designed to monitor and maintain the health of plants using various sensors. This application is developed as part of a university requirement, demonstrating the use of FreeRTOS for creating a multi-threaded application. By integrating sensors such as light, temperature, humidity, and soil moisture sensors, the application can track the plant's status and provide recommendations for optimal care.

## Features

- **Multi-threaded Architecture**: Utilizes FreeRTOS to handle multiple tasks and manage mutexes, ensuring efficient and synchronized access to shared resources.
- **Sensor Integration**: 
  - **Light Sensor**: Monitors the amount of light the plant receives to ensure it gets adequate sunlight.
  - **Temperature Sensor**: Tracks the ambient temperature around the plant to maintain an optimal growing environment.
  - **Humidity Sensor**: Measures the humidity level in the environment to prevent conditions that are too dry or too moist.
  - **Soil Moisture Sensor**: Checks the moisture level in the soil to ensure the plant is properly hydrated.
- **Data Analysis and Recommendations**: Analyzes the data collected from the sensors and provides actionable recommendations to improve plant health.
- **Future Extensions**: The application is designed to be extendable, with potential features like an automatic watering system and additional sensor integrations.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/smart-plant-application.git
