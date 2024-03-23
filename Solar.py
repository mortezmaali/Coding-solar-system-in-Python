# -*- coding: utf-8 -*-
"""
Created on Sat Mar 23 10:31:45 2024

@author: Morteza
"""

import cv2
import numpy as np
import math

# Constants for simulation
G = 6.67430e-11  # Gravitational constant (m^3 kg^-1 s^-2)
AU = 1.496e11     # Astronomical unit in meters
day_in_seconds = 86400  # 1 day in seconds

# Planet data (semi-major axis in AU, period in days, radius in km)
planet_data = {
    "Mercury": {"semi_major_axis": 0.39, "period": 87.97, "radius": 2439.7},  # Adjusted Mercury semi-major axis
    "Venus": {"semi_major_axis": 0.723, "period": 224.7, "radius": 6051.8},  # Adjusted Venus radius
    "Earth": {"semi_major_axis": 1.0, "period": 365.25, "radius": 6371.0},   # Adjusted Earth radius
    "Mars": {"semi_major_axis": 1.524, "period": 686.98, "radius": 3389.5},
    "Jupiter": {"semi_major_axis": 3.5, "period": 4332.59, "radius": 39911.0},  # Adjusted Jupiter semi-major axis and radius
    "Saturn": {"semi_major_axis": 9.537, "period": 10759.22, "radius": 58232.0},
    "Uranus": {"semi_major_axis": 19.191, "period": 30688.5, "radius": 25362.0},
    "Neptune": {"semi_major_axis": 30.069, "period": 60182.0, "radius": 24622.0},
}

# Function to calculate planet positions
def calculate_position(planet, time_elapsed):
    period = planet_data[planet]["period"] * day_in_seconds  # Convert period to seconds
    semi_major_axis = planet_data[planet]["semi_major_axis"] * AU  # Convert AU to meters
    theta = 2 * math.pi * time_elapsed / period
    x = semi_major_axis * math.cos(theta)
    y = semi_major_axis * math.sin(theta)
    return x, y

# Function to draw the solar system
def draw_solar_system(frame, time_elapsed):
    center = (frame.shape[1] // 2, frame.shape[0] // 2)  # Center of the frame

    # Draw the sun
    sun_radius = 70
    cv2.circle(frame, center, sun_radius, (0, 255, 255), -1)  # Yellow color for sun

    for planet, data in planet_data.items():
        x, y = calculate_position(planet, time_elapsed)
        position = (int(center[0] + x * 1e-9), int(center[1] + y * 1e-9))  # Scaling for visualization
        radius = int(10*data["radius"] * 2e-4)  # Scaling for visualization and making planets smaller
        color = (255, 255, 255)  # White color for planets

        # Ensure the planet doesn't overlap with the sun
        min_distance = sun_radius + radius + 20  # Add a padding of 20 pixels
        distance = math.sqrt((position[0] - center[0])**2 + (position[1] - center[1])**2)
        if distance >= min_distance:
            cv2.circle(frame, position, radius, color, -1)

            # Put the name of the planet nearby
            name_position = (position[0] + radius + 20, position[1])
            cv2.putText(frame, planet, name_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

# Main function
def main():
    # Simulation parameters
    time_elapsed = 0  # in seconds

    # Create a black background
    width, height = 1600, 1000
    frame = np.zeros((height, width, 3), dtype=np.uint8)

    while True:
        # Clear the frame
        frame.fill(0)

        # Draw the solar system
        draw_solar_system(frame, time_elapsed)

        # Display the frame
        cv2.imshow("Solar System Simulation", frame)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Update time elapsed (Increase increment to make the planets move faster)
        time_elapsed += 100000  # Increment time by 500000 seconds for faster motion

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
