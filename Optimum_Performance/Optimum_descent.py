import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Class_I.AltitudeVelocity import AltitudeVelocity
from utils import Data, MissionType, ISA
from Optimum_speeds import OptimumSpeeds

def calculate_descent_distance(altitude_velocity, h_start, h_end, num_points=None):
    """
    Calculate the total horizontal distance covered during descent using numerical integration.
    
    Parameters:
    - altitude_velocity: AltitudeVelocity object
    - h_start: Starting altitude (higher altitude)
    - h_end: Ending altitude (lower altitude)
    - num_points: Number of integration points (default: altitude difference + 1)
    
    Returns:
    - total_distance: Total horizontal distance covered (m)
    - final_AoD: Final angle of descent (rad)
    - final_V_min_aod: Final minimum velocity for angle of descent (m/s)
    """
    if num_points is None:
        num_points = int(abs(h_start - h_end)) + 1
    
    altitudes = np.linspace(h_start, h_end, num_points)
    distances = []
    
    # Variables to store the final values
    final_AoD = 0
    final_V_min_aod = 0
    
    for i in range(len(altitudes) - 1):
        h_current = altitudes[i]
        h_next = altitudes[i + 1]
        
        # Calculate angle of descent and velocity at current altitude
        AoD, V_min_aod = altitude_velocity.calculate_min_AoD(h_current)
        
        # Store the values (will be the last calculated values after loop)
        final_AoD = AoD
        final_V_min_aod = V_min_aod
        
        # Calculate altitude change (positive for descent)
        dh = abs(h_current - h_next)
        
        # Calculate horizontal distance for this segment
        if AoD != 0 and np.tan(abs(AoD)) != 0:
            dx = dh / np.tan(abs(AoD))
            distances.append(dx)
        else:
            distances.append(0)
    
    total_distance = np.sum(distances)
    return total_distance, final_AoD, final_V_min_aod

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.DESIGN  # Example mission type
    altitude_velocity = AltitudeVelocity(aircraft_data, mission_type)

    V_cruise = aircraft_data.data['requirements']['cruise_speed']
    h_WIG = 5  # Wing in ground effect altitude
    h_WOG = 3048  # Wing out of ground effect altitude
    RoC_req = 1000/196.85  # Required rate of climb

    # Calculate descent distance using integration
    total_distance_covered, final_AoD, final_V_min_aod = calculate_descent_distance(
        altitude_velocity, h_WOG, h_WIG, num_points=500
    )
    print("===============Optimum speeds==================")
    
    # Convert to IAS if needed (assuming final_V_min_aod is already in appropriate units)
    V_min_aod_ias = final_V_min_aod  # Adjust conversion if needed
    
    print(f'Optimum descent speed for range: {V_min_aod_ias:.2f} m/s IAS at AoD: {final_AoD*180/np.pi:.2f} degrees')
    print(f"Distance covered: {total_distance_covered/1000:.2f} km in {h_WOG-h_WIG} m of descent")

    # altitude_velocity.plot_force_curve([h_WIG, h_WOG])