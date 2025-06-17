import numpy as np
import matplotlib.pyplot as plt
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Optimum_Performance.Range_speed import RangeAnalyzer
from Optimum_Performance.Optimum_speeds import OptimumSpeeds
from utils import Data, MissionType, plt

if __name__ == "__main__":
    file_path = "design3.json"
    mission_type = MissionType.DESIGN
    aircraft_data = Data(file_path)
    analyzer = RangeAnalyzer(file_path, mission_type)
    optimizer = OptimumSpeeds(aircraft_data, mission_type)

    def v_max(opt, h, W=None):
        """Use optimal range speed (default)"""
        return opt.v_max(h, W=W)
    
    def v_test(opt, h, W=None):
        """Example"""
        return opt.v_range(h, W=W)*1.25
    
    def v_endurance(opt, h, W=None):
        """Endurance speed"""
        return opt.v_endurance(h, W=W)
    
    def v_range(opt, h, W=None):
        """Range speed"""
        return opt.v_range(h, W=W)


    range_m, time1, time2 = analyzer.calculate_range_variable(vfunc_leg1=v_range, 
                                                      vfunc_leg2=v_range, 
                                                      payload_leg1=None,
                                                      payload_leg2=0, 
                                                      h=10,
                                                      numerical=True)
    print(f"Range: {range_m/1852/2:.2f} nmi, Time leg 1: {time1/3600:.2f} h, Time leg 2: {time2/3600:.2f} h")

    # Define speed percentages to test
    speed_percentages = np.arange(0.5, 1.45, 0.05)
    ranges = []
    times = []
    speeds = []
    vert_lines = []


    for percentage in speed_percentages:
        print(f"Testing speed factor: {percentage:.2f}")
        optimizer._current_weight = optimizer._mtow  # Reset weight to MTOW for each test
        def v_variable(opt, h, W=None):
            """Variable speed as percentage of range speed"""
            return opt.v_range(h, W=W) * percentage

        range_m_var, time1, time2 = analyzer.calculate_range_variable(vfunc_leg1=v_variable, 
                                                                        vfunc_leg2=None, 
                                                                        payload_leg1=None,
                                                                        payload_leg2=0, 
                                                                        h=10,
                                                                        numerical=True)
        ranges.append(range_m_var/1852/2)  # Convert to nautical miles
        times.append(time1/3600)  # Convert to hours
        speeds.append(percentage)


    # Plot range vs cruise speed with time on second y-axis
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    # Plot range
    color = 'tab:blue'
    ax1.set_xlabel('Speed Factor (× Range Speed)')
    ax1.set_ylabel('Range (nmi)', color=color)
    line1 = ax1.plot(speeds, ranges, 'b-o', linewidth=2, markersize=4, label='Range')
    ax1.tick_params(axis='y', labelcolor=color)
    
    # Create second y-axis for time
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('Time in Air (hours)', color=color)
    line2 = ax2.plot(speeds, times, 'r-s', linewidth=2, markersize=4, label='Time')
    ax2.tick_params(axis='y', labelcolor=color)
    
    # Add legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, 
                labels1 + labels2, 
                loc='upper right')
    
    plt.title('Range and Time vs Cruise Speed')
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()