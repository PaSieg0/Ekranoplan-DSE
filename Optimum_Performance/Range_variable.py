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
    
    def v_stall(opt, h, W=None):
        """Stall speed"""
        return opt.v_stall(h, W=W)

    range_m, time1, time2 = analyzer.calculate_range_variable(vfunc_leg1=v_range, 
                                                      vfunc_leg2=v_range, 
                                                      payload_leg1=None,    
                                                      payload_leg2=0, 
                                                      h=10,
                                                      numerical=True)

    W4_1, W5_1 = analyzer.calculate_cruise_weights_leg1()
    W4_2, W5_2 = analyzer.calculate_cruise_weights_leg2()
    analyzer.plot_speed_vs_weight(analyzer.opt._mtow, W5_2, velocity_func_list=[v_max, v_range, v_endurance, v_stall],
                                  labels=["Max Speed", "Range Speed", "Endurance Speed", "Stall Speed"])
    print(f"Range: {range_m/1852/2:.2f} nmi, Time leg 1: {time1/3600:.2f} h, Time leg 2: {time2/3600:.2f} h")
    # Define speed percentages to test
    speed_percentages = np.arange(0.7, 1, 0.02)
    ranges = []
    times = []
    speeds = []
    vert_lines = []
    max_stall_speed_range = 0
    min_max_speed_range = 0
    legs = 1

    for percentage in speed_percentages:
        
        def v_variable(opt, h, W=None):
            """Variable speed as percentage of range speed"""
            return opt.v_range(h, W=W) * percentage

        range_m_var, time1, time2 = analyzer.calculate_range_variable(vfunc_leg1=v_variable, 
                                                                        vfunc_leg2=None if legs == 2 else v_variable, 
                                                                        payload_leg1=None if legs == 2 else 0,
                                                                        payload_leg2=0, 
                                                                        h=10,
                                                                        numerical=True, 
                                                                        no_stop=True if legs == 1 else False)
        
        if range_m_var is None or time1 is None or time2 is None:
            print(f"Speed Factor {percentage:.2f} Failed. Skipping.")
            continue
        else:
            # Remember first valid speed (above stall)
            if not max_stall_speed_range:
                max_stall_speed_range = percentage            
            # Update max valid speed
            min_max_speed_range = percentage

        if legs == 2:
            ranges.append(range_m_var/1852/2)  # Convert to nautical miles
            times.append(time1/3600)  # Convert to hours
            speeds.append(percentage)
        elif legs == 1:
            ranges.append(range_m_var/1852)
            times.append((time1+time2)/3600)  # Convert to hours
            speeds.append(percentage)
        print(f"Speed Factor {percentage:.2f} Succeeded.")


    # Calculate the endurance to range speed ratio
    v_range_speed = optimizer.v_range(10)  # Get range speed at altitude 10
    v_endurance_speed = optimizer.v_endurance(10)  # Get endurance speed at altitude 10
    endurance_factor = v_endurance_speed / v_range_speed
    
    # Plot range vs cruise speed with time on second y-axis
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    # Plot range
    color = 'tab:blue'
    ax1.set_xlabel('Speed Factor (× Range Speed)')
    ax1.set_ylabel('Range (nmi)', color=color)
    line1 = ax1.plot(speeds, ranges, '-o', linewidth=2, markersize=4, label='Range', color=color)
    ax1.tick_params(axis='y', labelcolor=color)
    
    # Create second y-axis for time
    ax2 = ax1.twinx()
    color = 'tab:orange'
    ax2.set_ylabel('Time in Air (hours)', color=color)
    line2 = ax2.plot(speeds, times, '-s', linewidth=2, markersize=4, label='Time', color=color)
    ax2.tick_params(axis='y', labelcolor=color)
    
    # Add vertical line for stall speed
    # ax1.axvline(x=max_stall_speed_range, color='red', linestyle='--', linewidth=2, alpha=0.7, label='Stall Speed')
    # ax1.axvline(x=min_max_speed_range, color='green', linestyle='--', linewidth=2, alpha=0.7, label='Max Speed')
    
    # Add vertical lines for range and endurance speeds
    ax1.axvline(x=1.0, color='tab:blue', linestyle=':', linewidth=2, alpha=0.7, label='Range Speed')
    ax1.axvline(x=endurance_factor, color='tab:orange', linestyle=':', linewidth=2, alpha=0.7, label='Endurance Speed')
    
    # Add legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, 
                labels1 + labels2, 
                loc='lower center')
    
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()