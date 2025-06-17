import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from typing import Tuple, Dict
import warnings

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType, plt
from Class_I.PayloadRange import RangeCalculator
from Optimum_Performance.Optimum_speeds import OptimumSpeeds
from Class_II.Small_Iteration import Ainf_Ah
import random

def plot_speed_vs_time(details, label, payload_drop_time=None):
    times = np.array(details['times']) / 3600  # Convert seconds to hours
    speeds = np.array(details['speeds'])

    plt.figure(figsize=(10, 6))
    # Plot leg 1 (start to payload drop)
    if payload_drop_time is not None:
        payload_drop_time_hr = payload_drop_time / 3600
        leg1_mask = times <= payload_drop_time_hr
        leg2_mask = times > payload_drop_time_hr

        plt.plot(times[leg1_mask], speeds[leg1_mask], color='tab:blue', label='Leg 1 (Outbound)')
        plt.plot(times[leg2_mask], speeds[leg2_mask], color='tab:orange', label='Leg 2 (Return)')
        plt.axvline(payload_drop_time_hr, color='k', linestyle=':', linewidth=2, label='Payload Drop')
    else:
        plt.plot(times, speeds, color='tab:red', label='Speed')

    plt.xlabel('Time (h)')
    plt.ylabel('Speed (m/s)')
    plt.title(f'Aircraft Speed vs Time ({label})')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

# Concatenate integration details for total mission (leg 1 + leg 2)
def concat_details(details_a, details_b):
    combined = {}
    for key in details_a:
        if isinstance(details_a[key], list):
            # For 'times' and 'weights', adjust the second leg so it continues from the end of the first
            if key == 'times':
                offset = details_a[key][-1] if details_a[key] else 0
                combined[key] = details_a[key] + [t + offset for t in details_b[key]]
            elif key == 'weights':
                combined[key] = details_a[key] + details_b[key]
            elif key == 'speeds':
                combined[key] = details_a[key] + details_b[key]
            elif key == 'lift_drag_ratios':
                combined[key] = details_a[key] + details_b[key]
            else:
                combined[key] = details_a[key] + details_b[key]
        elif isinstance(details_a[key], (int, float)):
            combined[key] = details_a[key] + details_b[key]
        else:
            combined[key] = [details_a[key], details_b[key]]
    return combined

class RangeAnalyzer:
    """
    Analyzes aircraft range using both numerical integration and analytical methods.
    Includes sanity checks to validate results.
    """
    
    def __init__(self, file_path: str, mission_type: MissionType, sea_state: int = 0):
        """Initialize the range analyzer with aircraft data and mission type."""
        self.aircraft_data = Data(file_path)
        self.mission_type = mission_type
        self.opt = OptimumSpeeds(self.aircraft_data, mission_type)
        self.rng = RangeCalculator(data_object=self.aircraft_data, mission_type=mission_type)

        self.max_wave_height = [0, 0.1, 0.5, 1.25, 2.5, 4.0, 6., 9., 14.][sea_state]  # Example wave heights in meters
        # Assume TO takes 10% more fuel per sea state unit approximation
        self.rng.Mff_nocruise *= (1-(1-self.rng.fuel_fracs_no_cruise[1])*(1+self.max_wave_height/3))/self.rng.fuel_fracs_no_cruise[1]
        # Adjust cruise altitude for sea state
        wave_margin_0 = self.aircraft_data.data['inputs']['cruise_altitude'] - self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage']
        self.aircraft_data.data['inputs']['cruise_altitude'] = wave_margin_0 + self.max_wave_height + self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage']

        # Calculate ground effect factor
        self.k_wing = np.sqrt(1/Ainf_Ah(self.aircraft_data.data['inputs']['cruise_altitude'],
                                        self.aircraft_data.data['outputs']['wing_design']['b'], 
                                        self.aircraft_data.data['inputs']['endplate_height']))
        self.h_tail = (self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['tail_height'] + self.aircraft_data.data['inputs']['cruise_altitude'])
        self.k_tail = np.sqrt(1/Ainf_Ah(self.h_tail, self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']))
        self.k = self.k_wing * self.k_tail  # Combined ground effect factor
        
        # Constants
        self.METERS_TO_NMI = 1852
        self.TOLERANCE_PERCENT = 1.0  # Acceptable difference percentage for sanity check
        self.weight_step = 100  # Weight step for numerical integration (N)
        
    def calculate_cruise_weights_leg1(self) -> Tuple[float, float]:
        """Calculate weights at start and end of cruise segment."""
        mass_fractions = self.rng.calculate_mass_fractions()
        W4_W5 = self.rng.calculate_weight_ratios(mass_fractions)
        
        W4 = self.opt._mtow * self.rng.Mff_nocruise  # Weight at start of cruise
        W5 = W4 / W4_W5['design']                     # Weight at end of cruise
        
        return W4, W5
    
    def calculate_cruise_weights_leg2(self) -> Tuple[float, float]:
        """Calculate weights at start and end of cruise segment."""
        mass_fractions = self.rng.calculate_mass_fractions()
        W4_W5 = self.rng.calculate_weight_ratios(mass_fractions)
        
        W4_leg1 = self.opt._mtow * self.rng.Mff_nocruise  # Weight at start of cruise
        W5_leg1 = W4_leg1 / W4_W5['design']                     # Weight at end of 
        
        W4_leg2 = (W5_leg1-self.aircraft_data.data['requirements']['design_payload']*9.81) * self.rng.Mff_nocruise  # Weight at start of cruise for leg 2
        W5_leg2 = W4_leg2 / W4_W5['design']  # Weight at end of cruise for leg 2

        # Check if W5_leg2 is close to zero fuel weight (OEW)
        oew = self.opt._oew + self.aircraft_data.data['outputs']['design']['reserve_fuel']
        if not np.isclose(W5_leg2, oew, rtol=0.01):
            warnings.warn(f"W5_leg2 ({W5_leg2:.2f} N) is not close to OEW ({oew:.2f} N)")

        return W4_leg2, W5_leg2
    
    def validate_flight_capability(self, V: float, weight: float, h: float) -> bool:
        """Check if the aircraft can fly at given speed and weight."""
        self.opt._current_weight = weight
        angle_of_climb = self.opt.calculate_AoC(V, h)
        
        if angle_of_climb < 0-1e-6:  # Allow small negative AoC for numerical stability
            print(f"Warning: Cannot fly at V={V:.2f} m/s with weight={weight/9.81:.2f} kg")
            print(f"Angle of climb: {angle_of_climb:.4f} degrees")
            return False
        return True
    
    def calculate_numerical_range(self, W4: float, W5: float, velocity_func=None, h: float = 10, dW: float = 100) -> Tuple[float, float, Dict]:
        """
        Calculate range using numerical integration.
        
        Args:
            W4: Weight at start of cruise (N)
            W5: Weight at end of cruise (N)
            velocity_func: Function to calculate velocity at each weight.
                          Should take (optimizer, altitude) as arguments and return velocity.
                          If None, uses default v_range (optimal range speed).
            h: Altitude (m)
            dW: Weight step for integration (N)
            
        Returns:
            Tuple of (range in meters, integration details dict)
        """
        R_numerical = 0
        t = 0
        W_array = np.arange(W4, W5, -dW)
        
        integration_details = {
            'weights': [],
            'times': [],	
            'speeds': [],
            'lift_drag_ratios': [],
            'valid_points': 0,
            'invalid_points': 0
        }
        
        for weight in W_array:
            self.opt._current_weight = weight
            
            # Calculate velocity using provided function or default
            if velocity_func is not None:
                V = velocity_func(self.opt, h, weight)
            else:
                V = self.opt.v_range(h, W=weight)  # Default: optimal range speed
            
            # Validate flight capability
            if not self.validate_flight_capability(V, weight, h):
                integration_details['invalid_points'] += 1
                continue
            
            # Calculate lift-to-drag ratio with ground effect correction
            Cl_Cd = self.opt.L_over_D(V, h, weight)
            Cl_Cd_ge = Cl_Cd * self.k
            
            # Integration step
            val_integration = (self.opt._prop_efficiency / 
                             self.aircraft_data.data['inputs']['prop_consumption'] * 
                             Cl_Cd_ge * (1/weight) / self.opt.g)
            
            dR = val_integration * dW
            R_numerical += dR
            t += dR / V  # Time increment
            
            # Store details for analysis
            integration_details['weights'].append(weight)
            integration_details['speeds'].append(V)
            integration_details['times'].append(t)
            integration_details['lift_drag_ratios'].append(Cl_Cd_ge)
            integration_details['valid_points'] += 1
        
        return R_numerical, t, integration_details
    
    def calculate_analytical_range(self, W4: float, W5: float, velocity_func=None, h: float = 10) -> float:
        """
        Calculate range using analytical formula (Breguet range equation).
        
        Args:
            W4: Weight at start of cruise (N)
            W5: Weight at end of cruise (N)
            h: Altitude (m)
            
        Returns:
            Range in meters
        """
        # Use representative weight for L/D calculation (geometric mean)
        W_representative = np.sqrt(W4 * W5)
        
        if velocity_func is not None:
            V = velocity_func(self.opt, h)
        else:
            V = self.opt.v_range(h, W=W_representative)  # Default: optimal range speed
        # Calculate representative L/D ratio
        Cl_Cd = self.opt.L_over_D(V, h, W_representative)
        Cl_Cd_ge = Cl_Cd * self.k
        
        # Breguet range equation
        R_analytical = (self.opt._prop_efficiency / 
                       self.aircraft_data.data['inputs']['prop_consumption'] * 
                       Cl_Cd_ge * np.log(W4 / W5) / self.opt.g)
        
        print(f"Analytical range: {R_analytical/1852:.2f} nmi, Cl/Cd: {Cl_Cd_ge:.4f}, W4: {W4/W5:.2f}")

        t_analytical = R_analytical / V  # Time for analytical range  
        
        return R_analytical, t_analytical
    
    def calculate_range_variable(self, 
                        vfunc_leg1: callable = None, 
                        vfunc_leg2: callable = None, 
                        payload_leg1: float = None, 
                        payload_leg2: float = 0, 
                        h: float = 10,
                        numerical: bool = False
                        ) -> Tuple[float, float, float]:
        """
        Calculate total range for the mission using both legs.
        
        Args:
            vfunc_leg1: Velocity function for leg 1 (optional).
            vfunc_leg2: Velocity function for leg 2 (optional).
            payload_leg1: Payload for leg 1 (kg), if None uses design payload.
            payload_leg2: Payload for leg 2 (kg), default is 0.
            h: Altitude (m)
            numerical: If True, use numerical integration, otherwise use analytical formula (only for constant L/D).
            
        Returns:
            Total range in meters and time in seconds for both legs.
        """
        if payload_leg1 is None:
            payload_leg1 = self.aircraft_data.data['requirements']['design_payload']
        else:
            # Update MTOW based on the new payload
            payload_diff = payload_leg1 - self.aircraft_data.data['requirements']['design_payload']
            self.opt._mtow += payload_diff * 9.81  # Adjust MTOW by payload difference

        W4_leg1 = self.opt._mtow * self.rng.Mff_nocruise  # Weight at start of cruise for leg 1
        W5_leg1 = 0 # this is the variable we want to calculate for R_leg1 = R_leg2
        W4_leg2 = (W5_leg1 - (payload_leg1-payload_leg2) * 9.81) * self.rng.Mff_nocruise  # Weight at start of cruise for leg 2
        W5_leg2 = self.aircraft_data.data['outputs']['design']['ZFW'] + self.aircraft_data.data['outputs']['design']['reserve_fuel'] - (self.aircraft_data.data['requirements']['design_payload']-payload_leg2) * 9.81

        # Iteratively find W5_leg1 such that R_leg1 equals R_leg2
        W5_leg1_min = W5_leg2
        W5_leg1_max = W4_leg1

        tolerance = 10000  # Big tolerance for convergence (meters)

        max_iterations = 100
        dW = self.weight_step*5  # Initial weight step for numerical integration
        range_diff = float('inf')  # Initialize range difference

        for iteration in range(max_iterations):
            prev_range_diff = range_diff
            W5_leg1 = (W5_leg1_min + W5_leg1_max) / 2 + random.uniform(-dW, dW)  # Randomize W5_leg1 within bounds
            W4_leg2 = (W5_leg1 - (payload_leg1 - payload_leg2) * 9.81) * self.rng.Mff_nocruise
            
            if numerical:
                # Calculate numerical range for both legs
                R_leg1, time_leg1, _ = self.calculate_numerical_range(W4_leg1, W5_leg1, vfunc_leg1, h, dW=dW)
                R_leg2, time_leg2, _ = self.calculate_numerical_range(W4_leg2, W5_leg2, vfunc_leg2, h, dW=dW)
            else:
                R_leg1, time_leg1 = self.calculate_analytical_range(W4_leg1, W5_leg1, vfunc_leg1, h)
                R_leg2, time_leg2 = self.calculate_analytical_range(W4_leg2, W5_leg2, vfunc_leg2, h)
            
            range_diff = R_leg1 - R_leg2
            
            if abs(range_diff) < tolerance:
                break
            elif range_diff < 0:  # R_leg1 > R_leg2, need to increase W5_leg1
                W5_leg1_max = W5_leg1
            else:  # R_leg1 < R_leg2, need to increase W5_leg1
                W5_leg1_min = W5_leg1

            if numerical and abs(range_diff - prev_range_diff) < 0.1 * abs(prev_range_diff):
                # If range difference is roughly equal to previous (within 1%), reduce weight step for better precision
                dW = dW // 2  # Reduce weight step

            # print(f"Iteration {iteration+1}: W5_leg1 = {W5_leg1:.2f} N, R_leg1 = {R_leg1/1852:.2f} nmi, R_leg2 = {R_leg2/1852:.2f} nmi, Range diff = {range_diff/1852:.2f} nmi")

        total_range= R_leg1 + R_leg2

        return total_range, time_leg1, time_leg2
    
    def perform_sanity_check(self, R_numerical: float, R_analytical: float, print_results: bool = False) -> bool:
        """
        Perform sanity check comparing numerical and analytical results.
        
        Args:
            R_numerical: Range from numerical integration (m)
            R_analytical: Range from analytical formula (m)
            
        Returns:
            True if results are within acceptable tolerance, False otherwise
        """
        relative_error = abs(R_numerical - R_analytical) / R_analytical * 100
        
        if print_results:
            print("\n" + "="*60)
            print("SANITY CHECK RESULTS")
            print("="*60)
            print(f"Numerical range:   {R_numerical:.2f} m ({R_numerical/self.METERS_TO_NMI:.2f} nmi)")
            print(f"Analytical range:  {R_analytical:.2f} m ({R_analytical/self.METERS_TO_NMI:.2f} nmi)")
            print(f"Absolute error:    {abs(R_numerical - R_analytical):.2f} m")
            print(f"Relative error:    {relative_error:.2f}%")
            print(f"Tolerance:         {self.TOLERANCE_PERCENT:.1f}%")
        
        if relative_error <= self.TOLERANCE_PERCENT:
            print(f"✓ SANITY CHECK PASSED, Relative error: {relative_error:.2f}%, weight step: {self.weight_step} N")
            return True
        else:
            print("✗ SANITY CHECK FAILED: Results exceed acceptable tolerance")
            print("  Consider:")
            print("  - Reducing integration step size (dW)")
            print("  - Checking for numerical instabilities")
            print("  - Verifying analytical assumptions")
            return False
    
    def plot_speed_vs_weight(
        self,
        W4: float,
        W5: float,
        velocity_func_list=None,
        labels=None,
        h: float = 10,
        dW: float = 10
    ):
        """
        Plot speed versus weight for the cruise segment for multiple strategies.

        Args:
            W4: Weight at start of cruise (N)
            W5: Weight at end of cruise (N)
            velocity_func_list: List of functions to calculate velocity at each weight.
                                Each should take (optimizer, altitude) and return velocity.
                                If None, uses default v_range (optimal range speed).
            labels: List of labels for each strategy.
            h: Altitude (m)
            dW: Weight step (N)
        """
        if velocity_func_list is None:
            velocity_func_list = [None]
        if labels is None:
            labels = [f"Strategy {i+1}" for i in range(len(velocity_func_list))]

        plt.figure(figsize=(10, 6))

        mtow = self.opt._mtow

        for idx, velocity_func in enumerate(velocity_func_list):
            weights_frac = []
            speeds = []
            W_array = np.arange(W4, W5, -dW)
            for weight in W_array:
                self.opt._current_weight = weight
                # Calculate velocity using provided function or default
                if velocity_func is not None:
                    V = velocity_func(self.opt, h)
                else:
                    V = self.opt.v_range(h)
                # Check if flight is possible at this speed/weight
                if self.opt.calculate_AoC(V, h) >= 0:
                    weights_frac.append(weight / mtow)  # Fraction of MTOW
                    speeds.append(V)
            if weights_frac:
                plt.plot(weights_frac, speeds, linewidth=2, marker='o', markersize=4, label=labels[idx])
            else:
                print(f"No valid flight points found for plotting: {labels[idx]}")

        plt.xlabel('Weight / MTOW')
        plt.ylabel('Speed (m/s)')
        plt.title('Speed vs Weight Fraction During Cruise')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.gca().invert_xaxis()  # Reverse the weight axis
        plt.tight_layout()
        plt.show()

    def check(self):
        """Main execution function demonstrating usage."""
        try:
            altitude = self.aircraft_data.data['inputs']['cruise_altitude']  # Example altitude in meters
            
            # Calculate cruise weights
            W4, W5 = self.calculate_cruise_weights_leg1()
            
            # Example velocity functions for different strategies
            def v_max_range(opt, h, W=None):
                """Use optimal range speed (default)"""
                return opt.v_range(h, W=W)

            # Numerical integration for range, endurance, and constant speed
            R_numerical_range, time_numerical, _ = self.calculate_numerical_range(W4, W5, v_max_range, altitude, dW=self.weight_step)
            
            # Analytical range calculation
            R_analytical_range, time_analytical = self.calculate_analytical_range(W4, W5, h=altitude)

            # Perform sanity check
            self.perform_sanity_check(R_numerical_range, R_analytical_range, print_results=True)

        except Exception as e:
            print(f"Error during analysis: {str(e)}")
            raise


def speed_comparison():
    """Main function to run the range analysis."""

    # Configuration
    file_path = "design3.json"
    mission_type = MissionType.DESIGN
    analyzer = RangeAnalyzer(file_path, mission_type, sea_state=0)  # Example sea state

    weight_step = analyzer.weight_step  # Weight step for integration (N)
    
    # Calculate cruise weights
    W4_1, W5_1 = analyzer.calculate_cruise_weights_leg1()
    W4_2, W5_2 = analyzer.calculate_cruise_weights_leg2()
    
    # Calculate numerical range
    def v_test(opt, h, W=None):
        """Example"""
        return opt.v_range(h, W=W)*1.25
    
    def v_max(opt, h, W=None):
        """Use optimal range speed (default)"""
        return opt.v_max(h, W=W)
    
    # Leg 1
    R_numerical_1, time_1, _ = analyzer.calculate_numerical_range(W4_1, W5_1, None, dW=weight_step)
    R_numerical_test_1, time_test_1, _ = analyzer.calculate_numerical_range(W4_1, W5_1, v_test, dW=weight_step)
    R_numerical_max_1, time_max_1, _ = analyzer.calculate_numerical_range(W4_1, W5_1, v_max, dW=weight_step)

    # details_total = concat_details(details_1, details_2)
    # details_total_test = concat_details(details_test_1, details_test_2)

    # plot_speed_vs_time(details_total, "Default Strategy", payload_drop_time=details_1['times'][-1])
    # plot_speed_vs_time(details_total_test, "Test Strategy", payload_drop_time=details_test_1['times'][-1])

    print(f"Total mission range (default strategy): {R_numerical_1/analyzer.METERS_TO_NMI:.2f} nmi, time: {time_1/3600:.2f} h")
    print(f"Total mission range (test strategy):    {R_numerical_test_1/analyzer.METERS_TO_NMI:.2f} nmi, time: {time_test_1/3600:.2f} h")
    print(f"Total mission range (max speed strategy): {R_numerical_max_1/analyzer.METERS_TO_NMI:.2f} nmi, time: {time_max_1/3600:.2f} h")

    # Create bar chart with dual y-axes for time and range
    strategies = ['Max Range', 'Test (1.25x)', 'Max Speed']
    ranges = [R_numerical_1/analyzer.METERS_TO_NMI, R_numerical_test_1/analyzer.METERS_TO_NMI, R_numerical_max_1/analyzer.METERS_TO_NMI]
    times = [time_1/3600, time_test_1/3600, time_max_1/3600]

    fig, ax1 = plt.subplots(figsize=(10, 6))

    # Bar chart for range
    color1 = 'tab:blue'
    ax1.set_xlabel('Strategy')
    ax1.set_ylabel('Range (nmi)', color=color1)
    bars1 = ax1.bar([x - 0.2 for x in range(len(strategies))], ranges, width=0.4, 
                   color=color1, alpha=0.7, label='Range')
    ax1.tick_params(axis='y', labelcolor=color1)

    # Second y-axis for time
    ax2 = ax1.twinx()
    color2 = 'tab:orange'
    ax2.set_ylabel('Time (hours)', color=color2)
    bars2 = ax2.bar([x + 0.2 for x in range(len(strategies))], times, width=0.4, 
                   color=color2, alpha=0.7, label='Time')
    ax2.tick_params(axis='y', labelcolor=color2)

    # Set x-axis labels
    ax1.set_xticks(range(len(strategies)))
    ax1.set_xticklabels(strategies)

    # Add value labels on bars
    for i, (bar1, bar2, range_val, time_val) in enumerate(zip(bars1, bars2, ranges, times)):
        ax1.text(bar1.get_x() + bar1.get_width()/2, bar1.get_height() + 5,
                 f'{range_val:.1f}', ha='center', va='bottom', color=color1, fontweight='bold')
        ax2.text(bar2.get_x() + bar2.get_width()/2, bar2.get_height() + 0.1,
                 f'{time_val:.2f}', ha='center', va='bottom', color=color2, fontweight='bold')

    plt.title('Range and Time Comparison by Strategy')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

def sea_state_comparison():
    """
    Compare range performance across different sea states.
    Prints cruise altitude and range for each sea state.
    """
    sea_states = [0, 1, 2, 3, 4]  # Example sea states
    results = {}

    for sea_state in sea_states:
        analyzer = RangeAnalyzer("design3.json", MissionType.DESIGN, sea_state=sea_state)
        # Calculate and store numerical range for each sea state
        W4, W5 = analyzer.calculate_cruise_weights_leg1()
        R_numerical, time, _ = analyzer.calculate_numerical_range(W4, W5, None, dW=analyzer.weight_step)
        cruise_altitude = analyzer.aircraft_data.data['inputs']['cruise_altitude']
        results[sea_state] = {
            'range_nmi': R_numerical / analyzer.METERS_TO_NMI,
            'cruise_altitude': cruise_altitude
        }

    print("Sea state comparison results:")
    for ss, res in results.items():
        print(f"Sea State {ss}: Range = {res['range_nmi']:.2f} nmi, Cruise Altitude = {res['cruise_altitude']:.2f} m")

    # Plotting
    sea_states_list = list(results.keys())
    ranges = [results[ss]['range_nmi'] for ss in sea_states_list]

    plt.figure(figsize=(8, 5))
    plt.plot(sea_states_list, ranges, marker='o', label='Range (nmi)')
    plt.xlabel('Sea State')
    plt.ylabel('Range (nmi)')
    plt.title('Range vs Sea State')
    plt.grid(True, alpha=0.3)
    plt.xticks(sea_states_list)  # Set x-axis ticks to sea state values (step 1)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Create analyzer
    # Configuration

    file_path = "design3.json"
    mission_type = MissionType.DESIGN
    analyzer = RangeAnalyzer(file_path, mission_type)
    analyzer.check()

    sea_state_comparison()