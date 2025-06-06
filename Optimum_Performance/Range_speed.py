import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from typing import Tuple, Dict
import warnings

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType
from Class_I.PayloadRange import RangeCalculator
from Optimum_Performance.Optimum_speeds import OptimumSpeeds
from Class_II.Small_Iteration import Ainf_Ah

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

        # Assume TO takes 10% more fuel per sea state unit approximation
        self.rng.Mff_nocruise *= (1-(1-self.rng.fuel_fracs_no_cruise[1])*(1+sea_state/10))/self.rng.fuel_fracs_no_cruise[1]
        # Adjust cruise altitude for sea state
        self.aircraft_data.data['inputs']['cruise_altitude'] *= 1+sea_state/10
        
        # Constants
        self.METERS_TO_NMI = 1852
        self.TOLERANCE_PERCENT = 1.0  # Acceptable difference percentage for sanity check
        self.weight_step = 10  # Weight step for numerical integration (N)
        
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

        # Check if W5_leg2 is close to zero fuel weight (ZFW)
        oew = self.opt._oew
        if not np.isclose(W5_leg2, oew, rtol=0.01):
            warnings.warn(f"W5_leg2 ({W5_leg2:.2f} N) is not close to OEW ({oew:.2f} N)")

        return W4_leg2, W5_leg2
    
    def validate_flight_capability(self, V: float, weight: float, h: float) -> bool:
        """Check if the aircraft can fly at given speed and weight."""
        self.opt._current_weight = weight
        angle_of_climb = self.opt.calculate_AoC(V, h)
        
        if angle_of_climb < 0:
            print(f"Warning: Cannot fly at V={V:.2f} m/s with weight={weight/9.81:.2f} kg")
            print(f"Angle of climb: {angle_of_climb:.4f} degrees")
            return False
        return True
    
    def calculate_numerical_range(self, W4: float, W5: float, velocity_func=None, h: float = 10, dW: float = 10) -> Tuple[float, Dict]:
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
        k = np.sqrt(1/Ainf_Ah(h,
                               self.aircraft_data.data['outputs']['wing_design']['b'], 
                               self.aircraft_data.data['inputs']['endplate_height']))
        print(f"Ground effect factor (k): {k:.4f}, {h} m altitude")
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
                V = velocity_func(self.opt, h)
            else:
                V = self.opt.v_range(h)  # Default: optimal range speed
            
            # Validate flight capability
            if not self.validate_flight_capability(V, weight, h):
                integration_details['invalid_points'] += 1
                continue
            
            # Calculate lift-to-drag ratio with ground effect correction
            Cl_Cd = self.opt.L_over_D(V, h, weight)
            Cl_Cd_ge = Cl_Cd * k
            
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
    
    def calculate_analytical_range(self, W4: float, W5: float, h: float = 10) -> float:
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
        
        V = self.opt.v_range(h)
        k = np.sqrt(1/Ainf_Ah(self.aircraft_data.data['inputs']['cruise_altitude'],
                               self.aircraft_data.data['outputs']['wing_design']['b'], 
                               self.aircraft_data.data['inputs']['endplate_height']))
        
        # Calculate representative L/D ratio
        Cl_Cd = self.opt.L_over_D(V, h, W_representative)
        Cl_Cd_ge = Cl_Cd * k
        
        # Breguet range equation
        R_analytical = (self.opt._prop_efficiency / 
                       self.aircraft_data.data['inputs']['prop_consumption'] * 
                       Cl_Cd_ge * np.log(W4 / W5) / self.opt.g)     
        
        return R_analytical
    
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
        dW: float = 100
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
            def v_max_range(opt, h):
                """Use optimal range speed (default)"""
                return opt.v_range(h)

            # Numerical integration for range, endurance, and constant speed
            R_numerical_range, time_range, _ = self.calculate_numerical_range(W4, W5, v_max_range, altitude, dW=self.weight_step)
            
            # Analytical range calculation
            R_analytical_range = self.calculate_analytical_range(W4, W5, altitude)

            # Perform sanity check
            self.perform_sanity_check(R_numerical_range, R_analytical_range, print_results=True)

        except Exception as e:
            print(f"Error during analysis: {str(e)}")
            raise


def main():
    """Main function to run the range analysis."""

    # Configuration
    file_path = "design3.json"
    mission_type = MissionType.DESIGN
    analyzer = RangeAnalyzer(file_path, mission_type, sea_state=0)  # Example sea state

    altitude = analyzer.aircraft_data.data['inputs']['cruise_altitude']  # Example altitude in meters
    weight_step = analyzer.weight_step  # Weight step for integration (N)
    
    # Calculate cruise weights
    W4_1, W5_1 = analyzer.calculate_cruise_weights_leg1()
    W4_2, W5_2 = analyzer.calculate_cruise_weights_leg2()
    
    # Calculate numerical range
    def v_test(opt, h):
        """Example"""
        return opt.v_range(h)*1.25
    
    # Leg 1
    R_numerical_1, time_1, details_1 = analyzer.calculate_numerical_range(W4_1, W5_1, None, dW=weight_step)
    R_numerical_test_1, time_test_1, details_test_1 = analyzer.calculate_numerical_range(W4_1, W5_1, v_test, dW=weight_step)
    # Leg 2
    R_numerical_2, time_2, details_2 = analyzer.calculate_numerical_range(W4_2, W5_2, None, dW=weight_step)
    R_numerical_test_2, time_test_2, details_test_2 = analyzer.calculate_numerical_range(W4_2, W5_2, v_test, dW=weight_step)

    # Combine ranges and times for total mission
    R_total = R_numerical_1 + R_numerical_2
    time_total = time_1 + time_2
    R_total_test = R_numerical_test_1 + R_numerical_test_2
    time_total_test = time_test_1 + time_test_2

    details_total = concat_details(details_1, details_2)
    details_total_test = concat_details(details_test_1, details_test_2)

    plot_speed_vs_time(details_total, "Default Strategy", payload_drop_time=details_1['times'][-1])
    plot_speed_vs_time(details_total_test, "Test Strategy", payload_drop_time=details_test_1['times'][-1])

    print(f"Total mission range (default strategy): {R_total/analyzer.METERS_TO_NMI:.2f} nmi, time: {time_total/3600:.2f} h")
    print(f"Total mission range (test strategy):    {R_total_test/analyzer.METERS_TO_NMI:.2f} nmi, time: {time_total_test/3600:.2f} h")


if __name__ == "__main__":
    # Create analyzer
    # Configuration
    # file_path = "design3.json"
    # mission_type = MissionType.DESIGN
    # analyzer = RangeAnalyzer(file_path, mission_type)
    # analyzer.check()

    main()