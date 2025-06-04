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


class RangeAnalyzer:
    """
    Analyzes aircraft range using both numerical integration and analytical methods.
    Includes sanity checks to validate results.
    """
    
    def __init__(self, file_path: str, mission_type: MissionType):
        """Initialize the range analyzer with aircraft data and mission type."""
        self.aircraft_data = Data(file_path)
        self.mission_type = mission_type
        self.opt = OptimumSpeeds(self.aircraft_data, mission_type)
        self.rng = RangeCalculator(data_object=self.aircraft_data, mission_type=mission_type)
        
        # Constants
        self.METERS_TO_NMI = 1852
        self.TOLERANCE_PERCENT = 1.0  # Acceptable difference percentage for sanity check
        
    def calculate_cruise_weights(self) -> Tuple[float, float]:
        """Calculate weights at start and end of cruise segment."""
        mass_fractions = self.rng.calculate_mass_fractions()
        W4_W5 = self.rng.calculate_weight_ratios(mass_fractions)
        
        W4 = self.opt._mtow * self.rng.Mff_nocruise  # Weight at start of cruise
        W5 = W4 / W4_W5['design']                     # Weight at end of cruise
        
        return W4, W5
    
    def validate_flight_capability(self, V: float, weight: float, h: float) -> bool:
        """Check if the aircraft can fly at given speed and weight."""
        self.opt._current_weight = weight
        angle_of_climb = self.opt.calculate_AoC(V, h)
        
        if angle_of_climb < 0:
            print(f"Warning: Cannot fly at V={V:.2f} m/s with weight={weight/9.81:.2f} kg")
            print(f"Angle of climb: {angle_of_climb:.4f} degrees")
            return False
        return True
    
    def calculate_numerical_range(self, W4: float, W5: float, velocity_func=None, h: float = 10, dW: float = 100) -> Tuple[float, Dict]:
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
        k = self.aircraft_data.data['outputs']['design']['k']
        W_array = np.arange(W4, W5, -dW)
        
        integration_details = {
            'weights': [],
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
            Cl_Cd = self.opt.L_over_D(h, V)
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
        self.opt._current_weight = W_representative
        
        V = self.opt.v_range(h)
        k = self.aircraft_data.data['outputs']['design']['k']
        
        # Calculate representative L/D ratio
        Cl_Cd = self.opt.L_over_D(h, V)
        Cl_Cd_ge = Cl_Cd * k
        
        # Breguet range equation
        R_analytical = (self.opt._prop_efficiency / 
                       self.aircraft_data.data['inputs']['prop_consumption'] * 
                       Cl_Cd_ge * np.log(W4 / W5) / self.opt.g)
        
        return R_analytical
    
    def perform_sanity_check(self, R_numerical: float, R_analytical: float, print_results: bool = True) -> bool:
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
            print("✓ SANITY CHECK PASSED: Results are within acceptable tolerance")
            return True
        else:
            print("✗ SANITY CHECK FAILED: Results exceed acceptable tolerance")
            print("  Consider:")
            print("  - Reducing integration step size (dW)")
            print("  - Checking for numerical instabilities")
            print("  - Verifying analytical assumptions")
            return False
    
    def plot_speed_vs_weight(self, W4: float, W5: float, velocity_func=None, h: float = 10, dW: float = 100):
        """
        Plot speed versus weight for the cruise segment.
        
        Args:
            W4: Weight at start of cruise (N)
            W5: Weight at end of cruise (N)
            velocity_func: Function to calculate velocity at each weight.
                          Should take (optimizer, altitude) as arguments and return velocity.
                          If None, uses default v_range (optimal range speed).
            h: Altitude (m)
            dW: Weight step (N)
        """
        weights = []
        speeds = []
        
        W_array = np.arange(W4, W5, -dW)
        
        for weight in W_array:
            self.opt._current_weight = weight
            
            # Calculate velocity using provided function or default
            if velocity_func is not None:
                V = velocity_func(self.opt, h)
            else:
                V = self.opt.v_range(h)  # Default: optimal range speed
            
            # Check if flight is possible at this speed/weight
            if self.opt.calculate_AoC(V, h) >= 0:
                weights.append(weight / 9.81)  # Convert to kg
                speeds.append(V)
        
        if not weights:
            print("No valid flight points found for plotting")
            return
        
        plt.figure(figsize=(10, 6))
        plt.plot(weights, speeds, 'b-', linewidth=2, marker='o', markersize=4)
        plt.xlabel('Weight (kg)')
        plt.ylabel('Speed (m/s)')
        
        # Set title based on velocity function used
        if velocity_func is not None:
            plt.title('Speed vs Weight During Cruise (Custom Speed Function)')
        else:
            plt.title('Optimal Range Speed vs Weight During Cruise')
            
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
        
        print(f"Speed range: {min(speeds):.2f} - {max(speeds):.2f} m/s")
        print(f"Weight range: {min(weights):.0f} - {max(weights):.0f} kg")


def check():
    """Main execution function demonstrating usage."""
    try:
        # Configuration
        file_path = "design3.json"
        mission_type = MissionType.DESIGN
        altitude = 10  # meters
        weight_step = 100  # N
        
        # Create analyzer
        analyzer = RangeAnalyzer(file_path, mission_type)
        
        # Calculate cruise weights
        W4, W5 = analyzer.calculate_cruise_weights()
        
        # Example velocity functions for different strategies
        def v_max_range(opt, h):
            """Use optimal range speed (default)"""
            return opt.v_range(h)
        
        def v_max_endurance(opt, h):
            """Use optimal endurance speed"""
            return opt.v_endurance(h)
        
        def v_test(opt, h):
            """Example"""
            return opt.v_range(h)*1.25
        
        # Calculate ranges with different speed strategies
        print("=== RANGE ANALYSIS WITH DIFFERENT SPEED STRATEGIES ===")
        # Analytical range calculation
        R_analytical = analyzer.calculate_analytical_range(W4, W5, altitude)

        # Numerical integration for range, endurance, and constant speed
        R_numerical_range, time_range, _ = analyzer.calculate_numerical_range(W4, W5, v_max_range, altitude, weight_step)
        R_numerical_endurance, time_endurance, _ = analyzer.calculate_numerical_range(W4, W5, v_max_endurance, altitude, weight_step)
        R_numerical_test, time_test, _= analyzer.calculate_numerical_range(W4, W5, v_test, altitude, weight_step)
        # Perform sanity check
        analyzer.perform_sanity_check(R_numerical_range, R_analytical, print_results=False)

        # Compare results
        print("\n=== COMPARISON ===")
        # Print results in a table with speed ranges
        print(f"{'Strategy':<18}{'Range (nmi)':>15}{'Time (hr)':>15}{'Speed range (m/s)':>22}")
        print("-" * 70)
        strategies = [
            ("Range speed", v_max_range, R_numerical_range, time_range),
            ("Endurance speed", v_max_endurance, R_numerical_endurance, time_endurance),
            ("Test speed", v_test, R_numerical_test, time_test)
        ]
        for name, v_func, rng, t in strategies:
            analyzer.opt._current_weight = W4
            speed_W4 = v_func(analyzer.opt, altitude)
            analyzer.opt._current_weight = W5
            speed_W5 = v_func(analyzer.opt, altitude)
            speed_range = f"{min(speed_W4, speed_W5):.2f} - {max(speed_W4, speed_W5):.2f}"
            print(f"{name:<18}{rng/1852:>15.1f}{t/3600:>15.2f}{speed_range:>22}")

                
    except Exception as e:
        print(f"Error during analysis: {str(e)}")
        raise

if __name__ == "__main__":
    check()