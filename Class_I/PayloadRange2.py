import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, plt
from Class_I.ClassIWeightEstimation import ClassI, MissionType


class RangeCalculator:
    """
    A class to calculate aircraft range and generate payload-range diagrams.
    
    This class encapsulates the range calculation logic and payload-range diagram 
    generation for different aircraft types with interpolation capabilities.
    """
    
    def __init__(self, data_file=None, data_object=None, mission_type=MissionType.DESIGN):
        """
        Initialize the RangeCalculator with either a data file or a Data object.
        
        Args:
            data_file (str, optional): Path to the JSON data file.
            data_object (Data, optional): Pre-initialized Data object.
        """
        self.g = 9.81  # Gravity constant (m/s²)
        
        if data_object:
            self.data = data_object
        elif data_file:
            self.data = Data(data_file)
        else:
            raise ValueError("Either data_file or data_object must be provided")
        
        self.mission_type = mission_type
            
        # Extract common parameters from data
        self.aircraft_type = self.data.data["inputs"]['aircraft_type']
        self.eta_p = self.data.data["inputs"]['prop_efficiency']
        self.cp = self.data.data["inputs"]['prop_consumption']
        self.cj = self.data.data["inputs"]['jet_consumption']
        self.V_cruise = self.data.data["requirements"]['cruise_speed']
        self.V_cruise_high = self.data.data["requirements"]['cruise_speed']
        
        # Calculate L/D ratio
        self.L_D = self.data.data["outputs"]['design']['LD']
        self.L_D_geometric = self.data.data["outputs"]['general']['LD_g']
        
        # Extract weight and fuel data
        self.fuel_design = self.data.data["outputs"]['design']['mission_fuel']
        self.fuel_reserve = self.data.data["outputs"]['design']['reserve_fuel']
        self.fuel_max = self.data.data["outputs"]['design']['max_fuel'] - self.fuel_reserve
        self.design_payload = self.data.data["requirements"]['design_payload']
        self.max_payload = 100 * 1_000  # kg This could be made configurable !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.maxrange_payload = self.design_payload - (self.fuel_max - self.fuel_reserve - self.fuel_design)/9.81
        self.MTOW = self.data.data["outputs"]['design']['MTOW']
        
        # Calculate fuel fractions without cruise
        self.class_i = ClassI(aircraft_data=self.data, mission_type=self.mission_type)
        self.fuel_fracs_no_cruise = self.class_i.fuel_fractions
        self.Mff_nocruise = 1
        for fraction in self.fuel_fracs_no_cruise.values():
            self.Mff_nocruise *= fraction
        if mission_type == MissionType.ALTITUDE:
            self.class_i.LD = self.class_i.calculate_LD()
            range_fraction_altitude = np.exp(-self.class_i.altitude_range_WOG*self.class_i.prop_consumption*9.81/self.class_i.prop_efficiency * (self.class_i.LD)**-1)
            self.Mff_nocruise *= range_fraction_altitude*self.class_i.climb_fraction

        
    def calculate_range(self, W4_W5):
        """
        Calculate the range using the Breguet range equation.
        
        Args:
            W4_W5 (float): Weight ratio
            
        Returns:
            float: Range in meters
        """

        if self.aircraft_type == "PROP":
            R = (self.eta_p * self.L_D * np.log(W4_W5)) / (self.g * self.cp)
        elif self.aircraft_type == "JET":
            R = (self.V_cruise * self.L_D * np.log(W4_W5)) / (self.g * self.cj)
        else:
            raise ValueError(f"Unknown aircraft type: {self.aircraft_type}")
        
        return R
    
    def calculate_mass_fractions(self):
        """
        Calculate different mass fractions for various mission scenarios.
        
        Returns:
            dict: Dictionary containing different mass fraction values
        """
        # Calculate mass fractions
        Mff_harmonic = 1 - (self.fuel_design - (self.max_payload - self.design_payload) * self.g) / self.MTOW
        Mff_design = 1 - self.fuel_design / self.MTOW
        Mff_maxrange = 1 - (self.fuel_max - self.fuel_reserve) / self.MTOW
        Mff_ferry = 1 - (self.fuel_max - self.fuel_reserve) / (self.MTOW - self.design_payload * self.g)
        
        return {
            "harmonic": Mff_harmonic,
            "design": Mff_design,
            "maxrange": Mff_maxrange,
            "ferry": Mff_ferry
        }
    
    def compute_Mff_1way(self, Mff, payload, MTOW):
        payload = payload * self.g  # Convert payload from tonnes to kg
        a = 1
        b = -payload / MTOW
        c = (payload / MTOW) - Mff
        
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            raise ValueError("No real solution for Mff_1way.")
        
        sqrt_discriminant = np.sqrt(discriminant)
        Mff_1way_1 = (-b + sqrt_discriminant) / (2 * a)
        Mff_1way_2 = (-b - sqrt_discriminant) / (2 * a)

        # Choose the root between 0 and 1 (physically meaningful fuel fraction)
        if 0 < Mff_1way_1 <= 1:
            return Mff_1way_1
        elif 0 < Mff_1way_2 <= 1:
            return Mff_1way_2
        else:
            raise ValueError("No physically valid Mff_1way in [0, 1].")
    
    def calculate_weight_ratios(self, mass_fractions):
        """
        Calculate weight ratios based on mass fractions.
        
        Args:
            mass_fractions (dict): Mass fractions dictionary
            
        Returns:
            dict: Dictionary containing weight ratios
        """
        if self.mission_type == MissionType.DESIGN or self.mission_type == MissionType.ALTITUDE:
            W4_W5_harmonic = self.Mff_nocruise/self.compute_Mff_1way(mass_fractions["harmonic"], self.max_payload, self.MTOW)
            W4_W5_design = self.Mff_nocruise/self.compute_Mff_1way(mass_fractions["design"], self.design_payload, self.MTOW)
            W4_W5_maxrange = self.Mff_nocruise/self.compute_Mff_1way(mass_fractions["maxrange"], self.maxrange_payload, self.MTOW)
            W4_W5_ferry = self.Mff_nocruise/self.compute_Mff_1way(mass_fractions["ferry"], 0, self.MTOW - self.design_payload * self.g)
        elif self.mission_type == MissionType.FERRY:
            W4_W5_harmonic = 1 / mass_fractions["harmonic"] * self.Mff_nocruise
            W4_W5_design = 1 / mass_fractions["design"] * self.Mff_nocruise
            W4_W5_maxrange = 1 / mass_fractions["maxrange"] * self.Mff_nocruise
            W4_W5_ferry = 1 / mass_fractions["ferry"] * self.Mff_nocruise
        
        return {
            "harmonic": W4_W5_harmonic,
            "design": W4_W5_design,
            "maxrange": W4_W5_maxrange,
            "ferry": W4_W5_ferry
        }
    
    def calculate_ranges(self, weight_ratios):
        """
        Calculate ranges for different mission scenarios.
        
        Args:
            weight_ratios (dict): Weight ratios dictionary
            
        Returns:
            dict: Dictionary containing range values in meters
        """
        range_harmonic = self.calculate_range(weight_ratios["harmonic"])
        range_design = self.calculate_range(weight_ratios["design"])
        range_max = self.calculate_range(weight_ratios["maxrange"])
        range_ferry = self.calculate_range(weight_ratios["ferry"])

        if self.mission_type == MissionType.ALTITUDE:
            range_harmonic += self.data.data['requirements']['altitude_range_WOG']
            range_design += self.data.data['requirements']['altitude_range_WOG']
            range_max += self.data.data['requirements']['altitude_range_WOG']
            range_ferry += self.data.data['requirements']['altitude_range_WOG']

        
        return {
            "harmonic": range_harmonic,
            "design": range_design,
            "maxrange": range_max,
            "ferry": range_ferry
        }
    
    def meters_to_nautical_miles(self, ranges_dict):
        """
        Convert ranges from meters to nautical miles.
        
        Args:
            ranges_dict (dict): Dictionary of ranges in meters
            
        Returns:
            dict: Dictionary of ranges in nautical miles
        """
        return {key: value / 1852 for key, value in ranges_dict.items()}
    
    def generate_payload_range_points(self, ranges_nm):
        """
        Generate points for payload-range diagram.
        
        Args:
            ranges_nm (dict): Dictionary of ranges in nautical miles
            
        Returns:
            list: List of (range, payload) points for the diagram
        """
        return [
            (0, self.max_payload / 1000),
            (ranges_nm["harmonic"], self.max_payload / 1000),
            (ranges_nm["design"], self.design_payload / 1000),
            (ranges_nm["maxrange"], (self.design_payload - (self.fuel_max - self.fuel_design) / self.g) / 1000),
            (ranges_nm["ferry"], 0)
        ]
    
    def interpolate_payload_range(self, ranges_nm, num_points=100):
        """
        Interpolate between payload-range points to create a smooth curve.
        
        Args:
            ranges_nm (dict): Dictionary of ranges in nautical miles
            num_points (int): Number of interpolated points
            
        Returns:
            tuple: (range_interp, payload_interp) arrays for interpolated points
        """
        # Get the main points
        points = self.generate_payload_range_points(ranges_nm)
        
        # Extract x and y coordinates
        range_points = [p[0] for p in points]
        payload_points = [p[1] for p in points]
        
        # Create interpolation function
        interp_func = interp1d(range_points, payload_points, kind='linear', 
                              bounds_error=False, fill_value="extrapolate")
        
        # Generate interpolated points
        range_min = min(range_points)
        range_max = max(range_points)
        range_interp = np.linspace(range_min, range_max, num_points)
        payload_interp = interp_func(range_interp)
        
        # Ensure no negative payloads
        payload_interp = np.maximum(payload_interp, 0)
        
        return range_interp, payload_interp
    
    def calculate_fuel_economy_for_payload_range(self, payload_tonnes, range_nm):
        """
        Calculate fuel economy for a specific payload and range combination.
        
        Args:
            payload_tonnes (float): Payload in tonnes
            range_nm (float): Range in nautical miles
            
        Returns:
            float: Fuel consumption in L/ton/km
        """
        if payload_tonnes <= 0 or range_nm <= 0:
            return np.nan
        
        # Calculate required fuel for this specific payload and range using interpolation
        range_m = range_nm * 1852  # Convert to meters

        # Get the key mission points for interpolation
        mass_fractions = self.calculate_mass_fractions()
        weight_ratios = self.calculate_weight_ratios(mass_fractions)
        ranges_m_key = self.calculate_ranges(weight_ratios)

        # Define key points for interpolation: (range_m, payload_kg, fuel_used_kg)
        key_points = [
            (ranges_m_key["harmonic"], self.max_payload, (1 - mass_fractions["harmonic"]) * self.MTOW),
            (ranges_m_key["design"], self.design_payload, (1 - mass_fractions["design"]) * self.MTOW),
            (ranges_m_key["maxrange"], self.design_payload - (self.fuel_max - self.fuel_design), (1 - mass_fractions["maxrange"]) * self.MTOW),
            (ranges_m_key["ferry"], 0, (1 - mass_fractions["ferry"]) * (self.MTOW - self.design_payload * self.g))
        ]

        # Sort points by range for interpolation
        key_points.sort(key=lambda x: x[0])
        ranges_key = [p[0] for p in key_points]
        fuels_key = [p[2] for p in key_points]

        # Interpolate fuel consumption based on range
        if range_m <= min(ranges_key):
            fuel_used_N = fuels_key[0]
        elif range_m >= max(ranges_key):
            fuel_used_N = fuels_key[-1]
        else:
            fuel_interp_func = interp1d(ranges_key, fuels_key, kind='linear')
            fuel_used_N = fuel_interp_func(range_m)

        # Convert fuel from N to liters (assuming fuel density of 0.76 kg/L)
        fuel_used_L = fuel_used_N / 9.81 / 0.76
        
        # Calculate fuel economy in L/ton/km
        range_km = range_nm * 1.852  # Convert nautical miles to kilometers
        
        if self.mission_type == MissionType.ALTITUDE or self.mission_type == MissionType.DESIGN:
            # Round trip mission
            fuel_economy = fuel_used_L / (payload_tonnes * range_km * 2)
        else:
            # One-way mission
            fuel_economy = fuel_used_L / (payload_tonnes * range_km)
            
        return fuel_economy
    
    def calculate_interpolated_fuel_economy(self, ranges_nm, num_points=100):
        """
        Calculate fuel economy for interpolated payload-range points.
        
        Args:
            ranges_nm (dict): Dictionary of ranges in nautical miles
            num_points (int): Number of interpolated points
            
        Returns:
            tuple: (range_interp, payload_interp, fuel_economy_interp) arrays
        """
        # Get interpolated points
        range_interp, payload_interp = self.interpolate_payload_range(ranges_nm, num_points)
        
        # Calculate fuel economy for each point
        fuel_economy_interp = np.array([
            self.calculate_fuel_economy_for_payload_range(payload, range_val)
            for range_val, payload in zip(range_interp, payload_interp)
        ])
        
        return range_interp, payload_interp, fuel_economy_interp
    
    def plot_payload_range_diagram(self, points, show=True, save_path=None, show_interpolation=True, num_interp_points=100):
        """
        Plot the payload-range diagram with optional interpolation.
        
        Args:
            points (list): List of (range, payload) points
            show (bool): Whether to display the plot
            save_path (str, optional): Path to save the plot image
            show_interpolation (bool): Whether to show interpolated curve
            num_interp_points (int): Number of interpolation points
            
        Returns:
            tuple: Figure and axis objects
        """
        # Extract x and y coordinates from points
        x_coords, y_coords = zip(*points)
        
        # Create plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
        # Top plot: Payload-Range diagram
        ax1.plot(x_coords, y_coords, marker='o', linestyle='-', color='b', 
                label='Key Points', linewidth=2, markersize=8)
        
        if show_interpolation:
            # Get ranges for interpolation
            ranges_nm = {
                "harmonic": points[1][0],
                "design": points[2][0], 
                "maxrange": points[3][0],
                "ferry": points[4][0]
            }
            
            # Calculate interpolated points
            range_interp, payload_interp, fuel_economy_interp = self.calculate_interpolated_fuel_economy(
                ranges_nm, num_interp_points)
            
            # Plot interpolated curve
            ax1.plot(range_interp, payload_interp, '-', color='red', alpha=0.7, 
                    linewidth=1, label='Interpolated Curve')
        
        # Fill the area under the curve with light green
        ax1.fill_between(x_coords, y_coords, color='lightgreen', alpha=0.3)
        
        # Highlight the design point (index 2)
        ax1.plot(*points[2], color='lime', label='Design Point', marker='o', markersize=10)

        # Annotate all points
        labels = ['0 nm', 'Harmonic', 'Design', 'Max', 'Ferry']
        xylabel = [(30, 5),(70, -6),(60, 0),(75, -20),(-15, 90)]
        for i, (x, y) in enumerate(points):
            if i == 0:
                continue
            x_format = int(float('%.3g' % x))
            y_format = float('%.3g' % y)
            ax1.annotate(f"{labels[i]}: {x_format} nm\nat {y_format} tonnes", 
                        (x, y), 
                        textcoords="offset points", 
                        xytext=xylabel[i], 
                        ha='center', 
                        fontsize=10,
                        arrowprops=dict(arrowstyle="->", color='black', lw=1))
        
        ax1.set_xlim(left=0)
        ax1.set_ylim(bottom=0)
        ax1.set_xlabel('Range (nautical miles)')
        ax1.set_ylabel('Payload (tonnes)')
        ax1.legend(loc='upper right')
        ax1.grid(True)
        ax1.set_title('Payload-Range Diagram')
        
        # Bottom plot: Fuel Economy vs Range
        if show_interpolation:
            # Filter out invalid fuel economy values
            valid_mask = ~np.isnan(fuel_economy_interp) & ~np.isinf(fuel_economy_interp)
            valid_range = range_interp[valid_mask]
            valid_fuel_economy = fuel_economy_interp[valid_mask]
            
            if len(valid_range) > 0:
                ax2.plot(valid_range, valid_fuel_economy, '-', color='orange', 
                        linewidth=2, label='Fuel Economy')
                ax2.set_xlabel('Range (nautical miles)')
                ax2.set_ylabel('Fuel Consumption (L/ton/km)')
                ax2.grid(True)
                ax2.legend()
                ax2.set_title('Fuel Economy vs Range')
                ax2.set_xlim(left=0)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            
        if show:
            plt.show()
            
        return fig, (ax1, ax2)
    
    def analyze_and_plot(self, show=True, save_path=None, show_interpolation=True, num_interp_points=100):
        """
        Perform a complete analysis and generate the payload-range diagram with interpolation.
        
        Args:
            show (bool): Whether to display the plot
            save_path (str, optional): Path to save the plot image
            show_interpolation (bool): Whether to show interpolated analysis
            num_interp_points (int): Number of interpolation points
            
        Returns:
            tuple: Ranges dictionary, points list, and interpolation data
        """
        # Calculate mass fractions
        mass_fractions = self.calculate_mass_fractions()
        
        # Calculate weight ratios
        weight_ratios = self.calculate_weight_ratios(mass_fractions)
        
        # Calculate ranges
        ranges_m = self.calculate_ranges(weight_ratios)
        
        # Convert to nautical miles
        ranges_nm = self.meters_to_nautical_miles(ranges_m)
        
        # Generate points for the diagram
        points = self.generate_payload_range_points(ranges_nm)
        
        # Calculate interpolated data if requested
        interpolation_data = None
        if show_interpolation:
            range_interp, payload_interp, fuel_economy_interp = self.calculate_interpolated_fuel_economy(
                ranges_nm, num_interp_points)
            interpolation_data = (range_interp, payload_interp, fuel_economy_interp)
        
        # Plot the diagram
        if show:
            self.plot_payload_range_diagram(points, show=show, save_path=save_path, 
                                          show_interpolation=show_interpolation,
                                          num_interp_points=num_interp_points)
        
        return ranges_nm, points, interpolation_data
    
    def calculate_fuel_per_ton_km(self, mass_fractions, ranges_m):
        """
        Calculate fuel consumption in kg/ton/km for each scenario except ferry.

        Args:
        mass_fractions (dict): Mass fractions for each scenario.
        ranges_m (dict): Ranges in meters for each scenario.

        Returns:
        dict: Fuel consumption in kg/ton/km for each scenario.
        """
        fuel_per_ton_km = {}
        for key in ['harmonic', 'design', 'maxrange']:
            payload_ton = {
                'harmonic': self.max_payload / 1000,
                'design': self.design_payload / 1000,
                'maxrange': (self.design_payload - (self.fuel_max - self.fuel_design) / self.g) / 1000,
            }[key]
            range_km = ranges_m[key] / 1000
            # Fuel used is (1 - mass_fraction) * MTOW
            fuel_used = (1 - mass_fractions[key]) * self.MTOW / self.g / 0.76
            if range_km > 0 and payload_ton > 0:
                if self.mission_type == MissionType.ALTITUDE or self.mission_type == MissionType.DESIGN:
                    fuel_per_ton_km[key] = fuel_used / (payload_ton * range_km * 2)
                elif self.mission_type == MissionType.FERRY:
                    fuel_per_ton_km[key] = fuel_used / (payload_ton * range_km)
            else:
                fuel_per_ton_km[key] = float('nan')

        return fuel_per_ton_km
            
    def get_results_summary(self, short_summary=False, include_interpolation=False, num_interp_points=10):
        """
        Get a formatted summary of the calculation results.
        
        Args:
            short_summary (bool): Whether to return a short summary
            include_interpolation (bool): Whether to include interpolated results
            num_interp_points (int): Number of interpolation points to show
        
        Returns:
            str: Text summary of the results
        """
        mass_fractions = self.calculate_mass_fractions()
        weight_ratios = self.calculate_weight_ratios(mass_fractions)
        ranges_m = self.calculate_ranges(weight_ratios)
        ranges_nm = self.meters_to_nautical_miles(ranges_m)

        if short_summary:
            summary = f"Mission Type: {self.mission_type.name}\n"
            summary += f"  - Design: {ranges_nm['design']:.2f} nm ({ranges_m['design']/1000:.2f} km)\n"
            summary += f"  - Ferry: {ranges_nm['ferry']:.2f} nm ({ranges_m['ferry']/1000:.2f} km)\n"
            return summary
        
        summary = f"Range Calculation Results Summary\n"
        summary += f"===============================\n"
        summary += f"Aircraft Type: {self.aircraft_type}\n"
        summary += f"L/D Ratio: {self.L_D:.4f}\n\n"
        
        summary += f"Mass Fractions:\n"
        summary += f"  - Harmonic: {mass_fractions['harmonic']:.4f}\n"
        summary += f"  - Design: {mass_fractions['design']:.4f}\n"
        summary += f"  - Max Range: {mass_fractions['maxrange']:.4f}\n"
        summary += f"  - Ferry: {mass_fractions['ferry']:.4f}\n\n"
        
        summary += f"Ranges:\n"
        summary += f"  - Harmonic: {ranges_nm['harmonic']:.2f} nm ({ranges_m['harmonic']/1000:.2f} km)\n"
        summary += f"  - Design: {ranges_nm['design']:.2f} nm ({ranges_m['design']/1000:.2f} km)\n"
        summary += f"  - Max Range: {ranges_nm['maxrange']:.2f} nm ({ranges_m['maxrange']/1000:.2f} km)\n"
        summary += f"  - Ferry: {ranges_nm['ferry']:.2f} nm ({ranges_m['ferry']/1000:.2f} km)\n\n"

        summary += f"Fuel Consumption (Key Points):\n"
        fuel_per_ton_km = self.calculate_fuel_per_ton_km(mass_fractions, ranges_m)
        summary += f"  - Harmonic: {fuel_per_ton_km['harmonic']:.4f} L/ton/km\n"
        summary += f"  - Design: {fuel_per_ton_km['design']:.4f} L/ton/km\n"
        summary += f"  - Max Range: {fuel_per_ton_km['maxrange']:.4f} L/ton/km\n"

        
        if include_interpolation:
            summary += f"\nInterpolated Fuel Economy Analysis:\n"
            summary += f"===================================\n"
            
            # Calculate interpolated data
            range_interp, payload_interp, fuel_economy_interp = self.calculate_interpolated_fuel_economy(
                ranges_nm, num_interp_points)
            
            # Show a sample of interpolated points
            summary += f"Sample of {num_interp_points} interpolated points:\n"
            for i in range(0, len(range_interp), max(1, len(range_interp)//num_interp_points)):
                if not np.isnan(fuel_economy_interp[i]) and not np.isinf(fuel_economy_interp[i]):
                    summary += f"  Range: {range_interp[i]:.1f} nm, "
                    summary += f"Payload: {payload_interp[i]:.2f} t, "
                    summary += f"Fuel Economy: {fuel_economy_interp[i]:.4f} L/t/km\n"

        # Find the maximum fuel economy point from interpolation
        if include_interpolation:
            valid_mask = ~np.isnan(fuel_economy_interp) & ~np.isinf(fuel_economy_interp)
            if np.any(valid_mask):
                max_efficiency_idx = np.argmin(fuel_economy_interp[valid_mask])
                valid_range_interp = range_interp[valid_mask]
                valid_payload_interp = payload_interp[valid_mask]
                valid_fuel_economy_interp = fuel_economy_interp[valid_mask]
                
                max_efficiency_range = valid_range_interp[max_efficiency_idx]
                max_efficiency_payload = valid_payload_interp[max_efficiency_idx]
                max_efficiency_value = valid_fuel_economy_interp[max_efficiency_idx]
                
                summary += f"\nMost Fuel Efficient Point:\n"
                summary += f"  Range: {max_efficiency_range:.1f} nm, "
                summary += f"Payload: {max_efficiency_payload:.2f} t, "
                summary += f"Fuel Economy: {max_efficiency_value:.4f} L/t/km\n"
        
        return summary


def analyze_3_missions():
    data_file = "design3.json"
    all_ranges = {}
    for mission_type in [MissionType.DESIGN, MissionType.ALTITUDE, MissionType.FERRY]:
        range_calculator = RangeCalculator(data_file=data_file, mission_type=mission_type)
        ranges_nm, points, interpolation_data = range_calculator.analyze_and_plot(show=False)
        all_ranges[mission_type.name] = ranges_nm
        print(range_calculator.get_results_summary(short_summary=True)) 
    return all_ranges

def plot_mission_with_interpolation(mission_type):
    data_file = "design3.json"
    range_calculator = RangeCalculator(data_file=data_file, mission_type=mission_type)
    ranges_nm, points, interpolation_data = range_calculator.analyze_and_plot(
        show=True, show_interpolation=True, num_interp_points=500)
    print(range_calculator.get_results_summary(
        short_summary=False, include_interpolation=True, num_interp_points=500)) 
    return ranges_nm, points, interpolation_data

if __name__ == "__main__":
    plot_mission_with_interpolation(MissionType.DESIGN)