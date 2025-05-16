import numpy as np
import matplotlib.pyplot as plt
from utils import Data
from ClassIWeightEstimation import ClassI, MissionType


class RangeCalculator:
    """
    A class to calculate aircraft range and generate payload-range diagrams.
    
    This class encapsulates the range calculation logic and payload-range diagram 
    generation for different aircraft types.
    """
    
    def __init__(self, data_file=None, data_object=None):
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
            
        # Extract common parameters from data
        self.aircraft_type = self.data.data["inputs"]['aircraft_type']
        self.eta_p = self.data.data["inputs"]['prop_efficiency']
        self.cp = self.data.data["inputs"]['prop_consumption']
        self.cj = self.data.data["inputs"]['jet_consumption']
        self.V = self.data.data["requirements"]['cruise_speed']
        
        # Calculate L/D ratio
        aspect_ratio = self.data.data["inputs"]['aspect_ratio']
        oswald_factor = self.data.data["inputs"]['oswald_factor']
        Cd0 = self.data.data["inputs"]['Cd0']
        self.L_D = self.data.data["outputs"]['design']['LD']
        
        # Extract weight and fuel data
        self.fuel_design = self.data.data["outputs"]['design']['mission_fuel']
        self.fuel_max = self.data.data["outputs"]['design']['max_fuel']
        self.fuel_reserve = self.data.data["outputs"]['design']['reserve_fuel']
        self.design_payload = self.data.data["requirements"]['design_payload']
        self.max_payload = 100 * 1_000  # kg This could be made configurable !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.MTOW = self.data.data["outputs"]['design']['MTOW']
        
        # Calculate fuel fractions without cruise
        self.class_i = ClassI(aircraft_data=self.data, mission_type=MissionType.DESIGN)
        self.fuel_fracs_no_cruise = self.class_i.fuel_fractions
        self.Mff_nocruise = 1
        for fraction in self.fuel_fracs_no_cruise.values():
            self.Mff_nocruise *= fraction
    
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
            R = (self.V * self.L_D * np.log(W4_W5)) / (self.g * self.cj)
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
    
    def calculate_weight_ratios(self, mass_fractions):
        """
        Calculate weight ratios based on mass fractions.
        
        Args:
            mass_fractions (dict): Mass fractions dictionary
            
        Returns:
            dict: Dictionary containing weight ratios
        """
        W4_W5_harmonic = np.sqrt(1 / mass_fractions["harmonic"] * self.Mff_nocruise**2)
        W4_W5_design = np.sqrt(1 / mass_fractions["design"] * self.Mff_nocruise**2)
        W4_W5_maxrange = np.sqrt(1 / mass_fractions["maxrange"] * self.Mff_nocruise**2)
        W4_W5_ferry = np.sqrt(1 / mass_fractions["ferry"] * self.Mff_nocruise**2)
        
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
    
    def plot_payload_range_diagram(self, points, show=True, save_path=None):
        """
        Plot the payload-range diagram.
        
        Args:
            points (list): List of (range, payload) points
            show (bool): Whether to display the plot
            save_path (str, optional): Path to save the plot image
            
        Returns:
            tuple: Figure and axis objects
        """
        # Extract x and y coordinates from points
        x_coords, y_coords = zip(*points)
        
        # Create plot
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(x_coords, y_coords, marker='o', linestyle='-', color='b', label='Payload-Range Curve')
        
        # Highlight the design point (index 2)
        ax.plot(*points[2], color='lime', label='Design Point', marker='o', markersize=8)
        
        ax.set_xlabel('Range (nautical miles)')
        ax.set_ylabel('Payload (tonnes)')
        ax.legend(loc='upper right')
        ax.grid(True)
        
        if save_path:
            plt.savefig(save_path)
            
        if show:
            plt.show()
            
        return fig, ax
    
    def analyze_and_plot(self, show=True, save_path=None):
        """
        Perform a complete analysis and generate the payload-range diagram.
        
        Args:
            show (bool): Whether to display the plot
            save_path (str, optional): Path to save the plot image
            
        Returns:
            tuple: Ranges dictionary and points list
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
        
        # Plot the diagram
        self.plot_payload_range_diagram(points, show=show, save_path=save_path)
        
        return ranges_nm, points
    
    def get_results_summary(self):
        """
        Get a formatted summary of the calculation results.
        
        Returns:
            str: Text summary of the results
        """
        mass_fractions = self.calculate_mass_fractions()
        weight_ratios = self.calculate_weight_ratios(mass_fractions)
        ranges_m = self.calculate_ranges(weight_ratios)
        ranges_nm = self.meters_to_nautical_miles(ranges_m)
        
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
        summary += f"  - Ferry: {ranges_nm['ferry']:.2f} nm ({ranges_m['ferry']/1000:.2f} km)\n"
        
        return summary
    
if __name__ == "__main__":
    # Example usage
    data_file = "design3.json"
    range_calculator = RangeCalculator(data_file=data_file)
    
    # Perform analysis and plot
    ranges_nm, points = range_calculator.analyze_and_plot(show=True)
    
    # Print summary
    print(range_calculator.get_results_summary())

    