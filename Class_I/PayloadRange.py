import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
from Class_I.ClassIWeightEstimation import ClassI, MissionType


class RangeCalculator:
    """
    A class to calculate aircraft range and generate payload-range diagrams.
    
    This class encapsulates the range calculation logic and payload-range diagram 
    generation for different aircraft types.
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
        self.fuel_max = self.data.data["outputs"]['design']['max_fuel']
        self.fuel_reserve = self.data.data["outputs"]['design']['reserve_fuel']
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
        # Fill the area under the curve with light green
        ax.fill_between(x_coords, y_coords, color='lightgreen', alpha=0.3)
        
        # Highlight the design point (index 2)
        ax.plot(*points[2], color='lime', label='Design Point', marker='o', markersize=8)

        # Annotate all points
        labels = ['0 nm', 'Harmonic', 'Design', 'Max', 'Ferry']
        xylabel = [(30, 5),(70, -6),(55, 10),(75, -10),(0, 50)]
        for i, (x, y) in enumerate(points):
            if i == 0:
                continue
            x_format = int(float('%.3g' % x))
            y_format = float('%.3g' % y)
            ax.annotate(f"{labels[i]}: {x_format} nm\n at {y_format} tonnes", 
                        (x, y), 
                        textcoords="offset points", 
                        xytext=xylabel[i], 
                        ha='center', 
                        fontsize=11,
                        arrowprops=dict(arrowstyle="->", color='black', lw=1))
        
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
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
                fuel_used = (1 - mass_fractions[key]) * self.MTOW / self.g / 0.82
                if range_km > 0 and payload_ton > 0:
                    fuel_per_ton_km[key] = fuel_used / (payload_ton * range_km * 2)
                else:
                    fuel_per_ton_km[key] = float('nan')

            return fuel_per_ton_km
            
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
        summary += f"  - Ferry: {ranges_nm['ferry']:.2f} nm ({ranges_m['ferry']/1000:.2f} km)\n\n"

        summary += f"Fuel Consumption:\n"

        fuel_per_ton_km = self.calculate_fuel_per_ton_km(mass_fractions, ranges_m)

        summary += f"  - Harmonic: {fuel_per_ton_km['harmonic']:.4f} L/ton/km\n"
        summary += f"  - Design: {fuel_per_ton_km['design']:.4f} L/ton/km\n"
        summary += f"  - Max Range: {fuel_per_ton_km['maxrange']:.4f} L/ton/km\n"
        
        return summary
    
if __name__ == "__main__":
    # Example usage
    data_file = "design3.json"
    range_calculator = RangeCalculator(data_file=data_file, mission_type=MissionType.DESIGN)
    
    # Perform analysis and plot
    ranges_nm, points = range_calculator.analyze_and_plot(show=True)
    
    # Print summary
    print(range_calculator.get_results_summary())

    