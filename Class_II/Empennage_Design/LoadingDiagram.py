import os
import sys
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from utils import Data
import matplotlib.pyplot as plt
from Class_II.weight_distributions import CGCalculation

class LoadingDiagram:
    def __init__(self, aircraft_data: Data, wing_placement):
        self.aircraft_data = aircraft_data
        self.design_id = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_id}.json"


        self.cargo_start = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_distance_from_nose'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose']
        self.cargo_end = self.cargo_start + self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_length']
        self.cargo_y = 0
        self.cargo_z = 0
        self.cargo_cross_section = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_width'] * self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_height']
        self.cargo_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_length']
        self.cargo_density = self.aircraft_data.data['requirements']['cargo_density']
        # Heavy case: 100000 kg payload distributed over cargo volume
        self.cargo_density_heavy = 100000 * 9.81 / (self.cargo_cross_section * self.cargo_length)
        self.step_size = np.array([0.1, 0.0, 0.0])

        self.wing_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['wing_height']
        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.X_LEMAC = wing_placement*self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']

        # self.fuel_pos = self.X_LEMAC + 0.5*self.MAC
        self.fuel_weight = self.aircraft_data.data['outputs']['max']['total_fuel']

        self.OEW_cg = CGCalculation(self.aircraft_data).calculate_cg(0, OEW=True)

    @property
    def X_LEMAC(self):
        return self._X_LEMAC

    @X_LEMAC.setter
    def X_LEMAC(self, value):
        self._X_LEMAC = value
        self.fuel_pos = np.array([self._X_LEMAC + 0.5 * self.MAC, 0, self.wing_height])
        self.aircraft_data.data['outputs']['wing_design']['X_LEMAC'] = self._X_LEMAC
    
    def load_front_to_back_regular(self):
        curr_weight = self.aircraft_data.data['outputs']['component_weights']['total_OEW']
        # curr_cg = self.determine_OEW_cg()
        curr_cg = self.OEW_cg
        res = [curr_cg]
        res_weight = [curr_weight]
        cargo_points = [np.array([x, self.cargo_y, 0]) for x in np.arange(self.cargo_start, self.cargo_end+self.step_size[0], step=self.step_size[0])]
        for step in cargo_points:
            weight_increase = self.cargo_density*9.81 * self.cargo_cross_section * self.step_size[0]
            new_cg = (curr_weight*curr_cg + weight_increase*(step+self.step_size/2)) / (curr_weight+ weight_increase)
            curr_weight += weight_increase
            curr_cg = new_cg
            res.append(curr_cg)
            # print(f"res: {res}")
            res_weight.append(curr_weight)
        return np.array(res), np.array(res_weight)

    def load_front_to_back_heavy(self):
        curr_weight = self.aircraft_data.data['outputs']['component_weights']['total_OEW']
        # curr_cg = self.determine_OEW_cg()
        curr_cg = self.OEW_cg
        res = [curr_cg]
        res_weight = [curr_weight]
        cargo_points = [np.array([x, self.cargo_y, 0]) for x in np.arange(self.cargo_start, self.cargo_end+self.step_size[0], step=self.step_size[0])]
        for step in cargo_points:
            weight_increase = self.cargo_density_heavy * self.cargo_cross_section * self.step_size[0]
            new_cg = (curr_weight*curr_cg + weight_increase*(step+self.step_size/2)) / (curr_weight+ weight_increase)
            curr_weight += weight_increase
            curr_cg = new_cg
            res.append(curr_cg)
            res_weight.append(curr_weight)
        return np.array(res), np.array(res_weight)

    def load_back_to_front_regular(self):
        curr_weight = self.aircraft_data.data['outputs']['component_weights']['total_OEW']
        # curr_cg = self.determine_OEW_cg()
        curr_cg = self.OEW_cg
        res = [curr_cg]
        res_weight = [curr_weight]
        cargo_points = [np.array([x, self.cargo_y, 0]) for x in np.arange(self.cargo_end, self.cargo_start-self.step_size[0], step=-self.step_size[0])]
        for step in cargo_points:
            weight_increase = self.cargo_density*9.81 * self.cargo_cross_section * self.step_size[0]
            new_cg = (curr_weight*curr_cg + weight_increase*(step+self.step_size/2)) / (curr_weight+ weight_increase)
            curr_weight += weight_increase
            curr_cg = new_cg
            res.append(curr_cg)
            res_weight.append(curr_weight)
        return np.array(res), np.array(res_weight)

    def load_back_to_front_heavy(self):
        curr_weight = self.aircraft_data.data['outputs']['component_weights']['total_OEW']
        # curr_cg = self.determine_OEW_cg()
        curr_cg = self.OEW_cg
        res_cg = [curr_cg]
        res_weight = [curr_weight]
        cargo_points = [np.array([x, self.cargo_y, 0]) for x in np.arange(self.cargo_end, self.cargo_start-self.step_size[0], step=-self.step_size[0])]
        for step in cargo_points:
            weight_increase = self.cargo_density_heavy * self.cargo_cross_section * self.step_size[0]
            new_cg = (curr_weight*curr_cg + weight_increase*(step+self.step_size/2)) / (curr_weight+ weight_increase)
            curr_weight += weight_increase
            curr_cg = new_cg
            res_cg.append(curr_cg)
            res_weight.append(curr_weight)
        return np.array(res_cg), np.array(res_weight)
    
    def add_fuel_regular(self):
        curr_cg, curr_weight = self.load_front_to_back_regular()
        curr_cg = np.asarray(curr_cg[-1])
        curr_weight = curr_weight[-1]
        MTOW = self.aircraft_data.data['outputs']['max']['MTOW']
        self.fuel_weight = MTOW - curr_weight
        new_cg = (curr_cg*curr_weight + self.fuel_pos*self.fuel_weight)/(curr_weight+ self.fuel_weight)
        return new_cg, curr_weight + self.fuel_weight
    
    def add_fuel_heavy(self):
        curr_cg, curr_weight = self.load_front_to_back_heavy()
        curr_cg = np.asarray(curr_cg[-1])
        curr_weight = curr_weight[-1]
        MTOW = self.aircraft_data.data['outputs']['max']['MTOW']
        fuel_weight = MTOW - curr_weight
        new_cg = (curr_cg*curr_weight + self.fuel_pos*fuel_weight)/(curr_weight+fuel_weight)
        return new_cg, curr_weight + fuel_weight
    
    def add_fuel_OEW(self):
        curr_cg = self.OEW_cg
        curr_weight = self.aircraft_data.data['outputs']['component_weights']['total_OEW']
        fuel_weight = self.aircraft_data.data['outputs']['max']['max_fuel']
        new_cg = (curr_cg*curr_weight + self.fuel_pos*fuel_weight)/(curr_weight+fuel_weight)
        return new_cg, curr_weight + fuel_weight
    
    def update_json(self):
        self.determine_range()

        MTOW_cg = list(CGCalculation(self.aircraft_data).calculate_cg(self.aircraft_data.data['inputs']['engine']['engine_length'], OEW=False))
        OEW_cg = list(CGCalculation(self.aircraft_data).calculate_cg(self.aircraft_data.data['inputs']['engine']['engine_length'], OEW=True))
        self.aircraft_data.data['outputs']['cg_range']['most_aft_cg'] = self.max_cg * self.MAC + self.X_LEMAC
        self.aircraft_data.data['outputs']['cg_range']['most_forward_cg'] = self.min_cg * self.MAC + self.X_LEMAC
        self.aircraft_data.data['outputs']['cg_range']['most_aft_cg_mac'] = self.max_cg
        self.aircraft_data.data['outputs']['cg_range']['most_forward_cg_mac'] = self.min_cg
        self.aircraft_data.data['outputs']['cg_range']['highest_cg'] = self.highest_cg
        self.aircraft_data.data['outputs']['cg_range']['lowest_cg'] = self.lowest_cg
        self.aircraft_data.data['outputs']['cg_range']['most_right_cg'] = self.most_right
        self.aircraft_data.data['outputs']['cg_range']['most_left_cg'] = self.most_left
        self.aircraft_data.data['outputs']['cg_range']['MTOW_cg'] = MTOW_cg
        self.aircraft_data.data['outputs']['cg_range']['OEW_cg'] = OEW_cg
        self.aircraft_data.save_design(self.design_file)

    def determine_range(self):
        OEW_cg = CGCalculation(self.aircraft_data).calculate_cg(0, OEW=True)
        f2b_cg, f2b_weight = self.load_front_to_back_regular()
        f2b_heavy_cg, f2b_heavy_weight = self.load_front_to_back_heavy()
        b2f_cg, b2f_weight = self.load_back_to_front_regular()
        b2f_heavy_cg, b2f_heavy_weight = self.load_back_to_front_heavy()
        fuel_cg, fuel_weight = self.add_fuel_regular()
        fuel_OEW_cg, fuel_OEW_weight = self.add_fuel_OEW()


        OEW_cg_mac = (OEW_cg[0] - self.X_LEMAC) / self.MAC
        f2b_cg_mac = (f2b_cg[:, 0] - self.X_LEMAC) / self.MAC
        b2f_cg_mac = (b2f_cg[:, 0] - self.X_LEMAC) / self.MAC
        f2b_heavy_cg_mac = (f2b_heavy_cg[:, 0] - self.X_LEMAC) / self.MAC
        b2f_heavy_cg_mac = (b2f_heavy_cg[:, 0] - self.X_LEMAC) / self.MAC
        fuel_cg_mac = (fuel_cg[0] - self.X_LEMAC) / self.MAC
        fuel_heavy_cg_mac = (fuel_cg[0] - self.X_LEMAC) / self.MAC
        fuel_OEW_cg_mac = (fuel_OEW_cg[0] - self.X_LEMAC) / self.MAC

        self.highest_cg = max(
            OEW_cg[2],
            f2b_cg[:, 2].max(),
            f2b_heavy_cg[:, 2].max(),
            fuel_cg[2],
            fuel_OEW_cg[2]
        )


        self.lowest_cg = min(
            OEW_cg[2],
            f2b_cg[:, 2].min(),
            f2b_heavy_cg[:, 2].min(),
            fuel_cg[2],
            fuel_OEW_cg[2]
        )

        self.most_right = max(
            OEW_cg[1],
            f2b_cg[:, 1].max(),
            f2b_heavy_cg[:, 1].max(),
            fuel_cg[1],
            fuel_OEW_cg[1]
        )
        self.most_left = min(
            OEW_cg[1],
            f2b_cg[:, 1].min(),
            f2b_heavy_cg[:, 1].min(),
            fuel_cg[1],
            fuel_OEW_cg[1]
        )

        self.min_cg = min(
            OEW_cg_mac,
            f2b_cg_mac.min(),
            f2b_heavy_cg_mac.min(),
            fuel_cg_mac,
            fuel_OEW_cg_mac
        )
        self.max_cg = max(
            OEW_cg_mac,
            f2b_cg_mac.max(),
            f2b_heavy_cg_mac.max(),
            fuel_cg_mac,
            fuel_heavy_cg_mac,
        )
        return 0.98*self.min_cg, 1.02*self.max_cg


    

    def plot(self):
        # OEW_cg = self.determine_OEW_cg()
        f2b_cg, f2b_weight = self.load_front_to_back_regular()
        f2b_heavy_cg, f2b_heavy_weight = self.load_front_to_back_heavy()
        b2f_cg, b2f_weight = self.load_back_to_front_regular()
        b2f_heavy_cg, b2f_heavy_weight = self.load_back_to_front_heavy()
        fuel_cg, fuel_weight = self.add_fuel_regular()
        fuel_heavy_cg, fuel_heavy_weight = self.add_fuel_heavy()
        fuel_OEW_cg, fuel_OEW_weight = self.add_fuel_OEW()

        OEW_cg_mac = (self.OEW_cg[0] - self.X_LEMAC) / self.MAC
        f2b_cg_mac = (f2b_cg[:, 0] - self.X_LEMAC) / self.MAC
        b2f_cg_mac = (b2f_cg[:, 0] - self.X_LEMAC) / self.MAC
        f2b_heavy_cg_mac = (f2b_heavy_cg[:, 0] - self.X_LEMAC) / self.MAC
        b2f_heavy_cg_mac = (b2f_heavy_cg[:, 0] - self.X_LEMAC) / self.MAC
        fuel_cg_mac = (fuel_cg[0] - self.X_LEMAC) / self.MAC
        fuel_heavy_cg_mac = (fuel_heavy_cg[0] - self.X_LEMAC) / self.MAC
        fuel_OEW_cg_mac = (fuel_OEW_cg[0] - self.X_LEMAC) / self.MAC

        OEW_kg = self.aircraft_data.data['outputs']['component_weights']['total_OEW'] / 9.81  # Convert to kg
        f2b_weight = f2b_weight / 9.81  # Convert to kg
        b2f_weight = b2f_weight / 9.81
        f2b_heavy_weight = f2b_heavy_weight / 9.81
        b2f_heavy_weight = b2f_heavy_weight / 9.81
        fuel_weight = fuel_weight / 9.81
        fuel_heavy_weight = fuel_heavy_weight / 9.81
        fuel_OEW_weight = fuel_OEW_weight / 9.81

        fig, axs = plt.subplots(1, 2, figsize=(16, 6))

        # self.aircraft_data.data['outputs']['fuselage_dimensions'] reference frame (x in meters)
        axs[0].plot(f2b_cg[:, 0], f2b_weight, label='Front to Back Regular', color='blue')
        # axs[0].plot(b2f_cg[:, 0], b2f_weight, label='Back to Front Regular', color='cyan')
        axs[0].plot(f2b_heavy_cg[:, 0], f2b_heavy_weight, label='Front to Back Heavy', color='red')
        # axs[0].plot(b2f_heavy_cg[:, 0], b2f_heavy_weight, label='Back to Front Heavy', color='orange')
        axs[0].scatter(fuel_cg[0], fuel_weight, color='green', label='Fuel CG', zorder=5)
        axs[0].plot([f2b_cg[-1, 0], fuel_cg[0]], [f2b_weight[-1], fuel_weight], color='green', linestyle='--', label='Fuel Loading')
        axs[0].scatter(fuel_heavy_cg[0], fuel_heavy_weight, color='darkgreen', label='Fuel CG (Heavy)', zorder=5)
        axs[0].plot([f2b_heavy_cg[-1, 0], fuel_heavy_cg[0]], [f2b_heavy_weight[-1], fuel_heavy_weight], color='darkgreen', linestyle='--', label='Fuel Loading (Heavy)')
        axs[0].scatter(fuel_OEW_cg[0], fuel_OEW_weight, color='purple', label='Fuel CG (OEW)', zorder=5)
        axs[0].plot([self.OEW_cg[0], fuel_OEW_cg[0]], [OEW_kg, fuel_OEW_weight], color='purple', linestyle='--', label='Fuel Loading (OEW)')
        # axs[0].axvline(self.X_LEMAC, color='black', linestyle=':', label='X_LEMAC')
        axs[0].set_xlabel('CG X Position (m)')
        axs[0].set_ylabel('Weight (kg)')
        axs[0].set_title('Loading Diagram Fuselage Reference Frame')
        axs[0].grid(True)

        # MAC reference frame (x in MAC)
        axs[1].plot(f2b_cg_mac, f2b_weight, color='blue')
        # axs[1].plot(b2f_cg_mac, b2f_weight, color='cyan')
        axs[1].plot(f2b_heavy_cg_mac, f2b_heavy_weight, color='red')
        # axs[1].plot(b2f_heavy_cg_mac, b2f_heavy_weight, color='orange')
        axs[1].scatter(fuel_cg_mac, fuel_weight, color='green', zorder=5)
        axs[1].plot([f2b_cg_mac[-1], fuel_cg_mac], [f2b_weight[-1], fuel_weight], color='green', linestyle='--')
        axs[1].scatter(fuel_heavy_cg_mac, fuel_heavy_weight, color='darkgreen', zorder=5)
        axs[1].plot([f2b_heavy_cg_mac[-1], fuel_heavy_cg_mac], [f2b_heavy_weight[-1], fuel_heavy_weight], color='darkgreen', linestyle='--')
        axs[1].scatter(fuel_OEW_cg_mac, fuel_OEW_weight, color='purple', zorder=5)
        axs[1].plot([OEW_cg_mac, fuel_OEW_cg_mac], [OEW_kg, fuel_OEW_weight], color='purple', linestyle='--')
        axs[1].set_xlabel('CG X Position (MAC)')
        axs[1].set_ylabel('Weight (kg)')
        axs[1].set_title('Loading Diagram (MAC Reference Frame)')
        axs[1].grid(True)

        # Single legend for all lines, placed outside the right of the figure
        handles, labels = axs[0].get_legend_handles_labels()
        handles1, labels1 = axs[1].get_legend_handles_labels()
        handles += [h for h, l in zip(handles1, labels1) if l not in labels]
        labels += [l for l in labels1 if l not in labels]
        fig.legend(handles, labels, bbox_to_anchor=(1.02, 1), loc='upper left')

        plt.subplots_adjust(right=0.75)  # Make space for the legend on the right
        plt.tight_layout()
        plt.show()

    def plot_all_dimensions(self):
        # Get all loading cases
        f2b_cg, f2b_weight = self.load_front_to_back_regular()
        f2b_heavy_cg, f2b_heavy_weight = self.load_front_to_back_heavy()
        b2f_cg, b2f_weight = self.load_back_to_front_regular()
        b2f_heavy_cg, b2f_heavy_weight = self.load_back_to_front_heavy()
        fuel_cg, fuel_weight = self.add_fuel_regular()
        fuel_heavy_cg, fuel_heavy_weight = self.add_fuel_heavy()
        fuel_OEW_cg, fuel_OEW_weight = self.add_fuel_OEW()

        OEW_kg = self.aircraft_data.data['outputs']['component_weights']['total_OEW'] / 9.81  # Convert to kg
        f2b_weight = f2b_weight / 9.81  # Convert to kg
        b2f_weight = b2f_weight / 9.81
        f2b_heavy_weight = f2b_heavy_weight / 9.81
        b2f_heavy_weight = b2f_heavy_weight / 9.81
        fuel_weight = fuel_weight / 9.81
        fuel_heavy_weight = fuel_heavy_weight / 9.81
        fuel_OEW_weight = fuel_OEW_weight / 9.81

        dim_labels = ['X (Longitudinal)', 'Y (Lateral)', 'Z (Vertical)']
        fig, axs = plt.subplots(3, 1, figsize=(10, 14), sharex=False)

        for dim in range(3):
            axs[dim].plot(f2b_cg[:, dim], f2b_weight, label='Front to Back Regular', color='blue')
            axs[dim].plot(f2b_heavy_cg[:, dim], f2b_heavy_weight, label='Front to Back Heavy', color='red')
            axs[dim].scatter(fuel_cg[dim], fuel_weight, color='green', label='Fuel CG', zorder=5)
            axs[dim].plot([f2b_cg[-1, dim], fuel_cg[dim]], [f2b_weight[-1], fuel_weight], color='green', linestyle='--', label='Fuel Loading')
            axs[dim].scatter(fuel_heavy_cg[dim], fuel_heavy_weight, color='darkgreen', label='Fuel CG (Heavy)', zorder=5)
            axs[dim].plot([f2b_heavy_cg[-1, dim], fuel_heavy_cg[dim]], [f2b_heavy_weight[-1], fuel_heavy_weight], color='darkgreen', linestyle='--', label='Fuel Loading (Heavy)')
            axs[dim].scatter(fuel_OEW_cg[dim], fuel_OEW_weight, color='purple', label='Fuel CG (OEW)', zorder=5)
            axs[dim].plot([self.OEW_cg[dim], fuel_OEW_cg[dim]], [OEW_kg, fuel_OEW_weight], color='purple', linestyle='--', label='Fuel Loading (OEW)')
            axs[dim].scatter(self.OEW_cg[dim], OEW_kg, color='black', label='OEW CG', zorder=6, marker='x', s=80)
            axs[dim].set_xlabel(f'CG {dim_labels[dim]} Position (m)')
            axs[dim].set_ylabel('Weight (kg)')
            axs[dim].set_title(f'Loading Diagram Fuselage Reference Frame ({dim_labels[dim]})')
            axs[dim].grid(True)
            axs[dim].legend(bbox_to_anchor=(1.05, 1), loc='upper left')

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    wing_placement = 0.349
    loading_diagram = LoadingDiagram(aircraft_data=aircraft_data, wing_placement=wing_placement)
    # loading_diagram.plot()
    # min_cg, max_cg = loading_diagram.determine_range()
    # print(f"Min CG (MAC): {min_cg:.3f}, Max CG (MAC): {max_cg:.3f}")
    loading_diagram.update_json()
    loading_diagram.plot_all_dimensions()