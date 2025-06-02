import os
import sys
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
import matplotlib.pyplot as plt
from Class_II.weight_distributions import CGCalculation

class LoadingDiagram:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_id = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_id}.json"


        # Precompute and store constants
        general = self.aircraft_data.data['outputs']['general']
        requirements = self.aircraft_data.data['requirements']
        self.cargo_start = general['cargo_distance_from_nose'] + self.aircraft_data.data['outputs']['general']['l_nose']
        self.cargo_end = self.cargo_start + general['cargo_length']
        self.cargo_y = general.get('cargo_y', 0.0)
        self.cargo_z = general.get('cargo_z', 0.0)
        self.cargo_cross_section = general['cargo_width'] * general['cargo_height']
        self.cargo_length = general['cargo_length']
        self.cargo_density = requirements['cargo_density']
        # Heavy case: 100000 kg payload distributed over cargo volume
        self.cargo_density_heavy = 100000 * 9.81 / (self.cargo_cross_section * self.cargo_length)
        self.step_size = np.array([0.1, 0.0, 0.0])

        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.X_LEMAC = 1*self.aircraft_data.data['outputs']['general']['l_fuselage']

        # self.fuel_pos = self.X_LEMAC + 0.5*self.MAC
        self.fuel_weight = self.aircraft_data.data['outputs']['max']['total_fuel']

    @property
    def X_LEMAC(self):
        return self._X_LEMAC

    @X_LEMAC.setter
    def X_LEMAC(self, value):
        self._X_LEMAC = value
        self.fuel_pos = self._X_LEMAC + 0.5 * self.MAC
        self.aircraft_data.data['outputs']['wing_design']['X_LEMAC'] = self._X_LEMAC

    # def determine_OEW_cg(self):
    #     self.component_cgs = {}
    #     for key, positions in self.component_positions.items():
    #         if key == 'wing':
    #             positions = np.asarray(positions)
    #             positions[0] = self.X_LEMAC + 0.5*self.MAC
    #             print(f"Positions for {key}: {positions}")
    #             weight = self.aircraft_data.data['outputs']['component_weights'][key]
    #             self.component_cgs[key] = positions * weight
    #         if key == 'engine':
    #             positions = np.asarray(positions)
    #             positions[0] = self.X_LEMAC
    #             weight = self.aircraft_data.data['outputs']['component_weights'][key]
    #             self.component_cgs[key] = positions * weight
    #         if key == 'nacelle_group':
    #             positions = np.asarray(positions)
    #             positions[0] = self.X_LEMAC
    #             weight = self.aircraft_data.data['outputs']['component_weights'][key]
    #             self.component_cgs[key] = positions * weight
    #         else:
    #             weight = self.aircraft_data.data['outputs']['component_weights'][key]
    #             self.component_cgs[key] = np.asarray(positions) * weight
    #     self.OEW_cg = np.sum(list(self.component_cgs.values()), axis=0) / np.sum(list(self.aircraft_data.data['outputs']['component_weights'].values()))
    #     self.OEW_cg[0] = self.X_LEMAC + 0.5*self.MAC
    #     print(f"OEW CG: {self.OEW_cg}")
    #     return self.OEW_cg
    
    def load_front_to_back_regular(self):
        curr_weight = self.aircraft_data.data['outputs']['component_weights']['total_OEW']
        # curr_cg = self.determine_OEW_cg()
        curr_cg = np.asarray([CGCalculation(self.aircraft_data).calculate_cg(0, OEW=True), 0, 0])
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
        curr_cg = np.asarray([CGCalculation(self.aircraft_data).calculate_cg(0, OEW=True), 0, 0])
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
        curr_cg = np.asarray([CGCalculation(self.aircraft_data).calculate_cg(0, OEW=True), 0, 0])
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
        curr_cg = np.asarray([CGCalculation(self.aircraft_data).calculate_cg(0, OEW=True), 0, 0])
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

    def determine_range(self):
        OEW_cg = np.asarray([CGCalculation(self.aircraft_data).calculate_cg(0, OEW=True), 0, 0])
        f2b_cg, f2b_weight = self.load_front_to_back_regular()
        f2b_heavy_cg, f2b_heavy_weight = self.load_front_to_back_heavy()
        b2f_cg, b2f_weight = self.load_back_to_front_regular()
        b2f_heavy_cg, b2f_heavy_weight = self.load_back_to_front_heavy()
        fuel_cg, fuel_weight = self.add_fuel_regular()

        f2b_cg_mac = (f2b_cg[:, 0] - self.X_LEMAC) / self.MAC
        b2f_cg_mac = (b2f_cg[:, 0] - self.X_LEMAC) / self.MAC
        f2b_heavy_cg_mac = (f2b_heavy_cg[:, 0] - self.X_LEMAC) / self.MAC
        b2f_heavy_cg_mac = (b2f_heavy_cg[:, 0] - self.X_LEMAC) / self.MAC
        fuel_cg_mac = (fuel_cg[0] - self.X_LEMAC) / self.MAC

        self.min_cg = min(
            f2b_cg_mac.min(),
            b2f_cg_mac.min(),
            f2b_heavy_cg_mac.min(),
            b2f_heavy_cg_mac.min(),
            fuel_cg_mac
        )
        self.max_cg = max(
            f2b_cg_mac.max(),
            b2f_cg_mac.max(),
            f2b_heavy_cg_mac.max(),
            b2f_heavy_cg_mac.max(),
            fuel_cg_mac
        )

        return self.min_cg, self.max_cg

    

    def plot(self):
        # OEW_cg = self.determine_OEW_cg()
        OEW_cg = np.asarray([CGCalculation(self.aircraft_data).calculate_cg(0, OEW=True), 0, 0])
        print(f"OEW CG: {OEW_cg}")
        f2b_cg, f2b_weight = self.load_front_to_back_regular()
        f2b_heavy_cg, f2b_heavy_weight = self.load_front_to_back_heavy()
        b2f_cg, b2f_weight = self.load_back_to_front_regular()
        b2f_heavy_cg, b2f_heavy_weight = self.load_back_to_front_heavy()
        fuel_cg, fuel_weight = self.add_fuel_regular()
        fuel_heavy_cg, fuel_heavy_weight = self.add_fuel_heavy()

        f2b_cg_mac = (f2b_cg[:, 0] - self.X_LEMAC) / self.MAC
        b2f_cg_mac = (b2f_cg[:, 0] - self.X_LEMAC) / self.MAC
        f2b_heavy_cg_mac = (f2b_heavy_cg[:, 0] - self.X_LEMAC) / self.MAC
        b2f_heavy_cg_mac = (b2f_heavy_cg[:, 0] - self.X_LEMAC) / self.MAC
        fuel_cg_mac = (fuel_cg[0] - self.X_LEMAC) / self.MAC
        fuel_heavy_cg_mac = (fuel_heavy_cg[0] - self.X_LEMAC) / self.MAC

        f2b_weight = f2b_weight / 9.81  # Convert to kg
        b2f_weight = b2f_weight / 9.81
        f2b_heavy_weight = f2b_heavy_weight / 9.81
        b2f_heavy_weight = b2f_heavy_weight / 9.81
        fuel_weight = fuel_weight / 9.81
        fuel_heavy_weight = fuel_heavy_weight / 9.81

        fig, axs = plt.subplots(1, 2, figsize=(16, 6))

        # General reference frame (x in meters)
        axs[0].plot(f2b_cg[:, 0], f2b_weight, label='Front to Back Regular', color='blue')
        axs[0].plot(b2f_cg[:, 0], b2f_weight, label='Back to Front Regular', color='cyan')
        axs[0].plot(f2b_heavy_cg[:, 0], f2b_heavy_weight, label='Front to Back Heavy', color='red')
        axs[0].plot(b2f_heavy_cg[:, 0], b2f_heavy_weight, label='Back to Front Heavy', color='orange')
        axs[0].scatter(fuel_cg[0], fuel_weight, color='green', label='Fuel CG', zorder=5)
        axs[0].plot([f2b_cg[-1, 0], fuel_cg[0]], [f2b_weight[-1], fuel_weight], color='green', linestyle='--', label='Fuel Loading')
        axs[0].scatter(fuel_heavy_cg[0], fuel_heavy_weight, color='darkgreen', label='Fuel CG (Heavy)', zorder=5)
        axs[0].plot([f2b_heavy_cg[-1, 0], fuel_heavy_cg[0]], [f2b_heavy_weight[-1], fuel_heavy_weight], color='darkgreen', linestyle='--', label='Fuel Loading (Heavy)')
        axs[0].axvline(self.X_LEMAC, color='black', linestyle=':', label='X_LEMAC')
        axs[0].set_xlabel('CG X Position (m)')
        axs[0].set_ylabel('Weight (kg)')
        axs[0].set_title('Loading Diagram (General Reference Frame)')
        axs[0].grid(True)

        # MAC reference frame (x in MAC)
        axs[1].plot(f2b_cg_mac, f2b_weight, color='blue')
        axs[1].plot(b2f_cg_mac, b2f_weight, color='cyan')
        axs[1].plot(f2b_heavy_cg_mac, f2b_heavy_weight, color='red')
        axs[1].plot(b2f_heavy_cg_mac, b2f_heavy_weight, color='orange')
        axs[1].scatter(fuel_cg_mac, fuel_weight, color='green', zorder=5)
        axs[1].plot([f2b_cg_mac[-1], fuel_cg_mac], [f2b_weight[-1], fuel_weight], color='green', linestyle='--')
        axs[1].scatter(fuel_heavy_cg_mac, fuel_heavy_weight, color='darkgreen', zorder=5)
        axs[1].plot([f2b_heavy_cg_mac[-1], fuel_heavy_cg_mac], [f2b_heavy_weight[-1], fuel_heavy_weight], color='darkgreen', linestyle='--')
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


if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    loading_diagram = LoadingDiagram(aircraft_data=aircraft_data)
    loading_diagram.plot()
    # min_cg, max_cg = loading_diagram.determine_range()
    # print(f"Minimum CG: {min_cg}, Maximum CG: {max_cg}")
    loading_diagram.X_LEMAC = 40
    loading_diagram.plot()