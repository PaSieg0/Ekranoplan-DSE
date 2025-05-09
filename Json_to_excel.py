from utils import Data
import openpyxl

def format_value(value):
    if isinstance(value, (int, float)):
        return f"{value:.3f}" if abs(value) >= 0.001 else f"{value:.3e}"
    return value

def design_json_to_excel(json_file: str, excel_file: str) -> None:
    """
    Convert a JSON file containing design parameters to an Excel file. If the Excel file already exists, add a new sheet to it.
    
    Args:
        json_file (str): Path to the input JSON file.
        excel_file (str): Path to the output Excel file.
    """
    # Load the JSON data
    data = Data(json_file).data

    variable_units = {
        "design_range": "[m]", "design_payload": "[kg]", "ferry_range": "[m]", "reserve_range": "[m]",
        "ferry_payload": "[kg]", "altitude_range_WIG": "[m]", "altitude_range_WOG": "[m]", "altitude_payload": "[kg]",
        "cruise_speed": "[m/s]", "jet_consumption": "[kg/(N·s)]", "prop_consumption": "[kg/J]",
        "reserve_fuel": "[N]", "take_off_power": "[W]", "take_off_thrust": "[N]", "cruise_altitude": "[km]",
        "MTOM": "[kg]", "MTOW": "[N]", "OEW": "[N]", "ZFW": "[N]", "EW": "[N]", "fuel": "[N]",
        "fuel_used": "[N]", "fuel_reserve": "[N]", "S": "[m²]", "b": "[m]", "MAC": "[m]", "WP": "[N/W]",
        "WS": "[N/m²]", "fuel_economy": "[L/ton/km]", "P": "[W]", "T": "[N]", "stall_speed_clean": "[m/s]",
        "stall_speed_takeoff": "[m/s]", "stall_speed_landing": "[m/s]", "high_altitude": "[m]", "L": "[m]", "r": "[m]",
        "hull_surface": "[m²]", "gravitational_acceleration": "[m/s²]", "kinematic_viscosity": "[m²/s]", "rho_water": "[kg/m³]", 
        "l_fuselage": "[m]", "l_cargo_straight": "[m]", "r_fuselage": "[m]", "d_fuselage": "[m]", "l_tailcone": "[m]", "l_nose": "[m]",
        "V_lof": "[m/s]", "cargo_width": "[m]", "cargo_height": "[m]", "cargo_length": "[m]", "cargo_density": "[kg/m³]",
        "sweep_c_4": "[deg]", "dihedral": "[deg]", "sweep_x_c": "[deg]", "sweep_TE": "[deg]", "chord_root": "[m]", "chord_tip": "[m]",
        "y_MAC": "[m]", "X_LEMAC": "[m]", "X_LE": "[m]"
    }

    requirements = data.get('requirements', {})
    inputs = data.get('inputs', {})
    general_outputs = data.get('outputs', {}).get('general', {})
    design_outputs = data.get('outputs', {}).get('design', {})
    max_outputs = data.get('outputs', {}).get('max', {})
    wing_design = data.get('outputs', {}).get('wing_design', {})

    del data['outputs']
    del data['requirements']
    del data['inputs']

    try:
        excelsheet = openpyxl.load_workbook(excel_file)
        if f"Design {data['design_id']} Data" in excelsheet.sheetnames:
            del excelsheet[f"Design {data['design_id']} Data"]
        sheet = excelsheet.create_sheet(title=f"Design {data['design_id']} Data")
    except FileNotFoundError:
        excelsheet = openpyxl.Workbook()
        sheet = excelsheet.active
        sheet.title = f"Design {data['design_id']} Data"

    # Requirements
    sheet.cell(row=1, column=1, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=2, value="Requirements")
    row = 2
    for key, value in requirements.items():
        label = f"{key} {variable_units[key]}" if key in variable_units else key
        sheet.cell(row=row, column=1, value=label)
        sheet.cell(row=row, column=2, value=format_value(value) if isinstance(value, (int, float)) else value)
        row += 1

    # General States
    sheet.cell(row=row+1, column=1, value=f"General States")
    row += 2
    for key, value in data.items():
        label = f"{key} {variable_units[key]}" if key in variable_units else key
        sheet.cell(row=row, column=1, value=label)
        sheet.cell(row=row, column=2, value=format_value(value) if isinstance(value, (int, float)) else value)
        row += 1

    # Inputs
    sheet.cell(row=1, column=4, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=5, value="Inputs")
    row_inputs = 2
    for key, value in inputs.items():
        label = f"{key} {variable_units[key]}" if key in variable_units else key
        sheet.cell(row=row_inputs, column=4, value=label)
        sheet.cell(row=row_inputs, column=5, value=format_value(value) if isinstance(value, (int, float)) else value)
        row_inputs += 1

    # General Outputs
    sheet.cell(row=1, column=7, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=8, value="General Outputs")
    row_gen_out = 2
    for key, value in general_outputs.items():
        label = f"{key} {variable_units[key]}" if key in variable_units else key
        sheet.cell(row=row_gen_out, column=7, value=label)
        sheet.cell(row=row_gen_out, column=8, value=format_value(value) if isinstance(value, (int, float)) else value)
        row_gen_out += 1

    # Design Mission & Max Outputs
    sheet.cell(row=1, column=10, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=11, value="Design Mission")

    row_design = 2
    excluded_keys = {"S", "b", "MAC", "MTOM", "fuel_economy"}
    for key, value in design_outputs.items():
        if key in excluded_keys:
            continue
        label = f"{key} {variable_units[key]}" if key in variable_units else key
        sheet.cell(row=row_design, column=10, value=label)
        sheet.cell(row=row_design, column=11, value=f"{value:.7f}" if isinstance(value, (int, float)) else value)
        row_design += 1

    for key, value in max_outputs.items():
        label = f"{key} {variable_units[key]}" if key in variable_units else key
        sheet.cell(row=row_design, column=10, value=label)
        sheet.cell(row=row_design, column=11, value=f"{value:.7f}" if isinstance(value, (int, float)) else value)
        row_design += 1

    # Wing Design
    sheet.cell(row=row_inputs + 1, column=4, value=f"Design {data['design_id']}")
    sheet.cell(row=row_inputs + 1, column=5, value="Wing Design")
    row_wing = row_inputs + 2
    for key, value in wing_design.items():
        label = f"{key} {variable_units[key]}" if key in variable_units else key
        sheet.cell(row=row_wing, column=4, value=label)
        sheet.cell(row=row_wing, column=5, value=format_value(value) if isinstance(value, (int, float)) else value)
        row_wing += 1

    # Save
    excelsheet.save(excel_file)


    

if __name__ == "__main__":
    json_file = "design1.json"  # Replace with your JSON file path
    excel_file = "Concept Data.xlsx"   # Replace with your desired Excel file path
    design_json_to_excel(json_file, excel_file)