from utils import Data
import openpyxl


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
    "design_range": "m",
    "design_payload": "kg",
    "ferry_range": "m",
    "ferry_payload": "kg",
    "altitude_range_WIG": "m",
    "altitude_range_WOG": "m",
    "altitude_payload": "kg",
    "cruise_speed": "m/s",
    "jet_consumption": "kg/(N·s)",
    "prop_consumption": "kg/J",
    "reserve_fuel": "N",
    "take_off_power": "W",
    "take_off_thrust": "N",
    "cruise_altitude": "km",
    "MTOM": "kg",
    "MTOW": "N",
    "OEW": "kg",
    "ZFW": "kg",
    "EW": "kg",
    "Fuel": "N",
    "Fuel_used": "N",
    "Fuel_reserve": "N",
    "S": "m²",
    "b": "m",
    "MAC": "m",
    "WP": "N/W",
    "WS": "N/m²",
    "fuel_economy": "L/ton/km",
    "P": "W",
    "T": "N",
    "stall_speed_clean": "m/s",
    "stall_speed_takeoff": "m/s",
    "stall_speed_landing": "m/s",
    "high_altitude": "m",
    "L": "m",
    "r": "m",
    "hull_surface": "m²",
    "gravitational_acceleration": "m/s²",
    "kinematic_viscosity": "m²/s",
    "rho_water": "kg/m³"
    }

    design_data = data.get('design', {})
    ferry_data = data.get('ferry', {})
    altitude_data = data.get('altitude', {})

    del data['design']
    del data['ferry']
    del data['altitude']

    # Check if the Excel file exists
    try:
        excelsheet = openpyxl.load_workbook(excel_file)
        # Remove the existing sheet if it exists
        if f"Design {data['design_id']} Data" in excelsheet.sheetnames:
            del excelsheet[f"Design {data['design_id']} Data"]
        sheet = excelsheet.create_sheet(title=f"Design {data['design_id']} Data")
    except FileNotFoundError:
        excelsheet = openpyxl.Workbook()
        sheet = excelsheet.active
        sheet.title = f"Design {data['design_id']} Data"

    sheet.cell(row=1, column=1, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=2, value="General Data")
    for i, (key, value) in enumerate(data.items()):
        if key in variable_units:
            sheet.cell(row=i+2, column=1, value=f"{key} ({variable_units[key]})")
        else:
            sheet.cell(row=i+2, column=1, value=key)
        sheet.cell(row=i+2, column=2, value=value)

    sheet.cell(row=1, column=4, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=5, value="Design Data")
    for i, (key, value) in enumerate(design_data.items()):
        if key in variable_units:
            sheet.cell(row=i+2, column=4, value=f"{key} ({variable_units[key]})")
        else:
            sheet.cell(row=i+2, column=4, value=key)
        sheet.cell(row=i+2, column=5, value=value)

    sheet.cell(row=1, column=7, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=8, value="Ferry Data")
    for i, (key, value) in enumerate(ferry_data.items()):
        if key in variable_units:
            sheet.cell(row=i+2, column=7, value=f"{key} ({variable_units[key]})")
        else:
            sheet.cell(row=i+2, column=7, value=key)
        sheet.cell(row=i+2, column=8, value=value)

    sheet.cell(row=1, column=10, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=11, value="Altitude Data")
    for i, (key, value) in enumerate(altitude_data.items()):
        if key in variable_units:
            sheet.cell(row=i+2, column=10, value=f"{key} ({variable_units[key]})")
        else:
            sheet.cell(row=i+2, column=10, value=key)
        sheet.cell(row=i+2, column=11, value=value)
    # Save the Excel file
    excelsheet.save(excel_file)

    

if __name__ == "__main__":
    json_file = "design2.json"  # Replace with your JSON file path
    excel_file = "Concept Data.xlsx"   # Replace with your desired Excel file path
    design_json_to_excel(json_file, excel_file)