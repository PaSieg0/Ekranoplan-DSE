from utils import Data
import openpyxl


def json_to_excel(json_file: str, excel_file: str) -> None:
    """
    Convert a JSON file containing design parameters to an Excel file. If the Excel file already exists, add a new sheet to it.
    
    Args:
        json_file (str): Path to the input JSON file.
        excel_file (str): Path to the output Excel file.
    """
    # Load the JSON data
    data = Data(json_file).data

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
        sheet.cell(row=i+2, column=1, value=key)
        sheet.cell(row=i+2, column=2, value=value)

    sheet.cell(row=1, column=4, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=5, value="Design Data")
    for i, (key, value) in enumerate(design_data.items()):
        sheet.cell(row=i+2, column=4, value=key)
        sheet.cell(row=i+2, column=5, value=value)

    sheet.cell(row=1, column=7, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=8, value="Ferry Data")
    for i, (key, value) in enumerate(ferry_data.items()):
        sheet.cell(row=i+2, column=7, value=key)
        sheet.cell(row=i+2, column=8, value=value)

    sheet.cell(row=1, column=10, value=f"Design {data['design_id']}")
    sheet.cell(row=1, column=11, value="Altitude Data")
    for i, (key, value) in enumerate(altitude_data.items()):
        sheet.cell(row=i+2, column=10, value=key)
        sheet.cell(row=i+2, column=11, value=value)
    
    # Save the Excel file
    excelsheet.save(excel_file)

if __name__ == "__main__":
    json_file = "design2.json"  # Replace with your JSON file path
    excel_file = "Concept Data.xlsx"   # Replace with your desired Excel file path
    json_to_excel(json_file, excel_file)