import os
import matplotlib.pyplot as plt

def cl_distribution(filename, plot=False):
    """
    Extracts the y-span and Cl values from an airfoil data file located in the AirfoilData folder.
    
    Parameters:
        filename (str): Name of the file.
        plot (bool): If True, plot the extracted data.
        
    Returns:
        tuple: Two lists (yspan_values, cl_values) containing floats.
    """
    # Construct path relative to this script's location
    folder_path = os.path.join(os.path.dirname(__file__), "AirfoilData")
    file_path = os.path.join(folder_path, filename)

    yspan_values = []
    cl_values = []
    chord_values = []
    header_found = False

    with open(file_path, 'r') as f:
        for line in f:
            stripped = line.strip()
            # Look for table header by checking for both 'y-span' and 'Cl'
            if not header_found and "y-span" in stripped and "Cl" in stripped and "Chord" in stripped:
                header_found = True
                continue  # skip the header line
            if header_found:
                # Stop reading if we hit an empty line
                if not stripped:
                    break
                # Split by whitespace. This assumes the columns are space-separated.
                parts = stripped.split()
                if len(parts) < 4:
                    continue
                try:
                    y_span = float(parts[0])
                    cl = float(parts[3])
                    chord = float(parts[1])
                    yspan_values.append(y_span)
                    cl_values.append(cl)
                    chord_values.append(chord)
                except ValueError:
                    # In case conversion fails, skip the row
                    continue

    if plot:
        plt.figure()
        plt.plot(yspan_values, cl_values, label=filename)
        plt.xlabel("y-span")
        plt.ylabel("Cl")
        plt.title("Cl Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.legend()
        plt.show()

    return yspan_values, cl_values, chord_values
def lift_distribution(yspan_values, cl_values, chord_values, rho, V, plot=False):
    """
    Calculates the lift distribution based on y-span and Cl values.
    
    Parameters:
        yspan_values (list): List of y-span values.
        cl_values (list): List of Cl values.
        
    Returns:
        list: Lift distribution values.
    """
    lift_values = []
    for cl, c in zip(cl_values, chord_values):
        if cl < 0 or c < 0:
            raise ValueError("Cl values must be non-negative.")
        else:
            l = cl * (0.5*rho*V**2*c)
            lift_values.append(l)

    if plot:
        plt.figure()
        plt.plot(yspan_values,lift_values, label=filename)
        plt.xlabel("y-span")
        plt.ylabel("Lift")
        plt.title("Lift Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.legend()
        plt.show()

    return lift_values



if __name__ == "__main__":
    filename = "MainWing_a=5.00_v=10.00ms non wig (1).txt"
    yspan, cl, chord = cl_distribution(filename, plot=False)
    lift_values = lift_distribution(yspan, cl, chord, rho=1.225, V=10.0, plot=True)
    print("y-span values:")
    print(yspan)
    print("\nCl values:")
    print(cl)