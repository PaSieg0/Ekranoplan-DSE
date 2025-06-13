import os
import numpy as np
import matplotlib.pyplot as plt

def load_airfoil(filepath):
    """Loads x, y coordinates from a file assuming whitespace separation."""
    try:
        data = np.loadtxt(filepath)
        x, y = data[:, 0], data[:, 1]
        return x, y
    except Exception as e:
        print(f"Failed to load {filepath}: {e}")
        return None, None

def plot_all_airfoils(folder_path):
    files = [f for f in os.listdir(folder_path) if f.endswith(('.dat', '.txt'))]
    files.sort()  # Optional: for consistent ordering

    num_files = len(files)
    cols = 3
    rows = int(np.ceil(num_files / cols))

    fig, axs = plt.subplots(rows, cols, figsize=(12, 2 * rows))
    axs = axs.flatten()

    for i, filename in enumerate(files):
        filepath = os.path.join(folder_path, filename)
        x, y = load_airfoil(filepath)

        if x is not None and y is not None:
            axs[i].plot(x, y, linewidth=1.5)
            axs[i].set_xlim(0, 1)
            axs[i].set_ylim(-0.1, 0.2)
            axs[i].set_aspect('auto')
            axs[i].grid(True)
            axs[i].set_title(os.path.splitext(filename)[0])
        else:
            axs[i].text(0.5, 0.5, "Failed to load", ha="center", va="center")
            axs[i].set_title("Error")

    # Remove unused subplots (if any)
    for j in range(num_files, len(axs)):
        fig.delaxes(axs[j])

    fig.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_all_airfoils(r"C:\Users\Martin\Downloads\DSE\DSE_code\Ekranoplan-DSE\aero\airfoils_points")
