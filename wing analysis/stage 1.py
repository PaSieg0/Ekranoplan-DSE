"""
    - A Vortex Lattice Method (VLM) is employed to calculate the spanwise lift distribution of the wing for a specified aoa 
    or lift coefficient (C_L).
    - The VLM code also computes the induced drag coefficient (C_Di) using the Trefftz plane analysis, 
    which is a technique for analyzing the induced drag of a lifting surface.
"""
# Done with XFLR

import subprocess

def run_javafoil(airfoil_name, alpha, re, mach):
    jfm_script = f"""
READ {airfoil_name}.dat
OPER
RE {re}
MACH {mach}
ITER 250
ALFA {alpha}
CPWR {airfoil_name}_cp.txt
PWRT {airfoil_name}_polar.txt
QUIT
"""
    with open("input.jfm", "w") as f:
        f.write(jfm_script.strip())

    javafoil_path = r"C:\Program Files (x86)\MH AeroTools\JavaFoil\java\javafoil.jar"

    result = subprocess.run(
        ["java", "-jar", javafoil_path, "-b", "input.jfm"],
        capture_output=True,
        text=True
    )

    # DEBUG: Print JavaFoil output
    print("------ JAVAFOIL STDOUT ------")
    print(result.stdout)
    print("------ JAVAFOIL STDERR ------")
    print(result.stderr)

    if result.returncode != 0:
        print("JavaFoil failed.")
        return None

    try:
        with open(f"{airfoil_name}_polar.txt", "r") as f:
            data = f.read()
            print("Polar output:\n", data)
            return data
    except FileNotFoundError:
        print("Output file not found.")
        return None

# Example use
run_javafoil("glenn_21", alpha=5, re=1e6, mach=0.1)

