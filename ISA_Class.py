import numpy as np

class ISA:
    """
    A class representing the International Standard Atmosphere (ISA) model.

    This class calculates atmospheric properties (pressure, temperature, 
    density, and speed of sound) based on altitude using the standard ISA model.

    Attributes:
        altitude (float): Altitude in meters.
        pressure (float): Air pressure at the given altitude in Pascals.
        temperature (float): Air temperature at the given altitude in Kelvin.
        rho (float): Air density at the given altitude in kg/m^3.
        speed_of_sound (float): Speed of sound at the given altitude in m/s.
    
    Methods:
        Mach(IAS): Computes the Mach number for a given Indicated Airspeed (IAS).
        TAS(IAS): Computes the True Airspeed (TAS) for a given IAS.
    """
    def __init__(self, altitude: float=0) -> None:
        """
        Initialize the ISA model for a given altitude.

        Args:
            altitude (float): Altitude in meters.
        """
        self.lamb = -0.0065  # Temperature lapse rate in K/m
        self.Temp0 = 288.15  # Sea level standard temperature in K
        self.p0 = 101325  # Sea level standard pressure in Pa
        self.rho0 = 1.225  # Sea level standard density in kg/m^3
        self.gamma = 1.4  # Ratio of specific heats for air
        self.R = 287.05  # Specific gas constant for dry air in J/(kg·K)
        self.g = 9.80665  # Acceleration due to gravity in m/s^2
        
        self.altitude = altitude


    @property
    def altitude(self) -> float:
        return self._altitude

    @altitude.setter
    def altitude(self, value: float) -> None:
        if value < 0:
            raise ValueError("Altitude cannot be negative")
        self._altitude = value
        self._pressure = self._compute_pressure()
        self._temperature = self._compute_temperature()
        self._rho = self._compute_rho()
        self._speed_of_sound = self._compute_speed_of_sound()

    def _compute_pressure(self) -> float:
        power = -self.g / (self.lamb * self.R)
        return self.p0 * (1 + self.lamb * self.altitude / self.Temp0) ** power

    def _compute_temperature(self) -> float:
        return self.Temp0 + self.lamb * self.altitude
    
    def _compute_rho(self) -> float:
        return self._pressure / (self.R * self._temperature)
    
    def _compute_speed_of_sound(self) -> float:
        return np.sqrt(self.gamma * self.R * self._temperature)
    
    @property
    def pressure(self) -> float:
        return self._pressure

    @property
    def temperature(self) -> float:
        return self._temperature
    
    @property
    def rho(self) -> float:
        return self._rho
    
    @property
    def speed_of_sound(self) -> float:
        return self._speed_of_sound
    
    def Mach(self, IAS: float) -> float:
        """
        Calculate the Mach number given Indicated Airspeed (IAS).
        ---------------------------------------------------------
        Parameters:
            IAS : float
                Indicated airspeed in m/s.

        Returns:
            float: Mach number.
        """
        curly_braces = (1 + ((self.gamma-1)/(2*self.gamma)) * self.rho0/self.p0 * IAS ** 2) ** (self.gamma/(self.gamma-1)) - 1 
        square_braces = (1 + self.p0/self.pressure * curly_braces) ** ((self.gamma-1)/self.gamma) - 1
        M = np.sqrt(2/(self.gamma-1) * square_braces)
        return M

    def TAS(self, IAS: float) -> float:
        """
        Calculate the true airspeed (TAS) given Indicated Airspeed (IAS).
        ------------------------------------------------------------------
        Parameters:
            IAS (float): Indicated airspeed in m/s.

        Returns:
            float: True airspeed in m/s.
        """
        M = self.Mach(IAS)
        return M * self.speed_of_sound
    
    def TAT_to_SAT(self, IAS: float, measured_TAT: float) -> float:
        """
        Calculate the static air temperature (SAT) given Indicated Airspeed (IAS).
        ---------------------------------------------------------------------------
        Parameters:
            IAS (float): Indicated airspeed in m/s.
            measured_TAT (float): Total air temperature in Kelvin (measured).

        Returns:
            float: Static air temperature in Kelvin.
        """
        M = self.Mach(IAS)
        return measured_TAT / (1 + (self.gamma-1)/2 * M ** 2)

    def EAS(self, TAS: float) -> float:
        """
        Calculate the equivalent airspeed (EAS) given True Airspeed (TAS).
        -------------------------------------------------------------------
        Parameters:
            TAS (float): True airspeed in m/s.
            rho (float): Air density at the given altitude in kg/m^3.

        Returns:
            float: Equivalent airspeed in m/s.
        """
        return TAS * np.sqrt(self.rho / self.rho0)


if __name__ == "__main__":
    isa = ISA(altitude=10048)
    print(f"Pressure: {isa.pressure} Pa")
    print(f"Temperature: {isa.temperature} K")
    print(f"Density: {isa.rho} kg/m^3")
    print(f"Speed of Sound: {isa.speed_of_sound} m/s")
