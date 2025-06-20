import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Class_I.__main__ import main
from utils import Data, ISA, MissionType, plt

if __name__ == "__main__":
    main(create_excel=False)