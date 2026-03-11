import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import kinematic_bicycle

kinematic_bicycle.some_fu(1.0, 2.0)
print(kinematic_bicycle.k)
