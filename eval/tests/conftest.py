# eval/ is a plain directory, not a package, and its name collides with the builtin
# `eval` — so put it on sys.path and import the modules by bare name.
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
