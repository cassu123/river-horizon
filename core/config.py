# River Horizon - core/config.py

import json
import os
from pathlib import Path

class Config:
    def __init__(self, profile_path="/home/hoke/river-horizon/units/drone_profile.json"):
        self.profile_path = profile_path
        self.data = self._load_profile()

    def _load_profile(self):
        if not os.path.exists(self.profile_path):
            return {}
        with open(self.profile_path, 'r') as f:
            return json.load(f)

    @property
    def drone_id(self):
        return self.data.get("drone_id", "UNKNOWN")

    @property
    def flight_params(self):
        return self.data.get("flight_params", {})

    @property
    def safety_limits(self):
        return self.data.get("safety_limits", {})

    @property
    def connectivity(self):
        return self.data.get("connectivity", {})

    def get(self, key, default=None):
        return self.data.get(key, default)

# Global config instance
config = Config()
