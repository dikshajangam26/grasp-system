import pytest
import yaml
from pathlib import Path

def test_config_loads():
    """Test: Configuration file exists and contains expected root keys"""
    # Navigate up 4 levels from tests/ to reach the repo root: ~/grasp-system/
    repo_root = Path(__file__).resolve().parents[4]
    config_file = repo_root / 'config' / 'system_config.yaml'
    
    assert config_file.exists(), f"Config file not found at {config_file}"
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
        
    assert 'vision_servo' in config, "Missing 'vision_servo' configuration block"
    assert 'grasp_predictor' in config, "Missing 'grasp_predictor' configuration block"
    assert 'ik_solver' in config, "Missing 'ik_solver' configuration block"