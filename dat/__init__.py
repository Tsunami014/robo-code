import json, os

def get_positions():
    with open('dat/positions.json') as f:
        d = json.load(f)
    return d

def path_exists():
    return os.path.exists('dat/path.json')

def load_path():
    with open('dat/path.json') as f:
        d = json.load(f)
    return d

def save_path(path):
    with open('dat/path.json', 'w+') as f:
        json.dump(path, f)
