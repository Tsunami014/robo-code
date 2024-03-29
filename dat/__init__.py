import json

def get_positions():
    with open('dat/positions.json') as f:
        d = json.load(f)
    return d
