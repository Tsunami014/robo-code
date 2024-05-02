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
    for elm in d:
        if elm[1] == "":
            elm[1] = None
    return d

def save_path(path):
    d = path.copy()
    for elm in d:
        if elm[1] is None:
            elm[1] = ""
    with open('dat/path.json', 'w+') as f:
        json.dump(d, f)
    for elm in d: # We must convert back because otherwise python will kill us, even though we're making a copy of the list (???)
                  # TODO: Fix this
        if elm[1] == "":
            elm[1] = None
