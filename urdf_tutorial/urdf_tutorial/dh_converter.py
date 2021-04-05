import json
import yaml

'''
zalozenia: 
ruchome: arm1, arm2, wrist
nieruchome: base, gripper
'''

def convert(jsonfile):

    data = read_dh(jsonfile)
  
    # czy dodawac fixed, prismatic, continuous?

    for item in data.values():
        item['rpy'] = {}
        item['xyz'] = {}
        item['rpy']['r'] = 0
        item['rpy']['p'] = item['alfa']
        item['rpy']['y'] = 0

        item['xyz']['x'] = item['a']
        item['xyz']['y'] = 0
        item['xyz']['z'] = item['d']

        item.pop('alfa')
        item.pop('theta')
        item.pop('a')
        item.pop('d')
    
    return data


def read_dh(jsonfile):
    with open(jsonfile, "r") as file:
        data = json.load(file)    
    return data

def write_yaml(data, filename):
    with open(filename, "w") as yamlfile:
        yaml.dump(data, yamlfile)


if __name__ == "__main__":
    data = convert("dh.json")
    write_yaml(data, "dh.yaml")




