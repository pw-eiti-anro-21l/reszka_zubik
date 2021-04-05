import json
import yaml
import math

'''
zalozenia: 
ruchome: arm1, arm2, wrist
nieruchome: base, gripper
'''

def get_x_sign(item):
    alfa = item['alfa']
    if alfa ==0:
        #alfa = 0 brak obrotu
        return 1
    elif alfa <= 1.5708:
        #mniejsza lub równa 90 st
        return -1
    elif alfa <= 3.14:
        # alfa do 90 do 180
        return -1
    elif alfa <=4.7124:
        #alfa od 180 do 270
        return 1
    else:
        #alfa od 270 do 360
        return 1

def get_z_sign(item):
    alfa = item['alfa']
    
    if alfa == 0:
        #alfa = 0 brak obrotu
        return 1
    elif alfa <= 1.5708:
        #mniejsza lub równa 90 st
        return 1
    elif alfa <= 3.14:
        # alfa do 90 do 180
        return -1
    elif alfa <=4.7124:
        #alfa od 180 do 270
        return -1
    else:
        #alfa od 270 do 360
        return 1

# def rotate_coordinates(coor_list):
#     alfa = item['alfa']
#     if alfa == 0:
#         coor_list[0], coor_list[1] = coor_list[1], coor_list[0]
#     elif alfa == 1.5708:
#         coo

#     elif alfa ==3.14:

#     elif alfa == 4.7124:

#     else:


def get_geometric_center(item):
    #końcówka do zrobienia
    alfa = item['alfa']
    x_sign = get_x_sign(item)
    z_sign = get_z_sign(item)


    if len(item) == 6:
        #its a cylider
        return [0,item['length']/2]

    elif len(item) == 5 and alfa == 0:
        #coordinates of box in home coordinates swap z with z 
        measures = item['size']
        return [x_sign*measures['z']/2,z_sign*measures['x']/2 ]
    else:
        #both x and z axiz changes directions -> -
        measures = item['size']
        return [x_sign*measures['x']/2,z_sign*measures['z']/2 ]




def convert(jsonfile):

    data = read_dh(jsonfile)
  
    # czy dodawac fixed, prismatic, continuous?



    for item in data.values():
        a,d = get_geometric_center(item)
        
        item['a'] = a
        item['d'] = d

        
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



