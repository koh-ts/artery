import subprocess
import xml.etree.ElementTree as ET
from xml.dom import minidom
import random


map_name = "grid"
from_path = "./" + map_name + "_un_fixed.rou.xml"
to_path = "./" + map_name + ".rou.xml"

route_num = 9
car_pop_interval = 0.1
sim_time = 600
min_distance = 200

try:
    print("$SUMO_HOME/tools/randomTrips.py --validate --min-distance=" + str(min_distance) + " --flows=" + str(route_num) + " -n " + map_name + ".net.xml --route-file=" + map_name + "_un_fixed.rou.xml")
    res = subprocess.getoutput("$SUMO_HOME/tools/randomTrips.py --validate --min-distance=" + str(min_distance) + " -e 100 -n " + map_name + ".net.xml --flow=" + str(route_num) + " --route-file=" + map_name + "_un_fixed.rou.xml")
    print(res)
except:
    print("Error.")

tree = ET.parse(from_path)
root = tree.getroot()

routes = ET.Element("routes")
tree2 = ET.ElementTree(element=routes)
vType = ET.SubElement(routes, "vType")
vType.attrib = { "id": "vtype0", "accel": "2.6", "decel": "4.5", "sigma": "0.5", "length": "2.5", "minGap": "2.5", "maxSpeed": "14", "color": "1,1,0" }


for i,vehicle in enumerate(root.findall("vehicle")):
    if float(vehicle.attrib["depart"]) != 0:
        continue
    for routee in vehicle.findall("route"):
        routee.attrib["id"] = "route" + str(i)
        route = ET.SubElement(routes, "route")
        route.attrib = routee.attrib
        flow = ET.SubElement(routes, "flow")
        flow.attrib = {"id": "flow" + str(i), "type": "vtype0", "route": "route" + str(i), "begin": "0",
        # str(random.random()/2.0),
        "period": str(car_pop_interval), "number": str(round(sim_time/car_pop_interval)) }

with open(to_path, mode="w") as f:
    f.write(minidom.parseString(ET.tostring(routes)).toprettyxml())
