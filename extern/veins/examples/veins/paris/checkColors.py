import xml.etree.ElementTree as ET

f = open("paris.poly1.xml","r")
data  = f.read()
root = ET.fromstring(data)
types = []
for child in root:
    if "shape" not in child.attrib.keys():
        continue
    # v = child.attrib["color"].split(",")
    if len(child.attrib["shape"].split(" ")) > 255:
        print(child.attrib["id"])