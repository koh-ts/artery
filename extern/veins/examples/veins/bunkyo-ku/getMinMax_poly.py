import xml.etree.ElementTree as ET

from_path = "./bunkyo-ku.poly1.xml"
max_num = 10
removed_count = 0


tree = ET.parse(from_path)
root = tree.getroot()

minX = 1000000000000
minY = 1000000000000
maxX = -1000000000000
maxY = -1000000000000

for poly in root.findall("poly"):
    shape = poly.attrib["shape"].split(" ")
    for xy in shape:
        x = float(xy.split(",")[0])
        y = float(xy.split(",")[1])
        if minX > x:
            minX = x
        if minY > y:
            minY = y
        if maxX < x:
            maxX = x
        if maxY < y:
            maxY = y
print("(minX minY maxX maxY):", minX, minY, maxX, maxY)
