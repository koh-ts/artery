import sys

def getShape(i, num):
    row = int(i/(num - 1) )
    col = i%(num - 1)
    # print(row,",",col)
    # print (str(col * 100 + 5.00) + "," + str(row * 100 + 5.00) + " " + str((col + 1) * 100 - 5.00) + "," + str(row * 100 + 5.00) + " " +str((col + 1) * 100 - 5.00) + "," + str((row + 1) * 100 - 5.00) + " " + str(col * 100 + 5.00) + "," + str((row + 1) * 100 - 5.00))

    return (str(col * 100 + 5.00)
        + "," + str(row * 100 + 5.00)
        + " " + str((col + 1) * 100 - 5.00)
        + "," + str(row * 100 + 5.00)
        + " " +str((col + 1) * 100 - 5.00)
        + "," + str((row + 1) * 100 - 5.00)
        + " " + str(col * 100 + 5.00)
        + "," + str((row + 1) * 100 - 5.00))



args = sys.argv
num = int(args[1])

path = "grid" + str(num) + ".poly.xml"

with open(path, mode="w") as f:
    f.write("<additionals>")
    for i in range((num-1)*(num-1)):
        f.write("    <poly id=\"poly_"
        + str(i)
        + "\" type=\"building\" color=\"green\" fill=\"1\" layer=\"0.00\" shape=\""
        + getShape(i, num)
        + "\"/>\n")
    f.write("</additionals>")
