import png

def displaypgm(bytes, width, name):
    a = []
    w = int(width)
    for i in range(len(bytes)/w):
        row = [bytes[x + (i * w)] for x in range(w)]
        a.append(row)
    png.from_array(a, "L").save(name)
