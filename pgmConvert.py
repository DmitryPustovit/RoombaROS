class pgm:
    def __init__(self, file):
        f = open(file, "rb") #Opens the file and reads the bytes
        #We don't care about the two lines of the header
        f.readline() #The format in our case P5, which means this is encoded in bytes
        f.readline() #The creator of the line
        self.width, self.height = f.readline().split(' ') #Width and Height seperated by a space
        l = f.readline() #The values per byte (0-255)
        self.bytes = bytearray(f.readline()) #Read the lines in and convert to a byte array