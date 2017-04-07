from pgmConvert import pgm

f = open('out.txt', 'w')
var = pgm("roomba_hall.pgm")

for x in range(0, len(var.bytes)):
    f.write(str(var.bytes[x]) + " ")
    
print len(var.bytes)
print var.width
print "DONE"
