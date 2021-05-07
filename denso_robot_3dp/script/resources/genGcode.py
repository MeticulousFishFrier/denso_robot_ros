import numpy as np


z_pos = np.linspace(.4,.2,100)*1000
z_pos = np.round(z_pos, decimals=3)
print(z_pos[2])
#translate from meters to mm
f = open("example.gcode", "w")


for z in z_pos:
    f.write("G0 Z")
    f.write(str(z))
    f.write("\n")
    
f.close()