def is_move_ext_pose(line):
    matches = ['X', 'Y', 'Z','E']


    for x in line:
        if(x in matches):
            print(x+" is in "+line)

    if any(x in line for x in matches):
        return True



    return False

print(is_move_ext_pose("G1 X91.357 Y89.779 E2.04999"))
print(is_move_ext_pose("G1 X91.357 Y89.779"))