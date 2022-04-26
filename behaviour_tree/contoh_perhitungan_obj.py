from tkinter import Y


obj = {
    'obj_len': [4,2,3],
    'obj_x':[1,6,2,8,6,3,1,2,3],
    'xc':[4,5,6],
}

def obstacle():
    global obj
    
    obs = []
    a = 0
    for i in range (len(obj['obj_len'])):
        b = a + obj['obj_len'][i]
        obj_row = {
            'id': i,
            'obj_x': obj['obj_x'][a:b],
            'xc': obj['xc'][i],
        }
        obs.append(obj_row)
        a = b
        print(obj_row)
    return obs

col = [False, False, False, False, True, False, False]
cel = [False, False, False]
cil = []

def cek(coll):
    for i in range (len(coll)):
        if coll[i] == True:
            return True
        else:
            False
            
print('col = ',cek(col))
print('cel = ',cek(cel))
print('cil = ',cek(cil))

if cek(cel) == None:
    print("No possible path")