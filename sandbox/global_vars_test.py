def myFunction():
    global a
    a = 20
    # globals()["a"] = 20

a = 10
myFunction()
print(a)


