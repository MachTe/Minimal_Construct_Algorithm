import turtle

filename = "TestPolygons.txt"   #name of the mape which is to be displayed
f = open(filename, "r")
lineXmax = float('-inf')
lineYmax = float('-inf')
lineXmin = float('inf')
lineYmin = float('inf')

#finding out maximal and minimal values on both axis
while True:
    lineX = f.readline()
    if lineX == "":
        break
    if lineX == "MAIN\n" or lineX == "HOLE\n":
        pass
    elif lineX == "PATH\n":
        break
    else:
        lineX = int(lineX)
        if lineX > lineXmax:
            lineXmax = lineX
        if lineX < lineXmin:
            lineXmin = lineX
        lineY = int(f.readline())
        if lineY > lineYmax:
            lineYmax = lineY
        if lineY < lineYmin:
            lineYmin = lineY
            
f.close()

#values used to scale the picture to the widnow size
X = abs(lineXmax - lineXmin)
Y = abs(lineYmax - lineYmin)

if X > Y:
    modifier = 750 / X
else:
    modifier = 750 / Y

turtle.setup(750,750)
turtle.tracer(0, 0)
turtle.width(2)
turtle.colormode(255)
f = open(filename, "r")

#going from node to node and printing them out
count = 0
while True:
    count +=1
    lineX = f.readline()
    if lineX == "":
        break
    if lineX == "MAIN\n" or lineX == "HOLE\n":
        turtle.penup()
        lineX = int(f.readline())
        lineY = int(f.readline())
        turtle.goto((int((lineX - lineXmin)*modifier)-375, -int((lineY-lineYmin)*modifier)+375))
        turtle.pendown()
    elif lineX == "PATH\n":
        turtle.penup()
        turtle.color(40,220, 5)
        turtle.width(4)
        lineX = int(f.readline())
        lineY = int(f.readline())
        turtle.goto((int((lineX - lineXmin)*modifier)-375, -int((lineY-lineYmin)*modifier)+375))
        turtle.pendown()
    else:
        lineX = int(lineX)
        lineY = int(f.readline())
        turtle.goto((int((lineX - lineXmin)*modifier)-375, -int((lineY-lineYmin)*modifier)+375))
        
print(str(count) + " vertices in the map")
        
turtle.update()
turtle.hideturtle()
turtle.exitonclick()
