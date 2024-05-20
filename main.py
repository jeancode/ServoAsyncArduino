import serial
import time

# Abre la conexión al puerto serie COM10 a 9600 baudios (ajusta el baud rate si es diferente)
ser = serial.Serial('COM10', 115200)

def recore(ser, serv, minimum, maximum, speed):
    if minimum <= maximum:
        for i in range(minimum, maximum):
            print(i)
            command = f"{serv},{i}\n"
            ser.write(command.encode())
            time.sleep(speed)
    else:
        for i in range(minimum, maximum - 1, -1):
            print(i)
            command = f"{serv},{i}\n"
            ser.write(command.encode())
            time.sleep(speed)
 

inicial = [
    [15,50],
    [14,1],

    [13,50],
    [12,50],
    [11,100],

    [10,60],


]

p1 = [
 
    
    [15,1],
    [14,1],

    [13,60],
    [12,80],
    [11,100],

    [10,80],
]


p2 = [
 
    [15,50],
    [14,130],
    [13,80],
    [12,80],
    [11,100],
    [10,80],
]

p3 = [
 
    [15,50],
    [14,1],
    [13,80],
    [12,80],
    [11,100],
    [10,80],
]

p4 = [
 
    [15,50],
    [14,1],

    [13,50],
    [12,30],
    [11,120],

    [10,80],
]





comandos = [

    inicial,
    p1,
    p2,
    p3,
    p4

]


indice = 1


def send(c):

    for i in range(0,len(comandos[c])):

               
        serv = comandos[c][i][0]
        angle = comandos[c][i][1]

        command = f"{serv},{angle}\n"

        print(command)
        ser.write(command.encode())
        time.sleep(0.001)


def start():

    while True:
        send(0)
        time.sleep(1)
        send(1)
        time.sleep(1)
        send(2)
        time.sleep(1)
        send(3)
        time.sleep(2)


while True:

    i = input("comand:")
    #send(int(i))

    if(i == "start"):
        start()
    




# Cierra la conexión después de usarla
ser.close()