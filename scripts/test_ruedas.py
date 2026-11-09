import serial
import time

s = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)

def mandar(l, r, segundos, etiqueta):
    print(etiqueta)
    cmd = f'{l},{r}\n'.encode()
    t0 = time.time()
    while time.time() - t0 < segundos:
        s.write(cmd)
        time.sleep(0.15)

mandar(200, 200, 4.0, '1) AMBOS lados: revisa que giren las 4 llantas, a la misma velocidad')
mandar(0, 0, 1.5, '   pausa')
mandar(200, 0, 3.0, '2) Solo IZQUIERDO: deben girar sus 2 llantas')
mandar(0, 0, 1.5, '   pausa')
mandar(0, 200, 3.0, '3) Solo DERECHO: deben girar sus 2 llantas')

s.write(b'0,0\n')
s.close()
