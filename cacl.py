import matplotlib.pyplot as plt
import numpy as np

def calcular_incremento(pasos_actuales, inicio_pasos, final_pasos, incremento, incremento_maximo):
    resultado = pasos_actuales
    while inicio_pasos <= final_pasos:
        while resultado < incremento_maximo:
            resultado += incremento
            print(f"Resultado actual: {resultado}")
        
        resultado -= incremento
        print(f"Decrementando resultado a: {resultado}")
        inicio_pasos += 1
        

