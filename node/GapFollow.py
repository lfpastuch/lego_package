#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 11 17:39:38 2020

@author: yohas
"""
#Importando as bibliotécas e as mensagens que serão usadas
import rospy
import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
#
gapArray = [0] #definindo a variavel global que será compartilhada, assim
bestGapTu = ()


def corrigeGap(lista, valorMin, x = 6):
    """
    Entradas são a lista de valores lidos pelo lidar o valorMinimo que é um float, e x que é um inteiro.
    Retorna a lista transformando todos os valores menores do que o valor minimo, e x de seus adjacentes em 0
    Ordem de grandeza máximo (2x+1)n+n -> O(n)
    Funciionamento: Varre a lista guardando o index dos valoress menores ou iguais ao valorMin em uma lista. Depois,
    usando essa lista de indexamentos, a função modifica o valor indexado e x valores, para mais e menos, para zero.
    """
    newlista= []
    for c in range(len(lista)):
        if lista[c] <= valorMin:
            newlista.append(c)
    for n in newlista:
        for i in range(2*x+1):
            a=(n + x - i)
            if   a >= 0 and a < len(lista):
                lista[a] = 0

    #print str(lista) #print para testes
    return lista

def callback(msg):
    '''
    Função que é ativada a cada leitura do scan e atualiza a "gap array"
    aumentando os obstáculos proximos para auxiliar a escolha da melhor direção.
    '''

    global gapArray
    #define-se o arco de visão onde estamos procurando os obstáculos, e a menor distância entre o carro e os obstáculos.
    rangeMax=int(270*1080/360)
    rangeMin=int(90*1080/360)

    #transformar em lista a tupple de valores de msgs.ranges adquiridas do Lidar
#    t=len(msg.ranges)
#    print "\n"
#    inicio = time.time()
#    tempList=[msg.ranges[i] for i in range(t) if (i>rangeMin and i<rangeMax)]
#    fim = time.time()
#    print "for "+str(fim - inicio)

    #transformar em lista a tupple de valores de msgs.ranges adquiridas do Lidar
    tempList=list(msg.ranges[rangeMin:rangeMax])


    #atualiza o gapArray com a leitura atual
    gapArray=tempList[:]

def melhorGap(listaGaps):
    """
    Entrada é uma lista de valores de lacunas e zeros.
    Retornar um tuple com o primeiro valor sendo o index central do melhor lacuna de não zeros dessa lista,
    o segundo valor sendo o numero de valores tem essa lacuna, o terceiro o valor da média desses numeros, e o
    quarto o numero de valores que tem a lista.
    Ordem de grandeza -> O(n²)
    """

   # print str(listaGaps)
    melhorTupla=(0, 0, 0, 0)
    tamLista=len(listaGaps)
    for v in range(len(listaGaps)):
        listaNova=[]
        med = 0
        centro=0
        tamLac=0
        if listaGaps[v] > 0:
            for i in range(len(listaGaps)-v):
                if listaGaps[v+i]>0:
                    listaNova.append(listaGaps[v+i])
                else:
                    break
            tamLac=len(listaNova)
            med=sum(listaNova)/len(listaNova)
            centro=v+int(len(listaNova)/2)
            if tamLac>melhorTupla[1] or (tamLac==melhorTupla[1] and med>melhorTupla[2]):
                melhorTupla=(centro, tamLac, med, tamLista)

    #print str(melhorTupla)     #usado para teste
    return melhorTupla


class FollowTheGap(object):

    direc_msg = AckermannDriveStamped()

    def __init__(self):

        rospy.init_node('follow_the_gap_node')

        self.sub = rospy.Subscriber('/scan', LaserScan, callback)



        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            global gapArray
            #print gapArray
            self.GapFollow(gapArray)
            rate.sleep()


    def GapFollow(self, listaScan):

        minDist = 4.0
        #função para retornar a lista de leituras com as lacunas sendo valores diferentes de zero
        GPArray=corrigeGap(listaScan, minDist)
        #rospy.loginfo(str(dist)) #usado para teste
        direc_msg = AckermannDriveStamped()
        direc_msg.drive.steering_angle = 0


        direc_msg.drive.speed = 0.5

        global gap_pub

        gapTuple=melhorGap(GPArray)
        if not gapTuple == (0,0,0,0):
            direc_msg.drive.steering_angle = ((gapTuple[0]-int(gapTuple[3]/2))*0.4/(gapTuple[3]/2))

            if GPArray[int(gapTuple[3]/2)] > 5:
                direc_msg.drive.speed = 2
            else:
                direc_msg.drive.speed = 1
            gap_pub.publish(direc_msg)


#    def shutdown(self):
#        global direc_msg
#        rospy.loginfo("Parada")
#
#        shut_msg = AckermannDriveStamped()
#        shut_msg.drive.speed = 0
#        shut_msg.drive.steering_angle = 0
#        gap_pub.publish(shut_msg)
#
#        rospy.sleep(1)
def main():

    fg = FollowTheGap()
    rospy.spin()

# O drive_pub foi declarado global para que ele possa ser usado tando no construtor quanto na funcao. callback
gap_pub = rospy.Publisher('/follow_the_gap', AckermannDriveStamped, queue_size=10)

main()
