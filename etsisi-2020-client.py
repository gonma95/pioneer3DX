#!/usr/bin/python3

print('### Script:', __file__)

import math
import sys
import time

import sim

# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Robot handle
    _,rbh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx',
                                     sim.simx_opmode_blocking)

    # Motor handles
    _,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                     sim.simx_opmode_blocking)
    _,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                     sim.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = sim.simxGetObjectHandle(clientID, str % (i+1),
                                       sim.simx_opmode_blocking)
        sonar[i] = h
        sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

    return [lmh, rmh], sonar, rbh

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
        if e == sim.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def avoid(sonar,flag):
    '''if paredDelante(sonar):
       if paredDelanteIzquierda(sonar):
          lspeed,rspeed = 0.6,-0.6
       elif paredDelanteDerecha(sonar):
          lspeed,rspeed = -0.6,0.6
       elif sonar[0] + sonar[15] < sonar[7] + sonar[8]:
          lspeed, rspeed, = 0.7, -0.7
       else: 
          lspeed,rspeed = -0.7, 0.7
    elif paredDelanteDerecha(sonar):
       lspeed,rspeed = -0.5,0.5
    elif paredDelanteIzquierda(sonar):
       lspeed,rspeed = 0.5,-0.5
    else:
       lspeed, rspeed = +1.0, +1.0
    return lspeed, rspeed'''
    if ((sonar[0] + sonar[7] < 0.8) and (sonar[1] + sonar[6] < 0.8) and (sonar[2] + sonar[5] < 0.8) and (sonar[3] + sonar[4] < 0.8)):
      lspeed, rspeed = 7, -7
    elif (sonar[0] > 0.8 and flag == 0):
       lspeed, rspeed = 1.55, 3.25
       flag =1
    else:
      if paredDelante(sonar):
        if paredDelanteIzquierda(sonar):
           lspeed,rspeed = 0.6,-0.6
        elif paredDelanteDerecha(sonar):
           lspeed,rspeed = -0.6,0.6
        else: 
           lspeed,rspeed = 0.7, -0.7
      elif paredDelanteDerecha(sonar):
        lspeed,rspeed = -0.5,0.5
      elif paredDelanteIzquierda(sonar):
        lspeed,rspeed = 0.4,-0.4
      else:
       lspeed, rspeed = +3.0, +3.0
    return lspeed, rspeed, flag
    
    '''if(centrar(sonar)):
       if(giroderecha(sonar)):
         lspeed, rspeed = 3.21, -3.21
       else:
         lspeed, rspeed = 0, 0
    else:
       lspeed, rspeed = 2, 2
    return lspeed, rspeed'''
# --------------------------------------------------------------------------
# FUNCIONES AUXILIARES
#---------------------------------------------------------------------------
def estaParaleloDerecha (sonar):
    if(sonar[7] == sonar[8]):
       return True
    else:
       return False
      
def estaParaleloIzquierda (sonar):
    if(sonar[0] == sonar[15]):
       return True
    else:
       return False

def paredDelante (sonar):
    if(sonar[3] < 0.4) or (sonar[4] < 0.4):
       return True
    else:
       return False
       
def paredDelanteDerecha (sonar):
    if(sonar[5] < sonar[4]) or (sonar[6] < sonar[4]):
       if(sonar[5] < 0.15) or (sonar[6] < 0.15):
          return True
    else:
       return False
       
def paredDelanteIzquierda (sonar):
    if(sonar[1] < sonar[3]) or (sonar[2] < sonar[3]):
       if(sonar[1] < 0.2) or (sonar[2] < 0.2):
          return True
    else:
       return False
#-----------------------------------------------------------------------------
def meta (sonar, contadormeta):
    if(sonar[7] > 0.8 and sonar [8] > 0.8):
       contadormeta = contadormeta + 1
    else:
       contadormeta = 0
    return contadormeta
 
def centrar (sonar):
   if(sonar[3] < 0.37) or (sonar[4] < 0.37):
      return True
   else:
      return False
      
def giroderecha (sonar):
   if (sonar [7] >0.5 and sonar [8] > 0.5):
      return True
   else:
      return False
#---------------------------------------------------------------------------
def main():
    contadormeta = 0
    flag = 0
    otroflag = 0
    otrocontador = 0
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)

        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            # print '### s', sonar

            # Planning
            lspeed, rspeed, flag = avoid(sonar, flag)
            # Action
            #print(contadormeta)
            contadormeta = meta(sonar, contadormeta)
            if (contadormeta == 62):
               while(1):
                  setSpeed(clientID, hRobot, -10, 10)
            setSpeed(clientID, hRobot, lspeed, rspeed)
            if(flag==1 ):
              time.sleep(0.90)
              flag = 0
            time.sleep(0.1)
        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
