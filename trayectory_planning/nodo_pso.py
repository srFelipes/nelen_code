#!/usr/bin/env python
import time
# Se importan paquetes importantes
import math
import rospy
import numpy as np
from tqdm import tqdm
import copy
import matplotlib.pyplot as plt

import s_curve_trayectory as planning
# import Float64 msg
from std_msgs.msg import Float64
# import control msg
from control_msgs.msg import JointControllerState

#necesario para llamar a los servicions de pausa despausa y reinicializacion
from std_srvs.srv import Empty

#paquete para cambiar los parametros del PID
import dynamic_reconfigure.client

#se crean funciones para operar gazebo
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
pause_simulation = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

#variables universales para guardar los datos de la realziacion
data_codo     =[]
tiempo_codo   =[]
data_hombro   =[]
tiempo_hombro =[]
accion_codo   =[]
accion_hombro =[]

def cubic_ref(not_t):
    #referencia cubica
    t_init=0.5
    t=not_t+t_init
    tf=0.666
    a0=-3/2.0*0.9420
    a_end=3.0/2.0*0.9420

    if t<=tf:
        a2=3.0*(a_end-a0)/(tf**2)
        a3=-2.0*(a_end-a0)/(tf**3)
        return a0+a2*(t**2)+a3*(t**3)
    elif t<0.0: return a0
    else: return a_end

def escalon(t):
    t_onset = 0.5
    if t<t_onset:
        return 0.
    else:
        return 1.


#Generar trayectoria de posicionamiento
back=planning.two_joints(-3.14/2,-14.0/18.0*3.14,S=50.,frec=100.)
#generar trayectoria rango completo
full=planning.two_joints(3.14*0.94,30.0/18.0*3.14*0.94,S=50.,frec=100.)
keep=np.linspace(0.,1.,1000)
hombro_dres=[0.0, 2.059488517353309e-05, 0.000165282680163863, 0.00055710909723659, 0.0013187707828069154, 0.0025712190540380466, 0.004433310832990797, 0.007021808646623586, 0.010450507962166447, 0.014829888121270618, 0.020266588741232956, 0.026863235182070726, 0.034717565881895604, 0.043922781422764096, 0.05456632280020112, 0.0667303950219756, 0.0804909199105493, 0.09591736157015157, 0.11307307545262982, 0.13201443569282348, 0.1527903115097884, 0.175442241739722, 0.20000600563229018, 0.22650883032382407, 0.2549681691068436, 0.28539815914829, 0.3178000221786395, 0.3521725364674158, 0.3885015648476778, 0.4267696540269055, 0.4669488787370669, 0.5090060777223743, 0.5528958724222757, 0.5985728842762192, 0.6459777720896372, 0.6950494493387098, 0.7457163575241051, 0.7979051928172396, 0.8515321794140175, 0.9065083055225868, 0.962744559351096, 1.0201397118029296, 1.0785925337814717, 1.1379948148730985, 1.1982383446641869, 1.259207931424105, 1.32078768529052, 1.3828575276108952, 1.4452984269302442, 1.5079854176741245, 1.5707963267948966, 1.6336072359156684, 1.6962942266595487, 1.758733380649646, 1.820804968299273, 1.8823899581534442, 1.9433543089256062, 2.0036030747044506, 2.062996629149817, 2.1214477057991075, 2.1788515848972008, 2.2350860933964585, 2.29006396483428, 2.3436804794555455, 2.395883277382696, 2.4465501855680913, 2.4956152305660066, 2.5430202929123493, 2.5886897998505094, 2.632584830538167, 2.6746472655112306, 2.7148247448921397, 2.7530823620958556, 2.7894201171223774, 2.8237856500941456, 2.85619641430368, 2.8866175031659416, 2.915083823265969, 2.941577921311243, 2.966152157179323, 2.9888065308702094, 3.0095759489689424, 3.02851277135308, 3.045669357900185, 3.061098068487815, 3.07486871628605, 3.0870336611724505, 3.097662716317096, 3.106878054767626, 3.1147320364016005, 3.1213293809741387, 3.1267573549478413, 3.1311421456276265, 3.134576429996776, 3.1371595172897275, 3.1390270195893617, 3.1402662033582778, 3.1410341482291546, 3.141427545442555, 3.141571884171694, 3.141592653589793]
codo_dres=[0.0, 3.490658504005327e-05, 0.0002617993877989113, 0.0008726646259973632, 0.0020594885173532214, 0.003996803987067249, 0.00689405054537778, 0.010925761117484585, 0.016249015336067412, 0.02307325271136508, 0.03152064629101779, 0.041783182292744345, 0.05400048705670445, 0.06832964021557787, 0.08487536152448423, 0.10379473061610282, 0.1252099205380733, 0.14920819775299535, 0.17589166402211076, 0.20535543978965265, 0.23767893753658773, 0.27291713513435334, 0.31112239246050905, 0.3523470693926153, 0.3966260725157113, 0.4439594018297976, 0.49436451062739384, 0.5478239456159802, 0.6043377067955567, 0.6638708875810833, 0.726365892107244, 0.7917860684597474, 0.8600598581392618, 0.9311122119879509, 1.0048541182139632, 1.0811878383791873, 1.1600034167407474, 1.2411856615680115, 1.3246054184963325, 1.4101248065148024, 1.4976024539540103, 1.5868830265105283, 1.677809444551677, 1.770214156469265, 1.863926119996596, 1.9587690568792198, 2.054557962216425, 2.151113067095255, 2.2482406399687394, 2.3457546287386153, 2.443460952792061, 2.541167276845507, 2.638681265615383, 2.7358088384888677, 2.8323639433676977, 2.9281528487049027, 3.0229957855875265, 3.1167077491148576, 3.2091124610324457, 3.3000388790735946, 3.3893194516301124, 3.4767970990693207, 3.5623164870877906, 3.645736244016111, 3.7269184888433755, 3.8057340672049356, 3.8820677873701595, 3.955809693596172, 4.026862047444861, 4.095135837124375, 4.160556013476879, 4.22305101800304, 4.282584198788566, 4.339097959968142, 4.392557394956729, 4.442962503754325, 4.49029583306841, 4.534574836191508, 4.575799513123613, 4.614004770449769, 4.649242968047535, 4.681566465794471, 4.711030241562012, 4.737713707831127, 4.761711985046049, 4.78312717496802, 4.802046544059639, 4.818592265368546, 4.832921418527418, 4.845138723291378, 4.855401259293104, 4.863848652872758, 4.870672890248055, 4.875996144466638, 4.880027855038746, 4.8829251015970545, 4.88486241706677, 4.886049240958125, 4.886660106196324, 4.886886998999082, 4.886921905584122]


## aca esta para gener trayectoria con Scurve
# total_codo   = back[4]+[back[4][-1] for ke in keep]+[fu+back[4][-1] for fu in full[4]]+[full[4][-1]+back[4][-1] for ke in keep]
# total_hombro = back[0]+[back[0][-1] for ke in keep]+[fu+back[0][-1] for fu in full[0]]+[full[0][-1]+back[0][-1] for ke in keep]
#
# #escalon
# total_codo   = [-3.14/2*0.94*escalon(t) for t in np.linspace(0,2,1000)]+[-3.14/2*0.94 for ke in keep]+[-3.14/2*0.94+3.14*0.94*escalon(t) for t in np.linspace(0,2,1000)]+[3.14/2*0.94 for ke in keep]
# total_hombro =  [-15.0/18.0*3.14*0.94*escalon(t) for t in np.linspace(0,2,1000)]+[-15.0/18.0*3.14*0.94 for ke in keep]+[-15.0/18.0*3.14*0.94+15.0/9.0*3.14*0.94*escalon(t) for t in np.linspace(0,2,1000)]+[15.0/18.0*3.14*0.94 for ke in keep]


# tray dres
total_codo   = back[4]+[back[4][-1] for ke in keep]+[fu+back[4][-1] for fu in codo_dres]+[codo_dres[-1]+back[4][-1] for ke in keep]
total_hombro = back[0]+[back[0][-1] for ke in keep]+[fu+back[0][-1] for fu in hombro_dres]+[hombro_dres[-1]+back[0][-1] for ke in keep]

# plt.plot(total_codo)
# plt.plot(total_hombro)
# plt.show()
def codo_tray(t):
    ind=t*500.0
    if ind+1>=len(total_codo):
        return total_codo[-1]
    elif ind==int(ind):
        return total_codo[int(ind)]
    else:
        return total_codo[int(ind)]*(1-ind+int(ind))+total_codo[int(ind)+1]*(ind-int(ind))

def hombro_tray(t):
    ind=t*100.0
    if ind+1>=len(total_hombro):
        return total_hombro[-1]
    elif ind==int(ind):
        return total_hombro[int(ind)]
    else:
        return total_hombro[int(ind)]*(1-ind+int(ind))+total_hombro[int(ind)+1]*(ind-int(ind))

def codo_callback(msg):
    #print('guardardato')

    #se guarda el error
    data_codo.append(msg.error)

    #se guarda el tiempo
    tiempo_codo.append(msg.header.stamp.to_sec())
    accion_codo.append(msg.command)

def hombro_callback(msg):
    #print('guardardato')

    #se guarda el error
    data_hombro.append(msg.error)

    #se guarda el tiempo
    tiempo_hombro.append(msg.header.stamp.to_sec())
    accion_hombro.append(msg.command)

#initialize ros stuff
rospy.init_node('run_simulation_once')
client_codo      = dynamic_reconfigure.client.Client('/nelen/codo_cont/pid')
client_hombro    = dynamic_reconfigure.client.Client('/nelen/hombro_control/pid')
pub_codo_cmd     = rospy.Publisher('/nelen/codo_cont/command', Float64, queue_size=10)
sub_codo_state   = rospy.Subscriber('/nelen/codo_cont/state', JointControllerState, codo_callback)
pub_hombro_cmd   = rospy.Publisher('/nelen/hombro_control/command', Float64, queue_size=10)
sub_hombro_state = rospy.Subscriber('/nelen/hombro_control/state', JointControllerState, hombro_callback)


class Particle:
    def __init__(self, x, c1, c2):
        self.dim = len(x)
        self.pos_i = x
        self.vel_i = np.random.uniform(-1, 1, self.dim)
        self.pos_best_i = []
        self.err_best_i = 100000000
        self.err_i = 100000000
        self.c1 = c1
        self.c2 = c2

    def evaluate(self, costFunc):
        self.err_i = costFunc(self.pos_i) # evaluate parameter's performance

        # update pos_best_i and err_best_i
        if self.err_i < self.err_best_i:
            self.pos_best_i = self.pos_i
            self.err_best_i = self.err_i

    def update_vel(self, pos_best_g, w):
        r1 = np.random.random(self.dim)
        r2 = np.random.random(self.dim)
        vel_cognitive = self.c1 * r1 * (np.array(self.pos_best_i) - np.array(self.pos_i))
        vel_social = self.c2 * r2 * (np.array(pos_best_g) - np.array(self.pos_i))
        self.vel_i = w * self.vel_i + vel_cognitive + vel_social

    def update_pos(self, bounds):
        self.pos_i += self.vel_i

        for i in range(self.dim):
            if self.pos_i[i] > bounds[i][1]:
                self.pos_i[i] = bounds[i][1]
            if self.pos_i[i] < bounds[i][0]:
                self.pos_i[i] = bounds[i][0]
class PSO():
    def __init__(self, costFunc, bounds, N, iter, c1, c2, pos_best_g=[], err_best_g=1):
        self.err_log = np.zeros((N, iter))
        xdim = len(bounds)

        self.pos_best_g = pos_best_g

        self.err_best_g = err_best_g

        swarm = []
        for i in range(N):
            xx = []
            for d in range(xdim):
                xx.append(np.random.uniform(bounds[d][0], bounds[d][1]))
            swarm.append(Particle(xx, c1, c2))

        print("Birds are exploring...")
        i = 0

        pbar = tqdm(total=iter * N)
        while i < iter:
            w = np.exp(-i / iter)

            for j in range(N):
                swarm[j].evaluate(costFunc)
                self.err_log[j, i] = swarm[j].err_i

                if swarm[j].err_i < self.err_best_g:
                    self.pos_best_g = copy.deepcopy(swarm[j].pos_i)
                    self.err_best_g = copy.deepcopy(swarm[j].err_i)
                pbar.update(1)
            print("Best position so far : ", self.pos_best_g)
            print("Best error so far : ", self.err_best_g)

            for j in range(N):
                swarm[j].update_vel(self.pos_best_g, w)
                swarm[j].update_pos(bounds)

            i += 1
        pbar.close()

        print("Iteration :", iter)
        print("Optimized parameters :", self.pos_best_g)
        print("Minimum error :", self.err_best_g)

#esta funcion se opera cada vez que el nodo recibe un mensaje

def sigmoide(t):
    t_tot = 2
    t_onset = 0.5
    agres = 25.0
    expon = 1/(1 + np.exp(-(t - t_onset) * agres))

    return expon



def run_once(hombro_pid,codo_pid, fun_codo,fun_hombro):

    t_it=len(total_codo)/100.0+0.25  #the whole realization+0.25s

    # Set the new gains
    params_hombro ={'p':hombro_pid[0],'i':hombro_pid[1],'d':hombro_pid[2]}
    config_hombro = client_hombro.update_configuration(params_hombro)
    params_codo   ={'p':codo_pid[0],'i':codo_pid[1],'d':codo_pid[2]}
    config_codo   = client_codo.update_configuration(params_codo)
    # Set publication rate
    rate = rospy.Rate(100) # 100hz
    pub_codo_cmd.publish(0.0)

    #resetear resultados
    data_codo[:]=[]
    tiempo_codo[:]=[]
    data_hombro[:]=[]
    tiempo_hombro[:]=[]
    accion_codo[:]=[]
    accion_hombro[:]=[]
    reset_simulation()
    unpause_simulation()
    time.sleep(0.01)
    t_init=rospy.get_time()
    while True:
        t = rospy.get_time() # get current time
        # Ver que publicar o retornar
        pub_codo_cmd.publish(fun_codo(t-t_init))
        pub_hombro_cmd.publish(fun_hombro(t-t_init))
        if t>=t_it+t_init:

            #pausar simulacion y cambiar el set point
            pause_simulation()
            pub_codo_cmd.publish(0.0)
            pub_hombro_cmd.publish(0.0)

            return ([tiempo_hombro,data_hombro,tiempo_codo,data_codo])

        # elif t>=t_escalon+t_init:
        #     pub_codo_cmd.publish(1.0)
        #
        # else:
        #     pub_codo_cmd.publish(0.0)


        # micro pause to reach the desired rate
        rate.sleep()



def ponderar(resultado):
    lamda=20.0
    time_back=len(back[0])/500.0
    time_keep=len(keep)/500.0
    time_full=len(full[0])/500.0
    el_k=0
    # print 'largos =' ,
    # print len(resultado),
    # print len(resultado[0]),
    # print len(resultado[1]),
    # print len(resultado[2]),
    # print len(resultado[3])
    this_hombro=[0. for l in resultado[1]]
    this_codo=[0. for l in resultado[3]]
    for t in resultado[0][0:-1]:
        if t>time_back and t<time_keep+time_back:
            this_hombro[el_k]=resultado[1][el_k]*lamda
        elif t>time_keep+time_back+time_full:
            this_hombro[el_k]=resultado[1][el_k]*lamda
        else:
            this_hombro[el_k]=resultado[1][el_k]

        # print 'el k hombro fue ',
        # print el_k,
        # print ' largo hombro = ',
        # print len(this_hombro),
        # print ' largo tiempo = ',
        # print len(resultado[1])
        el_k=el_k+1


    el_k=0
    for t in resultado[2][0:-1]:
        if t>time_back and t<time_keep+time_back:
            this_codo[el_k]=resultado[3][el_k]*lamda
        elif t>time_keep+time_back+time_full:
            this_codo[el_k]=resultado[3][el_k]*lamda
        else:
            this_codo[el_k]=resultado[3][el_k]
        # print 'el k codo fue ',
        # print el_k,
        # print ' largo codo = ',
        # print len(this_hombro),
        # print ' largo tiempo = ',
        # print len(resultado[3])
        el_k=el_k+1


    return [resultado[0],this_hombro,resultado[2],this_codo]

def costFunc(x):
  # simulate with Gazebo with parameters x=[p,i,d]



  sin_ponder=run_once(x[0:3],x[3:6],codo_tray,hombro_tray)


  resultados=ponderar(sin_ponder)

  operado_codo     = errorCuadratico(resultados[2],resultados[3])
  operado_hombro   = errorCuadratico(resultados[0],resultados[1])
  # op_action_codo   = errorCuadratico(resultados[2],resultados[5])
  # op_action_hombro = errorCuadratico(resultados[0],resultados[4])


  if operado_codo<0.0 or operado_hombro<0.0:
      print(resultados[0])
      print(resultados[2])
      pause_simulation()
      time.sleep(0.5)
      reset_simulation()
      time.sleep(0.5)
      resultados=run_once(x[0:3],x[3:6],codo_tray,hombro_tray)
      operado_codo= errorCuadratico(resultados[2],resultados[3])
      operado_hombro= errorCuadratico(resultados[0],resultados[1])
      if operado_codo<0.0 or operado_hombro<0.0:
          print(resultados[0])
          print(resultados[2])
          return 10000
      else:
          # print(100.*(operado_codo+operado_hombro)+(op_action_codo+op_action_hombro)/1000.)
          return (operado_codo+operado_hombro)
  else:
      # print x,
      print(operado_codo+operado_hombro)
      # print((operado_codo+operado_hombro)+(op_action_codo+op_action_hombro)/1000.)
      return (operado_codo+operado_hombro)





def errorCuadratico(tiempo,data):
  # alpha=1.2
  # data2=ponderar(data,alpha)
  data2=[thisData**2 for thisData in data]
     #se remueve el primer dato
  data2.remove(data2[0])

     #calcular la diferencia de tiempo por el error al cuadrado
  ponderados=[thisTiempo*thisData for thisTiempo,thisData in zip(np.diff(tiempo),data2)]
  return sum(ponderados)/len(ponderados)

if __name__ == '__main__':
    try:

        # run_once([350.0,5.0,15.0],[350.0,5.0,15.0],codo_tray,hombro_tray)


        # #
        # # # #programa final
        # iter_num      = 20 # number of iterations
        # particle_num  = 50 # particle size
        # bounds_params = [[0.0,1000.0],[0.0,500.],[0.0,0.0],[0.0,1000.],[0.,500.],[0.0,0.0]]
        # pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=1,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
        # print(pso.pos_best_g)
        #
        # resultados=run_once(pso.pos_best_g[0:3],pso.pos_best_g[3:6],codo_tray,hombro_tray)
        # # resultados=run_once([715.68699424, 408.18883589,0.],[442.2078436,156.67895682,20.16516559],codo_tray,hombro_tray)
        #
        # hombro_sp=[hombro_tray(t) for t in resultados[0]]
        # hombro_pv=[sp-er for er,sp in zip(resultados[1],hombro_sp)]
        # codo_sp=[hombro_tray(t) for t in resultados[2]]
        # codo_pv=[sp-er for er,sp in zip(resultados[3],codo_sp)]
        # plt.subplot(3,2,1)
        # plt.plot(resultados[0],hombro_sp)
        # plt.plot(resultados[0],hombro_pv)
        # plt.subplot(3,2,3)
        # plt.plot(resultados[0],resultados[1])
        # plt.subplot(3,2,2)
        # plt.plot(resultados[2],codo_sp)
        # plt.plot(resultados[2],codo_pv)
        # plt.subplot(3,2,4)
        # plt.plot(resultados[2],resultados[3])
        # plt.subplot(3,2,5)
        # plt.plot(resultados[0],accion_hombro)
        # plt.subplot(3,2,6)
        # plt.plot(resultados[2],accion_codo)
        # plt.savefig('PI_cuadrado.png')
        #


        #
        # # # #programa final
        iter_num      = 20 # number of iterations
        particle_num  = 50 # particle size
        bounds_params = [[0.0,100000.0],[0.0,50000.0],[0.0,50000.0],[0.0,100000.],[0.,50000.],[0.0,50000.0]]
        pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=1,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
        print(pso.pos_best_g)

        # resultados=run_once(pso.pos_best_g[0:3],pso.pos_best_g[3:6],codo_tray,hombro_tray)
        # resultados=run_once([715.68699424, 408.18883589,0.],[442.2078436,156.67895682,20.16516559],codo_tray,hombro_tray)

        # print(costFunc([100.,0.,0.,100.,0.,0.]))

        resultados=run_once(pso.pos_best_g[0:3],pso.pos_best_g[3:6],codo_tray,hombro_tray)
        # resultados=run_once([715.68699424, 408.18883589,0.],[442.2078436,156.67895682,20.16516559],codo_tray,hombro_tray)

        hombro_sp=[hombro_tray(t) for t in resultados[0]]
        hombro_pv=[sp-er for er,sp in zip(resultados[1],hombro_sp)]
        codo_sp=[hombro_tray(t) for t in resultados[2]]
        codo_pv=[sp-er for er,sp in zip(resultados[3],codo_sp)]
        plt.subplot(3,2,1)
        plt.plot(resultados[0],hombro_sp)
        plt.plot(resultados[0],hombro_pv)
        plt.subplot(3,2,3)
        plt.plot(resultados[0],resultados[1])
        plt.subplot(3,2,2)
        plt.plot(resultados[2],codo_sp)
        plt.plot(resultados[2],codo_pv)
        plt.subplot(3,2,4)
        plt.plot(resultados[2],resultados[3])
        plt.subplot(3,2,5)
        plt.plot(resultados[0],np.clip(accion_hombro,-45.,45.))
        plt.subplot(3,2,6)
        plt.plot(resultados[2],np.clip(accion_codo,-35.,35.))
        plt.show()





        # pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=1,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
        # print(pso.pos_best_g)







    except rospy.ROSInterruptException:
        pass
