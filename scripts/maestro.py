#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import actionlib
import dynamic_reconfigure.client # Para mudar a velocidade dinamicamente
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler # Para converter Yaw em Quaternion

class Maestro:
    def __init__(self):
        rospy.init_node('maestro_integrador')
        
        # --- 1. Carregar Dados do Param Server ---
        try:
            self.locais = rospy.get_param('/locais')
            self.catalogo = rospy.get_param('/catalogo_pecas')
            self.plano = rospy.get_param('/plano')
            rospy.loginfo("Maestro: Dados carregados. Total de tarefas: {}".format(len(self.plano)))
        except KeyError as e:
            rospy.logerr("Erro crítico ao ler parâmetros: {}".format(e))
            sys.exit(1)

        # --- 2. Conectar ao Move Base (Navegação) ---
        rospy.loginfo("Maestro: Aguardando servidor move_base...")
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()
        rospy.loginfo("Maestro: move_base conectado!")

        # --- 3. Conectar ao Dynamic Reconfigure (Velocidade) ---
        # ATENCAO: Verifique se o nome do no do seu planejador e este mesmo.
        # No Trabalho 2, configuramos como "DWAPlannerROS".
        nome_planejador = "/move_base/DWAPlannerROS" 
        rospy.loginfo("Maestro: Conectando ao reconfigurador de velocidade em {}...".format(nome_planejador))
        try:
            self.dyn_client = dynamic_reconfigure.client.Client(nome_planejador, timeout=4.0)
            rospy.loginfo("Maestro: Reconfigurador conectado!")
        except Exception as e:
            rospy.logwarn("AVISO: Nao foi possivel conectar ao Dynamic Reconfigure. O robo nao mudara de velocidade.")
            rospy.logwarn("Erro: {}".format(e))
            self.dyn_client = None

    def ajustar_velocidade(self, max_vel):
        """ Altera a velocidade maxima do robô via Dynamic Reconfigure """
        if self.dyn_client:
            rospy.loginfo(">>> Ajustando VELOCIDADE MAXIMA para: {:.1f} m/s".format(max_vel))
            # O nome do parametro no DWA geralmente e 'max_vel_trans' ou 'max_vel_x'.
            # Vamos tentar ajustar ambos para garantir.
            params = {
                'max_vel_x': max_vel,
                'max_vel_trans': max_vel 
            }
            self.dyn_client.update_configuration(params)

    def navegar_para(self, nome_local):
        """ Envia o robo para uma coordenada do dicionario 'locais' """
        coords = self.locais[nome_local] # Pega [x, y, yaw]
        x, y, yaw = coords[0], coords[1], coords[2]

        rospy.loginfo(">>> Navegando para {} ({:.1f}, {:.1f})...".format(nome_local, x, y))

        # Cria a meta
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Converte Yaw (radianos) para Quaternion
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # Envia e espera
        self.nav_client.send_goal(goal)
        sucesso = self.nav_client.wait_for_result()
        
        if sucesso:
            rospy.loginfo(">>> Chegou em {}!".format(nome_local))
        else:
            rospy.logerr(">>> Falha ao navegar para {}.".format(nome_local))
        
        return sucesso

    def executar_plano(self):
        """ Loop principal """
        
        # Define velocidade padrao inicial
        self.ajustar_velocidade(0.5)

        for i, tarefa in enumerate(self.plano):
            peca = tarefa['peca']
            origem = tarefa['origem']
            destino = 'C4' # O destino final e sempre a mesa de entrega C4
            
            rospy.loginfo("\n========== TAREFA {}/{} : Peca {} ==========".format(i+1, len(self.plano), peca))
            
            # --- 1. CONFIGURA VELOCIDADE PARA "VAZIO" (RAPIDO) ---
            self.ajustar_velocidade(0.5) # Velocidade padrao para ir buscar

            # --- 2. IR PARA COLETA ---
            if not self.navegar_para(origem):
                rospy.logerr("Abortando tarefa devido a erro de navegacao.")
                continue

            # --- 3. SIMULAR CARGA ---
            rospy.loginfo(">>> Carregando peca {}... (3s)".format(peca))
            rospy.sleep(3)

            # --- 4. CONFIGURA VELOCIDADE DA PECA (DEVAGAR se fragil) ---
            vel_peca = self.catalogo[peca]['max_vel']
            self.ajustar_velocidade(vel_peca)

            # --- 5. IR PARA ENTREGA ---
            if not self.navegar_para(destino):
                rospy.logerr("Erro ao ir para entrega.")
                continue

            # --- 6. HANDSHAKE (Por enquanto simulado) ---
            lado = self.catalogo[peca]['destino_tm']
            rospy.loginfo(">>> Solicitando ao TM: Colocar na {}".format(lado))
            rospy.sleep(2) # Simula o tempo do braço
            rospy.loginfo(">>> TM finalizou. Tarefa concluida.")
            
        rospy.loginfo("\n========== PLANO CONCLUIDO ==========")

if __name__ == '__main__':
    try:
        m = Maestro()
        m.executar_plano()
    except rospy.ROSInterruptException:
        pass
