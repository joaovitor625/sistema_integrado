#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import actionlib
import dynamic_reconfigure.client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

class Maestro:
    def __init__(self):
        rospy.init_node('maestro_integrador')
        
        # --- Configuração do Publicador para o Braço TM ---
        rospy.loginfo("Maestro: Criando publicador para o braço TM...")
        self.tm_pub = rospy.Publisher('/comando_robo', String, queue_size=10)
        
        rospy.sleep(1.0) # Garante conexão
        rospy.loginfo("Maestro: Canal de comunicação com TM pronto!")

        # --- 1. Carregar Dados ---
        try:
            self.locais = rospy.get_param('/locais')
            self.catalogo = rospy.get_param('/catalogo_pecas')
            self.plano = rospy.get_param('/plano')
            rospy.loginfo("Maestro: Dados carregados. Total de tarefas: {}".format(len(self.plano)))
        except KeyError as e:
            rospy.logerr("Erro crítico ao ler parâmetros: {}".format(e))
            sys.exit(1)

        # --- 2. Conectar ao Move Base ---
        rospy.loginfo("Maestro: Aguardando servidor move_base...")
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()
        rospy.loginfo("Maestro: move_base conectado!")

        # --- 3. Conectar ao Dynamic Reconfigure ---
        nome_planejador = "/move_base/DWAPlannerROS" 
        try:
            self.dyn_client = dynamic_reconfigure.client.Client(nome_planejador, timeout=4.0)
        except Exception as e:
            self.dyn_client = None

    def ajustar_velocidade(self, max_vel):
        if self.dyn_client:
            params = {'max_vel_x': max_vel, 'max_vel_trans': max_vel}
            self.dyn_client.update_configuration(params)

    def navegar_para(self, nome_local):
        coords = self.locais[nome_local]
        x, y, yaw = coords[0], coords[1], coords[2]

        rospy.loginfo(">>> Navegando para {}...".format(nome_local))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.nav_client.send_goal(goal)
        return self.nav_client.wait_for_result()

    def enviar_comando_braco(self, comando, tempo_espera):
        """ Envia string para o braço e espera o movimento """
        rospy.loginfo(">>> MAESTRO -> ROBO: {}".format(comando.upper()))
        
        msg = String()
        msg.data = comando
        self.tm_pub.publish(msg)
        
        # Espera o tempo estimado para o movimento concluir
        rospy.sleep(tempo_espera)

    def executar_plano(self):
        """ Loop principal com a Lógica de Produção """
        
        self.ajustar_velocidade(0.5)

        for i, tarefa in enumerate(self.plano):
            peca = tarefa['peca']
            origem = tarefa['origem']
            destino = 'C4' 
            
            rospy.loginfo("\n========== INICIANDO TAREFA {}/{} : Peca {} ==========".format(i+1, len(self.plano), peca))
            
            # 1. BUSCAR PEÇA NA ORIGEM (Prateleira)
            self.ajustar_velocidade(0.5) 
            if not self.navegar_para(origem):
                continue

            rospy.loginfo(">>> Simulando carga da peça {}...".format(peca))
            rospy.sleep(2)

            # 2. LEVAR PARA C4 (Mesa de Entrega)
            if peca in self.catalogo:
                self.ajustar_velocidade(self.catalogo[peca]['max_vel'])
            
            if not self.navegar_para(destino):
                rospy.logerr("Erro ao chegar na mesa de entrega C4.")
                continue

            # ====================================================
            # AQUI COMEÇA A SEQUÊNCIA DO BRAÇO (A COREOGRAFIA)
            # ====================================================
            rospy.loginfo(">>> Chegamos em C4. Iniciando transferencia com o braço...")
            
            if peca in self.catalogo:
                lado_destino = self.catalogo[peca]['destino_tm']
                
                # PASSO A: Mandar o braço PEGAR a peça no robô móvel
                # Tempo estimado: 6 segundos para ir da Home até a peça
                self.enviar_comando_braco("pegar", 6.0)

                # PASSO B: Mandar o braço levar para o lado correto (DIREITA ou ESQUERDA)
                # Tempo estimado: 6 segundos para mover e soltar
                self.enviar_comando_braco(lado_destino, 6.0)

                # PASSO C: Mandar o braço voltar para HOME (Segurança)
                # Importante para o braço não ficar esticado na frente do Pioneer
                self.enviar_comando_braco("home", 5.0)

            else:
                rospy.logwarn("Peca desconhecida, pulando etapa do braço.")

            rospy.loginfo(">>> Transferência concluída. Pioneer liberado.")
            
        rospy.loginfo("\n========== PLANO DE PRODUÇÃO FINALIZADO ==========")
        # Garante que o braço termine em Home
        self.enviar_comando_braco("home", 2.0)

if __name__ == '__main__':
    try:
        m = Maestro()
        m.executar_plano()
    except rospy.ROSInterruptException:
        pass
