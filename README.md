# Sistema Integrado - Robótica Avançada

Pacote ROS para integração de sistema robótico multi-agente com robô móvel e manipulador industrial.

## Descrição

Este pacote implementa um sistema integrado de automação que coordena um robô móvel (AMR) equipado com sensores laser e um braço manipulador Omron TM5x-900 para realizar tarefas de pick-and-place em um ambiente de produção.

O sistema é composto por:
- **Maestro**: Nó coordenador que gerencia a navegação do robô móvel e sincroniza as operações com o braço manipulador
- **TM Robot Controller**: Interface de controle do braço manipulador Omron TM5x-900 usando MoveIt
- **Plano de Produção**: Configuração YAML com locais, catálogo de peças e sequência de tarefas

## Arquitetura do Sistema

**IMPORTANTE**: Este sistema requer uma arquitetura distribuída de **três computadores**:

### 📌 Computador 1 - Robô Móvel
Localizado no próprio robô móvel (AMR), executando:
- **ROSARIA**: Driver para controle do robô móvel
- **sicktoolbox_wrapper**: Driver para sensor laser SICK

### 📌 Computador 2 - Maestro
Computador central que coordena todo o sistema:
- **Nó Maestro** (`maestro.py`): Coordenador principal
- **ROS Master**: Núcleo da rede ROS
- **Map Server**: Servidor de mapa para navegação
- **Move Base**: Navegação autônoma
- **AMCL**: Localização

### 📌 Computador 3 - Manipulador TM5x-900
Computador dedicado ao controle do braço robótico:
- **TM Robot Controller** (`tm_robot.py`): Interface MoveIt para o braço
- **Driver TM5x-900**: Driver do fabricante com MoveIt
- **IP do Robô**: 169.254.21.120

## Dependências

- ROS (testado com ROS Noetic)
- Python 2.7 ou 3.x
- MoveIt
- Navigation Stack (move_base, amcl, map_server)
- ROSARIA
- sicktoolbox_wrapper
- tm5x-900-moveit_config (driver do Omron TM5x-900)
- dynamic_reconfigure
- actionlib

## Estrutura do Pacote

```
sistema_integrado/
├── config/
│   └── plano_producao.yaml     # Configuração de locais, peças e plano
├── launch/
│   └── sistema.launch           # Launch file principal
├── scripts/
│   ├── maestro.py               # Nó coordenador
│   └── tm_robot.py              # Controlador do braço TM
├── srv/
│   └── Manipular.srv            # Definição do serviço
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Configuração

### Arquivo de Configuração (`config/plano_producao.yaml`)

O arquivo YAML contém três seções principais:

1. **locais**: Coordenadas (x, y, yaw) dos pontos de interesse no mapa
2. **catalogo_pecas**: Características de cada peça (velocidade máxima, destino no braço)
3. **plano**: Sequência de tarefas a serem executadas

Exemplo:
```yaml
locais:
  C1: [5.0, 0.9, 0.0]
  C2: [0.0, -1.0, 0.0]
  C4: [3.2, 0.9, 0.0]  # Ponto de entrega

catalogo_pecas:
  'P1': { max_vel: 0.5, destino_tm: 'esquerda' }
  'P2': { max_vel: 0.2, destino_tm: 'direita' }

plano:
  - { peca: 'P1', origem: 'C1' }
  - { peca: 'P2', origem: 'C2' }
```

## Como Executar

### ⚠️ ORDEM DE INICIALIZAÇÃO CRÍTICA

Siga esta ordem rigorosamente:

### 1️⃣ Computador do Robô Móvel
```bash
# Terminal 1 - ROSARIA
rosrun rosaria RosAria

# Terminal 2 - Sensor SICK
rosrun sicktoolbox_wrapper sicklms
```

### 2️⃣ Computador do Manipulador TM5x-900

**IMPORTANTE**: O driver do robô TM deve ser iniciado **ANTES** de executar `tm_robot.py`

```bash
# Terminal 1 - Iniciar o driver do robô TM5x-900 com MoveIt
roslaunch tm5x-900-moveit_config tm5x-900_moveit_planning_execution.launch sim:=False robot_ip:=169.254.21.120
```

Aguarde até que o MoveIt esteja completamente carregado e o robô conectado. Então:

```bash
# Terminal 2 - Controlador do braço (somente após o driver estar rodando)
rosrun sistema_integrado tm_robot.py
```

### 3️⃣ Computador Maestro

```bash
# Terminal 1 - Move Base (navegação)
roslaunch trabalho_slam navegacao_remota.launch

# Terminal 1 - Maestro (sistema integrado)
roslaunch sistema_integrado sistema.launch
```

## Funcionamento

### Fluxo de Operação

1. O **Maestro** carrega o plano de produção do arquivo YAML
2. Para cada tarefa no plano:
   - Navega até o local de origem da peça
   - Simula o carregamento da peça
   - Navega até o ponto de entrega (C4)
   - Envia comando para o braço TM posicionar-se
   - Envia comando para pegar a peça
   - Aguarda o movimento do braço
   - Envia comando para retornar à posição home

### Comandos do Braço TM

O braço TM aceita os seguintes comandos via tópico `/comando_robo`:
- `home`: Posição inicial/repouso
- `esquerda`: Posição de entrega à esquerda
- `direita`: Posição de entrega à direita
- `pegar` ou `pegarpeça`: Posição para pegar peça

### Comunicação

- **Tópico**: `/comando_robo` (std_msgs/String)
- **Publicador**: Maestro
- **Subscriber**: TM Robot Controller

## Configuração de Rede ROS

Como o sistema usa três computadores, configure corretamente:

1. **Definir ROS_MASTER_URI** em todos os computadores apontando para o Maestro:
```bash
export ROS_MASTER_URI=http://IP_DO_MAESTRO:11311
```

2. **Definir ROS_IP** em cada computador:
```bash
export ROS_IP=IP_LOCAL_DA_MAQUINA
```

3. **Garantir conectividade**: Teste ping entre todos os computadores

## Sincronismo de Relógio

**IMPORTANTE**: É essencial sincronizar o relógio do Maestro com o relógio do robô Pioneer 3DX e do sensor LIDAR SICK. Desincronizações podem causar problemas graves de timestamps nas mensagens ROS.

Para sincronizar os relógios, use o NTP (Network Time Protocol):

```bash
# Em todos os computadores (Maestro, Pioneer 3DX e LIDAR SICK)
sudo ntpdate -s pool.ntp.br

# Ou instale o serviço NTP para sincronização contínua
sudo apt-get install ntp
sudo service ntp start
```

Verifique se os relógios estão sincronizados:
```bash
date
```

## Posições do Braço TM5x-900

As posições são definidas em radianos no arquivo `tm_robot.py`:
- **home**: Posição de repouso
- **pegar**: Posição para coletar peça do robô móvel
- **esquerda**: Posição de entrega no lado esquerdo
- **direita**: Posição de entrega no lado direito

## Vídeo do Projeto

Confira uma demonstração prática do sistema em funcionamento:

📹 **[Vídeo do Projeto no YouTube](https://youtube.com/shorts/5f5ZQC2pe5w)**

## Slides da Apresentação

A pasta `slides/` contém uma apresentação em PDF com detalhes sobre a arquitetura, funcionamento e resultados do projeto.

## Troubleshooting

### Problema: "Não foi possível conectar ao grupo 'tmr_arm'"
- Verifique se o driver do TM está rodando corretamente
- Confirme que o nome do grupo no MoveIt é 'tmr_arm' 
- Verifique conexão de rede com o robô (ping 169.254.21.120)

### Problema: "move_base não responde"
- Verifique se o Map Server está publicando o mapa
- Confirme que AMCL está localizado
- Verifique configuração dos parâmetros do DWA Planner

### Problema: "Robô móvel não se move"
- Verifique se ROSARIA está conectado ao robô
- Confirme que não há obstáculos bloqueando o caminho
- Verifique se o sensor SICK está publicando dados

## Autores

### Contribuidores

- Henrique Xavier Vincetini
- João Vitor Barbosa Pinheiro
- Julia Da Cruz Viana

### Professor Responsável

- Guilherme de Souza Bastos

Desenvolvido para a disciplina de Robótica Avançada - 8° Período
UNIFEI - Universidade Federal de Itajubá

## Licença

Este projeto é de uso acadêmico.
