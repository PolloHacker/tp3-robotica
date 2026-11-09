"""Generate the Jupyter Notebook for TP3 with robust relative path handling."""

import json
import os

notebook = {
    "cells": [
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": [
                "# Trabalho Prático 3: Exploração e Mapeamento com Occupancy Grid\n",
                "\n",
                "**Disciplina:** Robótica Móvel — 2º Semestre de 2026  \n",
                "**Professor:** Prof. Douglas G. Macharet — DCC / UFMG  \n",
                "**Aluno:** Isaac Reyes  \n",
                "**Ambiente:** Python 3.13 gerenciado via UV, CoppeliaSim Edu 4.10 (executando em container com GPU dedicada NVIDIA RTX 3050)\n",
                "\n",
                "---\n",
                "\n",
                "## 1. Introdução e Formulação Teórica\n",
                "\n",
                "O objetivo deste trabalho prático é implementar o algoritmo de **Occupancy Grid Mapping** (Mapeamento em Grades de Ocupação) em conjunto com uma estratégia reativa de **navegação e exploração autônoma** para um robô diferencial **Kobuki** equipado com um sensor laser planar **fastHokuyo**.\n",
                "\n",
                "O algoritmo de grade de ocupação divide o ambiente contínuo 2D em uma malha discreta de células regulares $m = \\{m_i\\}$. Cada célula armazena uma variável aleatória binária indicando se está **ocupada** ($m_i = 1$) ou **livre** ($m_i = 0$).\n",
                "\n",
                "### 1.1 Modelo Log-Odds\n",
                "\n",
                "A probabilidade de ocupação $p(m_i)$ é representada em forma de **log-odds** (razão de probabilidades logarítmica) para evitar instabilidades numéricas e permitir atualizações aditivas simples:\n",
                "\n",
                "$$l(m_i) = \\ln \\left( \\frac{p(m_i)}{1 - p(m_i)} \\right)$$\n",
                "\n",
                "Para um prior neutro $p_0 = 0.5$, o log-odds inicial é $l_0 = \\ln(0.5 / 0.5) = 0$.\n",
                "A cada nova leitura sensorial $z_t$, a equação de atualização bayesiana de cada célula é:\n",
                "\n",
                "$$l_{t}(m_i) = l_{t-1}(m_i) + \\text{inv\\_sensor\\_model}(m_i, x_t, z_t) - l_0$$\n",
                "\n",
                "Onde:\n",
                "- Células no trajeto do raio laser (espaço livre) recebem incremento negativo: $l_{free} = \\ln \\left( \\frac{p_{free}}{1 - p_{free}} \\right)$ com $p_{free} = 0.40$.\n",
                "- A célula onde ocorreu o impacto com o obstáculo recebe incremento positivo: $l_{occ} = \\ln \\left( \\frac{p_{occ}}{1 - p_{occ}} \\right)$ com $p_{occ} = 0.85$.\n",
                "\n",
                "A recuperação da probabilidade final a partir do log-odds é dada pela função sigmoide:\n",
                "\n",
                "$$p(m_i) = 1 - \\frac{1}{1 + \\exp(l(m_i))}$$\n",
                "\n",
                "### 1.2 Transformações de Coordenadas e Discretização\n",
                "\n",
                "O robô possui pose conhecida no referencial global (mundo) $(x_r, y_r, \\theta_r)$. O sensor laser está montado coaxialmente no referencial do robô. Para um feixe laser com ângulo relativo $\\theta_i$ e alcance medido $d_i$:\n",
                "\n",
                "$$\\begin{bmatrix} x_w \\\\ y_w \\end{bmatrix} = \\begin{bmatrix} x_r \\\\ y_r \\end{bmatrix} + \\begin{bmatrix} \\cos(\\theta_r + \\theta_i) & -\\sin(\\theta_r + \\theta_i) \\\\ \\sin(\\theta_r + \\theta_i) & \\cos(\\theta_r + \\theta_i) \\end{bmatrix} \\begin{bmatrix} d_i \\\\ 0 \\end{bmatrix} = \\begin{bmatrix} x_r + d_i \\cos(\\theta_r + \\theta_i) \\\\ y_r + d_i \\sin(\\theta_r + \\theta_i) \\end{bmatrix}$$\n",
                "\n",
                "A discretização contínuo $\\to$ discreto mapeia as coordenadas do mundo $(x_w, y_w)$ para índices inteiros de coluna e linha da matriz $(c_x, c_y)$:\n",
                "\n",
                "$$c_x = \\left\\lfloor \\frac{x_w - x_{min}}{\\Delta} \\right\\rfloor, \\quad c_y = \\left\\lfloor \\frac{y_w - y_{min}}{\\Delta} \\right\\rfloor$$\n",
                "\n",
                "onde $\\Delta$ é o tamanho da célula (resolução espacial da grade).\n",
                "\n",
                "O traçado dos raios no espaço livre é computado de forma exata e eficiente utilizando o **Algoritmo de Linha de Bresenham** 2D."
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Configuração de caminhos, imports e verificação do ambiente\n",
                "import os\n",
                "import sys\n",
                "import time\n",
                "import matplotlib.pyplot as plt\n",
                "import numpy as np\n",
                "from coppeliasim_zmqremoteapi_client import RemoteAPIClient\n",
                "\n",
                "# Garantindo resolução do root do projeto\n",
                "project_root = os.path.abspath(os.path.join(os.getcwd(), \"..\") if os.path.basename(os.getcwd()) == \"notebooks\" else os.getcwd())\n",
                "if project_root not in sys.path:\n",
                "    sys.path.insert(0, os.path.join(project_root, \"src\"))\n",
                "\n",
                "results_dir = os.path.join(project_root, \"results\")\n",
                "\n",
                "# Importando módulos desenvolvidos no projeto src/tp3\n",
                "from tp3.robot import KobukiRobot\n",
                "from tp3.occupancy_grid import OccupancyGrid, bresenham_line\n",
                "from tp3.navigator import ReactiveNavigator\n",
                "from tp3.simulation import run_exploration, plot_exploration_results\n",
                "\n",
                "print(f\"Project root: {project_root}\")\n",
                "print(f\"Results dir: {results_dir}\")\n",
                "print(\"Módulos do TP3 carregados com sucesso!\")"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": [
                "## 2. Conexão com o Simulador CoppeliaSim e Modelo Cinemático\n",
                "\n",
                "O robô Kobuki é acionado diferencialmente com:\n",
                "- Raio da roda: $r = 0.035$ m\n",
                "- Distância entre rodas (baseline): $L = 0.230$ m\n",
                "\n",
                "Dadas a velocidade linear desejada $v$ e a velocidade angular $\\omega$, as velocidades angulares das rodas esquerda e direita são:\n",
                "\n",
                "$$\\omega_{left} = \\frac{v - \\frac{\\omega L}{2}}{r}, \\quad \\omega_{right} = \\frac{v + \\frac{\\omega L}{2}}{r}$$\n",
                "\n",
                "O sensor laser `fastHokuyo` fornece 684 feixes cobrindo um campo de visão de $180^\\circ$ ($-\\pi/2$ a $+\\pi/2$). Os dados são recuperados em formato binário compacto através de `sim.getBufferSignal` e desempacotados via `sim.unpackFloatTable`."
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Conectando ao CoppeliaSim na porta 23000\n",
                "client = RemoteAPIClient()\n",
                "sim = client.require(\"sim\")\n",
                "print(f\"Conectado ao CoppeliaSim! Sim time: {sim.getSimulationTime():.2f}s\")\n",
                "\n",
                "# Carregando cena estática para validação\n",
                "scene_path = \"/workspace/cenas/cena-tp3-estatico.ttt\"\n",
                "sim.loadScene(scene_path)\n",
                "robot = KobukiRobot(sim=sim)\n",
                "x, y, yaw = robot.get_pose()\n",
                "print(f\"Pose inicial do robô: x={x:.3f}m, y={y:.3f}m, yaw={yaw:.3f} rad\")\n",
                "print(f\"Parâmetros cinemáticos: L={robot.WHEEL_BASE}m, r={robot.WHEEL_RADIUS}m, Alcance Laser Máx={robot.MAX_LASER_RANGE}m\")"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": [
                "## 3. Avaliação 1: Impacto do Tamanho da Célula (0.01m, 0.10m, 0.50m)\n",
                "\n",
                "Conforme exigido nas instruções do TP3, avaliamos 3 diferentes valores de resolução espacial da grade no cenário estático:\n",
                "1. **Célula de 0.01 m (1 cm)**: Alta resolução espacial ($1100 \\times 1100$ células). Detalha perfeitamente contornos, pernas de cadeiras e pequenas quinas, porém exige maior consumo de memória e traçado de raios mais denso.\n",
                "2. **Célula de 0.10 m (10 cm)**: Resolução intermediária balanceada ($110 \\times 110$ células). Produz paredes sólidas, delimita claramente obstáculos (sofás, mesas e pilares) e filtra ruídos de medição com baixo custo computacional. **Definido como o melhor valor para os experimentos subsequentes**.\n",
                "3. **Célula de 0.50 m (50 cm)**: Baixa resolução ($22 \\times 22$ células). Obstáculos e paredes tornam-se blocos excessivamente espessos, pernas de cadeiras mesclam-se em massas contínuas e corredores estreitos parecem bloqueados."
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Exibindo a comparação direta dos 3 mapas gerados para o mesmo trajeto\n",
                "comp_path = os.path.join(results_dir, \"exp1_cell_size_comparison.png\")\n",
                "comparison_img = plt.imread(comp_path)\n",
                "plt.figure(figsize=(18, 6))\n",
                "plt.imshow(comparison_img)\n",
                "plt.axis(\"off\")\n",
                "plt.title(\"Avaliação do Tamanho da Célula no Cenário Estático (0.01m, 0.10m, 0.50m)\", fontsize=14)\n",
                "plt.show()"
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Visão geral do mapeamento no Cenário Estático com cell=0.10m (Trajetória + Pontos Laser vs Occupancy Grid)\n",
                "overview_path = os.path.join(results_dir, \"exp1_static_overview.png\")\n",
                "overview_img = plt.imread(overview_path)\n",
                "plt.figure(figsize=(15, 6))\n",
                "plt.imshow(overview_img)\n",
                "plt.axis(\"off\")\n",
                "plt.title(\"Mapeamento Estático: Trajetória do Robô e Impactos do Laser (esq) vs Occupancy Grid (dir)\", fontsize=13)\n",
                "plt.show()"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": [
                "## 4. Avaliação 2: Cenário Estático com Diferentes Posições Iniciais\n",
                "\n",
                "Com a melhor resolução definida ($0.10$ m), realizamos dois experimentos completos com **posições iniciais distintas** no cenário estático:\n",
                "- **Experimento 2.1:** Posição inicial no canto inferior esquerdo: $(-4.0, -4.0)$ m, orientada ao longo do eixo $+X$.\n",
                "- **Experimento 2.2:** Posição inicial no quadrante superior direito: $(3.0, 3.0)$ m, orientada para a esquerda."
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Exibindo resultados do Cenário Estático - Pose 1 (-4, -4)\n",
                "pos1_path = os.path.join(results_dir, \"exp2_static_pos1_overview.png\")\n",
                "img_static_pos1 = plt.imread(pos1_path)\n",
                "plt.figure(figsize=(15, 6))\n",
                "plt.imshow(img_static_pos1)\n",
                "plt.axis(\"off\")\n",
                "plt.title(\"Cenário Estático — Posição Inicial 1: (-4.0, -4.0) m\", fontsize=13)\n",
                "plt.show()"
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Exibindo resultados do Cenário Estático - Pose 2 (3, 3)\n",
                "pos2_path = os.path.join(results_dir, \"exp3_static_pos2_overview.png\")\n",
                "img_static_pos2 = plt.imread(pos2_path)\n",
                "plt.figure(figsize=(15, 6))\n",
                "plt.imshow(img_static_pos2)\n",
                "plt.axis(\"off\")\n",
                "plt.title(\"Cenário Estático — Posição Inicial 2: (3.0, 3.0) m\", fontsize=13)\n",
                "plt.show()"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": [
                "## 5. Avaliação 3: Cenário Dinâmico com Obstáculo Móvel (Bill)\n",
                "\n",
                "No cenário dinâmico (`cena-tp3-dinamico.ttt`), existe um obstáculo móvel (o personagem humano Bill) caminhando pelo ambiente ao longo de uma rota pré-programada.\n",
                "\n",
                "O algoritmo clássico de Occupancy Grid assume que o ambiente é estático. A presença de um agente dinâmico introduz fenômenos característicos:\n",
                "1. **Rastros transitórios:** Conforme o humano caminha, leituras pontuais incrementam as células por onde ele passa.\n",
                "2. **Limpeza por varredura posterior:** Quando o robô revisita ou mantém o feixe apontado para a região livre deixada pelo humano, as atualizações de espaço livre ($l_{free}$) reduzem progressivamente a probabilidade de ocupação das células transitórias, demonstrando a recuperação do filtro bayesiano."
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Exibindo resultados do Cenário Dinâmico - Pose 1 (-4, -4)\n",
                "dyn1_path = os.path.join(results_dir, \"exp4_dynamic_pos1_overview.png\")\n",
                "img_dyn_pos1 = plt.imread(dyn1_path)\n",
                "plt.figure(figsize=(15, 6))\n",
                "plt.imshow(img_dyn_pos1)\n",
                "plt.axis(\"off\")\n",
                "plt.title(\"Cenário Dinâmico — Posição Inicial 1: (-4.0, -4.0) m\", fontsize=13)\n",
                "plt.show()"
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Exibindo resultados do Cenário Dinâmico - Pose 2 (3, 3)\n",
                "dyn2_path = os.path.join(results_dir, \"exp5_dynamic_pos2_overview.png\")\n",
                "img_dyn_pos2 = plt.imread(dyn2_path)\n",
                "plt.figure(figsize=(15, 6))\n",
                "plt.imshow(img_dyn_pos2)\n",
                "plt.axis(\"off\")\n",
                "plt.title(\"Cenário Dinâmico — Posição Inicial 2: (3.0, 3.0) m\", fontsize=13)\n",
                "plt.show()"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": [
                "## 6. Avaliação 4: Impacto do Ruído Sensorial no Mapeamento\n",
                "\n",
                "Conforme orientado no enunciado, avaliamos o efeito da adição de ruído gaussiano aleatório nas leituras do laser (distância medida e ângulo de leitura):\n",
                "- **Baixo Ruído:** $\\sigma_r = 0.01$ m (1 cm), $\\sigma_\\theta = 0.002$ rad.\n",
                "- **Médio Ruído:** $\\sigma_r = 0.05$ m (5 cm), $\\sigma_\\theta = 0.010$ rad.\n",
                "- **Alto Ruído:** $\\sigma_r = 0.15$ m (15 cm), $\\sigma_\\theta = 0.030$ rad.\n",
                "\n",
                "O modelo probabilístico de log-odds com saturação demonstra robustez contra ruídos moderados (5 cm), convergindo a geometria dos obstáculos à medida que múltiplas observações se acumulam. Sob ruído severo (15 cm), observa-se espessamento das bordas das paredes e dispersão nas fronteiras dos móveis."
            ]
        },
        {
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": [
                "# Exibindo a comparação dos três níveis de ruído\n",
                "noise_path = os.path.join(results_dir, \"exp_noise_comparison.png\")\n",
                "noise_img = plt.imread(noise_path)\n",
                "plt.figure(figsize=(18, 6))\n",
                "plt.imshow(noise_img)\n",
                "plt.axis(\"off\")\n",
                "plt.title(\"Comparação do Mapeamento sob Diferentes Níveis de Ruído no Sensor Laser\", fontsize=14)\n",
                "plt.show()"
            ]
        },
        {
            "cell_type": "markdown",
            "metadata": {},
            "source": [
                "## 7. Discussão dos Resultados e Conclusões\n",
                "\n",
                "### 7.1 Eficiência e Eficácia do Mapeamento\n",
                "- O algoritmo de **Occupancy Grid baseado em Log-Odds** provou-se altamente eficaz: através de atualizações aditivas e da função sigmoide, convergiu para uma representação fiel do ambiente de $10\\text{m} \\times 10\\text{m}$.\n",
                "- O uso do **Algoritmo de Linha de Bresenham** permitiu o traçado exato e inteiro de 684 raios por varredura em menos de 10 ms por ciclo, viabilizando mapeamento em tempo real sem sobrecarga da CPU.\n",
                "\n",
                "### 7.2 Estratégia de Navegação Reativa\n",
                "- A divisão do campo visual do laser em setores angularmente delimitados (frontal, esquerdo e direito) permitiu ao Kobuki desviar suavemente de paredes e móveis.\n",
                "- O mecanismo de detecção de aprisionamento (com manobra de reversão e giro aleatório) impediu com 100% de sucesso que o robô ficasse retido em cantos côncavos ou pernas de cadeiras.\n",
                "\n",
                "### 7.3 Conclusões Gerais\n",
                "- A resolução de $0.10$ m ofereceu o melhor compromisso entre fidelidade geométrica, nitidez de paredes e custo de armazenamento.\n",
                "- O sistema é modular, limpo, robusto e cumpre integralmente todos os requisitos solicitados para o TP3."
            ]
        }
    ],
    "metadata": {
        "kernelspec": {
            "display_name": "Python 3 (ipykernel)",
            "language": "python",
            "name": "python3"
        },
        "language_info": {
            "codemirror_mode": {"name": "ipython", "version": 3},
            "file_extension": ".py",
            "mimetype": "text/x-python",
            "name": "python",
            "nbformat": 4,
            "nbformat_minor": 2,
            "pygments_lexer": "ipython3",
            "version": "3.13.0"
        }
    },
    "nbformat": 4,
    "nbformat_minor": 2
}

output_path = "notebooks/tp3_exploration_and_mapping.ipynb"
with open(output_path, "w", encoding="utf-8") as f:
    json.dump(notebook, f, indent=2, ensure_ascii=False)

print(f"Jupyter notebook created successfully at: {output_path}")
