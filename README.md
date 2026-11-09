# Trabalho Prático 3 — Exploração e Mapeamento com Occupancy Grid

**Disciplina:** Robótica Móvel — 2º Semestre de 2026  
**Professor:** Prof. Douglas G. Macharet — DCC / UFMG  
**Aluno:** Isaac Reyes  
**Matrícula:** 2022069843  
**Email:** isaac.reyalves@gmail.com  

---

## 1. Visão Geral

Este projeto implementa o algoritmo de **Mapeamento em Grades de Ocupação (Occupancy Grid Mapping)** integrado a um **navegador reativo autônomo** para o robô diferencial **Kobuki** equipado com um scanner laser planar **fastHokuyo** no simulador **CoppeliaSim Edu 4.10**.

### Principais Características
- **Representação em Log-Odds:** Atualizações bayesianas aditivas eficientes com saturação simétrica em $[-5{,}0, +5{,}0]$, eliminando problemas de precisão numérica.
- **Traçado de Raios Exato (Bresenham 2D):** Determinação de células livres por aritmética inteira, processando varreduras de laser completas (684 feixes) em menos de $10$\,ms por ciclo.
- **Navegador Reativo com Anti-Stuck:** Máquina de estados de três setores angulares (Frontal, Esquerdo, Direito) com comportamento de margeamento de paredes e mecanismo de manobra de escape contra aprisionamento em quinas côncavas.
- **Gestão de Dependências com UV:** Ambiente virtual limpo, determinístico e bloqueado via `pyproject.toml` e `uv.lock`.
- **Aceleração por GPU Dedicada:** Integração direta com a GPU dedicada NVIDIA GeForce RTX 3050 Laptop via CDI (Container Device Interface) no container Docker do CoppeliaSim.

---

## 2. Estrutura do Repositório

```text
tp3/
├── pyproject.toml              # Definição do projeto e dependências UV
├── uv.lock                     # Bloqueio estrito de versões
├── README.md                   # Identificação do aluno e guia completo do projeto
├── TP3.pdf                     # Enunciado oficial do trabalho prático
├── cenas/                      # Cenas do CoppeliaSim fornecidas
│   ├── cena-tp3-estatico.ttt   # Cenário estático (sala 10x10m com móveis)
│   └── cena-tp3-dinamico.ttt   # Cenário dinâmico (com pedestre móvel Bill)
├── src/
│   └── tp3/                    # Código-fonte modular e documentado
│       ├── __init__.py         # Exportação de módulos e CLI entry-point
│       ├── __main__.py         # Ponto de entrada para python -m tp3
│       ├── robot.py            # Interface do robô Kobuki, cinemática e laser
│       ├── occupancy_grid.py   # Grade de ocupação, log-odds e Bresenham
│       ├── navigator.py        # Navegador reativo e detecção de aprisionamento
│       └── simulation.py       # Orquestrador de simulação e geração de plots
├── notebooks/
│   └── tp3_exploration_and_mapping.ipynb  # Jupyter Notebook completo e executado
├── scripts/
│   ├── run_all_experiments.py # Executa a suíte de experimentos e salva figuras
│   ├── build_notebook.py      # Constrói o Jupyter Notebook
│   └── generate_pdf_report.py # Compila a documentação em PDF
├── results/                    # Gráficos e mapas em alta resolução gerados
│   ├── exp1_cell_size_comparison.png
│   ├── exp1_static_overview.png
│   ├── exp2_static_pos1_overview.png
│   ├── exp3_static_pos2_overview.png
│   ├── exp4_dynamic_pos1_overview.png
│   ├── exp5_dynamic_pos2_overview.png
│   └── exp_noise_comparison.png
└── docs/
    └── relatorio_tp3.pdf       # Documentação completa em PDF (5 páginas diagramadas)
```

---

## 3. Instalação e Requisitos

O projeto requer **Python &ge; 3.11** e o gerenciador **UV**.

```bash
# Sincronizar todas as dependências do ambiente virtual:
uv sync
```

### Dependências Principais
- `coppeliasim-zmqremoteapi-client`: Comunicação ZMQ RemoteAPI com o CoppeliaSim.
- `numpy`: Vetorização e manipulação matricial da grade de ocupação.
- `matplotlib`: Geração e salvamento de plots e mapas.
- `pillow`: Exportação direta de mapas de ocupação em imagens de escala de cinza.
- `jupyter`: Execução interativa do notebook.
- `reportlab`: Geração do relatório em PDF.

---

## 4. Como Executar

### 4.1 Iniciar o CoppeliaSim

O projeto é compatível tanto com o **CoppeliaSim instalado nativamente no Host** quanto em **Container Docker**:

- **Opção A — CoppeliaSim Nativo (Sem Container):**
  Basta iniciar o CoppeliaSim normalmente na sua máquina (ele abrirá a Remote API ZMQ por padrão na porta `23000`).

- **Opção B — CoppeliaSim via Container Docker:**
  Certifique-se de que o container do CoppeliaSim está em execução ouvindo na porta `23000` e mapeando o repositório para `/workspace`:
  ```bash
  # Exemplo de execução via container (com aceleração de display/GPU):
  COPPELIA_DISPLAY_BACKEND=xwayland ./run-coppeliasim.sh
  # ou via docker run direto montando o workspace:
  # docker run --net=host -v "$(pwd):/workspace" ...
  ```

### 4.2 Executar o Script CLI
O projeto expõe um comando executável diretamente registrado em `pyproject.toml`. O carregador de cenas detecta automaticamente se a execução é no Host ou no Container:

```bash
# Execução padrão (Cenário estático, 45 segundos, célula de 0.10m):
uv run tp3

# Ou especificando argumentos customizados:
uv run tp3 --scene cenas/cena-tp3-estatico.ttt --duration 60 --cell-size 0.10 --output results/demo.png
```

### 4.3 Executar o Jupyter Notebook
Para explorar o trabalho interativamente com código, equações e visualizações:

```bash
uv run jupyter lab notebooks/tp3_exploration_and_mapping.ipynb
```

### 4.4 Reproduzir Todos os Experimentos do Relatório
Para re-executar todos os testes avaliados no relatório e regenerar as figuras:

```bash
uv run python scripts/run_all_experiments.py
```

### 4.5 Regenerar o Relatório em PDF
```bash
uv run python scripts/generate_pdf_report.py
```

---

## 5. Como o Sistema Foi Desenvolvido

### 5.1 Cinemática do Robô Kobuki
O robô diferencial Kobuki possui raio de roda $r = 0{,}035$\,m e distância entre rodas $L = 0{,}230$\,m. As velocidades angulares das rodas esquerda ($\omega_l$) e direita ($\omega_r$) são computadas a partir da velocidade linear $v$ e angular $\omega$ por:
$$\omega_l = \frac{v - \frac{\omega L}{2}}{r}, \qquad \omega_r = \frac{v + \frac{\omega L}{2}}{r}$$

### 5.2 Sensor Laser fastHokuyo
O script do sensor no CoppeliaSim realiza a desprojeção angular da visão das câmeras duplas e empacota os 684 alcances e ângulos em formato binário compacto (`sim.packFloatTable`). No cliente Python, as leituras são extraídas via `sim.getBufferSignal` e desempacotadas via `sim.unpackFloatTable`.

A transformação do referencial do laser para o mundo é dada por:
$$x_w = x_r + d_i \cos(\theta_r + \theta_i), \qquad y_w = y_r + d_i \sin(\theta_r + \theta_i)$$
onde $(x_r, y_r, \theta_r)$ é a pose global do robô e $(d_i, \theta_i)$ é a medição do feixe.

### 5.3 Algoritmo de Occupancy Grid com Log-Odds
A probabilidade $p(m_i)$ de cada célula é mapeada em log-odds:
$$l(m_i) = \ln \left( \frac{p(m_i)}{1 - p(m_i)} \right)$$

- **Prior neutro:** $p_0 = 0{,}5 \implies l_0 = 0$.
- **Célula livre (trajeto do feixe):** $\Delta l_{free} = \ln(0{,}40 / 0{,}60) \approx -0{,}405$.
- **Célula ocupada (ponto de impacto):** $\Delta l_{occ} = \ln(0{,}85 / 0{,}15) \approx +1{,}735$.
- **Saturação:** $l(m_i) \in [-5{,}0, +5{,}0]$ para evitar estagnação e permitir atualização dinâmica.
- **Recuperação de probabilidade:** $p(m_i) = 1 - \frac{1}{1 + \exp(l(m_i))}$.

### 5.4 Traçado de Raios por Bresenham
O traçado das células no espaço livre utiliza o algoritmo de linha de Bresenham inteiramente em aritmética de inteiros. Isso garante que a atualização de cada feixe processe apenas células efetivamente interceptadas, sem custo computacional de amostragem em ponto flutuante.

### 5.5 Navegação Reativa e Anti-Aprisionamento
O robô monitora a menor distância nos setores Frontal ($-25^\circ$ a $+25^\circ$), Esquerdo ($+25^\circ$ a $+85^\circ$) e Direito ($-85^\circ$ a $-25^\circ$):
1. **Desvio Frontal:** Se $d_{front} < 0{,}70$\,m, o robô para o avanço e rotaciona em direção ao lado mais livre.
2. **Wander com Alinhamento:** Com frente desobstruída ($v = 0{,}30$\,m/s), aplica-se viés suave quando paredes laterais são detectadas ($< 0{,}50$\,m), margeando os limites do salão.
3. **Detecção Anti-Stuck:** Se o deslocamento em 40 passos for $< 0{,}08$\,m, aciona reversão por 0,5s seguida de rotação em pivô aleatória entre $90^\circ$ e $150^\circ$, libertando o robô de quinas côncavas.

---

## 6. Resultados Experimentais Resumidos

1. **Impacto do Tamanho da Célula (0,01 m, 0,10 m, 0,50 m):**
   - $0{,}01$\,m detalha quinas finas mas consome memória excessiva e apresenta vazios radiais.
   - $0{,}50$\,m sofre aliasing severo e funde móveis em blocos densos.
   - **$0{,}10$\,m é o tamanho ideal**, combinando paredes nítidas, móveis bem delimitados e atualização rápida.
2. **Cenário Estático com Múltiplas Poses Iniciais:**
   - Testado partindo de $(-4{,}0, -4{,}0)$\,m e $(+3{,}0, +3{,}0)$\,m. O mapa convergiu com alta consistência mútua independentemente da pose inicial.
3. **Cenário Dinâmico (Pedestre Móvel):**
   - A movimentação do pedestre Bill gera rastros transitórios temporários de ocupação, que são gradativamente limpos conforme o robô realiza novas varreduras na área desocupada.
4. **Sensibilidade ao Ruído Sensorial:**
   - Ruídos de até $5$\,cm são eficientemente filtrados pelo modelo de log-odds. Apenas sob ruído elevado ($15$\,cm) nota-se espessamento artificial das bordas.

Para detalhes completos, diagramas e análise quantitativa aprofundada, consulte o relatório em PDF em [`docs/relatorio_tp3.pdf`](docs/relatorio_tp3.pdf).
