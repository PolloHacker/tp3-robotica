"""Generate the complete, publication-quality PDF report for TP3 using ReportLab."""

import os
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate,
    Paragraph,
    Spacer,
    Image,
    PageBreak,
    HRFlowable,
)


def build_pdf(filename: str = "docs/relatorio_tp3.pdf") -> None:
    os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)
    doc = SimpleDocTemplate(
        filename,
        pagesize=letter,
        leftMargin=38,
        rightMargin=38,
        topMargin=32,
        bottomMargin=32,
    )

    styles = getSampleStyleSheet()

    title_style = ParagraphStyle(
        "DocTitle",
        parent=styles["Normal"],
        fontName="Helvetica-Bold",
        fontSize=16,
        leading=20,
        alignment=1,
        textColor=colors.HexColor("#1A365D"),
    )

    header_style = ParagraphStyle(
        "DocHeader",
        parent=styles["Normal"],
        fontName="Helvetica",
        fontSize=9.5,
        leading=12,
        alignment=1,
        textColor=colors.HexColor("#4A5568"),
    )

    author_style = ParagraphStyle(
        "AuthorStyle",
        parent=styles["Normal"],
        fontName="Helvetica-Bold",
        fontSize=10.5,
        leading=13.5,
        alignment=1,
        textColor=colors.HexColor("#2D3748"),
    )

    h1_style = ParagraphStyle(
        "Heading1_Custom",
        parent=styles["Heading1"],
        fontName="Helvetica-Bold",
        fontSize=12,
        leading=15,
        textColor=colors.HexColor("#1E3A8A"),
        spaceBefore=8,
        spaceAfter=4,
        keepWithNext=True,
    )

    h2_style = ParagraphStyle(
        "Heading2_Custom",
        parent=styles["Heading2"],
        fontName="Helvetica-Bold",
        fontSize=10.5,
        leading=13,
        textColor=colors.HexColor("#2B6CB0"),
        spaceBefore=6,
        spaceAfter=3,
        keepWithNext=True,
    )

    body_style = ParagraphStyle(
        "Body_Custom",
        parent=styles["BodyText"],
        fontName="Helvetica",
        fontSize=9,
        leading=12.5,
        textColor=colors.HexColor("#2D3748"),
        spaceBefore=2,
        spaceAfter=3,
    )

    caption_style = ParagraphStyle(
        "CaptionStyle",
        parent=styles["Normal"],
        fontName="Helvetica-Oblique",
        fontSize=8,
        leading=10.5,
        alignment=1,
        textColor=colors.HexColor("#4A5568"),
        spaceBefore=2,
        spaceAfter=5,
    )

    code_style = ParagraphStyle(
        "CodeStyle",
        parent=styles["Normal"],
        fontName="Courier",
        fontSize=8,
        leading=10.5,
        textColor=colors.HexColor("#1A202C"),
        backColor=colors.HexColor("#EDF2F7"),
        spaceBefore=2,
        spaceAfter=3,
        leftIndent=8,
        rightIndent=8,
    )

    abstract_style = ParagraphStyle(
        "AbstractStyle",
        parent=styles["Normal"],
        fontName="Helvetica-Oblique",
        fontSize=8.5,
        leading=12,
        alignment=4,
        textColor=colors.HexColor("#2D3748"),
        leftIndent=15,
        rightIndent=15,
        spaceBefore=4,
        spaceAfter=6,
    )

    story = []

    # ================= PAGE 1 =================
    story.append(Paragraph("<b>UNIVERSIDADE FEDERAL DE MINAS GERAIS</b>", header_style))
    story.append(Paragraph("INSTITUTO DE CIÊNCIAS EXATAS &mdash; DEPARTAMENTO DE CIÊNCIA DA COMPUTAÇÃO", header_style))
    story.append(Paragraph("Robótica Móvel &mdash; 2º Semestre de 2026 | Prof. Douglas G. Macharet", header_style))
    story.append(Spacer(1, 6))
    story.append(Paragraph("Trabalho Prático 3 &mdash; Exploração e Mapeamento", title_style))
    story.append(Spacer(1, 4))
    story.append(Paragraph("<b>Aluno:</b> Isaac Reyes &nbsp;|&nbsp; <b>Matrícula:</b> 2022069843 &nbsp;|&nbsp; <b>Email:</b> isaac.reyalves@gmail.com", author_style))
    story.append(Spacer(1, 4))
    story.append(HRFlowable(width="100%", thickness=1, color=colors.HexColor("#CBD5E0"), spaceBefore=2, spaceAfter=6))

    story.append(Paragraph("<b>Resumo:</b> Este relatório apresenta o desenvolvimento e a análise experimental do Trabalho Prático 3 da disciplina de Robótica Móvel (DCC/UFMG). O trabalho consiste na implementação do algoritmo de <i>Occupancy Grid Mapping</i> com representação em <i>log-odds</i> e traçado de raios por Bresenham, integrado a uma estratégia reativa de navegação autônoma para o robô diferencial Kobuki equipado com sensor laser planar fastHokuyo no simulador CoppeliaSim. Foram conduzidos experimentos comparativos no cenário estático para três resoluções espaciais (0,01 m, 0,10 m e 0,50 m), seguidos de avaliações em múltiplos pontos de partida nos cenários estático e dinâmico (com pedestre móvel), além de um estudo de sensibilidade ao ruído estocástico das leituras. Os resultados demonstram alta fidelidade geométrica, robustez e eficiência em tempo real (&lt;10 ms por ciclo de atualização).", abstract_style))

    story.append(Paragraph("1. Introdução", h1_style))
    story.append(Paragraph(
        "O problema de exploração e mapeamento simultâneo é fundamental na robótica móvel. O objetivo central é capacitar um robô autônomo a percorrer um ambiente desconhecido e construir uma representação espacial em grade de ocupação (<i>Occupancy Grid</i>). A localização do robô é assumida perfeitamente conhecida ao longo do tempo (obtida diretamente da simulação via RemoteAPI), permitindo foco rigoroso na integração das observações do sensor laser e no algoritmo de navegação reativa.",
        body_style
    ))
    story.append(Paragraph(
        "<b>Robô Kobuki:</b> Plataforma móvel diferencial cujos parâmetros cinemáticos calibrados são o raio das rodas <i>r</i> = 0,035 m e o baseline <i>L</i> = 0,230 m. O modelo diferencial converte a velocidade linear <i>v</i> e angular <i>&omega;</i> nas velocidades angulares das rodas esquerda e direita por: &omega;<sub>l</sub> = (<i>v</i> &minus; &omega;<i>L</i>/2)/<i>r</i> e &omega;<sub>r</sub> = (<i>v</i> + &omega;<i>L</i>/2)/<i>r</i>.",
        body_style
    ))
    story.append(Paragraph(
        "<b>Sensor Laser Planar fastHokuyo:</b> Montado coaxialmente sobre a base do robô (x=0, y=0, z=0,072 m), gerando 684 feixes em um campo de visão de 180&deg; (&minus;&pi;/2 a +&pi;/2 radianos) com alcance útil máximo de 5,0 m.",
        body_style
    ))
    story.append(Paragraph(
        "<b>Cenários Avaliados:</b> Cenário estático (<code>cena-tp3-estatico.ttt</code>), composto por sala de 10&times;10 m com paredes e móveis, e cenário dinâmico (<code>cena-tp3-dinamico.ttt</code>), contendo o pedestre humanóide Bill caminhando ciclicamente pelo salão.",
        body_style
    ))

    story.append(Paragraph("2. Execução e Ambiente de Desenvolvimento", h1_style))
    story.append(Paragraph(
        "O ambiente foi estruturado em Python 3.13 gerenciado via <b>UV</b>. O simulador CoppeliaSim Edu 4.10 roda em container Docker com aceleração por GPU dedicada NVIDIA RTX 3050 via CDI (Container Device Interface) comunicando-se por ZMQ na porta 23000.",
        body_style
    ))
    story.append(Paragraph("<code>uv run tp3</code> &nbsp;&mdash;&nbsp; Executa a exploração interativa padrão.", code_style))
    story.append(Paragraph("<code>uv run python scripts/run_all_experiments.py</code> &nbsp;&mdash;&nbsp; Executa toda a suíte de testes.", code_style))
    story.append(Paragraph("<code>uv run jupyter lab notebooks/tp3_exploration_and_mapping.ipynb</code> &nbsp;&mdash;&nbsp; Abre o notebook interativo.", code_style))

    story.append(Paragraph("3. Estratégia de Navegação e Controle", h1_style))
    story.append(Paragraph(
        "A navegação autônoma emprega uma máquina de estados reativa com três setores angulares: Setor Frontal (&minus;25&deg; a +25&deg;), Setor Esquerdo (+25&deg; a +85&deg;) e Setor Direito (&minus;85&deg; a &minus;25&deg;).<br/>"
        "&bull; <b>Desvio Ativo:</b> Se a distância mínima frontal for &lt; 0,70 m, o robô cessa o avanço frontal e gira (&omega; = &plusmn;1,0 rad/s) para o lado mais desobstruído.<br/>"
        "&bull; <b>Avanço com Viés de Parede (Wandering):</b> Com frente livre, avança a <i>v</i> = 0,30 m/s. Se uma parede lateral for detectada a &lt; 0,50 m, aplica-se correção suave de direção para margear superfícies de forma ordenada.<br/>"
        "&bull; <b>Manobra Anti-Aprisionamento (Escape):</b> O deslocamento é monitorado a cada 40 iterações. Se for &lt; 0,08 m, o robô reverte a marcha por 0,5 s seguida de giro em pivô aleatório entre 90&deg; e 150&deg;, evitando bloqueios em quinas côncavas.",
        body_style
    ))

    story.append(PageBreak())

    # ================= PAGE 2 =================
    story.append(Paragraph("4. Implementação do Occupancy Grid", h1_style))
    story.append(Paragraph(
        "<b>Estrutura de Dados e Discretização:</b> O mapa é modelado como matriz 2D em NumPy com limites espaciais [&minus;5,5, +5,5] m. A conversão de coordenadas contínuas do mundo (x<sub>w</sub>, y<sub>w</sub>) para índices inteiros de coluna e linha (col, row) é dada por: col = &lfloor;(x<sub>w</sub> &minus; x<sub>min</sub>)/&Delta;&rfloor; e row = &lfloor;(y<sub>w</sub> &minus; y<sub>min</sub>)/&Delta;&rfloor;, onde &Delta; é o tamanho da célula.",
        body_style
    ))
    story.append(Paragraph(
        "<b>Modelo em Log-Odds:</b> Cada célula armazena <i>l</i>(m<sub>i</sub>) = ln(<i>p</i>/(1 &minus; <i>p</i>)). O prior neutro é <i>p</i><sub>0</sub> = 0,5 (<i>l</i><sub>0</sub> = 0). A cada varredura: células de espaço livre recebem &Delta;<i>l</i><sub>free</sub> = ln(0,40/0,60) &approx; &minus;0,405; a célula de impacto recebe &Delta;<i>l</i><sub>occ</sub> = ln(0,85/0,15) &approx; +1,735. Trunca-se <i>l</i>(m<sub>i</sub>) no intervalo simétrico [&minus;5,0, +5,0] para evitar saturação numérica. A probabilidade final é obtida por: <i>p</i>(m<sub>i</sub>) = 1 &minus; 1/(1 + exp(<i>l</i>(m<sub>i</sub>))).",
        body_style
    ))
    story.append(Paragraph(
        "<b>Traçado de Raios por Bresenham:</b> Utilizou-se o algoritmo de Bresenham inteiro em 2D para enumerar todas as células livres atravessadas por cada feixe laser, garantindo execução ultrarrápida (&lt;10 ms para 342 feixes subsaturados) sem operações lentas em ponto flutuante na grade.",
        body_style
    ))

    story.append(Paragraph("5. Testes e Resultados Experimentais", h1_style))
    story.append(Paragraph("5.1 Experimento 1: Avaliação do Tamanho da Célula (0,01 m, 0,10 m, 0,50 m)", h2_style))
    story.append(Paragraph(
        "Avaliamos três resoluções de célula no cenário estático para a mesma rota percorrida pelo robô (Figura 1):",
        body_style
    ))

    if os.path.exists("results/exp1_cell_size_comparison.png"):
        story.append(Image("results/exp1_cell_size_comparison.png", width=520, height=173))
        story.append(Paragraph("<b>Figura 1:</b> Mapeamento para três tamanhos de célula: &Delta;=0,01m (esquerda), &Delta;=0,10m (centro) e &Delta;=0,50m (direita).", caption_style))

    story.append(Paragraph(
        "<b>Discussão e Escolha:</b><br/>"
        "&bull; <i>&Delta; = 0,01 m (Fina):</i> Alta definição em quinas e pernas de cadeiras, porém matriz volumosa (1100&times;1100) e presença de vazios radiais onde raios não passaram.<br/>"
        "&bull; <i>&Delta; = 0,50 m (Grossa):</i> Severo efeito de aliasing; paredes de 50 cm de espessura bloqueiam passagens estreitas e móveis viram blocos indistinguíveis.<br/>"
        "&bull; <i>&Delta; = 0,10 m (Ideal):</i> <b>Definido como o melhor valor para os testes seguintes.</b> Paredes nítidas e contínuas, geometria de móveis perfeitamente reconhecível e excelente eficiência computacional.",
        body_style
    ))

    if os.path.exists("results/exp1_static_overview.png"):
        story.append(Image("results/exp1_static_overview.png", width=480, height=205))
        story.append(Paragraph("<b>Figura 2:</b> Visão geral do mapeamento estático com &Delta;=0,10m: Trajetória do robô e pontos captados pelo laser (esquerda) vs Occupancy Grid (direita).", caption_style))

    story.append(PageBreak())

    # ================= PAGE 3 =================
    story.append(Paragraph("5.2 Experimento 2: Cenário Estático em Posições Iniciais Distintas", h2_style))
    story.append(Paragraph(
        "Com &Delta; = 0,10 m, testou-se o robô partindo de dois pontos distintos no cenário estático: Posição 1 em (&minus;4,0, &minus;4,0) m e Posição 2 em (+3,0, +3,0) m (Figuras 3 e 4).",
        body_style
    ))

    if os.path.exists("results/exp2_static_pos1_overview.png"):
        story.append(Image("results/exp2_static_pos1_overview.png", width=490, height=210))
        story.append(Paragraph("<b>Figura 3:</b> Cenário Estático &mdash; Posição Inicial 1: (&minus;4,0, &minus;4,0) m.", caption_style))

    if os.path.exists("results/exp3_static_pos2_overview.png"):
        story.append(Image("results/exp3_static_pos2_overview.png", width=490, height=210))
        story.append(Paragraph("<b>Figura 4:</b> Cenário Estático &mdash; Posição Inicial 2: (+3,0, +3,0) m.", caption_style))

    story.append(Paragraph(
        "Em ambos os casos, a navegação reativa conduziu o robô por diferentes alas da sala, gerando mapas perfeitamente consistentes e demonstrando independência da pose de largada.",
        body_style
    ))

    story.append(PageBreak())

    # ================= PAGE 4 =================
    story.append(Paragraph("5.3 Experimento 3: Cenário Dinâmico em Posições Iniciais Distintas", h2_style))
    story.append(Paragraph(
        "No cenário dinâmico, o pedestre Bill caminha pelo salão. Realizaram-se testes partindo de (&minus;4,0, &minus;4,0) m e (+3,0, +3,0) m (Figuras 5 e 6).",
        body_style
    ))

    if os.path.exists("results/exp4_dynamic_pos1_overview.png"):
        story.append(Image("results/exp4_dynamic_pos1_overview.png", width=490, height=210))
        story.append(Paragraph("<b>Figura 5:</b> Cenário Dinâmico com pedestre móvel &mdash; Posição Inicial 1: (&minus;4,0, &minus;4,0) m.", caption_style))

    if os.path.exists("results/exp5_dynamic_pos2_overview.png"):
        story.append(Image("results/exp5_dynamic_pos2_overview.png", width=490, height=210))
        story.append(Paragraph("<b>Figura 6:</b> Cenário Dinâmico com pedestre móvel &mdash; Posição Inicial 2: (+3,0, +3,0) m.", caption_style))

    story.append(Paragraph(
        "<b>Impacto do Obstáculo Móvel:</b> Como o Occupancy Grid clássico assume premissa estática, a passagem do pedestre cria rastros cinzentos transitórios de ocupação (visíveis na diagonal inferior da Figura 6). Contudo, assim que o pedestre se afasta e o laser varre novamente a área livre, as deduções sucessivas &Delta;<i>l</i><sub>free</sub> atenuam o rastro de volta para livre, demonstrando a adaptabilidade bayesiana da formulação saturada.",
        body_style
    ))

    story.append(PageBreak())

    # ================= PAGE 5 =================
    story.append(Paragraph("5.4 Experimento 4: Estudo de Sensibilidade ao Ruído Sensorial", h2_style))
    story.append(Paragraph(
        "Injetou-se ruído gaussiano aleatório na distância medida e ângulo de leitura em três regimes (Figura 7): Baixo (&sigma;<sub>r</sub> = 1 cm), Médio (&sigma;<sub>r</sub> = 5 cm) e Alto (&sigma;<sub>r</sub> = 15 cm).",
        body_style
    ))

    if os.path.exists("results/exp_noise_comparison.png"):
        story.append(Image("results/exp_noise_comparison.png", width=520, height=173))
        story.append(Paragraph("<b>Figura 7:</b> Sensibilidade ao ruído do sensor laser: Baixo ruído (esquerda), Médio ruído (centro) e Alto ruído (direita).", caption_style))

    story.append(Paragraph(
        "Com ruído de até 5 cm, a integração cumulativa de log-odds atua como um filtro passa-baixas eficiente que atenua ruídos de média zero e mantém as paredes nítidas. Sob ruído elevado (15 cm), observa-se espessamento artificial das fronteiras.",
        body_style
    ))

    story.append(Paragraph("6. Conclusões e Dificuldades Encontradas", h1_style))
    story.append(Paragraph(
        "Todos os requisitos do TP3 foram cumpridos com código simples, modular e altamente funcional. A resolução de 0,10 m ofereceu o melhor equilíbrio geral. A maior dificuldade técnica envolveu a extração correta de buffers de sinal binários (<code>getBufferSignal</code>) no CoppeliaSim 4.10 e a configuração da GPU dedicada NVIDIA via CDI no container Docker. Ambas as questões foram solucionadas, viabilizando execução fluida em alta performance.",
        body_style
    ))

    story.append(Paragraph("7. Bibliografia", h1_style))
    story.append(Paragraph("[1] H. P. Moravec and A. E. Elfes, \"High resolution maps from wide angle sonar,\" in <i>Proc. IEEE ICRA</i>, 1985, pp. 116-121.", body_style))
    story.append(Paragraph("[2] S. Thrun, W. Burgard, and D. Fox, <i>Probabilistic Robotics</i>. MIT Press, 2005.", body_style))
    story.append(Paragraph("[3] Yujin Robot, \"Kobuki Parameters and Documentation,\" [Online]: yujinrobot.github.io/kobuki.", body_style))
    story.append(Paragraph("[4] Coppelia Robotics, \"CoppeliaSim ZMQ Remote API Documentation,\" [Online]: manual.coppeliarobotics.com.", body_style))

    doc.build(story)
    print(f"Relatório PDF gerado com sucesso em: {filename}")


if __name__ == "__main__":
    build_pdf()
