%Universidade Autónoma de Lisboa
%Sistemas de Robótica | Trabalho Final | Prof.: Laércio Cruvinel
%Robô Simples com 2 juntas - Cinemática Direta e Inversa
%Trabalho Realizado por:
%Miguel Lima, Aluno 30003444, Lic. Engenharia Informática
%Tiago Alves, Aluno 30003460, Lic. Engenharia Informática



mdl_KR5                     % Cria o Robô
KR5.tool = SE3(0, 0, 0.2)   % Cria o comprimento da ferramenta

s = 0.2                     % Desenha o tamanho do quadrado

p = [s s 0 pi/2 % inicio
    -s s 0 pi/2 % lado1
    -s s 0 0    % roda
    -s -s 0 0   % lado2
    -s -s 0 pi/2% roda
     s -s 0 pi/2% lado3
     s -s 0 0   % roda
     s  s 0 0   % lado4
     ]

t1 = 5  % tempo para cada lado
t2 = 1  % tempo cada rodação

% Cria a trajetória, e a matriz com cada linha considerando (x, y, z,
% theta)
pt = mstraj(p, [], [t1 t2 t1 t2 t1 t2 t1], [], 0.1, 0.4)

% Translação para cada ponto da trajetória
X = SE3([0.5 0 -s])

% Cada ponto na trajetória, faz a traslação, define o eixo do z para baixo
% e roda o "gripper" do eixo z
Tt = SE3(pt(:,1:3)) * X * SE3.Ry(pi) * SE3.Rz(pt(:,4))

qt = KR5.ikine6s(Tt, 'qn') % Cinemática Inversa com o cotovelo para cima

ps = X * p(1:2:end,1:3)' % Posição dos cantos do quadrado, e pontos ímpares

clf
plot_sphere(ps, 0.03, 'b') % cria a esfera

KR5.plot(qt, 'trail', {'y', 'LineWidth', 5}, 'nowrist', 'view', [138 8], 'movie', 'kukaKR5sphere.mp4')