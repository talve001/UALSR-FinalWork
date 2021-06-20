%Universidade Autónoma de Lisboa
%Sistemas de Robótica | Trabalho Final | Prof.: Laércio Cruvinel
%Robô Simples com 2 juntas - Cinemática Direta e Inversa
%Trabalho Realizado por:
%Miguel Lima, Aluno 30003444, Lic. Engenharia Informática
%Tiago Alves, Aluno 30003460, Lic. Engenharia Informática



mdl_KR5         % Cria o Robô
KR5             % Mostra os detalhes do Robô (Parametros DH)              
KR5.plot(qz)  %Plota o Robô no seu estado inicial  (qz = angulos a zero)

pos_A = [0 0 0 0 0 0] % Posição Inicial do robô (em qz)
pos_B = [0 pi/3 -pi/10 -pi/6 pi/5 pi/3] %posição final do robô
trajcd = jtraj(pos_B, pos_B, tempo) %calculo da trajetória
%plot(KR5, trajcd, 'movie', 'kukaKR5atob.mp4') % plota o cálculo da trajetória, assim como guarda a mesma em video
T = KR5.fkine(trajcd) %calculo da cinemática direta
cininv = KR5.ikine(T) %cálculo da cinemática inversa
cininvCotBaixo = KR5.ikine6s(T,'rd') %cálculo da cinemática inversa com o cotovelo para baixo
plot(KR5, cininv) %KR5 em cinemática inversa
plot(KR5, cininvCotBaixo) %KR5 em cinemática inversa com o cotovelo para baixo

tempo2 = [0:0.02:5]
% Coordenadas em X Y e Z
ponto1 = SE3(0.9, 0.2, 0.5);
ponto2 = SE3(.5,-0.5, -2); 
ponto3 = SE3(1.5, 1.5, 1.5);
ponto4 = SE3(0.5, 0.04, 0.5);
ponto5 = SE3(0.9, -5, 3);
ponto6 = SE3(-15, -10, -16);

% Definir a cinemática inversa para obter os ângulos a partir das
% coordenadas X Y e Z dos pontos acima criados
% vamos usar o método ikcon em vez de ikine, uma vez que este método está
% otimizado para o limite das juntas.
% https://www.petercorke.com/RTB/r9/html/SerialLink.html

q1 = KR5.ikcon(ponto1)
q2 = KR5.ikcon(ponto2)
q3 = KR5.ikcon(ponto3)
q4 = KR5.ikcon(ponto4)
q5 = KR5.ikcon(ponto5)
q6 = KR5.ikcon(ponto6)

%vamos definir as matrizes jacobianas, para garantir que não temos
%qualquer tipo de singularidade.
J1 = KR5.jacobn(q1)
det(J1)
J2 = KR5.jacobn(q2)
det(J2)
J3 = KR5.jacobn(q3)
det(J3)
J4 = KR5.jacobn(q4)
det(J4)
J5 = KR5.jacobn(q5)
det(J5)
J6 = KR5.jacobn(q6)
det(J6)
J7 = KR5.jacobn(pos_B)
det(J7)

%cálculo das trajetórias:

path=mtraj(@tpoly,pos_A,q1,tempo2); 
plot(KR5,path, 'trail', {"r", 'LineWidth', 8}, 'movie', 'kukaKR5Ato1.mp4'); %Posição A até Ponto1

path=mtraj(@tpoly,q1,q2,tempo2);
plot(KR5,path, 'trail', {"r", 'LineWidth', 8}, 'movie', 'kukaKR51to2.mp4'); % Posição Ponto1 até Ponto2
 
path=mtraj(@tpoly,q2,q3,tempo2);
plot(KR5,path, 'trail', {"r", 'LineWidth', 8}, 'movie', 'kukaKR52to3.mp4');% Posição Ponto2 até Ponto3

path=mtraj(@tpoly,q3,q4,tempo2);
plot(KR5,path, 'trail', {"r", 'LineWidth', 8}, 'movie', 'kukaKR53to4.mp4');% Posição Ponto3 até Ponto4

path=mtraj(@tpoly,q4,q5,tempo2);
plot(KR5,path, 'trail', {"r", 'LineWidth', 8}, 'movie', 'kukaKR54to5.mp4'); % Posição Ponto4 até Ponto5
 
path=mtraj(@tpoly,q5,q6,tempo2);
plot(KR5,path, 'trail', {"r", 'LineWidth', 8}, 'movie', 'kukaKR55to6.mp4'); % Posição Ponto5 até Ponto6

% e Finalmente..!
path=mtraj(@tpoly,q6,pos_B,tempo2);
plot(KR5,path, 'trail', {"r", 'LineWidth', 8}, 'movie', 'kukaKR56toB.mp4');% E finalmente. Posição Ponto6 até Posição B
