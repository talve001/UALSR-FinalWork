%Universidade Autónoma de Lisboa
%Sistemas de Robótica | Trabalho Final | Prof.: Laércio Cruvinel
%Robô Simples com 2 juntas - Cinemática Direta e Inversa
%Trabalho Realizado por:
%Miguel Lima, Aluno 30003444, Lic. Engenharia Informática
%Tiago Alves, Aluno 30003460, Lic. Engenharia Informática

mdl_planar2 %cria o Robô
load hershey %carrega a fonte hershey

D = hershey{'S'} % Define a letra que vamos usar
D.stroke % desenho da letra
path = [1*D.stroke; zeros(1, numcols(D.stroke))] % caminho da caneta, cada NaN indica o fim de um segmento
k = find(isnan(path(1, :))) % escala do caminho
path(:, k) = path(:, k-1) % caminho

traj = mstraj(path(:,2:end)', [2 2 2], [], path(:,1)', 0.02, 0.2) % converte a trajetória num movimento continuo
tempo = numrows(traj) * 0.02 % vetor tempo, ou seja, vai levar cerca de 7 segundos a desenhar a letra
Tp = SE3(0, 0, 0) * SE3(traj) * SE3.oa([0 1 0], [0 0 -1]) % Orientação da caneta
q = p2.ikine(Tp, 'mask', [1 1 0 0 0 0]) % Aplicação da Cinemática Inversa

plot3(traj(:, 1), traj(:, 2), traj(:, 3)) % faz o plot das coordenadas no espaço a 3 dimensões
grid
p2.plot(q, 'trail', {'y', 'LineWidth', 4}, 'movie', '2DOF_S.mp4') % faz plot do Robô e exporta a animação para um ficheiro de vídeo

D1 = hershey{'R'} % Define a letra que vamos usar
D1.stroke % desenho da letra
path1 = [1*D1.stroke; zeros(1, numcols(D1.stroke))] % caminho da caneta, cada NaN indica o fim de um segmento
k1 = find(isnan(path1(1, :))) % escala do caminho
path1(:, k1) = path1(:, k1-1) % caminho

traj1 = mstraj(path1(:,2:end)', [2 2 2], [], path1(:,1)', 0.02, 0.2) % converte a trajetória num movimento continuo
tempo1 = numrows(traj1) * 0.02 % vetor tempo, ou seja, vai levar cerca de 7 segundos a desenhar a letra
Tp1 = SE3(0, 0, 0) * SE3(traj1) * SE3.oa([0 1 0], [0 0 -1]) % Orientação da caneta
q1 = p2.ikine(Tp1, 'mask', [1 1 0 0 0 0]) % Aplicação da Cinemática Inversa

plot3(traj1(:, 1), traj1(:, 2), traj1(:, 3))
grid
p2.plot(q1, 'trail', {'y', 'LineWidth', 4}, 'movie', '2DOF_R.mp4')
