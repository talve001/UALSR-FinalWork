%Universidade Autónoma de Lisboa
%Sistemas de Robótica | Trabalho Final | Prof.: Laércio Cruvinel
%Robô Simples com 2 juntas - Cinemática Direta e Inversa
%Trabalho Realizado por:
%Miguel Lima, Aluno 30003444, Lic. Engenharia Informática
%Tiago Alves, Aluno 30003460, Lic. Engenharia Informática

mdl_planar2 % Cria o Robô
p2 % Mostra os dados do Robô
p2.plot(qz) % Mostra o Robô na posição inicial

m1 = p2.fkine([0 0]) % cinemática direta com os ângulos a zero
m2 = p2.fkine([pi pi/4]) % cinemática direta em pi pi/4
p2.plot([pi pi/4]) % Robô em pi pi/4

m3 = p2.ikine(m1, 'mask', [1 1 0 0 0 0]) % Cinemática inversa
m4 = p2.ikine(m2, 'mask', [1 1 0 0 0 0]) % Cinemática inversa
p2.plot(m3) % Robô em cinemática inversa pi pi/4

tempo = [0:0.01:5] % Definimos um vetor de tempo
q = jtraj(m3, m4, tempo) % Definimos a trajetória
p2.plot(q, 'trail', {'r', "LineWidth", 6}, 'movie', '2DOFatob.mp4') % Animação com a trajetória
