close all %Fechar graficos
clear all %Limpar variaveis


%--------DESEMPENHO DE CONTROLE----------
figure; %Inicializa plotagem do desempenho de controle
subplot(2,1,1);
hz1 = animatedline('Color', 'k', 'LineWidth', 1);
hz2 = animatedline('Color', 'r', 'LineWidth', 1);
title('Desempenho de controle para seguimento de SP');
legend('Altura do drone','SetPoint');
xlabel('Tempo (em segundos)');
ylabel('Altura (em metros)');
hold on;
grid minor;
ylim([0 2.2]);
yticks([0:0.2:2.2]);


%---------------ACAO DE CONTROLE---------
subplot(2,1,2); %Plotagem da acao de controle
hz3 = animatedline('Color', 'b', 'LineWidth', 1);
title('Acao de controle (U)');
xlabel('Tempo (em segundos)');
ylabel('Altura (em metros)');
hold on;
grid minor;
ylim([-2 2.4]);
yticks([-2:0.2:2.4]);


pause(3); %Espera abrir os graficos
r = ryze; %Estabelece comunicacao com o drone

% Parametros de controle
Kp = input('Informe o ganho proporcional (Kp > 0):'); %Solicita informacao do ganho proporcional
sp = input('Informe o valor do SetPoint (0.5 a 1.5):'); %Solicita informacao do SetPoint

takeoff(r); %Decolagem automatica do drone
tObj = tic; %Inicia temporizacao
t = toc(tObj); %Coleta tempo de amostragem

while t < 15 %Executa o loop de controle por 70 segundos

%height_IR = []; %Zera a variavel de altura
dadoIR = [];
dadoBar = [];

while (numel(dadoIR) == 0) && (numel(dadoBar) == 0)

dadoIR = readHeightIR(r);
dadoBar = readHeight(r);
end

%Corrige a leitura

h = dadoIR + 0.06800479*(dadoBar-dadoIR);

% variaveis do processo
pv = h;
erro = sp - pv;

% Acao de controle
u0 = Kp*erro;

if (u0 >= 0.2) && (u0 <= 5)
u = u0;
moveup(r,'distance',u,'WaitUntilDone',false)
end

if (u0 <= -0.2) && (u0 >= -5)
u = u0*-1;
movedown(r,'distance',u,'WaitUntilDone',false)
u = u0;
end

% Plotar dados em tempo real
t = toc(tObj);

addpoints(hz1, t, h);
addpoints(hz2, t, sp);
addpoints(hz3, t, u);

end
land(r); %Pousa o drone apos tempo estabelecido