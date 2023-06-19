clc %Limpa os comandos
close %Fecha os graficos
clear %Limpa variaveis

n = input('Numero (inteiro) do experimento :'); %Identificacao do experimento
experimento = sprintf('experimento %d.mat', n);

r = ryze; %Estabelece comunicacao com o drone
cam = webcam(1); %Habilita a camera externa
tObj = tic; %Inicia tempo de experimento
bat_inicial = r.BatteryLevel; %Armazena informacao da bateria
takeoff(r); %Decola o drone
pause(5); %Espera 5 segundos

y_verde = height_g(cam); %Armazena coordenada y da etiqueta do drone
y_cm(1) = ((-(y_verde - 440)/439)*220)/100; %Converte altura da etiqueta em metros

tStamp(1) = toc(tObj); %Coleta tempo de amostragem
heightIR = []; %Zera as leituras iniciais
heightbar = [];

while numel(heightIR) ~= numel(tStamp) %Ler infravermelho
    heightIR(1) = readHeightIR(r); %Precisa garantir que a leitura seja feita
end

while numel(heightbar) ~= numel(tStamp) %Ler barometrico
    heightbar(1) = readHeight(r); %Precisa garantir que a leitura seja feita
end

i = 2;
for i=2:8 %Inicia 7 subidas de 20cm

    moveup(r,'distance',0.2); %Sobe 20cm
    pause(3); %Espera 3 segundos para estabilziar o drone

    y_verde = height_g(cam); %Armazena coordenada y da etiqueta do drone
    y_cm(i) = ((-(y_verde - 440)/439)*220)/100; %Converte altura da etiqueta em metros

    tStamp(i) = toc(tObj); %Coleta tempo de amostragem

    while numel(heightIR) ~= numel(tStamp) %Ler infravermelho
        heightIR(i) = readHeightIR(r); %Precisa garantir que a leitura seja feita
    end

    while numel(heightbar) ~= numel(tStamp) %Ler barometrico
        heightbar(i) = readHeight(r); %Precisa garantir que a leitura seja feita
    end

end

i = 9;
for i=9:15 %Inicia 7 descidas de 20cm

    movedown(r,'distance',0.2); %Desce 20cm
    pause(3); %Aguarda 3 segundos para o drone estabilizar

    y_verde = height_g(cam); %Armazena coordenada y da etiqueta do drone
    y_cm(i) = ((-(y_verde - 440)/439)*220)/100; %Converte altura da etiqueta em metros

    tStamp(i) = toc(tObj); %Coleta tempo de amostragem

    while numel(heightIR) ~= numel(tStamp) %Ler infravermelho
        heightIR(i) = readHeightIR(r); %Precisa garantir que a leitura seja feita
    end

    while numel(heightbar) ~= numel(tStamp) %Ler barometrico
        heightbar(i) = readHeight(r); %Precisa garantir que a leitura seja feita
    end

end

land(r); %Pousa a aeronave
time_end = toc(tObj); %Armazena duracao do voo
bat_final = r.BatteryLevel; %Armazena informacao final da bateria

figure %Abre nova figura
hold on
plot(tStamp,heightbar,'*','Color','b'); %Plota dados do barometrico para o experimento
plot(tStamp,heightIR,'*','Color','r'); %Plota dados do infravermelho para o experimento
plot(tStamp,y_cm,'*','Color','g'); %Plota dados da camera externa para o experimento
title('Medicoes da altura');
legend('Trena','Barometro','Infravermelho','Camera externa');
ylabel('Altura (m)');
xlabel('Tempo (s)');
saveas(gcf,'plot'); %Salva o plot do experimento
save(experimento); %Salva todos os dados do experimento

X = sprintf('-FIM DE EXPERIMENTO \n-Bateria restante: %d %; \n-Duracao de voo: %f segundos;',bat_final, time_end);
disp(X)