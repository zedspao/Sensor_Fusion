function resultado = height_g(cam) %Define a funcao

img_full = snapshot(cam); %Tira foto com camera ext. e armazena
im = imcrop(img_full,[199 41 300 480]); %Recorta a imagem fotografada anteriormente
[m,n,p] = size(im); %Armazena as dimensoes da foto

%%% procura o verde
im_robo_g = im; %Imagem do robo eh a foto tirada
area_robo_g=0; %Area do robo inicia zerada
x_robo_g=0; %Localizacao X inicia zerada
y_robo_g=0; %Localizacao Y inicia zerada

for i=1:m %Inicia a verredura em cada linha da imagem
    for j=1:n %Inicia varredura em cada coluna da imagem
        im_robo_g(i,j,1:3)=0;
        if im(i,j,1)<131 && im(i,j,2)>115 && im(i,j,3)<86 %Se os limites RGB da etiqueta forem encontrados:
            im_robo_g(i,j,2)=255;
            area_robo_g = area_robo_g+1; %Contabiliza o pixel;
            x_robo_g = x_robo_g+j; %Contabiliza a posicao X do pixel;
            y_robo_g = y_robo_g+i; %Contabiliza a posicao Y do pixel;
        else
            im_robo_g(i,j,2)=0;
        end
    end
end %Conclui as buscas em cada pixel da imagem

x_robo_g = round(x_robo_g/area_robo_g); %Divide a soma das coordenadas X pela quantidade de pixels encontrados
y_robo_g = round(y_robo_g/area_robo_g); %Divide a soma das coordenadas Y pela quantidade de pixels encontrados
resultado = y_robo_g; %Encontra a coordenada Y da etiqueta na imagem

end %Fim da funcao