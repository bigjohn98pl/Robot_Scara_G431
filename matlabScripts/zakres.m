close all;
a = [200, 130];
l1=a(1); %d³ugoœæ pierwszego cz³onu
l2=a(2); %d³ugoœæ drugiego cz³onu

J1_max = 140;
J2_max = 125;
theta1=deg2rad(-J1_max):0.02:deg2rad(J1_max); %tworzenie wektora k¹ta obrotu J1 co 0.1 radiana
theta2=deg2rad(-J2_max):0.02:deg2rad(J2_max); %tworzenie wektora k¹ta obrotu J2 co 0.1 radiana

[THETA1, THETA2]=meshgrid(theta1, theta2); %tworzenie tablicy k¹tów

X=l1*cos(THETA1)+l2*cos(THETA1+THETA2); %obliczamy dla ka¿dego k¹ta wartoœæ wspó³rzêdnej X
Y=l1*sin(THETA1)+l2*sin(THETA1+THETA2); %obliczamy dla ka¿dego k¹ta wartoœæ wspó³rzêdnej Y

data1=[X(:) Y(:) THETA1(:)]; %tworzenie tablicy wspó³rzêdnych dla konkretnego theta1
data2=[X(:) Y(:) THETA2(:)]; %tworzenie tablicy wspó³rzêdnych dla konkretnego theta2

figure(1)
x = [300 200 100 0, 330 230 130 30, 270 170 70 -30 , 200 200 200 200];
y = [0 -100 -200 -300, 0 -100 -200 -300, 0 -100 -200 -300, 100 0 -100 -200]
plot(X(:),Y(:), 'r.',330,0,'bx',x,y,'kx');
axis([-350 350 -350 350]);
axis equal;
grid on;
xlabel('X [mm]','fontsize', 15)
ylabel('Y [mm]','fontsize', 15)
title('Wykres osi¹galnych punktów ramienia, zakres ruchów');
legend('zakres','punkt pocz¹tkowy [330,0]','pocz¹tek uk³adu wspó³rzêdnych')

