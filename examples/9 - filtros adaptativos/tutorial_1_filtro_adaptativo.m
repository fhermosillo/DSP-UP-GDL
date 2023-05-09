%% Diseño de un filtro adaptativo
% En teoria se vieron los filtros adaptativos, en los cuales se busca
% obtener una respuesta al impulso variante en tiempo, es decir que cambian
% los valores de sus coeficientes en cada instante de tiempo.
%
% Este tipo de filtros asume que existe una señal de referencia, la cual es
% la señal deseada d(n) a obtener al pasar la señal medida x(n) por nuestro
% filtro adaptativo h(n) que se asume de tener N coeficientes. El modelo de
% la señal aproximada es el siguiente:
%                   dh(n) = x(n) * h(n)
%
% Se conoce al filtro como adaptativo justamente por ello, dado que la 
% señal medida x(n) pudiera cambiar, el filtro necesita adaptarse a esos 
% cambios y proveer la mejor aproximación de la señal de referencia d(n). 
% Esto va a introducir un error por la aproximación no exacta, dado por
%                   e(n) = d(n) - dh(n) = d(n) - x(n) * h(n)
%
% Lo que busca el filtro adaptativo es minimizar el error cuadratico medio
%           min {E[e^2(n)]}
%
% Cuya solución vista en clase fue por medio del algoritmo steepest descent
%               h(n) = h(n-1) + 2*step*e(n)*z
%
% donde z es un vector que contiene las ultimas N muestras de la señal x(n)
%       z = [x(n),x(n-1),...,x(n-(N-1))]
% mientras que "step" es un parametro que regulariza la velocidad de
% aprendizaje de h(n), el cual en la práctica debe de tener un valor 
% relativamente pequeño para no desestabilizar el sistema

% Vamos a diseñar un filtro adaptativo para el siguiente problema:
% Se sabe que una señal s(n) debe de ser transmitida por cierto sistema
% de comunicación. Esta señal tiene la forma de una señal senoidal de 
% frecuencia F = 100Hz. En el receptor, la señal que se recibe es x(n) la
% cual por efectos del canal o del medio de transmisión, no llega justo
% como se transmitio, por el contrario siempre llega sumada con una señal 
% de ruido gaussiano w(n). Se desea entonces diseñar un filtro que permita
% extraer dicha señal de ruido w(n) de la señal que se transmitio por medio
% de un filtro adaptativo. Asuma que la frecuencia de muestreo es de 1000Hz
% y que la cantidad de muestras que llegaron al receptor fueron 300
Fs=1000;
F=100;
N=300;
n=(0:N-1)';
d=cos(2*pi*(F/Fs)*n);

% Señar recibida contaminada por ruido gausiano con una potencia de 0.7
x = 0.9*d + randn(size(d))*0.7;

% Graficamos ambas señales
figure;
plot(n,d,'LineWidth',2);
hold on
stem(n,d);
hold off
xlabel('Tiempo (n)');
ylabel('Amplitud (v)');
grid on;
legend('d(n)');
figure;
plot(n,x,'LineWidth',2);
hold on
stem(n,x);
hold off
xlabel('Tiempo (n)');
ylabel('Amplitud (v)');
grid on;
legend('x(n)');

% Suponga que el orden del filtro es 15
% Inicialización del filtro
M=15;           % Orden
hk=ones(M,1);   % Respuesta al impulso h(n)
e=zeros(N,1);   % Señal de eror e(n)
dh=zeros(N,1);  % Salida estimada d(n)

% Steepest descent
step = 0.05;
% Se accesa a las muestras conforme se esperarian recibir, a partir de la
% muestra que corresponde con el orden del filtro (M)
for n = M:N
    % Vector de los M valores pasados de x(n)
    xk = x(n:-1:n-M+1); % xk=[x(n),x(n-1),...,x(n-N+1)]

    % Calcular el error e(n) = d(n) - x(n)*h(n)
    e(n) = d(n) - xk'*hk;

    % Actualizar a h(n) = h(n-1) + 2*step*e(n)*xk
    hk = hk + 2*step*e(n)*xk;

    % Estimar la señal dh(n) = x(n) * h(n)
    dh(n) = xk'*hk;
end

% Graficar los resultados
subplot(2,2,1);
plot(d);
legend('d(n)');

subplot(2,2,2);
plot(x)
legend('x(n)');

subplot(2,2,3);
plot(d);
hold on
plot(dh)
hold off
legend('d(n)','dh(n)');

subplot(2,2,4);
plot(e);
legend('e(n)');