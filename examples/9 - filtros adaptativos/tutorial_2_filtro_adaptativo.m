%% Diseño de un filtro adaptativo II
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
% Una señal puede contener frecuencias entre 20Hz hasta 120Hz. Sin embargo,
% dicha señal siempre viene contaminada por una señal de 50Hz, la cual es
% una frecuencia que no se desea tener en esta señal. Se desea diseñar un
% filtro adaptativo que permita filtrar únicamente la señal de 50Hz. Asuma 
% que la frecuencia de muestreo es de 10000Hz y que la cantidad de muestras 
% que se adquieren son 5000. Ademas suponga que la señal muestreada es
% sin(2*pi*30*t) + sin(2*pi*50*t) + sin(2*pi*80*t) + sin(2*pi*100*t)
%
% Al final visualice las ultimas 1000 muestras
close all
F1 = 300;
F2=50;
F3=800;
F4=1000;
Fs=10000;
N=5000;
n=(0:N-1)';

% Señal latente (la que se desea obtener)
s = sin(2*pi*(F1/Fs)*n) + sin(2*pi*(F3/Fs)*n) + sin(2*pi*(F4/Fs)*n);

% En este caso, dado que la señal latente pudiera ser más compleja, nos
% combiene que la señal x(n) sea la de 50Hz
x=cos(2*pi*(F2/Fs)*n);

% Y que la señal de referencia sea la muestreada
d = s + 3*x;

% Es decir, se estan invirtiendo los papeles para este caso, puesto que va
% permitir que la señal de error sea la señal que capture la señal latente
% Si x(n) = cos(2*pi*(50/Fs)*n) y además, d(n) es la señal medida
% La señal de eror seria e(n) = d(n) - dh(n), donde dh(n) se aproximaria a
% la señal de 50Hz

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

% Suponga que el orden del filtro es 25
% Inicialización del filtro
M=25;           % Orden
hk=ones(M,1);   % Respuesta al impulso h(n)
e=zeros(N,1);   % Señal de eror e(n)
dh=zeros(N,1);  % Salida estimada d(n)

% Steepest descent
step = 0.0009;
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
figure;
subplot(2,2,1);
plot(s);
legend('s(n): señal latente');

subplot(2,2,2);
plot(x)
legend('x(n): señal de 50Hz');

subplot(2,2,3);
plot(d);
hold on
plot(dh)
hold off
legend('d(n)','dh(n)');

subplot(2,2,4);
plot(e);
legend('e(n): Señal latente estimada');

% En este ejercicio la señal de interes no es la deseada, sino la del error
% puesto que es a la que se le elimina la señal de frecuencia de 50Hz
figure;
plot(s);
hold on;
plot(e);
hold off
grid on;
legend('s(n)','e(n)')
ylim([-4,4])
xlim([4000,5000]);
% Otro ejemplo donde se puede plantear esto es para señales ecg, donde el
% ancho de banda de estas señales es de aproximadamente 150Hz, es decir la
% señal contempla señales desde 0 hasta 150Hz, sin embargo pudiera existir
% un acoplamiento electromagnetico por la señal de la red electrica que es
% de 60Hz con esta señal ECG, y termine montandose la señal de interes en
% la señal de 60Hz (ver grafico de la señal deseada en este caso)