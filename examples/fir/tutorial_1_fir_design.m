%% Tutorial 10. Diseño de filtros FIR por su la respuesta en frecuencia
% En teoria se planteo la metodología de diseño de un filtro FIR por medio
% de la respuesta en frecuencia ideal de los filtros clásicos, conocida
% como la tecnica de ventaneo:
% 1. Pasa-bajas (elimina todas las componentes frecuenciales que sean 
% mayores a cierta frecuencia de corte Fc)
% 2. Pasa-altas (elimina todas las componentes frecuenciales que sean 
% menores a cierta frecuencia de corte Fc)
% 3. Pasa-bandas (elimina todas las componentes frecuenciales que sean 
% menores a a cierta frecuencia de corte Fc1 y que sean mayores a Fc2)
% 4. Rechaza-bandas: elima todas las componentes frecuenciales que sean
% mayores a cierta frecuencia de corte Fc1 y que sean menores que Fc2
%
%
% El proceso se describe a continuación:
% #1. Especificar el parametro de la frecuencia de muestreo Fs. Este 
% parametro delimitara el intervalo de frecuencias que van a poder
% representarse en el dominio de la frecuencia. Dicho intervalor esta dado  
% desde -Fs/2 hasta Fs/2. Por ejemplo, si Fs= 1000, las únicas frecuencias
% que van a poder representarse en el dominio de la frecuencia, son desde
% -500 hasta 500 Hz.
% #2. Establecer el tipo de filtro que se va a diseñar, así como la o las
% frecuencias de corte que se requieran. Obviamente dichas frecuencias de
% corte estan limitadas por la frecuencia de muestreo, es decir Fc <= Fs/2.
% #3. Normalizar las frecuencias de corte Fc por medio de la frecuencia de
% muestreo. Para normalizarla simplemente dividimos entre la frecuencia de
% muestreo: fc = Fc/Fs. Recuerde que fc debe estar especificado como el 
% un cociente de dos numeros enteros, y dicha frección debe ser lo más 
% posible reducible. Finalmente, se procede a multiplicar a f por 2*pi, a 
% fin de obtener la frecuencia de corte angular, es decir wc = 2*pi*fc.
% #4. Diseñar el filtro en el dominio de la frecuencia, en función del tipo
% de filtro especificado en (#2). Con ello se obtendra una respuesta en
% frecuencia H(w) ideal, la cual debera de contener cambios abruptos 
% (transiciones entre 0 y 1) localizados en los puntos de wc.
% #5. Aplicar la transformada inversa de Fourier de tiempo discreto (IDTFT)
% a fin de obtener la respuesta al impulso ideal h(n).
% #6. Especificar el orden M, que nuestro filtro tendra, donde M debe ser
% un número impar.
% #7. Evaluar los valores de n = -(M-1)/2 hasta n = (M-1)/2 de la respuesta
% al impulso h(n). Con ello se obtendran M valores o coeficientes.
% #8. Con los coeficientes h(-(M-1)/2) hasta h((M-1)/2), se forma la
% ecuación en diferencias del filtro:
% y(n) = h(-(M-1)/2)*x(n) + h(-(M-1)/2 + 1)*x(n-1) + ... + h((M-1)/2)*x(n-M+1)
%
% Y con ello, nuestro filtro puede ser implementado en un microcontrolador.
%



% Vamos a diseñar un filtro por medio de esta técnica.
% #1. Para ello especificaremos que la frecuencia de muestreo es de 1000Hz,
% por ello, el intervalo de frecuencias sera de -500 a 500 Hz.
Fs = 1000;

% #2. Se diseñara un filtro pasa-bajas con una frecuencia de corte Fc = 120Hz.
Fc=120;

% #3. Normalizamos a Fc, fc = Fc/Fs = 120/1000 = 12/100 = 6/50. Luego se
% multiplica por 2PI para obtener wc = 2*pi*6/50 = pi*12/50 = pi*6/25
fc = Fc/Fs;
wc = 2*pi*fc;

% #4. Al ser un filtro pasabajas, tiene la forma de un cuadrado centrado en
% w=0 y esta representado por la siguiente ecuación:
%        { 1, |w| < wc
% H(w) = {
%        { 0, otro caso
%
% Para visualizarlo, primero contruimos un vector de frecuencia, el cual
% debera de estar a cotado desde -pi a pi:
N=1024; % Cuantos valores se quieren observar
w = -pi:(2*pi)/N:pi;
H=zeros(1,N+1);
H(abs(w) < wc) = 1;
plot(w,abs(H),'LineWidth',2);
xlabel('Frecuencia (w)');
ylabel('Amplitud |H(w)|');
grid on

% #5. Al aplicar la IDTFT se obtiene la respuesta al impulso
%           h(n) = wc/pi * sinc(wc*n)

% #6. Se selecciona el orden del filtro M = 5
M = 5;

% #7. Se evalua a h(n) en los valores de n = -(5-1)/2 hasta (5-1)/2
% Recuerde que:
% 1. sinc(wc*n) = sin(wc*n) / (wc*n)
% 2. sinc(wc*n) = 1, en n = 0
n=-(M-1)/2:(M-1)/2;
h = (wc/pi)*sin(wc*n)./(wc*n);
h(n==0) = wc/pi;


% Graficaremos además la respuesta al impulso truncada
stem(n,h,'LineWidth',2);
xlabel('Tiempo (n)');
ylabel('Amplitud (x(n))');
grid on; % Visualizar las rejillas del area de la gráfica

% Además vamos a visualizar la nueva respuesta en frecuencia de h(n)
% truncada, para ello volvemos a visualizar H(w) y traslapamos la H(w)
% truncada
N=1024; % Cuantos valores se quieren observar
w = -pi:(2*pi)/N:pi;
H=zeros(1,N+1);
H(abs(w) < wc) = 1;
Ht = fftshift(fft(h,N+1));
plot(w,abs(H),'LineWidth',2);
hold on;
plot(w,abs(Ht),'LineWidth',2);
hold off;
xlabel('Frecuencia (w)');
ylabel('Amplitud |H(w)|');
legend('H(w)','Ht(w)')
grid on

% Observe que ya no tenemos la misma respuesta en frecuencia, pero nuestro
% filtro se asemejó lo más que se pudo a la respuesta en frecuencia ideal

% #8. Los coeficientes del filtro son
fprintf('const uint32_t FIR_TAPS = %d;\n',length(h));
fprintf("const float h[FIR_TAPS] = {");
fprintf("%.20fF,\t",h(1:end-1));
fprintf("%.20fF};\n",h(end));
% Copiar estos valores en el archivo .c para su implementacion en la ESP32
% Si se desea incluir mas filtros, asegurarse de cambiar los nombres de 
% "FIR_TAPS" y "h" en cada filtro

% A fin de evaluar nuestro filtro, vamos a definir una señal que estara 
% definida como la suma de dos señales senoidales de frecuencias F1=50Hz y 
% F2 = 200Hz. Vamos a discretizar las primeras 50 muestras
n = 0:49;
F1=50;
F2=200;
x1 = sin(2*pi*(F1/Fs)*n);
x2 = sin(2*pi*(F2/Fs)*n);
x = x1 + x2;

% Para aplicar el filtro, puede usar la función "conv" que le permite
% calcular la convolucion entre dos señales: y(n) = x(n)*h(n)
y = conv(x,h);

% Graficamos las señales
subplot(2,2,1:2);
stem(n,x,'LineWidth',2);
hold on
stem(y,'LineWidth',2);
hold off
xlabel('Tiempo (n)');
ylabel('Amplitud (x(n))');
grid on; % Visualizar las rejillas del area de la gráfica
legend('x(n)','y(n)');

subplot(2,2,3);
stem(n,x1,'LineWidth',2);
hold on
stem(y,'LineWidth',2);
hold off
xlabel('Tiempo (n)');
ylabel('Amplitud (x(n))');
grid on; % Visualizar las rejillas del area de la gráfica
legend('sin(w_1 n)','y(n)');

subplot(2,2,4);
stem(n,x2,'LineWidth',2);
hold on
stem(y,'LineWidth',2);
hold off
xlabel('Tiempo (n)');
ylabel('Amplitud (x(n))');
grid on; % Visualizar las rejillas del area de la gráfica
legend('sin(w_2 n)','y(n)');

% Observe que la señal filtrada tiene mayor similitud con la señal seno
% de frecuencia de 50 Hz, debido a que al pasar por nuestro filtro, toda
% señal senoidal de frecuencia superior a 120 Hz será atenuada por el
% filtro pasa-bajas diseñado.

%% Experimentación
% 1. Reproduzca el experimento anterior para distintos valores de M:
%    a) Para M = 11
%    b) Para M = 21
%    c) Para M = 51
%    d) Para M = 101
% ¿Que efectos prácticos tiene el valor de M?

% 2. Diseñe ahora un filtro pasa-bandas utilizando la misma Fs que en el
% experimento pasado pero para las frecuencias de corte de Fc1 = 500 y Fc2 = 600
% Asuma que el orden del filtro es M = 11. Visualice los mismos graficos
% que en el experimento anterior.


