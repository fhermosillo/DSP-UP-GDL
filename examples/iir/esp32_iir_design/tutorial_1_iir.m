%% FILTROS IIR. #1 Diseño de filtros IIR mediante la Transformada Bilineal
% A diferencia de los filtros FIR (finite impulse response), los filtros 
% IIR (infinite impulse response) se caracterizan por tener una respuesta
% al impulso infinita en duración. La ecuación general que describe el 
% comportamiento de un filtro IIR de orden N, es la siguiente:
%
%   y(n) = b[0]*x(n) + b[1]*x(n-1) + ... + b[M]x(n-M) - a[1]y(n-1) - ... -
%   a[n-N]y(n-N)
%
% donde M <= N
%
% Como se puede apreciar en la ecuación anterior, la salida o respuesta de
% un filtro IIR depende tanto de los valores pasados de la entrada como de
% la salida. Es por ello que usualmente se les conoce como filtros
% recursivos, ya que usan recursión respecto a la salida para calcular el
% estado actual del sistema.
%
% Para diseñar filtros IIR, usualmente se sigue con la metodología:
% 1. Especificar los parámetros del filtro (tipo del filtro, frecuencias de
%    corte, frecuencia de muestreo, etc).
% 2. Diseñar el filtro en tiempo continuo (prototipo analógico)
% 3. Obtener la función de transferencia H(s)
% 4. Discretizar H(s) por medio de técnicas como la Transformada Bilineal
%    a fin de obtener H(z).
% 5. Obtener la ecuación en diferencias que caracteriza el comportamiento
%    de H(s) en el dominio discreto.
%
% Vamos a ver un ejemplo sencillo, para ello vamos a diseñar un filtro IIR
% basandonos en el prototipo analógico del filtro pasa-bajas RC.
%
% 1. Especificación: Se va a discretizar el diseño de un filtro pasa-bajas 
% RC de orden 1, con frecuencia de corte de 100Hz para una frecuencia de 
% muestreo Fs=1000Hz
Fs=1000;    % Frecuencia de muestreo
Ts=1/Fs;    % Periodo de muestreo
Fc=100;     % Frecuencia de corte
Wc=2*pi*Fc; % Frecuencia de corte

% FREQUENCY PREWARPING: ESTA PARTE ES OPCIONAL
% fc=Fc/Fs; % Normalizamos
% Wc=(2/Ts)*tan(pi*fc); % Prewarping

% 2 y 3. Prototipo analógico: Por medio del análisis de circuitos, es posible 
% llegar a la función de transferencia H(s) [Revisar notas "Bilinear Transform"]
%
%   H(s) = Ωc/(s + Ωc)
% para Ωc = 1/(RC), con R resistencia y C capacitancia
%
% De un análisis en respuesta en frecuencia, se sabe que la frecuencia de
% corte de este filtro esta dada por Ωc.
%
% Entonces, Ωc = 2πFc = 2π100 = 1/(RC).
%
% 4. Transformación Bilineal, reemplazamos s = (2/Ts) * (1 - z^-1) / (1 + z^-1)
% Con ello obtenemos H(z), i.e., H(z) = H(s=(2/Ts) * (1 - z^-1) / (1 + z^-1))
% Con Ts = 1/Fs = 1/1000 = 0.001
%
% La función de transferencia discreta
%             1 + z^-1
%   H(z) = K ----------
%             a - z^-1
%           ΩcTs              2 + ΩcTs
% con K = ----------,  y a = ----------
%          2 - ΩcTs           2 - ΩcTs


% 5. Ecuación en diferencias: A partir de H(z) podemos deconstruirla para
% obtener la relación de la función de transferencia:
%         Y(z)       1 + z^-1
% H(z) = ------ = K ----------
%         X(z)       a - z^-1
%
% Y(z) [a - z^-1] = X(z) K [1 + z^-1]
%
% Si invertimos la Transformada Z, obtenemos
%
%   ay(n) - y(n-1) = Kx(n) + Kx(n - 1)
%
% Despejando para y(n)
%           K          K            1
%   y(n) = ----x(n) + --- x(n-1) + --- y(n-1)
%           a          a            a
% Podemos simplificar esta  ecuación un poco más:
%  1     2 - ΩcTs        K       ΩcTs
% --- = ----------,     --- = ---------
%  a     2 + ΩcTs        a     2 + ΩcTs
%
% Resultando en la ecuación en diferencias final:
% Ωc = 2π100, Ts = .001
% ΩcTs = (2π100)(0.001) = 0.628318531
% 2 - ΩcTs = 2 - (2π100)(0.001) = 1.37168147
% 2 + ΩcTs = 2.62831853
%
% Sustituyendo:
%       y(n) = 0.2391 x(n) + 0.2391 x(n-1) - 0.5219 y(n-1)
b0 = (Wc*Ts)/(Wc*Ts + 2);
b1 = (Wc*Ts)/(Wc*Ts + 2);
a1 = (Wc*Ts - 2)/(Wc*Ts + 2);

% Esa es la ecuación que implementariamos en el microcontrolador
%
% Por ejemplo, asumiremos que tenemos una señal 
%       x(t) = sin(2π50t) + sin(2π250t)
% La cual es discretizada por un ADC (simulado por el for)
% Frecuencias de la señal de entrada x(n)
N=200;  % Número de muestras a procesar
% Señal de entrada y salida (solo simulación)
x=zeros(1,N);
y=zeros(1,N);
F1 = 50;
F2 = 250;

% Simulación del filtro 
% Memoria de valores pasados de la entrada y la salida
xprev=0; % x(n-1)
yprev=0; % y(n-1)
% Procesamiento de la señal muestra a muestra
for n = 0:N-1 % Simulación de la discretización
    % Discretizamos x(t)
    x(n+1) = sin(2*pi*(F1/Fs)*n) + sin(2*pi*(F2/Fs)*n);

    % Aplicamos el filtro
    y(n+1) = b0*x(n+1) + b1*xprev - a1*yprev;

    % Actualizamos los valores pasados
    xprev = x(n+1);
    yprev = y(n+1);
end

% Graficamos la entrada y salida
close all;
figure;
stem(0:N-1, x);
hold on
stem(0:N-1, y);
hold off
legend('x(n)','y(n)');

% La respuesta en frecuencia nos determina el comportamiento que tendra
% nuestro filtro en frecuencia.

% Calculamos la respuesta en frecuencia del filtro diseñado
a=[1,a1];   % Coef. de y(n)
b=[b0,b1];  % Coef. de x(n)
[Hw,F]=freqz(b,a,2000,Fs);

% Calculamos la transformada de fourier de x(n), y(n) con las mismas
% dimensiones de la respuesta en frecuencia.
[Xw,~]=freqz(x,1,2000,Fs);
[Yw,~]=freqz(y,1,2000,Fs);


% Graficamos H(w) y X(w) y lo normalizamos X(w) entre su longitud en tiempo
% discreto
figure;
L=length(x);
subplot(2,1,1);
plot(F,abs(Xw)/L); % Necesita normalizar
hold on
plot(F,abs(Hw)); % Ya esta normalizado
hold off;
legend('|X(\omega)|','|H(\omega)|');
xlabel('Frecuencia (F)');

% Graficamos Y(w)
subplot(2,1,2);
plot(F,abs(Yw)/L);  % Necesita normalizar
ylabel('|Y(\omega)|');
xlabel('Frecuencia (F)');
ylim([0,1]);

% Como se observa, la respuesta en frecuencia del filtro atenua la primer
% frecuencia (F1) en un factor de 0.8988 (ubique en la curva anaranjada el 
% cursor en la frecuencia de 50Hz), por lo que en la salida y(n) su
% amplitud (A=1) se vera atenuada por este factor.
%
% Para la segunda frecuencia (F2=250), la respuesta en frecuencia nos
% determina que la atenuación que tendra en esta frecuencia sera de 0.3090,
% en este caso la amplitud original era de "1" por lo que despues de pasar
% por el filtro, se espera que su amplitud sea A*0.3090 = 0.3090

% En resumen, el filtro cumple con su cometido, sin embargo la atenuación
% que se percibe en el rango de frecuencias que se deseaban atenuar se
% pueden mejorar al conectar un segundo filtro pasa-bajas en cascada, con
% las mismas caracteristicas que el original.


% Finalmente la respuesta en frecuencia del filtro RC analógico es
%               H(jΩ) = H(s=jΩ) = Ωc/(jΩ + Ωc)
% A fin de graficarla y compararla con la H(ω) calculada anteriormente
% necesitamos definir el vector de frecuencia analógica, la cual a fin de
% tener el mismo rango [0,Fs/2]
W = linspace(0, 2*pi*Fs/2, length(Hw));
Ha=Wc./(1i*W + Wc);

% Graficamos H(Ω)[Del filtro analogico] y H(ω)[Del filtro digital]
% La teoria nos dice que debemos tener una buena aproximación de H(Ω) en H(ω)
figure;
plot(F,abs(Ha));
hold on
plot(F,abs(Hw));
hold off
legend('|H(\Omega)|','|H(\omega)|');

% Mostramos los coeficientes que deben de ser puestos en el Microcontrol.
fprintf('const uint32_t IIR_FILTER_TAPS = %d\n',length(a));
fprintf('const float iir_a_coefs[IIR_FILTER_TAPS] = {');
for i = 1:length(a)
    fprintf('%.20f\t',a(i));
end
fprintf('\b}\n');
fprintf('const float iir_b_coefs[IIR_FILTER_TAPS] = {');
for i = 1:length(b)
    fprintf('%.20f\t',b(i));
end
fprintf('\b}\n');

% En el siguiente tutorial vamos a aprender como diseñar filtros en Matlab
% sin necesidad de utilizar explicitamente la transformación bilineal.

%% Experimentación
% 1. Diseñe un filtro pasa-altas de primer orden basado en el prototipado 
% analógico Rc Recuerde que si quiere diseñar un filtro pasa-altas basado 
% en el prototipo pasa-bajas debe de aplicar la siguiente ecuación:
%
%       Hhp(s) = 1 - H(s)
%
% Escriba aqui su respuesta en frecuencia:
%                   NUM
%     Hhp(s) = -----------------
%                   DEN

% 2. Simule el filtro para una frecuencia de corte Fc=500Hz y una frecuencia
% de muestreo Fs=2000Hz.

% 3. Aplique una señal de prueba
%   x(t) = sin(2π100t) + sin(2π750t)

% 4. Corrobore los resultados mediante la el diagrama de magnitud de la
% respuesta en frecuencia. ¿Que atenuación espera recibir la frecuencia de
% 750Hz? ¿Corresponde la teoria con la práctica?