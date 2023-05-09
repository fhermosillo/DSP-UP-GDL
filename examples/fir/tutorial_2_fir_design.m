%% Segunda forma de diseñar filtros en Matlab
M=4;        % Orden deseado - 1, es decir el orden real será "4 + 1" = "5"
Fs=1000;    % Sampling freq
Fc=120;     % Cutoff freq
type='low'; % 'low', 'high', 'bandpass', 'stop'

% La tecnica se conoce como ventaneo debido a que trunca la respuesta al
% impulso por medio de una fincion w(n) conocida como ventaneo, la cual
% debe estar centrada en 0, y debera de contener tantos valores como el
% orden deseado de nuestro filtro, en este caso M+1. Finalmente, el filtro
% truncado ht(n) = h(n)w(n), es decir es igual al producto de h(n) con
% w(n). 
% Anteriormente, solo tomabamos los valores entre -(M-1)/2 y (M-1)/2, eso
% era puesto que w(n) era una señal cuadrada que valia 1 desde n = -(M-1)/2
% hasta (M-1)/2. En matlab, podemos especificar el tipo de ventana
% añadiendo los valores que toman en ese intervalo. Por ejemplo, a
% continuación se establece una ventana cuadrada
wn = ones(1,M+1);

% Luego, la función fir1 aplica el mismo metodo que en el experimento
% pasado, solamente resumido en una sola línea. Esta función requiere que
% la frecuencia de corte wc este normalizada de manera distinta a lo visto
% en teoria. Esta nueva normalización es wc = Fc/Fn, donde Fn = Fs/2 es la
% frecuencia de Nyquist.
Fn=Fs/2;    % Nyquist freq
wc=Fc/Fn;   % Digital cutoff  freq
h2 = fir1(M,wc,type,wn);

% Graficamos la respuesta en frecuencia ideal con la del filtro truncado
% con ventana cuadrada
N=1024;
H=zeros(1,N+1);
wc=2*pi*Fc/Fs;
H(abs(w) < wc) = 1;
w = -pi:(2*pi)/N:pi;
Ht = fftshift(fft(h2,N+1));
plot(w,abs(H),'LineWidth',2);
hold on;
plot(w,abs(Ht),'LineWidth',2);
hold off;
xlabel('Frecuencia (w)');
ylabel('Amplitud |H(w)|');
legend('H(w)','Ht(w)')
grid on

% Observe que la respuesta en frecuencia calculada por medio de los 8 pasos
% del tutorial pasado es aproximadamente igual a aquella calculada por la 
% función fir1.

% Otras ventanas posibles son las siguientes
% hamming
% hanning
% blackman
% Por ejemplo, si se quiere usar la ventana de hamming
wc=Fc/Fn;   % Digital cutoff  freq
h2_hamming = fir1(M,wc,type,hamming(M+1));

% Comparo las respuesta en frecuencia
N = 1024;
Ht2_hamming = fftshift(fft(h2_hamming,N+1));
plot(w,abs(H),'LineWidth',2);
hold on;
plot(w,abs(Ht),'LineWidth',2);
plot(w,abs(Ht2_hamming),'LineWidth',2);
hold off;
xlabel('Frecuencia (w)');
ylabel('Amplitud |H(w)|');
legend('H(w)','Ht2(w)','Hhamming(w)')
grid on

% Imprimir los coeficientes del filtro para su implementación en
% microcontrolador, se van a copiar estos valores en el archivo .c
fprintf('const uint32_t FIR_TAPS = %d;\n',length(h2_hamming));
fprintf("const float h[FIR_TAPS] = {");
fprintf("%.20fF,\t",h2_hamming(1:end-1));
fprintf("%.20fF};\n",h2_hamming(end));
% Si se desea incluir mas filtros, asegurarse de cambiar los nombres de 
% "FIR_TAPS" y "h" en cada filtro

%% Experimentación
% Cambie el tipo de ventana por hanning y luego por blackman, graficando
% las respuestas en frecuencia generadas por las cuatro ventanas vistas
% 1. Ventana rectangular
% 2. Ventana hamming
% 3. Ventana hanning
% 4. Ventana blackman
%