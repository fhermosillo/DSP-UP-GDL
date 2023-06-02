%% FILTROS IIR #2 Diseño de filtros IIR en Matlab
% En el tutorial pasado, se aplico la metodología de diseño de filtros IIR 
% mediante el uso de la transformada Bilineal de manera explicita.
% Recordando que a partir de una función de transferencia H(s) diseñada en
% tiempo continuo, es posible aplicar un mapeo que permita relacionar los 
% dominios "s" y "z" a travez de la aproximación:
%
%                    2  1 - z^-1
%               s = --- ---------
%                    T  1 + z^-1
%
% En este tutorial vamos a ver una forma implicita para el diseño de
% filtros IIR en Matlab. Esta tecnica solo será aplicable en filtros 
% específicos como Butterworth y Chebyshev, sin embargo, como veremos al 
% final existe otra función de que permitira aplicar la transformada
% bilineal en cualquier función de transferencia H(s).

% Los filtros Butterworth se caracterizan por minimizar los risos en la
% banda de paso de un filtro digital. Esto es permiten una máxima
% planicidad en esta zona (donde no ocurre atenuación). En matlab, se
% encuntra la función "butter" para hacer el cálculo de los coeficientes
% asociados al filtro. Para más detalles sobre como diseñar un filtro
% Butterworth vease la nota asociada a los filtros IIR.
%
% El prototipado de la función butter es la siguiente:
% [b,a] = butter(M,Wn,ftype,'s')
% 
% • "M" especifica el orden del filtro
% • "Wn" la frecuencia de corte (digital). Si queremos diseñar un filtro
%   digital Butterworth, Wn debe estar normalizada entre 0 y 1 por medio
%   de la frecuencia de Nyquist (Fs/2).
% • "ftype" especifica el típo de filtro {'low', 'high', 'bandpass', 'stop'}
% • 's', si se anexa la bandera 's' (entre comillas simples), el diseño del
%   filtro será en tiempo continuo y no discreto.
% 
% La función retorna los valores de coeficientes asociados a la entrada en 
% el vector "b", y los coeficientes asociados a la salida en el vector "a".
% b -> [b0 b1 b2 ... bM]
% a -> [a0 a1 a2 ... aM]
% Donde típicamente a0 esta normalizado a "1",
%
% Finalmente, la ecuación en diferencias asociada al diseño de este filtro
% estará dado por:
% y(n) = b0 x(n) + b1 x(n-1) + .... + bn x(n-M) - a1 y(n-1) + ... + aM y(n-M)
%
% Siendo esta la ecuación a implementar (siempre y cuando no se haya 
% especificado la bandera 's').
close all;

%% Ejemplo: 
% Diseñar un filtro butterworth digital de orden 2 para una frecuencia de 
% corte de 100Hz a una frecuencia de muestreo de 1000Hz.
Fs=1000;
Fc=100;
Fn=Fs/2; % Frecuencia de Nyquist
M=2;
[b,a]=butter(M,Fc/Fn,'low');

% Evaluamos su respuesta en frecuencia de 2000 muestras
[Hw,F]=freqz(b,a,2000,Fs);

% Graficamos su diagrama de magnitud
figure;
plot(F,abs(Hw));
xlabel('Frecuencia (F)');
ylabel('Magnitud |H(F)|');

% Vamos a compararlo con un filtro FIR bajo las mismas condiciones
bfir=fir1(M,Fc/Fn,'low');
[Hwfir,~]=freqz(bfir,1,2000,Fs);
figure;
plot(F,abs(Hw));
hold on
plot(F,abs(Hwfir));
hold off
legend('IIR Butterworth','FIR');
xlabel('Frecuencia (F)');
ylabel('Magnitud');

% Como se puede observar en la figura 2, la banda de transición entre la
% zona de paso y la de rechazo tiene una transición más abrupta en el
% filtro butterworth a comparación del filtro FIR, aun cuando el orden del 
% filtro es muy pequeño. Esto tiene ventajas respecto a la eficiencia
% computacional.

% NOTA. Los filtros pasa-banda y rechaza-banda generán el doble de
% coeficientes requeridos, esto por las propiedades teoricas de diseño de
% estos filtros. Por ejemplo si se especifica que M=2, la cantidad de
% coeficientes obtenidos tanto para "a" como para "b" serán 2*M + 1 = 5
% coeficientes.
% Ejemplo
F1=750;
F2=1250;
Fs=44100;
Fn=Fs/2;
M=2;
[b,a]=butter(M,[F1/Fn,F2/Fn],'bandpass');
fprintf("Numero de coeficientes: %d\n", length(a));

% Respuesta en frecuencia (2000 muestras)
[Hw,F]=freqz(b,a,2000,Fs);
figure;
subplot(2,1,1);
plot(F,abs(Hw));
xlabel('Frecuencia (F)');
ylabel('Magnitud |H(F)|');
subplot(2,1,2);
plot(F,(angle(Hw)));
xlabel('Frecuencia (F)');
ylabel('Fase \Phi[H(F)]');

% Ejecutar estas lineas paso por paso
% Vamos a cargar un audio para ver los efectos de este filtro
[x,xFs]=audioread('guitar_1.wav'); 	% Asegurarse que el archivo "guitar_1.wav" se
									% encuentre en la misma carpeta que este script
input('Archivo guitar_1 cargado, presiona ENTER para continuar...');

% Vamos a ver si la frecuencia de muestreo usada para discretizar este
% audio corresponde con nuestra frecuencia de muestreo usada
fprintf('Audio.Fs=%d\n',xFs);

% Escucharemos primero la señal de audio y posteriormente la vamos a
% procesar
input('Presiona ENTER para escuchar la señal original...');
sound(x,xFs);


% Procesamos la señal por medio de nuestro filtro Butterworth pasa-bandas 
% diseñado a las frecuencias de corte de 750 a 1250. Esto permitira atenuar
% todas aquellas frecuencias que se encuentren fuera de este rango de
% frecuencias.
y = filter(b,a,x); % Filtrado en terminos de los coeficientes
input('Presione ENTER para escuchar la señal filtrada...');

% Escucharemos la señal de audio procesada
sound(y,xFs);
% Los cambios son más perceptibles con este filtro IIR respecto a los
% filtros FIR, para orden o numero de coeficientes más pequeños


% Es de resaltar nuevamente que con pocos coeficientes se logra una
% respuesta en frecuencia muy cercana a la ideal. Sin embargo es de
% restaltar que esto es unicamente en cuando a magnitud. En la fase, notese
% que la respuesta en fase presenta un comportamiento no lineal. Esto
% quiere decir que para dos frecuencias distintas no se van a ver
% afectados por el mismo cambio de fase en la salida del filtro. Por
% ejemplo si la entrada previo a ser discretizada es:
%       x(t) = sin(2π430t) + sin(2π2500t)
% La salida del filtro sería aproximadamente
% y(t) = 0.08*sin(2π430t + 2.72rad) + 0.054*sin(2π2500t - 2.8rad)
% (Puede buscar en los diagramas de fase y magnitud los valores mas cercanos
% a las frecuencias F=430 y F=2500 y sus valores de fase y magnitud asociados)
%
% Notese que el cambio de fase no es lineal, esto ocaciona que la señal de
% salida presente distorsiones respecto a su forma intrinseca.
%
% Esto tiende a afectar a señales que tengan una forma bien definida, como
% las señales ECG. 
%
% A fin de corregir estos cambios no lineales en la fase, se utilizan otro
% típo de filtros conocimos como pasa-todo, los cuales como su nombre
% indica, no afectan la magnitud de ninguna frecuencia, ya que dejan pasar
% toda la seña. Por el contrario, afecta unicamente a la fase de la señal
% de entrada.
%
% Si se conecta en cascada el filtro IIR seguido de un filtro pasa-todo
% diseñado especificamente para corregir la fase no lineal del filtro IIR
% se elimina el efecto de no linealidad de la fase.
%

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


%% NOTA FINAL:
% Recurde especificar de manera correcta a Fs en dependencia del problema
% que se este abordando. Por ejemplo para procesamiento de audio la
% frecuencia de muestreo más comun es Fs=44100, o Fs=48000, la elección de
% la frecuencia de muestreo depende del hardware usado. En los laboratorios
% se ha usado Fs=44100 debido a la especificación del hardware.
%
% Así mismo, la frecuencia de corte siempre debe ser menor que la
% frecuencia de Nyquist, esto es Fn = Fs/2 y Fc < Fn, de otro modo se
% tendran problemas de alias en su filtro y la frecuencia de corte en 
% realidad no sería Fc, sino su frecuencia alias.