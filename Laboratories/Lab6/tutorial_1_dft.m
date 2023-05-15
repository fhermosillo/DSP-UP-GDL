%% Tutorial 5. Transformada Discreta de Fourier (DFT)
% La DFT prove una nueva representación para señales discretas en tiempo.
% Esta nueva representación tiene como dominio (variable de entrada) la
% frecuencia, mientras que el codominio (salida) es una representación de
% amplitud-fase que presenta dicha frecuencia en particular.
%
% Formalmente, la DFT para una señal discreta x(n) de longitud L (muestras)
% se define por medio de la sumatoria:
%           X(k) = sum(x(n)*exp(-j*(2*pi/N)*n*k), n = 0 a L-1)
% donde k va desde 0 hasta N-1
%
% Ahora bien, la DFT es una versión discreta de la Transformada de Fourier
% de Tiempo Discreto (DTFT), por lo cual, se tienen frecuencias discretas. 
% Es por ello que cada valor de k calculado, tendrá un valor de frecuencia
% asociado, el cual esta dado por:
%                       Fk=k*dF = k*(Fs/N)
% Donde a Fs/N se le conoce como resolución espectral, o la mínima
% frecuencia que puede ser detectada por la DFT.
%
% For ejemplo, si Fs=1000, y N = 10, se tiene que la resolución es 1000/10
% que es igual a 100, esto implica que cada valor de k va a estar en
% función de esta resolución, así por ejemplo cuando k = 1, la frecuencia
% asociada será 1*100 = 100, para k = 3, será 3*100 = 300 y así
% sucesivamente.
%
% Cabe decir que la frecuencia máxima detectada por la DFT es Fs/2, cuando
% nos pasamos de este valor (que es cuando k > N/2), la frecuencia asociada
% sufre un desplazamiento frecuencial dado por  
%                       Fk = k*dF - Fs
%
% Por ejemplo, para el caso de que N = 10, y Fs = 1000, cuando k > N/2 = 5,
% se desplaza la frecuencia asociada por Fs. En este caso, dF = Fs/N = 
% 100, y si k = 6, k >5 , por lo que la frecuencia asociada a k = 6 será
%                   F6 = 6*100 - 1000 = 600 - 1000 = -400
%
% A continuación se ilustra una tabla que relaciona a "k" con "Fk" para el
% ejemplo de Fs=1000 y N=10 (dF=100)
%
%   k : 0   1   2   3   4   5   6       7       8       9
%   Fk: 0   100 200 300 400 500 -400    -300    -200    -100
%
% Es por ello que comunmente se gráfica la DFT de la siguiente manera:
%   k:  6       7         8       9     0   1   2   3   4   5
%   Fk: -400    -300    -200    -100    0   100 200 300 400 500
% Matlab aplica estos cambios usando la función fftshift
%
% Finalmente, la DFT se puede usar para aplicar filtros en frecuencia para
% señales procesadas por bloques (no en tiempo real) dentro de un micro de
% forma eficiente.


%% Ejemplo
% Veamos un ejemplo, supongamos que discretizamos la señal 
%               x(t) = sin(2*pi*100*t)
% a una tasa de 1000 muestras por segundo (Fs = 1000Hz), la señal
% resultante será 
%               x(n) = sin(2*pi*(100/1000)*n) = sin((2*pi/10)*n)
% donde f = 1/10, k = 1, N = 100, es decir se tiene que el periódo de esta
% señal es N = 10 muestras por k = 1 ciclo.
F=10;
Fs=1000;
L=100;
n = 0:L-1;  % Observaremos un solo periodo
x = sin(2*pi*F/Fs*n);

% El espectro teórico esta dado por
%   X(w) = 0.5*j*[delta(F + 10) - delta(F - 10)]
% Es decir dos deltas centradas en la frecuencia de 10Hz
% Su magnitud es: 0.5*[delta(F + 10) + delta(F - 10)]
% Su fase es pi/2 para F + 10, y es -pi/2 para F - 10, fuera de esos valores 
% la fase vale "0".
%
% Para calcular la DFT, en matlab hacemos uso de la FFT (Fast Fourier Transform)
% Que es un algoritmo eficiente para el calculo de la DFT:
N=L;    % N >= L    (longitud de la DFT)
Xf = fft(x,N);

% Para definir nuestro vector de frecuencias Fk
dF = Fs/N;  % Resolución de frecuencia
if mod(N,2)==0
    F=-Fs/2: dF : (Fs/2-dF);
else
    F=-Fs/2: dF : Fs/2;
end

% Afin de desplazar el espectro Xf obtenido, usamos la función fftshift
Xf = fftshift(Xf);

% Finalmente graficamos el espectro obtenido en magnitud y en fase, debido
% a que X(k) es una señal compleja
% Magnitud:
% abs(Xf) == |X(k)|, se normaliza respecto a N a fin de obtener una
% aproximación de X(w)
subplot(2,1,1);
plot(F,abs(Xf)/L);
xlabel('Frecuencia (F)');
ylabel('Magnitud |X(F)|');
% Fase
subplot(2,1,2);
% atan es una función sensible a valores pequeños de su argumento, es decir
% presenta errores de calculo frente a valores pequeños de su argumento
im=imag(Xf);
im(abs(im) < 1e-3) = 0;
% "./" division elemento a elemento
plot(F,atan(im./real(Xf)));
xlabel('Frecuencia (F)');
ylabel('Fase \Phi[X(F)]');




%% Ejercicio 1
% Repita el experimento anterior, para una señal x(t) discretizada a una 
% tasa de 1000 muestras por segundo, si x(t) esta dada por
%               x(t) = sin(2*pi*10*t) + 0.1*sin(2*pi*50*t)
Fs=1000;
F1=10;
F2=50;
% 1. Determine los periodos fundamentales L1 y L2 de cada señal por separado
L1=;
L2=;
% 2. Establezca el parametro "L" como dos veces el periodo máximo de L1 y L2
L=2*max(L1,L2);
% 3. Defina la señal x(n) desde n = 0 hasta L-1
n=;
x=;
% 4. Calcule su DFT para N = L
Xf=;
% 5. Defina su vector de frecuencias asociado

% 6. Grafique sus espectro de magnitud y fase

% 7. Repita el paso 4 - 6, ahora usando N = 2*L, ¿Que sucede con el
% espectro de frecuencias?¿Porque?


%% Ejercicio 2
% 1. Para la señal anterior, agregue ruido a la señal x(n) con una potencia
% de 1/4 de la potencia de la señal, usando la función y = awgn(x,dB), donde 
% dB es la potencia del ruido a añadir:
%
% dB = -20*log10(Anoise/Asignal) = -20*log(r)
%
% Donde "Asignal" es la amplitud máxima de la señal x(n) y "Anoise" es la
% amplitud máxima que tendra el ruido, por ejemplo si dB = 0, se especifica
% que la potencia del ruido es igual a la potencia de la señal x(n)
% Si dB=6, la potencia del ruido es igual a la mitad de la potencia de la
% señal. "r" por otra parte representa la relación de las amplitudes, por
% ejemplo si r = 1,  la potencia del ruido es igual a la potencia de la
% señal x(n), si r = 0.5 la potencia del ruido es igual a la mitad de la
% potencia de la señal, etc.
dB=;    % Calcule cuantos dB's se requieren para r = 1/4
y = awgn(x,dB);

% 2. Grafique el espectro de la señal y(n)
% 3. ¿Que efectos en frecuencia tiene el ruido (magnitud y fase)?

% 4. Calcule el espectro de la señal y(n) sin desplazamiento por la función
% fftshift
Xk=fft(x,N);

% 5. Basandose en el ídice "k" asociado a las frecuencias de interes,
% haga cero a todos los indices k a excepción de los indices de las
% frecuencias de interes (k->Fk=10 y -10, 50 y -50), puede hacerlo
% calculando
%           k=mod(round(N*Fk/Fs) + N + 1, N);
%
k10=;   % k para Fk=10
km10=;   % k para Fk=-10
k50=;   % k para Fk=50
km50=;   % k para Fk=-50
k=[k10,km10,k50,km50];
% 6. Haga "cero" los valors de las componentes frecuenciales que no se 
% encuentren dentro de esos valores (filtrado)
kleft=setdiff(1:N,k);
Xk(kleft)=0;
% 7. Invierta el espectro resultante por medio de la función ifft,
yfilt = ifft(Xf);
% 8. Grafique la señal x(n) y la señal yfilt(n) en la misma grafica

% 9. ¿Que ocurrio con la señal yfilt(n)?