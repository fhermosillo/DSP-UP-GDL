% Signal definition
F=100;          % Frequency
Fs=8000;        % Sampling frequency
f = F/Fs;       % Digital frequency
[k,N]=rat(f);   % f=k/N

n = 0:N-1;          % Time vector
x = sin(2*pi*f*n);  % Signal

% Plot signal
stem(n,x);
grid on;



%% Impulse response
M=15;
nh = -(M-1)/2:(M-1)/2; 
Fc=1000;
wc = 2*pi*Fc/Fs; 
htrun = (wc/pi)*sin(wc*nh)./(wc*nh);
htrun(nh==0) = wc/pi;

% Plot impulse response
stem(nh,htrun);
grid on;

%% Apply system
y = conv(x,htrun);
stem(x);
hold on
stem(y);
hold off
grid on
legend('x(n)','y(n)','fontsize',15);
xlabel('Time (n)')
ylabel('Amplitude')

%% Truncated Frequency Response
Ns=8000; % Frequency resolution = Fs/Ns,
         % this is the smallest frequency
         % that will be detected in freqz
freqz(htrun,1,8000,Fs); % This will plot frequency response
                        % Magnitude: |H(w)|^2
                        % Phase: Phi(H(w)) in radians
[Htrun,F]=freqz(htrun,1,Ns,Fs); % This return frequency response

% Magnitude response
Hmag=abs(Htrun);    % |H(w)|
Fx=100;
fprintf('|H(%.1f)| = %.4f\n', Fx, Hmag(F==Fx))

% Phase response
Hphase = angle(Htrun); % Phi(H(w))
fprintf('Phi(H(%.1f)) = %.2f °\n', Fx, rad2deg(Hphase(F==Fx)));