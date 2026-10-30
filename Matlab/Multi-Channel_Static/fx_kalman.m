function filtro = fx_kalman(filtro);
% Algoritmo que resuelve el filtrado de Kalman aplicado al ANC propagando
% la matriz de Covarianza P(n).


%%% INICIALIZACIÓN
% Extracción de los datos que definen el filtrado de Kalman
w=filtro.w;
P=filtro.P_n1;
F=filtro.F_n;
Q_n=filtro.Q_n;
R_n=filtro.R_n;


% Extracción de la señales y sistemas que intervienen en el algoritmo
e=filtro.e;
in=filtro.x;        % Señal de referencia
s=filtro.s;         % Estimaciones de los caminos acústicos 
buffS=filtro.buffS; % Estado de los filtrados por las estimaciones de los caminos acústicos


[N,I]=size(in);  % N-> Número de muestras del buffer de entrada de datos. I -> Número de señales de referencia
[M,J,K]=size(s); % M-> Tamaño de las estimaciones de los caminos secundariod, J- > Número de actuadores, ,K-> Número de sensores de error
[L,~,~]=size(w); % L-> Número de coeficientes del filtro
TamA=min(filtro.TamA,N);   % Número de muestras entre las que se realiza el procesado (tiempo entre actualización de los filtros)
w=w(:);


H_n=filtro.buffL;
xp=zeros(L,N,I,J,K);
for dim=1:N
xp(:,dim,:,:,:)=[H_n(N-dim+(1:L),:,:,:)];      % 'Matriz de medida'
end


% Reordenamos matriz de medida para que sea una matriz bidimensional de
% tamaño IJLxKN
x=zeros(I*J*L,K*N);

for i=1:I
    for j=1:J
        for k=1:K
           x(1+L*(i-1)+L*I*(j-1):i*L+L*I*(j-1),1+N*(k-1):N*k)=xp(:,:,i,j,k); 
        end
    end
end


% Calculamos x(n) a partir de la señal de entrada y de los caminos
% secundarios (señal de referecia filtrada)
% Estas operaciones no pertenencen al filtrado de Kalman en sí, sino que
% obedecen al problema de ANC que hay que resolver donde vamos a usar la
% estructura de filtrado-x (fx)

for nref=1:I
    for nsec=1:J
        for nmic=1:K
          [vijk(:,nref,nsec,nmic),buffS(:,nref,nsec,nmic)]=filter(s(:,nsec,nmic),1,in(1:TamA,nref),buffS(:,nref,nsec,nmic));  
        end
    end
end

buffL=[-vijk(end:-1:1,:,:,:);filtro.buffL(1:L+(N-TamA)-1,:,:,:)];


%%%


% COMIENZA ALGORITMO DE KALMAN


Pp = P + Q_n;
wp = w;
% Cálculo de ganancia de Kalman

Z=Pp*x;
S=x'*Z+R_n;
Kg = filtro.mu*Z/S;



% Actualización 

w = wp + Kg * e; 

filtro.w=reshape(w,L,I,J);  % Reestructuramos los datos en una matriz 3D de tamaño LxIxJ para que en la parte de la simulación acústica se generen la señales correctamente  
filtro.P_n1=Pp-Kg*x'*Pp;

%%%%  FIN DEL ALGORITMO DE KALMAN

filtro.buffL=buffL;
filtro.buffS=buffS;

end

