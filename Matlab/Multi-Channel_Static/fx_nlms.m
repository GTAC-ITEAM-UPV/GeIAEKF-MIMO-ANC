function filtro = fx_nlms(filtro)

s=filtro.s;
in=filtro.x;
e=filtro.e;
buffL=filtro.buffL;
buffS=filtro.buffS;
w=filtro.w;
mu=filtro.mu;

L=filtro.L;
[~,I]=size(in);
[~,J,K]=size(s);
[N,~]=size(in);
e=reshape(e,N,K);

% Adaptamos coeficientes
for n=1:N

    for nref=1:I
        
    for nsec=1:J
        paso=zeros(L,1);
        for nmic=1:K
            paso=paso+buffL((N-n)+[1:L],nref,nsec,nmic)*e(n,nmic);
        end
        
            w(:,nref,nsec)=w(:,nref,nsec)-(2*mu/(1e-8 + sum( squeeze(sum( buffL((N-n)+[1:L],nref,nsec,:).^2 , 1)) )))*paso; 
    end
    end
    
end

% filtramos señal de referencia

for nref=1:I
    for nsec=1:J
        for nmic=1:K
          %  buffS(:,nref,nsec,nmic)=[in(:,nref);buffS(1:end-1,nref,nsec,nmic)];
          %   vijk(:,nref,nsec,nmic)=(buffS(:,nref,nsec,nmic))'*s(:,nsec,nmic);
          [vijk(:,nref,nsec,nmic),buffS(:,nref,nsec,nmic)]=filter(s(:,nsec,nmic),1,in(:,nref),buffS(:,nref,nsec,nmic));  
        end
    end
end

buffL=[vijk(end:-1:1,:,:,:);buffL(1:L-1,:,:,:)];

%actualización
filtro.w=w;
filtro.buffL=buffL;
filtro.buffS=buffS;



end

