%% hasankaantuna // 19015036 // Trajectory Creator Function

function [psi_ref,X_ref,Y_ref]=trajectoryCreator(t, x_dot,trajectoryType)

if trajectoryType==1
    x=linspace(0,x_dot*t(end),length(t));
    y = 3 * tanh(t - t(end)/2)+3;
    
else
    x=linspace(0,x_dot*t(end),length(t));
    y=-9*ones(1,length(t));
end

dx=x(2:end)-x(1:end-1);
dy=y(2:end)-y(1:end-1);

psi=zeros(1,length(x));
psiInt=psi;

psi(1)=atan2(dy(1),dx(1));
psi(2:end)=atan2(dy(:),dx(:));
dpsi=psi(2:end)-psi(1:end-1);

psiInt(1)=psi(1);
for i = 2:length(psiInt)
    if dpsi(i-1)<-pi
        psiInt(i)=psiInt(i-1)+(dpsi(i-1)+2*pi);
    elseif dpsi(i-1)>pi
        psiInt(i)=psiInt(i-1)+(dpsi(i-1)-2*pi);
    else
        psiInt(i)=psiInt(i-1)+dpsi(i-1);
    end
end


X_ref = [t' x'];
Y_ref = [t' y'];
psi_ref = [t' psiInt'];

end
