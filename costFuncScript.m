%% hasankaantuna // 19015036 // MPC Cost Function

%% Reference
% V. P. Mark Misin, “LPV MPC Control of an Autonomous Aerial Vehicle,” 
% 2020 28th Mediterranean Conference on Control and Automation (MED),
% France, 2020.

function [Hdb,Fdbt,Cdb,Adb] = costFuncScript(Ad,Bd,Cd,Dd,hz)
    
    Q=[100 0;0 0.1];
    S=[1 0;0 1];
    R=100;
    
    A_aug=[Ad,Bd;zeros(length(Bd(1,:)),length(Ad(1,:))),eye(length(Bd(1,:)))];
    B_aug=[Bd;eye(length(Bd(1,:)))];
    C_aug=[Cd,zeros(length(Cd(:,1)),length(Bd(1,:)))];
    D_aug=Dd;
    
    CQC=C_aug'*Q*C_aug;
    CSC=C_aug'*S*C_aug;
    QC=Q*C_aug; 
    SC=S*C_aug;
    
    Qdb=zeros(length(CQC(:,1))*hz,length(CQC(1,:))*hz);
    Tdb=zeros(length(QC(:,1))*hz,length(QC(1,:))*hz);
    Rdb=zeros(length(R(:,1))*hz,length(R(1,:))*hz);
    Cdb=zeros(length(B_aug(:,1))*hz,length(B_aug(1,:))*hz);
    Adb=zeros(length(A_aug(:,1))*hz,length(A_aug(1,:)));
    
    for i = 1:hz
       if i == hz
           Qdb(1+length(CSC(:,1))*(i-1):length(CSC(:,1))*i,1+length(CSC(1,:))*(i-1):length(CSC(1,:))*i)=CSC;
           Tdb(1+length(SC(:,1))*(i-1):length(SC(:,1))*i,1+length(SC(1,:))*(i-1):length(SC(1,:))*i)=SC;           
       else
           Qdb(1+length(CQC(:,1))*(i-1):length(CQC(:,1))*i,1+length(CQC(1,:))*(i-1):length(CQC(1,:))*i)=CQC;
           Tdb(1+length(QC(:,1))*(i-1):length(QC(:,1))*i,1+length(QC(1,:))*(i-1):length(QC(1,:))*i)=QC;
       end
       
       Rdb(1+length(R(:,1))*(i-1):length(R(:,1))*i,1+length(R(1,:))*(i-1):length(R(1,:))*i)=R;
       
       for j = 1:hz
           if j<=i
               Cdb(1+length(B_aug(:,1))*(i-1):length(B_aug(:,1))*i,1+length(B_aug(1,:))*(j-1):length(B_aug(1,:))*j)=A_aug^(i-j)*B_aug;
           end
       end
       Adb(1+length(A_aug(:,1))*(i-1):length(A_aug(:,1))*i,1:length(A_aug(1,:)))=A_aug^(i);
    end
    Hdb=Cdb'*Qdb*Cdb+Rdb;
    Fdbt=[Adb'*Qdb*Cdb;-Tdb*Cdb];
end