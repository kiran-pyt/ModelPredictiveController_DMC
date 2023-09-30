clc
clear all
close all

G11=tf(-2.5,[20 1],'inputdelay',7);
G22=tf(2.5,[20 1],'inputdelay',7);
ts=5; n=24;                                     %Calculating step response coefficients for each transfer function
[y,t] = step(G11,0:ts:n*ts);
S11=y(2:end);
S12=zeros(n,1);
S21=zeros(n,1);
[y,t] = step(G22,0:ts:n*ts);
S22=y(2:end);

S=[];

for k=1:n
    S = [S;[S11(k),S12(k);S21(k),S22(k)]];     %Combining to form S matrix
end


[x,y]=size(S);
nu=y;   % Number of Inputs
ny=nu; % Number of outputs
%% control parmeters 
yref=[2;2];
duMax=[0.05;0.05];%0.05*ones(nu,1);     % Input rate constraint
duMin=-duMax;
umax =[1;0.5];%40*ones(nu,1);
umin=-umax;
m=4;
p=10;
%  %ymin1=yref(1)-0.05;
% % ymin2=yref(2)-0.05;
% % ymax1=yref(1)+0.05;
% % ymax2=yref(2)+0.05;
% yymin=[1;1];
% yymax=[3;2];
% Ymin=repmat(yymin,ny,2);
% Ymax=repmat(yymax,ny,2);
 

%% Initialization
maxTime=50;
uprev=zeros(nu,1);
Yk0=zeros(n*ny,1);
Y_SAVE=zeros(maxTime+1,ny);
T_SAVE=zeros(maxTime+1,1);
U_SAVE=zeros(maxTime,nu);

%%
big_Su=[];
big_im=[];
Im_col=ones(nu*m,2);

for i=1:m
    newcol=[repmat(zeros(ny,nu),i-1,1);S(1:ny*(p-i+1),:)];
    big_Su=[big_Su,newcol];
    new_col1=[repmat(zeros(nu,nu),i-1,1);Im_col(1:nu*(m-i+1),:)];
    big_im=[big_im,new_col1];

end
%% Weights
Gamma_y=diag(repmat([10,1],1,p));
Gamma_u=diag(repmat([0.04 0.04],1,m));
%%
Hess=big_Su'*Gamma_y*big_Su+Gamma_u;
Hess=(Hess+Hess')/2;
%%
big_R = repmat(yref,p,1);   %p*ny

%% Constraints 
Im=(eye(nu*m));
C_LHS=[Im;-Im;big_im;-big_im];

for k=1:maxTime+1
    time=(k-1)*ts;   % Current time 
    if(k==1)
        YHAT=Yk0;
        err=0;
    else
        err=0;      % Replace this by "y_plant - YHat(1)"
    end
   
   predErr=YHAT(3:p*ny+2)-big_R;
   grad=big_Su'*Gamma_y*predErr;
   %opt=optimset('TolFun',1e-6,'TolX',1e-6,'Display','off');
   
  C_RHS=[repmat(duMax,m,1);repmat(-duMin,m,1)];
  C_RHS=[C_RHS; repmat( (umax-uprev),m,1)];
  C_RHS=[C_RHS; repmat(-(umin-uprev),m,1)];
  
  big_dU=quadprog(Hess,grad,C_LHS,C_RHS,[],[],[],[],[],[]);
  du=big_dU(1:nu);
  uk=uprev+du;
  % Store results

  U_SAVE(k,:)=uk;
  uprev=uk;
  
  % Implementing current step
  YHAT=[YHAT(3:end);YHAT(end-1:end)]+S*du;
  yk=YHAT(1:2);

  % Storing results
  Y_SAVE(k+1,:)=yk;
  T_SAVE(k+1)=k*ts;

    
end

figure(1)
plot(T_SAVE(1:maxTime),Y_SAVE(1:maxTime,1),'-b','linewidth',1)
figure(2)
plot(T_SAVE(1:maxTime),Y_SAVE(1:maxTime,2),'-b','linewidth',1)
figure(3)
stairs(T_SAVE(1:maxTime),U_SAVE(1:maxTime,1),'-b','linewidth',1)
figure(4)
stairs(T_SAVE(1:maxTime),U_SAVE(1:maxTime,2),'-b','linewidth',1)
