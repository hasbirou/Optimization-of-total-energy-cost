clc
clear all
% Lower bound of unknown parameters
P_grid_max=500;   %Power of grid
P_bat_max=1;     %Power of battery
SOCmin=0.2;      %SOC
% Upper bound of unknown parameters
P_grid_min=-10;  %Power of grid
P_bat_min=-1;    %Power of battery
SOCmax=1;        %SOC
% the initial parameters
W=0.99;          %The inertia weight coefficient for particles
C1=1.5;          %the personal and social acceleration coefficients 
C2=C1;           %the personal and social acceleration coefficients 
nvar=2;          %number of variables
Tmax=301;          % Maximum iterations
ec=0.95;         %The battery charging coefficient
ed=0.9;          %the battery discharging coefficient
Nomb=10;         %the nominal battery capacity
swarm=20;         %number of particles
for k=1:24
%%Initial
P_load=[3.5,5,5,5,5.7,0,7.3,0,0,1.6,0.9,1,0.2,0.2,0.2,0.7,1.12,2.5,3.12,2.35,0.12,0.12,6,0];
P_PV=[0,0,0,0,0.34875,1.04625,1.74375,2.266875,2.79,3.13875,3.4875,3.313125,3.13875,2.79,2.266875,1.395,1.04625,0.0279,0,0,0,0,0,0];
P_WT=[0.18,0,0,0,0,0,0,0,0,0,4.3,4.95,4.8,4.5,3.7,2.7,2.15,1.6,1.2,0.59,0,0,0,0];
 %Particle position
P_grid=P_grid_min+rand(1,swarm).*(P_grid_max-P_grid_min);
P_bat=P_bat_min+rand(1,swarm).*(P_bat_max-P_bat_min);
%P_SOC=SOCmin+rand(1,swarm).*(SOCmax-SOCmin);
 % Best particles position 
% Particle.bestposition=Particle.position;
 P_bp_grid=P_grid;
 %P_grid_min+rand(1,swarm).*(P_grid_max-P_grid_min);
 P_bp_bat=P_bat;
% P_bat_min+rand(1,swarm).*(P_bat_max-P_bat_min);
% P_bp_SOC=P_SOC;
 %P_bp_SOC=SOCmin+rand(1,swarm).*(SOCmax-SOCmin);
 %Fitness initial position & best position
 for i=1:swarm
%Fitness.P(i,1)=abir_function(Particle.position(i,:)); 
P_OF(i,1)=PSO_function( P_grid(i),P_bat(i),k );    
%Fitness.P(n)=SOC_function(P_SOC(n),k);
%Fitness.BP(i,1)=abir_function(Particle.bestposition(i,:));    
PB_OF(i,1)=PSO_function(P_bp_grid(i),P_bp_bat(i),k);
%Fitness.BP(n)=SOC_function(P_bp_SOC(n),k);
 end
 %Velocity
% V1=zeros(m,nvar);
V1=zeros(1,swarm);
V_bat=V1;
%V_SOC=V2;
 % Global Best particles position
 %[Fitness.Global,index]=min(Fitness.P);
 [Fitness.Global.GB,index]=min(P_OF);
% [Fitness.Global.SOC,index]=min(Fitness.P);
 %intial global position
 %Particle.globalposition=Particle.position(index,:); 
P_gp_grid=P_grid(index); 
%P_gp_bat=P_bat(index);
%P_gp_SOC=0.6;
t=1; 
%P_gp_grid=P_load(k)-P_PV(k)-P_WT(k)+P_gp_bat;  
P_gp_bat=-P_load(k)+P_PV(k)+P_WT(k)+P_gp_grid;  
while (t<Tmax)
    %Update of velocity & position
  for i=1:swarm
      
%V1(i,:)=V1(i,:).*w+C1.*rand().*(Particle.bestposition(i,:)-Particle.position(i,:))+C2.*rand().*(Particle.globalposition(1,:)-Particle.position(i,:));
%Particle.position(i,:)=Particle.position(i,:)+V1(i,:); 
V1(i)=V1(i).*W+C1.*rand().*(P_bp_grid(i)-P_grid(i))+C2.*rand().*(P_gp_grid-P_grid(i));
V_bat(i)=V_bat(i).*W+C1.*rand().*(P_bp_bat(i)-P_bat(i))+C2.*rand().*(P_gp_bat-P_bat(i));
%V_SOC=V_SOC.*W+C1.*rand(1,swarm).*(P_bp_SOC-P_SOC)+C2.*rand(1,swarm).*(P_gp_SOC-P_SOC);
P_grid(i)=P_grid(i)+V1(i);
P_bat(i)=P_bat(i)+V_bat(i);
%P_SOC=P_SOC+V_SOC;
    % Boundaries constraints
  
%       if (Particle.position(i,nvar)<LB(nvar)) || (Particle.position(i,nvar)>UB(nvar))
%          Particle.position=LB+rand(m,nvar).*(UB-LB);
%       end
if (P_grid(i)<P_grid_min) || (P_grid(i)>P_grid_max)
    P_grid(i)=P_grid_min+rand*(P_grid_max-P_grid_min);
end
        
if (P_bat(i)<P_bat_min) || (P_bat(i)>P_bat_max)
    P_bat(i)=P_bat_min+rand*(P_bat_max-P_bat_min);
end
% if (P_SOC(n)<SOCmin) || (P_SOC(n)>SOCmax)
%     P_SOC(n)=SOCmin+rand*(SOCmax-SOCmin);
% end
     
   % Evaluate the objective
    
F(i,1)=PSO_function(P_grid(i),P_bat(i),k);
   
   %Update Best position & Fitness
  
        if F(i)< PB_OF(i)  

        P_bp_grid(i)=P_grid(i);
        P_bp_bat(i)=P_bat(i);
        PB_OF(i)=F(i);

        end

  %Update global position & Fitness

   if PB_OF(i)<  Fitness.Global.GB 
 
      P_gp_grid=P_bp_grid(i);
      P_gp_bat=P_bp_bat(i);
      Fitness.Global.GB=PB_OF(i);
   end
   
 end  
%Final Fitness Global  
FF(t)=Fitness.Global.GB;
t=t+1; % Iteration counter
end
% P_GRID(k)=Particle.globalposition(1);
% P_BAT(k)=Particle.globalposition(2);
P_GRID(k)=P_gp_grid;
P_BAT(k)=P_gp_bat;
%P_soc(k)=P_gp_SPC;
end
plot(FF)
figure(3)
plot(P_GRID,':*')
hold on
plot(P_BAT,'r:o')
hold on
plot(P_load,'k')
hold on
plot(P_PV,'g--')
hold on
plot(P_WT,'y--')
xlabel('Time (h)')
ylabel('Energy (KW)')
legend('P_grid','P_Bat','P_load','P_PV','P_WT')
