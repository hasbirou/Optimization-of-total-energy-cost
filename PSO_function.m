function [Out]=PSO_function(P_GRID,P_BAT,k)
%The step 1hour
Dt=1;
%The price of electricity
Ct=[0.033 0.027 0.020 0.017 0.017 0.029 0.033 0.054 0.215 0.572 0.572 0.572 0.215 0.572 0.286 0.279 0.086 0.059 0.050 0.061 0.181 0.077 0.043 0.037]; 
%The price of grid,PV,PW and battery 
C_Grid=Ct;
C_PV=0.01;
C_WT=0.01;
C_Bat=0.05;
% The load power
P_load=[3.5,5,5,5,5.7,0,7.3,0,0,1.6,0.9,1,0.2,0.2,0.2,0.7,1.12,2.5,3.12,2.35,0.12,0.12,6,0];
% The net power generated by PV
P_PV=[0,0,0,0,0.34875,1.04625,1.74375,2.266875,2.79,3.13875,3.4875,3.313125,3.13875,2.79,2.266875,1.395,1.04625,0.0279,0,0,0,0,0,0];
% The net power generated by turbine
P_WT=[0.18,0,0,0,0,0,0,0,0,0,4.3,4.95,4.8,4.5,3.7,2.7,2.15,1.6,1.2,0.59,0,0,0,0];
%The objective to minimize the electricity cost 
Cost=sum(abs(P_GRID.*C_Grid(k).*Dt+P_BAT.*C_Bat.*Dt+P_PV(k).*C_PV.*Dt+P_WT(k).*C_WT.*Dt));
%The power balance constraintes
Power_balance=sum( abs(P_GRID+P_PV(k)+P_WT(k)-P_BAT-P_load(k)));
%Multi objective
alpha=0.4;
Out=sum((alpha.*Power_balance+(1-alpha).*Cost));
end