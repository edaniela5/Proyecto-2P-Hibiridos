%% MODELO ANALITICO Y OSCILOGRAMAS DEL MOTOR - KIA NIRO EV 2022
% Punto 6 de la guia del proyecto.
% Genera las relaciones y senales minimas del motor:
% torque, velocidad, potencia mecanica, potencia electrica,
% corriente del bus DC, eficiencia y potencia regenerativa.

clearvars;
close all;
clc;

%% 1. Parametros principales del motor
Pmax_W = 150e3;          % Potencia maxima: 150 kW
Tmax_Nm = 395;           % Torque maximo: 395 N.m
Vdc_V = 356;             % Tension nominal del bus DC
eta_traccion = 0.92;     % Eficiencia asumida en traccion
eta_generacion = 0.88;   % Eficiencia asumida en regeneracion
nmax_rpm = 11000;        % Supuesto tecnico de velocidad maxima

%% 2. Tiempo de simulacion
t_s = (0:0.1:60)';

%% 3. Demanda de torque
% 0-10 s: rampa de torque
% 10-20 s: traccion moderada
% 20-30 s: maxima demanda
% 30-40 s: reduccion de demanda
% 40-45 s: rodaje libre
% 45-55 s: frenado regenerativo
% 55-60 s: vehiculo detenido

Tsol_Nm = zeros(size(t_s));

idx = t_s >= 0 & t_s < 10;
Tsol_Nm(idx) = 25*t_s(idx);

idx = t_s >= 10 & t_s < 20;
Tsol_Nm(idx) = 250;

idx = t_s >= 20 & t_s < 30;
Tsol_Nm(idx) = 395;

idx = t_s >= 30 & t_s < 40;
Tsol_Nm(idx) = 395 - 39.5*(t_s(idx)-30);

idx = t_s >= 45 & t_s < 55;
Tsol_Nm(idx) = -180;

%% 4. Velocidad del motor
n_rpm = zeros(size(t_s));

idx = t_s >= 0 & t_s < 10;
n_rpm(idx) = 250*t_s(idx);

idx = t_s >= 10 & t_s < 20;
n_rpm(idx) = 2500 + 50*(t_s(idx)-10);

idx = t_s >= 20 & t_s < 30;
n_rpm(idx) = 3000 + 400*(t_s(idx)-20);

idx = t_s >= 30 & t_s < 40;
n_rpm(idx) = 7000 + 100*(t_s(idx)-30);

idx = t_s >= 40 & t_s < 45;
n_rpm(idx) = 8000;

idx = t_s >= 45 & t_s < 55;
n_rpm(idx) = 8000 - 600*(t_s(idx)-45);

idx = t_s >= 55;
n_rpm(idx) = max(0,2000 - 400*(t_s(idx)-55));

n_rpm = min(n_rpm,nmax_rpm);

%% 5. Relaciones del punto 6.2
% Velocidad angular: omega = 2*pi*n/60
omega_rad_s = 2*pi*n_rpm/60;

% Limite de torque por potencia maxima
Tlim_Nm = Tmax_Nm*ones(size(t_s));
enMovimiento = omega_rad_s > 1;
Tlim_Nm(enMovimiento) = min(Tmax_Nm, ...
    Pmax_W./omega_rad_s(enMovimiento));

% Torque real limitado por la envolvente del motor
Tmotor_Nm = max(-Tlim_Nm,min(Tsol_Nm,Tlim_Nm));

% Potencia mecanica: Pmec = T*omega
Pmec_W = Tmotor_Nm.*omega_rad_s;

% Eficiencia y potencia electrica
eta_motor = NaN(size(t_s));
Pelec_W = zeros(size(t_s));

traccion = Pmec_W > 1;
regeneracion = Pmec_W < -1;

eta_motor(traccion) = eta_traccion;
eta_motor(regeneracion) = eta_generacion;

% En traccion la potencia electrica de entrada es mayor que la mecanica
Pelec_W(traccion) = Pmec_W(traccion)/eta_traccion;

% En regeneracion solo una parte de la potencia mecanica vuelve a bateria
Pelec_W(regeneracion) = Pmec_W(regeneracion)*eta_generacion;

% Corriente del bus: I = P/V
% Convencion: I positiva = descarga; I negativa = regeneracion
Idc_A = Pelec_W/Vdc_V;

% Potencia regenerativa disponible
Pregen_W = max(-Pelec_W,0);

% Perdidas del conjunto motor-inversor
Perdidas_W = abs(Pelec_W-Pmec_W);

% Energia electrica consumida y recuperada
Econs_kWh = trapz(t_s,max(Pelec_W,0))/3.6e6;
Eregen_kWh = trapz(t_s,max(-Pelec_W,0))/3.6e6;
Eperdidas_kWh = trapz(t_s,Perdidas_W)/3.6e6;

%% 6. Exportar tabla y datos
ResultadosMotor = table( ...
    t_s(:),Tsol_Nm(:),Tmotor_Nm(:),n_rpm(:),omega_rad_s(:), ...
    Pmec_W(:)/1000,Pelec_W(:)/1000,Idc_A(:), ...
    eta_motor(:)*100,Pregen_W(:)/1000,Perdidas_W(:)/1000, ...
    VariableNames={'Tiempo_s','TorqueSolicitado_Nm','TorqueMotor_Nm', ...
    'VelocidadMotor_rpm','VelocidadAngular_rad_s','PotenciaMecanica_kW', ...
    'PotenciaElectrica_kW','CorrienteDC_A','Eficiencia_pct', ...
    'PotenciaRegenerativa_kW','Perdidas_kW'});

writetable(ResultadosMotor,"Resultados_Oscilogramas_Motor_Niro.csv");

ResumenMotor = table( ...
    max(Tmotor_Nm),max(n_rpm),max(Pmec_W)/1000, ...
    max(Pelec_W)/1000,max(Idc_A),min(Idc_A), ...
    Econs_kWh,Eregen_kWh,Eperdidas_kWh, ...
    VariableNames={'TorqueMax_Nm','VelocidadMax_rpm', ...
    'PotenciaMecanicaMax_kW','PotenciaElectricaMax_kW', ...
    'CorrienteDescargaMax_A','CorrienteRegenerativaMin_A', ...
    'EnergiaConsumida_kWh','EnergiaRecuperada_kWh', ...
    'EnergiaPerdida_kWh'});

writetable(ResumenMotor,"Resumen_Oscilogramas_Motor_Niro.csv");

save("Oscilogramas_Motor_Kia_Niro_2022.mat", ...
    "ResultadosMotor","ResumenMotor","Pmax_W","Tmax_Nm","Vdc_V", ...
    "eta_traccion","eta_generacion");

%% 7. Oscilogramas obligatorios
fig1 = figure("Color","white","Position",[80 60 1150 760]);
tiledlayout(2,2,"TileSpacing","compact","Padding","compact");

nexttile;
yyaxis left;
plot(t_s,Tmotor_Nm,"LineWidth",1.6);
ylabel("Torque (N.m)");
yyaxis right;
plot(t_s,n_rpm,"LineWidth",1.6);
ylabel("Velocidad (rpm)");
xlabel("Tiempo (s)");
title("Torque y velocidad del motor");
grid on;

nexttile;
plot(t_s,Pelec_W/1000,"LineWidth",1.6);
hold on;
plot(t_s,Pmec_W/1000,"LineWidth",1.6);
yline(0,"k--");
grid on;
xlabel("Tiempo (s)");
ylabel("Potencia (kW)");
title("Potencia electrica y mecanica");
legend("Potencia electrica","Potencia mecanica", ...
    "Location","best");

nexttile;
plot(t_s,Idc_A,"LineWidth",1.6,"Color",[0.15 0.55 0.30]);
yline(0,"k--");
grid on;
xlabel("Tiempo (s)");
ylabel("Corriente DC (A)");
title("Corriente del bus DC");

nexttile;
plot(t_s,eta_motor*100,"LineWidth",1.6, ...
    "Color",[0.62 0.25 0.72]);
grid on;
xlabel("Tiempo (s)");
ylabel("Eficiencia (%)");
ylim([0 100]);
title("Eficiencia del motor");

sgtitle("Oscilogramas del motor - Kia Niro EV 2022", ...
    "FontWeight","bold");
exportgraphics(fig1,"Oscilogramas_Motor_Kia_Niro_2022.png", ...
    "Resolution",200);

%% 8. Mapa torque-velocidad con trayectoria
fig2 = figure("Color","white","Position",[120 100 900 520]);
scatter(n_rpm,Tmotor_Nm,20,t_s,"filled");
hold on;
nMapa = linspace(1,nmax_rpm,400)';
wMapa = 2*pi*nMapa/60;
TMapa = min(Tmax_Nm,Pmax_W./wMapa);
plot(nMapa,TMapa,"k--","LineWidth",1.5);
plot(nMapa,-TMapa,"k--","LineWidth",1.5);
grid on;
xlabel("Velocidad del motor (rpm)");
ylabel("Torque del motor (N.m)");
title("Trayectoria torque-velocidad del motor");
cb = colorbar;
cb.Label.String = "Tiempo (s)";
legend("Puntos de operacion","Limite del motor", ...
    "Location","best");
exportgraphics(fig2,"Mapa_Torque_Velocidad_Motor_Niro.png", ...
    "Resolution",200);

%% 9. Mostrar resumen
disp("RESUMEN DEL MOTOR KIA NIRO EV 2022");
disp(ResumenMotor);

fprintf("\nINTERVALOS DE LA PRUEBA:\n");
fprintf("0-10 s: rampa de torque.\n");
fprintf("20-30 s: maxima demanda.\n");
fprintf("45-55 s: modo generador y frenado regenerativo.\n");
fprintf("\nARCHIVOS CREADOS:\n");
fprintf("Oscilogramas_Motor_Kia_Niro_2022.png\n");
fprintf("Mapa_Torque_Velocidad_Motor_Niro.png\n");
fprintf("Resultados_Oscilogramas_Motor_Niro.csv\n");
fprintf("Resumen_Oscilogramas_Motor_Niro.csv\n");
fprintf("Oscilogramas_Motor_Kia_Niro_2022.mat\n");
