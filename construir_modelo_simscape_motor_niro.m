%% CONSTRUCTOR DEL MODELO SIMSCAPE - MOTOR KIA NIRO EV 2022
% Ejecutar este archivo una sola vez. El script crea y guarda:
% Modelo_Simscape_Motor_Kia_Niro_2022.slx
%
% Requiere:
% Simulink, Simscape y Simscape Electrical.

clearvars;
close all;
clc;

%% Datos del Kia Niro EV 2022
Pmax_motor_W = 150e3;
Tmax_motor_Nm = 395;
Vbus_V = 356;
eficiencia_pct = 92;
inercia_equivalente_kgm2 = 1.2;

%% Perfil de prueba: rampa, maxima demanda y regeneracion
tTorque = [0 2 8 15 22 30 38 45 52 60]';
vTorque = [0 0 150 250 395 395 100 0 -180 0]';
TorqueSolicitado = timeseries(vTorque,tTorque);

assignin("base","TorqueSolicitado",TorqueSolicitado);
assignin("base","Pmax_motor_W",Pmax_motor_W);
assignin("base","Tmax_motor_Nm",Tmax_motor_Nm);
assignin("base","Vbus_V",Vbus_V);
assignin("base","eficiencia_pct",eficiencia_pct);

%% Abrir librerias
load_system("simulink");
load_system("simscape");
load_system("fl_lib");
load_system("ee_lib");
load_system("nesl_utility");

%% Crear modelo
modelo = "Modelo_Simscape_Motor_Kia_Niro_2022";

if bdIsLoaded(modelo)
    close_system(modelo,0);
end

if isfile(modelo + ".slx")
    nombreRespaldo = modelo + "_respaldo_" + ...
        string(datetime("now","Format","yyyyMMdd_HHmmss")) + ".slx";
    copyfile(modelo + ".slx",nombreRespaldo);
end

new_system(modelo);
open_system(modelo);

%% Localizar bloques de Simscape disponibles
rutaMotor = buscarBloque("ee_lib","Motor & Drive (System Level)");
rutaFuente = buscarBloque("fl_lib","DC Voltage Source");
rutaSensorI = buscarBloque("fl_lib","Current Sensor");
rutaSensorV = buscarBloque("fl_lib","Voltage Sensor");
rutaRefElec = buscarBloque("fl_lib","Electrical Reference");
rutaSolver = "nesl_utility/Solver Configuration";
rutaRefMec = buscarBloque("fl_lib","Mechanical Rotational Reference");
rutaInercia = buscarBloque("fl_lib","Inertia");
rutaSensorT = buscarBloque("fl_lib","Ideal Torque Sensor");

%% Insertar bloques fisicos
bFuente = add_block(rutaFuente,modelo+"/Bateria equivalente 356 V", ...
    Position=[80 150 150 230]);
bSensorI = add_block(rutaSensorI,modelo+"/Sensor de corriente", ...
    Position=[205 125 260 185]);
bSensorV = add_block(rutaSensorV,modelo+"/Sensor de tension", ...
    Position=[300 235 360 295]);
bMotor = add_block(rutaMotor,modelo+"/Motor PMSM Kia Niro", ...
    Position=[430 105 595 270]);
bSensorT = add_block(rutaSensorT,modelo+"/Sensor de torque", ...
    Position=[665 110 730 180]);
bInercia = add_block(rutaInercia,modelo+"/Inercia equivalente", ...
    Position=[800 110 870 180]);
bRefElec = add_block(rutaRefElec,modelo+"/Referencia electrica", ...
    Position=[205 315 255 365]);
bRefMec = add_block(rutaRefMec,modelo+"/Referencia mecanica", ...
    Position=[545 330 595 380]);
bSolver = add_block(rutaSolver,modelo+"/Solver Configuration", ...
    Position=[80 315 155 365]);

%% Insertar convertidores y entrada de torque
bTorque = add_block("simulink/Sources/From Workspace", ...
    modelo+"/Demanda de torque",Position=[80 35 190 70]);
set_param(bTorque,"VariableName","TorqueSolicitado");

rutaSimPS = "nesl_utility/Simulink-PS Converter";
rutaPSSim = "nesl_utility/PS-Simulink Converter";

bTorqueConv = add_block(rutaSimPS,modelo+"/Conversor torque", ...
    Position=[260 30 350 75]);
bVelConv = add_block(rutaPSSim,modelo+"/Conversor velocidad", ...
    Position=[655 250 745 295]);
bTmedConv = add_block(rutaPSSim,modelo+"/Conversor torque medido", ...
    Position=[770 205 870 250]);
bIConv = add_block(rutaPSSim,modelo+"/Conversor corriente", ...
    Position=[275 85 365 125]);
bVConv = add_block(rutaPSSim,modelo+"/Conversor tension", ...
    Position=[385 280 475 320]);

configurarPorTexto(bTorqueConv,"Input signal unit","N*m");
configurarPorTexto(bVelConv,"Output signal unit","rpm");
configurarPorTexto(bTmedConv,"Output signal unit","N*m");
configurarPorTexto(bIConv,"Output signal unit","A");
configurarPorTexto(bVConv,"Output signal unit","V");

%% Parametrizar motor, fuente e inercia
configurarPorTexto(bMotor,"Parameterize by","Maximum torque and power");
configurarPorTexto(bMotor,"Maximum torque","Tmax_motor_Nm");
configurarPorTexto(bMotor,"Maximum power","Pmax_motor_W");
configurarPorTexto(bMotor, ...
    "Motor and driver overall efficiency","eficiencia_pct");
configurarPorTexto(bFuente,"Constant voltage","Vbus_V");
configurarPorTexto(bInercia,"Inertia", ...
    num2str(inercia_equivalente_kgm2));

%% Conexiones Simulink y senal fisica de torque
add_line(modelo,bTorque+"/1",bTorqueConv+"/1","autorouting","on");
conectarPuertos(modelo,bTorqueConv,"",bMotor,"Tr");

%% Conexiones electricas
conectarPuertos(modelo,bFuente,"+",bSensorI,"+");
conectarPuertos(modelo,bSensorI,"-",bMotor,"+");
conectarPuertos(modelo,bFuente,"-",bMotor,"-");
conectarPuertos(modelo,bFuente,"-",bRefElec,"");
conectarPuertos(modelo,bFuente,"-",bSolver,"");
conectarPuertos(modelo,bSensorV,"+",bMotor,"+");
conectarPuertos(modelo,bSensorV,"-",bMotor,"-");

%% Conexiones mecanicas
conectarPuertos(modelo,bMotor,"R",bSensorT,"R");
conectarPuertos(modelo,bSensorT,"C",bInercia,"");
conectarPuertos(modelo,bMotor,"C",bRefMec,"");

%% Conectar salidas de sensores
conectarPuertos(modelo,bMotor,"W",bVelConv,"");
conectarPuertos(modelo,bSensorT,"T",bTmedConv,"");
conectarPuertos(modelo,bSensorI,"I",bIConv,"");
conectarPuertos(modelo,bSensorV,"V",bVConv,"");

%% Calculo de potencias en Simulink
bProdMec = add_block("simulink/Math Operations/Product", ...
    modelo+"/Calculo potencia mecanica",Position=[960 190 1010 235]);
bProdElec = add_block("simulink/Math Operations/Product", ...
    modelo+"/Calculo potencia electrica",Position=[960 300 1010 345]);
bGainMec = add_block("simulink/Math Operations/Gain", ...
    modelo+"/Pmec kW",Position=[1050 190 1110 235]);
bGainElec = add_block("simulink/Math Operations/Gain", ...
    modelo+"/Pelec kW",Position=[1050 300 1110 345]);
set_param(bGainMec,"Gain","1/1000");
set_param(bGainElec,"Gain","1/1000");

% Para potencia mecanica se utiliza velocidad en rpm convertida a rad/s.
bRpmRad = add_block("simulink/Math Operations/Gain", ...
    modelo+"/rpm a rad_s",Position=[805 265 875 305]);
set_param(bRpmRad,"Gain","2*pi/60");

add_line(modelo,bVelConv+"/1",bRpmRad+"/1","autorouting","on");
add_line(modelo,bRpmRad+"/1",bProdMec+"/1","autorouting","on");
add_line(modelo,bTmedConv+"/1",bProdMec+"/2","autorouting","on");
add_line(modelo,bProdMec+"/1",bGainMec+"/1","autorouting","on");

add_line(modelo,bVConv+"/1",bProdElec+"/1","autorouting","on");
add_line(modelo,bIConv+"/1",bProdElec+"/2","autorouting","on");
add_line(modelo,bProdElec+"/1",bGainElec+"/1","autorouting","on");

%% Osciloscopios
bMuxTV = add_block("simulink/Signal Routing/Mux", ...
    modelo+"/Mux torque velocidad",Position=[950 30 955 90]);
set_param(bMuxTV,"Inputs","2");
bScopeTV = add_block("simulink/Sinks/Scope", ...
    modelo+"/Scope Torque y Velocidad",Position=[1040 25 1120 95]);

bMuxP = add_block("simulink/Signal Routing/Mux", ...
    modelo+"/Mux potencias",Position=[1160 210 1165 275]);
set_param(bMuxP,"Inputs","2");
bScopeP = add_block("simulink/Sinks/Scope", ...
    modelo+"/Scope Potencias",Position=[1220 210 1300 280]);

bScopeI = add_block("simulink/Sinks/Scope", ...
    modelo+"/Scope Corriente DC",Position=[470 30 550 75]);

add_line(modelo,bTmedConv+"/1",bMuxTV+"/1","autorouting","on");
add_line(modelo,bVelConv+"/1",bMuxTV+"/2","autorouting","on");
add_line(modelo,bMuxTV+"/1",bScopeTV+"/1","autorouting","on");

add_line(modelo,bGainMec+"/1",bMuxP+"/1","autorouting","on");
add_line(modelo,bGainElec+"/1",bMuxP+"/2","autorouting","on");
add_line(modelo,bMuxP+"/1",bScopeP+"/1","autorouting","on");
add_line(modelo,bIConv+"/1",bScopeI+"/1","autorouting","on");

%% Guardar senales en Workspace
crearSalida(modelo,bTmedConv,"TorqueMotor_Nm",[900 120 990 155]);
crearSalida(modelo,bVelConv,"VelocidadMotor_rpm",[790 330 900 365]);
crearSalida(modelo,bIConv,"CorrienteDC_A",[470 90 565 125]);
crearSalida(modelo,bVConv,"TensionDC_V",[520 365 610 400]);
crearSalida(modelo,bGainMec,"PotenciaMecanica_kW",[1150 145 1270 180]);
crearSalida(modelo,bGainElec,"PotenciaElectrica_kW",[1150 330 1270 365]);

%% Configuracion, anotaciones y guardado
set_param(modelo, ...
    "StopTime","60", ...
    "Solver","ode23t", ...
    "SolverType","Variable-step", ...
    "MaxStep","0.02");

Simulink.BlockDiagram.arrangeSystem(modelo);

nota = sprintf([ ...
    "KIA NIRO EV 2022\\n" ...
    "Motor PMSM: 150 kW y 395 N.m\\n" ...
    "Bus DC nominal: 356 V\\n" ...
    "Pruebas: rampa, maxima demanda y regeneracion"]);
anotacion = Simulink.Annotation(modelo,nota);
anotacion.Position = [30 410 290 500];

save_system(modelo);
open_system(modelo);

fprintf("\nMODELO CREADO CORRECTAMENTE:\n");
fprintf("%s.slx\n",modelo);
fprintf("\nPresione Run para simular durante 60 segundos.\n");
fprintf("Abra los tres Scopes para observar los oscilogramas.\n");

%% FUNCIONES LOCALES
function ruta = buscarBloque(libreria,nombre)
    candidatos = find_system(libreria, ...
        "LookUnderMasks","all", ...
        "FollowLinks","on", ...
        "Type","Block", ...
        "Name",nombre);
    if isempty(candidatos)
        error("No se encontro el bloque '%s' en la libreria %s.", ...
            nombre,libreria);
    end
    ruta = candidatos{1};
end

function configurarPorTexto(bloque,textoPrompt,valor)
    nombres = get_param(bloque,"MaskNames");
    prompts = get_param(bloque,"MaskPrompts");
    if isempty(nombres) || isempty(prompts)
        warning("No se pudo parametrizar automaticamente: %s",bloque);
        return;
    end
    idx = find(contains(lower(string(prompts)), ...
        lower(string(textoPrompt))),1);
    if isempty(idx)
        warning("No se encontro el parametro '%s' en %s.", ...
            textoPrompt,bloque);
        return;
    end
    try
        set_param(bloque,nombres{idx},valor);
    catch ME
        warning("Revise manualmente '%s' en %s: %s", ...
            textoPrompt,bloque,ME.message);
    end
end

function conectarPuertos(modelo,bloqueA,nombreA,bloqueB,nombreB)
    puertoA = obtenerPuerto(bloqueA,nombreA,false);
    puertoB = obtenerPuerto(bloqueB,nombreB,true);
    add_line(modelo,puertoA,puertoB,"autorouting","on");
end

function puerto = obtenerPuerto(bloque,nombre,preferirEntrada)
    ph = get_param(bloque,"PortHandles");
    categorias = ["LConn","RConn","Inport","Outport"];
    todos = [];
    for k = 1:numel(categorias)
        campo = char(categorias(k));
        if isfield(ph,campo)
            todos = [todos ph.(campo)]; %#ok<AGROW>
        end
    end

    if strlength(string(nombre)) > 0
        for k = 1:numel(todos)
            try
                etiqueta = string(get_param(todos(k),"Name"));
                if strcmpi(strtrim(etiqueta),strtrim(string(nombre)))
                    puerto = todos(k);
                    return;
                end
            catch
            end
        end
    end

    if preferirEntrada && ~isempty(ph.Inport)
        puerto = ph.Inport(1);
    elseif ~preferirEntrada && ~isempty(ph.Outport)
        puerto = ph.Outport(1);
    elseif preferirEntrada && ~isempty(ph.LConn)
        puerto = ph.LConn(1);
    elseif ~preferirEntrada && ~isempty(ph.RConn)
        puerto = ph.RConn(1);
    elseif ~isempty(todos)
        puerto = todos(1);
    else
        error("No se encontro un puerto util en %s.",bloque);
    end
end

function crearSalida(modelo,bloqueOrigen,nombreVariable,posicion)
    destino = add_block("simulink/Sinks/To Workspace", ...
        modelo+"/"+nombreVariable,Position=posicion);
    set_param(destino, ...
        "VariableName",nombreVariable, ...
        "SaveFormat","Timeseries");
    add_line(modelo,bloqueOrigen+"/1",destino+"/1","autorouting","on");
end
