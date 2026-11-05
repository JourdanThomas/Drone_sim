% Script simulation simple drone
% Ce script initialise les parametres, lance la simulation dynamique et
% affiche les resultats sous forme de graphiques et d'une animation 3D.
clear all;
close all;
clc;

%% Definition des Parametres Physiques et Geometriques
% Parametres de la structure du drone
param.m = 2.0;% Masse du drone (kg)
param.r = 0.10;% Rayon du corps du drone (m)
param.H = 0.04;% Hauteur du corps du drone (m)
param.l = 0.3;% Longueur d'un bras (m)
param.g = 9.81;% Acceleration gravitationnelle (m/s^2)

% Matrice d'inertie
Jxx = (param.m / 12) * (3 * param.r^2 + param.H^2);
Jyy = Jxx;
Jzz = (param.m * param.r^2) / 2;
param.I = diag([Jxx, Jyy, Jzz]);

% Parametres des moteurs
param.beta = ones(1, 6) * 1e-5 * 1.293 * pi * (0.25^5);
param.alpha = ones(1, 6) * 1e-7 * 1.293 * pi * (0.25^5);
param.alpha([2, 4, 6]) = -param.alpha([2, 4, 6]);

% Geometrie des bras
param.gamma = [0, pi/3, 2*pi/3, pi, 4*pi/3, 5*pi/3];

%% Parametres de la Simulation
param.dt = 0.01;
t_fin = 5.0;
temps_sim = 0:param.dt:t_fin;

%% Conditions Initiales et Commande des Moteurs

V0 = [0; 0; 0];
W0 = [0; 0; 0];
angles0 = [0; 0; 0];
etat_initial = [V0; W0; angles0];
position_initiale_R0 = [0; 0; 1];


%determination de la commande
poussee_stationnaire = param.m * param.g;
w2_stationnaire = poussee_stationnaire / sum(param.beta);
facteur_montee = 1.1;
w2_montee = w2_stationnaire * facteur_montee;
param.u = ones(length(temps_sim), 6) * w2_montee;

%% Lancement du Solveur
fprintf('Lancement de la simulation\n');
[t, solution] = ode45(@(t,x) NewtonEuler(t,x,param), temps_sim, etat_initial);
fprintf('Simulation terminee.\n');

%% Post-traitement des Resultats
V_R1 = solution(:, 1:3);
W_R1 = solution(:, 4:6);
angles = solution(:, 7:9);

position_R0 = zeros(length(t), 3);
position_R0(1,:) = position_initiale_R0';
for i = 2:length(t)
    R = Matrice_rotation(angles(i,3), angles(i,2), angles(i,1));
    V_R0_i = (R * V_R1(i,:)')';
    dt_step = t(i) - t(i-1);
    position_R0(i,:) = position_R0(i-1,:) + V_R0_i * dt_step;
end

%% Affichage des Graphiques et Animation

% Graphique de la trajectoire (Altitude dans le temps)
figure('Name', 'Trajectoire du Drone');
plot(t, position_R0(:,3), 'b-', 'LineWidth', 2);
title('Altitude du drone en fonction du temps');
xlabel('Temps (s)');
ylabel('Altitude Z (m)');
grid on;

% b) Animation 3D
fprintf('Lancement de l''animation 3D...\n');

% Determiner les limites des axes pour toute l'animation
marge = param.l * 2; % Marge autour de la trajectoire
limites.x = [min(position_R0(:,1)) - marge, max(position_R0(:,1)) + marge];
limites.y = [min(position_R0(:,2)) - marge, max(position_R0(:,2)) + marge];
limites.z = [0, max(position_R0(:,3)) + marge]; % Ordonnee Z minimale fixee a 0

figure(100); % Cree la figure une seule fois en dehors de la boucle


for i = 1:5:length(t)
    clf; 
    
    pos_i = position_R0(i,:)';
    R_i = Matrice_rotation(angles(i,3), angles(i,2), angles(i,1));
    
    % Appeler la fonction qui ne fait que dessiner
    visualiser_drone(pos_i, R_i, param);
    
  
    hold on; % pour ajouter le repere fixe si besoin
    % plot3([0 1],[0 0],[0 0],'k-','LineWidth',2); % Exemple repere fixe
    hold off;
    
    title(sprintf('Visualisation 3D du Drone (t = %.2f s)', t(i)));
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    axis equal;
    xlim(limites.x);
    ylim(limites.y);
    zlim(limites.z);
    view(30, 20);
    
    pause(param.dt);
end
fprintf('Animation terminee.\n');


