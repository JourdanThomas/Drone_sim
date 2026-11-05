% Cette fonction dessine une representation 3D de l'hexacoptere a une
% position et une orientation donnees.
function visualiser_drone(position, orientation_R, param)
%
% Entrees :
% position       : Vecteur de position [x; y; z] dans le repere R0.
% orientation_R  : Matrice de rotation 3x3 pour orienter le drone.
% param          : Structure contenant les parametres geometriques.

% -------------------------------------------------
% Definir la geometrie du drone a l'origine (0,0,0)
% Corps du drone (cylindre)
n_points_cercle = 50; % Nombre de points pour dessiner les cercles
angles_cercle = linspace(0, 2*pi, n_points_cercle);

% Coordonnees des cercles superieur et inferieur du cylindre
x_cercle = param.r * cos(angles_cercle);
y_cercle = param.r * sin(angles_cercle);
z_haut = (param.H/2) * ones(1, n_points_cercle);
z_bas = -(param.H/2) * ones(1, n_points_cercle);

% Bras du drone
% On definit les points de depart (origine) et d'arrivee de chaque bras.
bras = zeros(6, 4);
for i = 1:6
    angle_bras = param.gamma(i);
    x_extremite = param.l * cos(angle_bras);
    y_extremite = param.l * sin(angle_bras);
    bras(i, :) = [x_extremite, y_extremite, 0, 1]; % Coordonnees homogenes
end

% -------------------------------------------------
% On cree la matrice de transformation homogene 

T = [orientation_R, position; 
              0, 0, 0,       1];
% -------------------------------------------------
% Appliquer la transformation a tous les points de la geometrie

% Transformer les cercles du corps
points_haut_homogenes = [x_cercle; y_cercle; z_haut; ones(1, n_points_cercle)];
points_bas_homogenes = [x_cercle; y_cercle; z_bas; ones(1, n_points_cercle)];

points_haut_transformes = (T * points_haut_homogenes)';
points_bas_transformes = (T * points_bas_homogenes)';

% Transformer les bras
bras_transformes = (T * bras')';

% -------------------------------------------------
% Dessiner le drone transforme

hold on;
grid on;

% Dessiner le corps du cylindre
fill3(points_haut_transformes(:,1), points_haut_transformes(:,2), points_haut_transformes(:,3), 'cyan', 'FaceAlpha', 0.7);
fill3(points_bas_transformes(:,1), points_bas_transformes(:,2), points_bas_transformes(:,3), 'cyan', 'FaceAlpha', 0.7);
surf([points_haut_transformes(:,1)'; points_bas_transformes(:,1)'], ...
     [points_haut_transformes(:,2)'; points_bas_transformes(:,2)'], ...
     [points_haut_transformes(:,3)'; points_bas_transformes(:,3)'], ...
     'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.7);

% Dessiner les bras
for i = 1:6
    % Chaque bras part du centre du drone (la position actuelle)
    % et va jusqu'a l'extremite transformee.
    plot3([position(1); bras_transformes(i,1)], ...
          [position(2); bras_transformes(i,2)], ...
          [position(3); bras_transformes(i,3)], 'k', 'LineWidth', 2);
end

% Dessiner le repere du corps du drone (R1)
longueur_axes = param.l * 1.5;
axe_x_R1 = T * [longueur_axes; 0; 0; 1];
axe_y_R1 = T * [0; longueur_axes; 0; 1];
axe_z_R1 = T * [0; 0; longueur_axes; 1];

plot3([position(1); axe_x_R1(1)], [position(2); axe_x_R1(2)], [position(3); axe_x_R1(3)], 'r-', 'LineWidth', 1.5); % Axe X en rouge
plot3([position(1); axe_y_R1(1)], [position(2); axe_y_R1(2)], [position(3); axe_y_R1(3)], 'g-', 'LineWidth', 1.5); % Axe Y en vert
plot3([position(1); axe_z_R1(1)], [position(2); axe_z_R1(2)], [position(3); axe_z_R1(3)], 'b-', 'LineWidth', 1.5); % Axe Z en bleu

hold off;

end
