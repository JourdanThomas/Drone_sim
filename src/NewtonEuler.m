% Cette fonction calcule la derivee du vecteur d'etat a un instant t a l'aide des equations de Newton-Euler. Elle est le coeur du simulateur et sera appelee par un solveur comme ode45.
function dx_dt = NewtonEuler(t, x, param)
% Entrees :
% % t        : Temps actuel
% x        : Vecteur d'etat actuel [Vx, Vy, Vz, Wx, Wy, Wz, phi, theta, psi]
% param    : Structure contenant tous les parametres physiques du drone
%            et les commandes des moteurs.
%
% Sortie :
% dx_dt  : Vecteur des derivees de l'etat [dV/dt, dW/dt, dphi/dt, ...]
%
% Repere utilises :
% R0 = (O, i0, j0, k0) 
% R1 = (G, i1, j1, k1) 
% ou R0 est le repere fixe (sol) et R1 est le repere mobile attache au drone.
% -------------------------------------------------
% Decomposition du vecteur d'etat 'x'
V = x(1:3);      % Vitesse lineaire [vx, vy, vz] dans le repere mobile R1.
W = x(4:6);      % Vitesse angulaire [p, q, r] dans le repere mobile R1.
phi = x(7);      % Roulis
theta = x(8);    % Tangage
psi = x(9);      % Lacet

%-------------------------------------------------
% Recuperer la commande des moteurs a l'instant t 
k = round(t/ param.dt) + 1;
if k > size( param.u, 1)
    k = size( param.u, 1);
end
commande_actuelle_w2 =  param.u(k,:); % Commande en vitesse de rotation au carre (w^2)

%-------------------------------------------------
% Bilan des Forces et Moments exterieurs dans le repere mobile R1

% Les forces
% La force de poussee des moteurs s'exerce le long de l'axe z du repere mobile (k1).
poussees_individuelles = param.beta.* commande_actuelle_w2;
force_poussee_R1 = [0; 0; sum(poussees_individuelles)];

% La force de gravite est definie dans le repere fixe R0.
% On doit la projeter dans le repere mobile R1 a l'aide de la matrice de rotation.
R = Matrice_rotation(psi, theta, phi); % Matrice de passage de R1 vers R0.
force_gravite_R0 = [0; 0; -param.m * param.g];
force_gravite_R1 = R' * force_gravite_R0; % R' est la matrice de passage de R0 vers R1.

somme_forces_R1 = force_poussee_R1 + force_gravite_R1;

% Les moments
moments_totaux_R1 = [0; 0; 0];
for i = 1:6
    angle_bras_i = param.gamma(i);
    poussee_i = poussees_individuelles(i);
    
    % Moment genere par la poussee (r x F) -> Roulis et Tangage.
    bras_de_levier = [param.l * cos(angle_bras_i); param.l * sin(angle_bras_i); 0];
    force_i_R1 = [0; 0; poussee_i];
    moments_totaux_R1 = moments_totaux_R1 + cross(bras_de_levier, force_i_R1);
    
    % Moment de reaction de l'helice -> Lacet.
    couple_reactif_i = param.alpha(i) * commande_actuelle_w2(i);
    moments_totaux_R1 = moments_totaux_R1 + [0; 0; couple_reactif_i];
end

%-------------------------------------------------
% Application des Equations de Newton-Euler dans R1

% Equation de la translation pour trouver l'acceleration lineaire dV/dt dans R1.
% dV/dt = (Somme_Forces / masse) - W x V
dV_dt = (somme_forces_R1 / param.m) - cross(W, V);

% Equation de la rotation pour trouver l'acceleration angulaire dW/dt dans R1.
% dW/dt = I^-1 * (Somme_Moments - W x (I*W))
dW_dt = param.I \ (moments_totaux_R1 - cross(W, param.I * W));

%-------------------------------------------------
%  Calcul de la derivee des angles d'Euler (Cinematique)
% Cette partie lie la vitesse angulaire W (exprimee dans R1) a la variation
% des angles d'Euler (qui definissent l'orientation de R1 par rapport a R0).
J = Matrice_jacobienne(psi, theta, phi);
d_angles_dt = J \ W;

%-------------------------------------------------
dx_dt = [
    dV_dt;
    dW_dt;
    d_angles_dt
];

end
