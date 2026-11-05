% Matrice_jacobienne.m calcule la matrice Jacobienne J a partir des angles d'Euler (psi, theta, phi).
function J = Matrice_jacobienne(psi,theta,phi)
    J = [1 0 -sin(theta);
    0 cos(phi) sin(phi)*cos(theta);
    0 -sin(phi) cos(phi)*cos(theta)];
end