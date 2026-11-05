% Matrice_rotation.m calcule la matrice de rotation R a partir des angles d'Euler (psi, theta, phi).
function R = Matrice_rotation(psi,theta,phi)

    Rx =[1 0 0;
    0 cos(phi) -sin(phi);
    0 sin(phi) cos(phi)];

    Ry = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];

    Rz = [cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0 0 1;];
    
    R = Rz*Ry*Rx;
end