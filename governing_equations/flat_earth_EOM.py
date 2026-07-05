import numpy as np

def flat_earth_EOM(t,states,vehicle):

    dstates = np.zeros(12)
    V_b_mps = states[0:3]
    w_b_rps = states[3:6]
    eular_rad = states[6:9]
    pos_n_m = states[9:12]

    #getting mass and inertias
    m_kg = vehicle.mass
    J_b_kgm2 = vehicle.inertia

    #GRAVITY:
    #acting in the earth frame
    g_n_mps = np.array([0,0,9.81])
    #rotation matrix NED to body
    phi = eular_rad[0]
    theta = eular_rad[1]
    psi = eular_rad[2]
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth,  sth  = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    R_n2b = np.array([
        [cth*cpsi,                     cth*spsi,                        -sth],
        [sphi*sth*cpsi - cphi*spsi,    sphi*sth*spsi + cphi*cpsi,   sphi*cth],
        [cphi*sth*cpsi + sphi*spsi,    cphi*sth*spsi - sphi*cpsi,   cphi*cth]
    ])


    #FORCES AND MOMENTS(coming soon)
    F_b_N = 0
    M_b_Nm = 0
    #translation EOM
    cross = np.cross(w_b_rps,V_b_mps)
    dstates[0:3] =  1/m_kg*F_b_N + R_n2b @ g_n_mps - (cross) 
    #rotation EOM
    dstates[3:6] = np.linalg.inv(J_b_kgm2)@(M_b_Nm-np.cross(w_b_rps,J_b_kgm2@w_b_rps))

    return dstates


