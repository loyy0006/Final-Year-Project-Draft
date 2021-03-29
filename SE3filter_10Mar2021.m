function G_final=SE3filter_10Mar2021(k,N,tau_R,tau_p,R0,p0,m_data,m_T0,com_T0,dT,L)
mm=1/1000;
%%
N_marker_task = length(m_T0);     % number of makers
br = tau_R*k*L^2;       % [Ns/m]
bp = tau_p*k;

R{1} = R0;
p{1} = p0;
G{1} = [R0 p0; [0 0 0 1]];
    for n =1:N
        for i =1:N_marker_task    %markers 1-6
            m_S_estimate{i} = G{n}*[m_T0{i};1];             %marker position estimated in S coordinate
            force_S{i} = k*(m_data{i} - p{n} - R{n}*m_T0{i});
            torque_T{i} = cross((R{n}'*force_S{i}),(com_T0-m_T0{i}));
        end
        store_force_S = [force_S{1:end}];
        store_torque_T = [torque_T{1:end}];
        w{n} = 1/br*sum(store_torque_T')';        
        store_marker{n} = [m_S_estimate{1:end}];
        R{n+1} = R{n}*ExpOmegaT(w{n},dT/N);
        p{n+1} = p{n} + dT/N*1/bp*sum(store_force_S')';
        G{n+1} = [R{n+1} p{n+1}; [0 0 0 1]];
    end
R_final=R{n+1};
p_final=p{n+1};
G_final=G{n+1};
return


function out=ExpOmegaT(Omega,T)
if norm(Omega)==0 
    out=eye(3);
else
    out=eye(3)...
    +SKEW(Omega)*sin(norm(Omega)*T)/norm(Omega)...
    +SKEW(Omega)*SKEW(Omega)...
    *(1-cos(norm(Omega)*T))/norm(Omega)^2;
end
return

function out=SKEW(a)
out=    [0      -a(3)   a(2);
         a(3)   0       -a(1);
         -a(2)  a(1)    0];
return

function out=UNIT(v)
if v(1)^2+v(2)^2+v(3)^2==0
    out=v;
else
    out=v/norm(v);
end
return