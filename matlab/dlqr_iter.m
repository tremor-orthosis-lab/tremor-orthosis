function K = dlqr_iter(A, B, Q, R)
%DLQR_ITER Discrete LQR gain by Riccati iteration (no toolbox needed).
%   u = -K x minimises sum(x'Qx + u'Ru) for x(k+1) = A x + B u.
P = Q;
for it = 1:50000
    K  = (R + B'*P*B) \ (B'*P*A);
    Pn = A'*P*A - A'*P*B*K + Q;
    if max(abs(Pn(:) - P(:))) < 1e-12*max(1, max(abs(P(:)))), P = Pn; break; end
    P = Pn;
end
K = (R + B'*P*B) \ (B'*P*A);
end
