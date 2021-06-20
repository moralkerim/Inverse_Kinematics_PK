function adjoint = Adj(T)
    R = T(1:3,1:3);
    p = T(4,1:3);
    adjoint = [R         zeros(3);
               skew(p)*R R];
end
