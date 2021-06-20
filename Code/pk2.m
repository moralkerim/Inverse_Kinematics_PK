function [alpha,beta,gama2,u,v] = pk2(w1, w2, pp, qp, r)
    u = pp-r;
    v = qp-r;
    alpha = ((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
    beta = ((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
    gama2 = (norm(u)^2-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(norm(cross(w1,w2))^2);
end